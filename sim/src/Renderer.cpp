#define _USE_MATH_DEFINES
#include "sim/Renderer.hpp"
#include "sim/Game.hpp"
#include "sim/View3D.hpp"
#include "sim/UiText.hpp"
#include <cmath>
#include <cstdio>
#include <array>
#include <cstring>
#include <cctype>
#include <string>
#include "pros/misc.h"

namespace sim {

Renderer::Renderer(SDL_Renderer* ren, int field_w, int field_h, const RobotConfig& cfg)
    : ren_(ren), fw_(field_w), fh_(field_h), cfg_(cfg) {}

void Renderer::setColor(SDL_Renderer* r, uint8_t red, uint8_t g, uint8_t b, uint8_t a) {
    SDL_SetRenderDrawColor(r, red, g, b, a);
}

void Renderer::drawBackground() {
    setColor(ren_, 20, 20, 20);
    SDL_Rect bg = {0, 0, fw_ * 2, fh_ + 80};
    SDL_RenderFillRect(ren_, &bg);
}

// Filled circle via horizontal scanlines
static void fillCircle(SDL_Renderer* ren, int cx, int cy, int r) {
    for (int dy = -r; dy <= r; ++dy) {
        int dx = (int)std::sqrt((double)(r * r - dy * dy));
        SDL_RenderDrawLine(ren, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

// Filled regular octagon (stop-sign orientation) with circumradius r.
// Derived from the flat-edge inradius = r * cos(π/8) ≈ 0.924r
// and the diagonal clipping: half_width = 1.307r − |dy| for corner rows.
static void fillOctagon(SDL_Renderer* ren, int cx, int cy, int r) {
    const int ir  = (int)(0.9239 * r + 0.5);   // inradius (flat-edge distance)
    const int cr  = (int)(1.3066 * r + 0.5);   // inradius + r*sin(π/8) for diagonal clip
    for (int dy = -ir; dy <= ir; ++dy) {
        int hw = (std::abs(dy) <= (int)(0.3827 * r + 0.5)) ? ir : cr - std::abs(dy);
        if (hw > 0)
            SDL_RenderDrawLine(ren, cx - hw, cy + dy, cx + hw, cy + dy);
    }
}

// Octagon outline only
static void drawOctagon(SDL_Renderer* ren, int cx, int cy, int r) {
    SDL_Point pts[9];
    for (int i = 0; i < 8; ++i) {
        double a = (i * 45.0 + 22.5) * M_PI / 180.0;
        pts[i] = { cx + (int)(r * std::cos(a) + 0.5),
                   cy + (int)(r * std::sin(a) + 0.5) };
    }
    pts[8] = pts[0];
    SDL_RenderDrawLines(ren, pts, 9);
}

void Renderer::drawField() {
    // All positions from game diagram are in inches (field = 140.41" × 140.41")
    const double S = (double)fw_ / 140.41;  // px per inch

    // Convert: field has (0,0) at bottom-left, Y increases up
    // SDL has (0,0) at top-left, Y increases down
    auto fx = [&](double in) -> int { return (int)(in * S + 0.5); };
    auto fy = [&](double in) -> int { return fh_ - (int)(in * S + 0.5); };

    // ── Field tiles (6×6 grid, ~23.4" each) ──────────────────────────────────
    setColor(ren_, 50, 50, 50);
    SDL_Rect fieldBg = {0, 0, fw_, fh_};
    SDL_RenderFillRect(ren_, &fieldBg);

    setColor(ren_, 70, 70, 70);
    for (int i = 0; i <= 6; ++i) {
        int pos = fx(i * 140.41 / 6.0);
        SDL_RenderDrawLine(ren_, pos, 0, pos, fh_);
        SDL_RenderDrawLine(ren_, 0, pos, fw_, pos);
    }

    // ── Tape (official drawing) ───────────────────────────────────────────────
    // A strip of tape from (ax, ay) to (bx, by) in field inches, wIn wide
    auto tape = [&](double ax, double ay, double bx, double by, double wIn, SDL_Color col) {
        double dx = bx - ax, dy = by - ay, len = std::max(1e-6, std::hypot(dx, dy));
        double nx = -dy / len * wIn / 2, ny = dx / len * wIn / 2;
        auto P = [&](double x, double y) { return SDL_FPoint{(float)(x * S), (float)(fh_ - y * S)}; };
        SDL_Vertex v[4] = {{P(ax + nx, ay + ny), col, {0, 0}}, {P(bx + nx, by + ny), col, {0, 0}},
                           {P(bx - nx, by - ny), col, {0, 0}}, {P(ax - nx, ay - ny), col, {0, 0}}};
        int idx[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(ren_, nullptr, v, 4, idx, 6);
    };
    const SDL_Color white = {225, 225, 225, 255}, red = {215, 35, 45, 255}, blue = {30, 150, 220, 255};
    const double F = FIELD_IN, c = F / 2, m = MIDFIELD_IN, lz = LOADZONE_X, dEnd = c - m / 2;
    // Midfield diamond
    tape(c, c + m, c + m, c, TAPE_IN, white);
    tape(c + m, c, c, c - m, TAPE_IN, white);
    tape(c, c - m, c - m, c, TAPE_IN, white);
    tape(c - m, c, c, c + m, TAPE_IN, white);
    // Diagonals from the load zones to the Midfield; top-left and bottom-right are the doubled Autonomous Line
    tape(lz, lz, dEnd, dEnd, TAPE_IN, white);
    tape(F - lz, F - lz, F - dEnd, F - dEnd, TAPE_IN, white);
    for (double o : {-0.875, 0.875}) {
        double ox = o * M_SQRT1_2, oy = o * M_SQRT1_2;
        tape(lz + ox, F - lz + oy, dEnd + ox, F - dEnd + oy, 0.75, white);
        tape(F - lz + ox, lz + oy, F - dEnd + ox, dEnd + oy, 0.75, white);
    }
    // Load zones: red Ls in the left corners, blue Ls in the right corners
    for (int sd = 0; sd < 2; ++sd) {
        double x = sd == 0 ? lz : F - lz, wallX = sd == 0 ? 0.0 : F;
        for (double yEdge : {TILE_IN, F - TILE_IN}) {
            double yCorner = yEdge < c ? 0.0 : F;
            tape(wallX, yEdge, x, yEdge, TAPE_IN * 1.4, sd == 0 ? red : blue);
            tape(x, yEdge, x, yCorner, TAPE_IN * 1.4, sd == 0 ? red : blue);
        }
    }

    // ── Loaders: dark base on the wall with the clear tube (alliance rim) seen from above ──
    for (const LoaderDef& ld : LOADERS) {
        int lx = fx(ld.x), ly = fy(ld.y);
        int hw = (int)(LOADER_W_IN / 2 * S + 0.5), hd = (int)(LOADER_D_IN / 2 * S + 0.5);
        setColor(ren_, 32, 33, 36);
        SDL_Rect body = {lx - hd, ly - hw, 2 * hd, 2 * hw};
        SDL_RenderFillRect(ren_, &body);
        if (ld.c == C_RED) setColor(ren_, 220, 60, 75);
        else               setColor(ren_, 60, 170, 225);
        fillCircle(ren_, lx, ly, (int)(LOADER_TUBE_D / 2 * S + 0.5));
        setColor(ren_, 200, 215, 225);
        fillCircle(ren_, lx, ly, (int)(LOADER_TUBE_D / 2 * S - 1.5));
    }

    // Toggles are drawn by the game layer (they change color in play)

    // ── Goals ─────────────────────────────────────────────────────────────────
    // Spec (Appendix A goal drawing): every goal is octagonal, 5.61" across the flats at the base,
    // 3.49" across the top, with a 2.37" opening; seen from above, the sloped faces show as a ring.
    const double K8    = 1.0 / std::cos(M_PI / 8);
    const int GOAL_R   = (int)(2.805 * K8 * S + 0.5);  // base corner radius (px)
    const int CENTER_R = GOAL_R;
    const int TOP_R    = (int)(1.745 * K8 * S + 0.5);  // top corner radius (px)
    const int HOLE_R   = (int)(1.185 * S + 0.5);       // opening radius (px)

    struct GoalDef { double xi, yi; uint8_t r, g, b; bool isTall; };
    static const GoalDef goals[] = {
        // Red alliance goals (bottom-left quadrant)
        {23.11,  46.66,  200,  45,  45, false},
        {46.66,  23.11,  200,  45,  45, false},
        // Blue alliance goals (top-right quadrant)
        {117.30, 93.75,  40, 150, 215, false},
        {93.75, 117.30,  40, 150, 215, false},
        // Neutral black goals (top-left and bottom-right quadrants)
        {46.66, 117.30,  70, 70, 70, false},
        {23.11,  93.75,  70, 70, 70, false},
        {117.30, 46.66,  70, 70, 70, false},
        {93.75,  23.11,  70, 70, 70, false},
        // Center tall goal
        {70.20, 70.20,  70, 70, 70, true},
    };

    for (const auto& g : goals) {
        int gx = fx(g.xi), gy = fy(g.yi);
        int R  = g.isTall ? CENTER_R : GOAL_R;
        int Hr = HOLE_R;

        // Base, then the lighter top face
        setColor(ren_, g.r, g.g, g.b);
        fillOctagon(ren_, gx, gy, R);
        setColor(ren_, 220, 220, 220);
        drawOctagon(ren_, gx, gy, R);
        setColor(ren_, (uint8_t)std::min(255, g.r + 30), (uint8_t)std::min(255, g.g + 30), (uint8_t)std::min(255, g.b + 30));
        fillOctagon(ren_, gx, gy, TOP_R);

        // Dark center hole (the opening / scoring cavity)
        setColor(ren_, 25, 25, 25);
        fillCircle(ren_, gx, gy, Hr);
        setColor(ren_, 90, 90, 90);
        drawOctagon(ren_, gx, gy, Hr);
    }

    // ── Field border ──────────────────────────────────────────────────────────
    setColor(ren_, 180, 180, 180);
    SDL_Rect border = {0, 0, fw_, fh_};
    SDL_RenderDrawRect(ren_, &border);
}

// ── Pixel-font text rendering ──────────────────────────────────────────────────
// 3×5 glyph bitmaps. Each uint8_t = one row, 3 LSBs: bit2=left, bit1=center, bit0=right.
// Rendered at 2× scale → each glyph is 6×10 px; advance = 7 px (6 + 1 gap).

static const struct { char ch; uint8_t rows[5]; } FONT[] = {
    {'0', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'1', {0b010, 0b110, 0b010, 0b010, 0b111}},
    {'2', {0b111, 0b001, 0b111, 0b100, 0b111}},
    {'3', {0b111, 0b001, 0b111, 0b001, 0b111}},
    {'4', {0b101, 0b101, 0b111, 0b001, 0b001}},
    {'5', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'6', {0b111, 0b100, 0b111, 0b101, 0b111}},
    {'7', {0b111, 0b001, 0b001, 0b001, 0b001}},
    {'8', {0b111, 0b101, 0b111, 0b101, 0b111}},
    {'9', {0b111, 0b101, 0b111, 0b001, 0b111}},
    {'.', {0b000, 0b000, 0b000, 0b000, 0b010}},
    {'-', {0b000, 0b000, 0b111, 0b000, 0b000}},
    {':', {0b000, 0b010, 0b000, 0b010, 0b000}},
    {'X', {0b101, 0b101, 0b010, 0b101, 0b101}},
    {'Y', {0b101, 0b101, 0b010, 0b010, 0b010}},
    {' ', {0b000, 0b000, 0b000, 0b000, 0b000}},
};

static void drawText(SDL_Renderer* ren, int x, int y, const char* text) {
    constexpr int SCALE = 2;
    constexpr int ADV   = 3 * SCALE + 1;   // 7 px advance per character
    int cx = x;
    for (const char* p = text; *p; ++p) {
        if (*p == '\n') { y += 5 * SCALE + 3;  cx = x;  continue; }
        // find glyph
        const uint8_t* rows = nullptr;
        for (const auto& g : FONT)
            if (g.ch == *p) { rows = g.rows; break; }
        if (rows) {
            for (int row = 0; row < 5; ++row) {
                uint8_t bits = rows[row];
                for (int col = 0; col < 3; ++col) {
                    if (bits & (1u << (2 - col))) {
                        SDL_Rect px = { cx + col * SCALE, y + row * SCALE, SCALE, SCALE };
                        SDL_RenderFillRect(ren, &px);
                    }
                }
            }
        }
        cx += ADV;
    }
}

// Top-down "3D": something h inches up is drawn this many pixels higher on screen
static constexpr double HEIGHT_PX_PER_IN = 2.0;

static void colorOf(Col c, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (c == C_RED)       { r = 215; g = 50;  b = 50; }
    else if (c == C_BLUE) { r = 30;  g = 150; b = 225; }  // VEX light blue
    else                  { r = 235; g = 205; b = 50; }
}

// Silhouette of a profiled piece (see PIN_HALF / CUP_PROF) in screen pixels: the axis starts at
// (ox, oy) and runs along unit (ux, uy); scale is pixels per inch; col[k] fills segment k.
static void fillProfile(SDL_Renderer* ren, double ox, double oy, double ux, double uy, double scale,
                        const ProfPt* prof, int np, const SDL_Color* col) {
    const double nx = -uy, ny = ux;
    for (int k = 0; k + 1 < np; ++k) {
        double z0 = prof[k].z * scale, z1 = prof[k + 1].z * scale;
        double r0 = prof[k].r * scale, r1 = prof[k + 1].r * scale;
        SDL_FPoint p[4] = {{(float)(ox + ux * z0 + nx * r0), (float)(oy + uy * z0 + ny * r0)},
                           {(float)(ox + ux * z0 - nx * r0), (float)(oy + uy * z0 - ny * r0)},
                           {(float)(ox + ux * z1 - nx * r1), (float)(oy + uy * z1 - ny * r1)},
                           {(float)(ox + ux * z1 + nx * r1), (float)(oy + uy * z1 + ny * r1)}};
        SDL_Vertex v[4];
        for (int i = 0; i < 4; ++i) v[i] = {p[i], col[k], {0, 0}};
        int idx[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(ren, nullptr, v, 4, idx, 6);
    }
}

static SDL_Color sdlCol(Col c) {
    uint8_t r, g, b;
    colorOf(c, r, g, b);
    return {r, g, b, 255};
}

// Whole pin profile (both halves) and its per-segment colors
static int pinProfile2D(const Piece& pc, ProfPt* prof, SDL_Color* col) {
    constexpr int NP = 2 * PIN_HALF_N - 1;
    for (int i = 0; i < PIN_HALF_N; ++i) prof[i] = PIN_HALF[i];
    for (int i = 0; i < PIN_HALF_N - 1; ++i)
        prof[PIN_HALF_N + i] = {PIECE_H_IN - PIN_HALF[PIN_HALF_N - 2 - i].z, PIN_HALF[PIN_HALF_N - 2 - i].r};
    for (int k = 0; k < NP - 1; ++k) col[k] = sdlCol(k < PIN_HALF_N - 1 ? pc.bot : pc.top);
    return NP;
}

// Pieces at true size from above (1 in = PX_IN px)
static void drawPiece(SDL_Renderer* ren, const Piece& pc, int x, int y) {
    if (pc.lying && !pc.isPin) {
        // Cup on its side, from above: its hourglass outline, the opaque half grey and the clear half
        // light, drawn from the opaque end (the outer end, toward `ang`, is its top)
        SDL_Color col[CUP_PROF_N - 1], edge[CUP_PROF_N - 1];
        for (int k = 0; k < CUP_PROF_N - 1; ++k) {
            col[k] = k < CUP_WAIST_I ? SDL_Color{118, 120, 126, 255} : SDL_Color{205, 226, 238, 255};
            edge[k] = {15, 15, 15, 255};
        }
        double ux = std::cos(pc.ang), uy = std::sin(pc.ang);
        if (pc.grayUp) { ux = -ux; uy = -uy; }  // opaque half at the outer end
        double ox = x - ux * HALF_IN * PX_IN, oy = y - uy * HALF_IN * PX_IN;
        ProfPt big[CUP_PROF_N];
        for (int k = 0; k < CUP_PROF_N; ++k) big[k] = {CUP_PROF[k].z, CUP_PROF[k].r + 0.2};
        fillProfile(ren, ox - ux * 0.2 * PX_IN, oy - uy * 0.2 * PX_IN, ux, uy, PX_IN * (PIECE_H_IN + 0.4) / PIECE_H_IN, big, CUP_PROF_N, edge);
        fillProfile(ren, ox, oy, ux, uy, PX_IN, CUP_PROF, CUP_PROF_N, col);
        return;
    }
    if (pc.lying) {
        // Pin on its side: its outline, inner half the bottom color, outer half the top color
        ProfPt prof[2 * PIN_HALF_N - 1];
        SDL_Color col[2 * PIN_HALF_N - 2];
        int np = pinProfile2D(pc, prof, col);
        double ux = std::cos(pc.ang), uy = std::sin(pc.ang);
        double ox = x - ux * HALF_IN * PX_IN, oy = y - uy * HALF_IN * PX_IN;
        SDL_Color edge[2 * PIN_HALF_N - 2];
        ProfPt big[2 * PIN_HALF_N - 1];
        for (int k = 0; k < np; ++k) big[k] = {prof[k].z, prof[k].r + 0.2};
        for (int k = 0; k < np - 1; ++k) edge[k] = {15, 15, 15, 255};
        fillProfile(ren, ox - ux * 0.2 * PX_IN, oy - uy * 0.2 * PX_IN, ux, uy, PX_IN * (PIECE_H_IN + 0.4) / PIECE_H_IN, big, np, edge);
        fillProfile(ren, ox, oy, ux, uy, PX_IN, prof, np, col);
        return;
    }
    if (!pc.isPin) {
        // Cup from above: the rim (3.15 in across), the inside wall of the top half, and the
        // opening at the waist. Light when the clear half is up, gray when the opaque half is up.
        const int rim = (int)(CUP_PROF[CUP_PROF_N - 1].r * PX_IN + 0.5);
        const int wall = (int)((CUP_PROF[CUP_PROF_N - 1].r - CUP_WALL_IN - 0.12) * PX_IN + 0.5);
        const int waist = (int)(CUP_PROF[CUP_WAIST_I].r * PX_IN + 0.5);
        if (pc.grayUp) SDL_SetRenderDrawColor(ren, 118, 120, 126, 255);
        else           SDL_SetRenderDrawColor(ren, 215, 232, 242, 255);
        fillCircle(ren, x, y, rim);
        if (pc.grayUp) SDL_SetRenderDrawColor(ren, 64, 66, 72, 255);
        else           SDL_SetRenderDrawColor(ren, 160, 186, 202, 255);
        fillCircle(ren, x, y, wall);
        if (pc.grayUp) SDL_SetRenderDrawColor(ren, 28, 28, 30, 255);
        else           SDL_SetRenderDrawColor(ren, 112, 134, 150, 255);
        fillCircle(ren, x, y, waist);
    } else {
        // Standing pin from above: the octagonal collar and tapered top half (top color), a thin
        // edge of the bottom half's color, and the small flat tip
        const int collar = (int)(PIN_HALF[PIN_HALF_N - 1].r / std::cos(M_PI / 8) * PX_IN + 0.5);
        uint8_t r, g, b;
        SDL_SetRenderDrawColor(ren, 15, 15, 15, 255);
        fillOctagon(ren, x, y, collar + 1);
        colorOf(pc.bot, r, g, b);
        SDL_SetRenderDrawColor(ren, r, g, b, 255);
        fillOctagon(ren, x, y, collar);
        colorOf(pc.top, r, g, b);
        SDL_SetRenderDrawColor(ren, r, g, b, 255);
        fillOctagon(ren, x, y, collar - 1);
        SDL_SetRenderDrawColor(ren, (uint8_t)std::min(255, r + 35), (uint8_t)std::min(255, g + 35), (uint8_t)std::min(255, b + 35), 255);
        const int tip = std::max(2, (int)(PIN_HALF[2].r * PX_IN + 0.5));
        fillOctagon(ren, x, y, tip);
        SDL_SetRenderDrawColor(ren, (uint8_t)(r * 0.55), (uint8_t)(g * 0.55), (uint8_t)(b * 0.55), 255);
        fillOctagon(ren, x, y, std::max(1, tip - 2));  // the hollow tip
    }
}

static void drawGameLayer(SDL_Renderer* ren, int fw, int fh) {
    Game& gm = game();
    if (gm.p.empty()) return;
    char buf[16];

    // Loaders: the top piece showing in the tube, with a count when more are inside
    for (int li = 0; li < 4; ++li) {
        const auto& ls = gm.loaderStack[li];
        if (ls.empty()) continue;
        int lx = (int)(LOADERS[li].x * PX_IN), ly = (int)(720.0 - LOADERS[li].y * PX_IN);
        drawPiece(ren, gm.p[ls.back()], lx, ly);
        if (gm.p[ls.front()].lz < 0.3) {  // the bottom one, sliding out to the opening once the cover is up
            double xin = LOADERS[li].x + (loaderOutX(li) - LOADERS[li].x) * gm.p[ls.front()].slide;
            drawPiece(ren, gm.p[ls.front()], (int)(xin * PX_IN), ly);
        }
        std::snprintf(buf, sizeof(buf), "%d", (int)ls.size());
        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        drawText(ren, lx + (LOADERS[li].x < 70 ? 12 : -20), ly - 5, buf);
    }

    // Toggles from above: 26 in bars over the middle of each wall, about 2 in deep, showing the
    // face of the color they are set to, with black mounts at the ends (white outline while touched)
    for (int k = 0; k < 4; ++k) {
        uint8_t r, g, b;
        colorOf(gm.toggle[k], r, g, b);
        int t = (int)TOGGLE_HALF_PX, d = (int)(2.19 * PX_IN + 0.5);
        SDL_Rect rc = k == 0 ? SDL_Rect{0, fh / 2 - t, d, 2 * t}
                    : k == 1 ? SDL_Rect{fw - d, fh / 2 - t, d, 2 * t}
                    : k == 2 ? SDL_Rect{fw / 2 - t, fh - d, 2 * t, d}
                             : SDL_Rect{fw / 2 - t, 0, 2 * t, d};
        SDL_SetRenderDrawColor(ren, r, g, b, 255);
        SDL_RenderFillRect(ren, &rc);
        SDL_SetRenderDrawColor(ren, 25, 25, 28, 255);
        SDL_RenderDrawRect(ren, &rc);
        bool vertical = k <= 1;
        for (int e = 0; e < 2; ++e) {
            SDL_Rect mnt = vertical ? SDL_Rect{rc.x, e == 0 ? rc.y - 4 : rc.y + rc.h, rc.w, 4}
                                    : SDL_Rect{e == 0 ? rc.x - 4 : rc.x + rc.w, rc.y, 4, rc.h};
            SDL_RenderFillRect(ren, &mnt);
        }
        if (gm.touching[k]) {
            SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
            SDL_RenderDrawRect(ren, &rc);
        }
    }

    // Loose pieces: cups first, then pins on top
    for (int pass = 0; pass < 2; ++pass)
        for (const Piece& pc : gm.p)
            if (pc.isPin == (pass == 1) && loose(pc)) drawPiece(ren, pc, (int)pc.x, (int)pc.y);

    // Goal stacks, drawn as a column rising toward the top of the screen (one step per piece)
    for (int i = 0; i < 9; ++i) {
        const auto& st = gm.stack[i];
        for (size_t k = 0; k < st.size(); ++k) {
            const Piece& pc = gm.p[st[k]];
            int lift = (int)((GOAL_H_IN[i] + HALF_IN * k) * HEIGHT_PX_PER_IN);
            drawPiece(ren, pc, (int)pc.x, (int)pc.y - lift);
        }
        if (st.size() > 1) {
            const Piece& top = gm.p[st.back()];
            std::snprintf(buf, sizeof(buf), "%d", (int)st.size());
            SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
            drawText(ren, (int)top.x + 9, (int)top.y - 8 - (int)(goalTopIn(gm, i) * HEIGHT_PX_PER_IN), buf);
        }
    }

    // Held piece: shadow on the floor, piece raised by the lift height
    // (green ring = high enough to go over the goal under it)
    if (gm.held >= 0) {
        const Piece& pc = gm.p[gm.held];
        int x = (int)pc.x, y = (int)pc.y;
        int up = (int)(gm.liftIn * HEIGHT_PX_PER_IN);
        SDL_SetRenderDrawColor(ren, 10, 10, 10, 255);
        fillCircle(ren, x, y, 5);
        SDL_SetRenderDrawColor(ren, 140, 140, 140, 255);
        SDL_RenderDrawLine(ren, x, y, x, y - up);
        if (gm.overGoal >= 0 && gm.overGoalOk) SDL_SetRenderDrawColor(ren, 90, 230, 90, 255);
        else                                   SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        fillCircle(ren, x, y - up, 10);
        drawPiece(ren, pc, x, y - up);
        if (gm.rider >= 0) drawPiece(ren, gm.p[gm.rider], x, y - up - (int)(HALF_IN * HEIGHT_PX_PER_IN));
    }

    // Scores, top right: red then blue
    SDL_SetRenderDrawColor(ren, 160, 35, 35, 255);
    SDL_Rect rbox = {fw - 120, 10, 56, 22};
    SDL_RenderFillRect(ren, &rbox);
    SDL_SetRenderDrawColor(ren, 35, 55, 170, 255);
    SDL_Rect bbox = {fw - 60, 10, 56, 22};
    SDL_RenderFillRect(ren, &bbox);
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    std::snprintf(buf, sizeof(buf), "%d", gm.red);
    drawText(ren, fw - 114, 16, buf);
    std::snprintf(buf, sizeof(buf), "%d", gm.blue);
    drawText(ren, fw - 54, 16, buf);

    // Lift gauge under the scores: height of the claw, from the bottom to the DR4B's top, with
    // ticks at the three goal heights (alliance 3.25, neutral 5.8, center 8.7). Square = claw (green closed).
    {
        const int gx0 = fw - 120, gy0 = 36, gw = 116, gh = 10;
        const double top = dr4bMaxLiftIn();
        SDL_SetRenderDrawColor(ren, 40, 40, 40, 255);
        SDL_Rect bg = {gx0, gy0, gw, gh};
        SDL_RenderFillRect(ren, &bg);
        int fill = (int)(std::clamp(gm.liftIn / top, 0.0, 1.0) * gw);
        SDL_SetRenderDrawColor(ren, 220, 200, 60, 255);
        SDL_Rect fr = {gx0, gy0, fill, gh};
        SDL_RenderFillRect(ren, &fr);
        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        for (double h : {3.25, 5.77, 8.77}) {
            int tx = gx0 + (int)(h / top * gw);
            SDL_RenderDrawLine(ren, tx, gy0 - 2, tx, gy0 + gh + 1);
        }
        if (gm.held >= 0) SDL_SetRenderDrawColor(ren, 80, 220, 80, 255);
        else              SDL_SetRenderDrawColor(ren, 90, 90, 90, 255);
        SDL_Rect cl = {gx0 - 14, gy0, 10, gh};
        SDL_RenderFillRect(ren, &cl);
    }
}

// Side view of the robot and whatever is straight ahead of the claw: robot on the left,
// lift and claw at its true height, goals and pieces in its lane at their distance and height.
static void drawSideView(SDL_Renderer* ren, int fw, int y0, int h, double rx, double ry, double heading,
                         double halfW, double halfH) {
    const Game& gm = game();
    const int ground = y0 + h - 14;
    // DR4B geometry (inches): tower pivots, bar angle, carriage top
    const double L = DR4B_BAR_IN, sp = DR4B_SPACING_IN, z0 = dr4bTowerPivotIn();
    const double cth = std::cos(gm.liftAngle), sth = std::sin(gm.liftAngle);
    const double carriageTop = gm.liftIn + 2.0 + sp + 3.0;  // (the lift type globals are set to the player's by the caller)  // top of the carriage and claw
    // px per inch: 7, zooming out when the lift rises past the top of the panel
    const double S = std::clamp((ground - y0 - 6) / std::max(ROBOT_START_H_IN + 1.0, carriageTop + 1.0), 1.9, 7.0);
    auto Y = [&](double in) { return ground - (int)(in * S); };
    SDL_BlendMode prevBlend;
    SDL_GetRenderDrawBlendMode(ren, &prevBlend);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    SDL_SetRenderDrawColor(ren, 28, 30, 34, 255);
    SDL_Rect bg = {0, y0, fw, h};
    SDL_RenderFillRect(ren, &bg);
    SDL_SetRenderDrawColor(ren, 90, 90, 90, 255);
    SDL_RenderDrawLine(ren, 0, ground, fw, ground);

    // Goal-height guide lines across the panel (alliance, neutral, center)
    for (double gh : {3.25, 5.77, 8.77}) {
        SDL_SetRenderDrawColor(ren, 55, 58, 64, 255);
        SDL_RenderDrawLine(ren, 0, Y(gh), fw, Y(gh));
    }

    // Robot: chassis from its back bumper to the front bumper at x = front
    double lenIn = 2 * halfW / PX_IN;
    const int front = 24 + (int)(lenIn * S);
    SDL_SetRenderDrawColor(ren, 150, 40, 40, 255);
    SDL_Rect body = {24, Y(2.6), front - 24, Y(0.6) - Y(2.6)};
    SDL_RenderFillRect(ren, &body);
    SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
    for (double wx : WHEEL_X_IN) fillCircle(ren, 24 + (int)((lenIn / 2 + wx) * S), Y(1.625), (int)(1.625 * S));

    // DR4B from the side: tower near the front, lower bars folding back to the middle link,
    // upper bars forward again to the carriage, which carries the claw straight up and down
    auto X = [&](double inFromFront) { return front - inFromFront * S; };
    auto bar = [&](double xa, double za, double xb, double zb, double thickIn, SDL_Color col) {
        double ax = X(xa), ay = Y(za), bx = X(xb), by = Y(zb);
        double dx = bx - ax, dy = by - ay, len = std::max(1e-6, std::hypot(dx, dy));
        double nx = -dy / len * thickIn * S / 2, ny = dx / len * thickIn * S / 2;
        SDL_Vertex v[4] = {{{(float)(ax + nx), (float)(ay + ny)}, col, {0, 0}}, {{(float)(bx + nx), (float)(by + ny)}, col, {0, 0}},
                           {{(float)(bx - nx), (float)(by - ny)}, col, {0, 0}}, {{(float)(ax - nx), (float)(ay - ny)}, col, {0, 0}}};
        int idx[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(ren, nullptr, v, 4, idx, 6);
    };
    const SDL_Color metal = {170, 172, 178, 255}, darkM = {95, 98, 105, 255}, jawC = {240, 225, 70, 255};
    // DR6B from the side (inches behind the front bumper): main post, the first stage's two sections
    // going back, the meshing gear pair between the stages, the second stage's two coming forward
    const double xt = DR4B_TOWER_BACK_IN, g2 = 2 * DR4B_MID_GEAR_R;
    const double clawY = gm.liftIn + 2.0;
    bar(xt, 2.6, xt, ROBOT_START_H_IN - 0.3, 1.0, metal);                       // main post (18 in tall)
    double xs0 = xt, zb0 = z0;
    const int NSEC = g_liftSections;
    const bool barLift = g_liftType == 1 || g_liftType == 2;
    if (g_liftType == 0) {          // cascade: two nested slides up the post, the carriage on the top one
        bar(xt + 1.0, 3.0, xt + 1.0, 3.0 + std::max(gm.liftIn * 0.5, 4.0) + 10.0, 0.8, darkM);
        bar(xt + 1.8, 3.0 + std::max(gm.liftIn - 4.0, 0.0), xt + 1.8, gm.liftIn + 8.0, 0.8, metal);
    } else if (g_liftType == 3) {   // clawbot: one arm from a pivot low on the post up to the claw
        bar(xt + 7.0, 4.0, xt + 7.0, 6.0, 0.9, darkM);
        bar(xt + 7.0, 5.0, xt, gm.liftIn + 2.0, 0.7, metal);
    }
    for (int k = 0; k < NSEC && barLift; ++k) {
        const int half = NSEC / 2;
        const double Lk = secLen(k);
        double xe0 = xs0 + (k < half ? Lk * cth : -Lk * cth), ze0 = zb0 + Lk * sth;
        bar(xs0, zb0, xe0, ze0, 0.6, metal);
        bar(xs0, zb0 + sp, xe0, ze0 + sp, 0.6, metal);
        if (k + 1 == half) {
            bar(xe0 + 0.3, ze0 - 0.4, xe0 + 0.3, ze0 + 2 * sp + 0.4, 0.7, darkM);  // middle tower
            SDL_SetRenderDrawColor(ren, 165, 32, 48, 255);
            for (double gxp : {xe0, xe0 - g2}) fillCircle(ren, (int)X(gxp), Y(ze0 + sp), std::max(2, (int)(DR4B_MID_GEAR_R * S)));
        }
        xs0 = xe0 - (k + 1 == half ? g2 : 0.0);
        zb0 = ze0 + sp;
    }
    const double xcar = barLift ? xs0 : xt;                                      // the last section's end
    bar(xcar, clawY - 0.4, xcar, clawY + sp + 0.4, 0.7, darkM);                  // carriage
    SDL_SetRenderDrawColor(ren, 30, 30, 32, 255);
    for (double pz : {z0, z0 + sp}) fillCircle(ren, (int)X(xt), Y(pz), 2);
    // Claw: plate from the carriage to the bumper, jaws out past it, ratchet wheel on top
    bar(xcar, gm.liftIn + 4.55, -REACH_IN + 1.4, gm.liftIn + 4.55, 0.3, metal);   // claw top plate, hanging back over the opening
    bar(-REACH_IN, gm.liftIn + 0.35, -REACH_IN, gm.liftIn + 4.9, 0.2, SDL_Color{25, 25, 27, 255});  // shaft
    for (double wzz : {0.85, 1.75, 2.65, 3.55})                                 // wheel stack
        bar(-REACH_IN + 1.35, gm.liftIn + wzz, -REACH_IN - 1.35, gm.liftIn + wzz, 0.56,
            wzz == 1.75 ? SDL_Color{32, 32, 35, 255} : SDL_Color{150, 154, 160, 255});
    bar(-ALIGN_BACK_X, (ALIGN_Z0 + ALIGN_Z1) / 2, -ALIGN_MOUTH_X, (ALIGN_Z0 + ALIGN_Z1) / 2, ALIGN_Z1 - ALIGN_Z0, SDL_Color{215, 60, 65, 255});  // alignment guide
    // Centre of mass and the front / back wheel contact points it tips over
    if (gm.set.physInfo) {
        const double hlIn = lenIn / 2;
        int cmx = (int)X(hlIn - gm.comX), cmy = Y(gm.comZ);
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        fillCircle(ren, cmx, cmy, 5);
        SDL_SetRenderDrawColor(ren, 250, 210, 40, 255);
        fillCircle(ren, cmx, cmy, 3);
        SDL_SetRenderDrawColor(ren, 250, 210, 40, 140);
        SDL_RenderDrawLine(ren, cmx, cmy, cmx, ground);  // straight down to the floor
        for (double e : {-1.0, 1.0}) {
            int ex = (int)X(hlIn - e * TIP_EDGE_X_IN);
            SDL_RenderDrawLine(ren, ex - 3, ground + 4, ex, ground);
            SDL_RenderDrawLine(ren, ex, ground, ex + 3, ground + 4);
        }
    }

    auto drawUpright = [&](const Piece& pc, int cxPx, double bottomIn) {
        if (pc.isPin) {
            ProfPt prof[2 * PIN_HALF_N - 1];
            SDL_Color col[2 * PIN_HALF_N - 2];
            int np = pinProfile2D(pc, prof, col);
            if (pc.lying)  // on its side, resting on the collar
                fillProfile(ren, cxPx - HALF_IN * S, Y(PIN_HALF[PIN_HALF_N - 1].r), 1, 0, S, prof, np, col);
            else
                fillProfile(ren, cxPx, Y(bottomIn), 0, -1, S, prof, np, col);
            return;
        }
        // Cup: opaque half solid, clear half see-through (pins nested in it show)
        SDL_Color col[CUP_PROF_N - 1];
        for (int k = 0; k < CUP_PROF_N - 1; ++k)
            col[k] = k < CUP_WAIST_I ? SDL_Color{95, 97, 103, 255} : SDL_Color{205, 226, 238, 110};
        if (pc.lying) {  // on its side, resting on both rims
            fillProfile(ren, cxPx - HALF_IN * S, Y(CUP_PROF[0].r), 1, 0, S, CUP_PROF, CUP_PROF_N, col);
            return;
        }
        if (pc.grayUp) fillProfile(ren, cxPx, Y(bottomIn + PIECE_H_IN), 0, 1, S, CUP_PROF, CUP_PROF_N, col);
        else           fillProfile(ren, cxPx, Y(bottomIn), 0, -1, S, CUP_PROF, CUP_PROF_N, col);
    };

    // Things in the claw's lane ahead (within 3 in of the center line, up to the panel edge)
    double c = std::cos(heading), sn = std::sin(heading);
    auto ahead = [&](double x, double y, double& distIn, double& sideIn) {
        double dx = x - rx, dy = y - ry;
        distIn = (dx * c + dy * sn - halfW) / PX_IN;  // from the front bumper
        sideIn = (-dx * sn + dy * c) / PX_IN;
    };
    const double maxIn = (fw - front - 10) / S;
    for (int i = 0; i < 9; ++i) {
        double d, sd;
        ahead(GOAL_POS_IN[i][0] * PX_IN, 720.0 - GOAL_POS_IN[i][1] * PX_IN, d, sd);
        if (std::fabs(sd) > 4.0 || d < -GOAL_W_IN || d > maxIn) continue;
        int gxp = front + (int)(d * S);
        if (i < 2)      SDL_SetRenderDrawColor(ren, 200, 45, 45, 255);
        else if (i < 4) SDL_SetRenderDrawColor(ren, 45, 45, 200, 255);
        else            SDL_SetRenderDrawColor(ren, 60, 60, 60, 255);
        int gw = (int)(GOAL_W_IN * S);
        SDL_Rect goal = {gxp - gw / 2, Y(GOAL_H_IN[i]), gw, (int)(GOAL_H_IN[i] * S)};
        SDL_RenderFillRect(ren, &goal);
        SDL_SetRenderDrawColor(ren, 210, 210, 210, 255);
        SDL_RenderDrawRect(ren, &goal);
        const auto& st = gm.stack[i];
        for (size_t k = 0; k < st.size(); ++k) drawUpright(gm.p[st[k]], gxp, stackBottomIn(i, (int)k));
    }
    for (int i = 0; i < (int)gm.p.size(); ++i) {
        const Piece& pc = gm.p[i];
        if (!loose(pc)) continue;
        double d, sd;
        ahead(pc.x, pc.y, d, sd);
        if (std::fabs(sd) > 3.0 || d < 0 || d > maxIn) continue;
        drawUpright(pc, front + (int)(d * S), floorBottomIn(gm, i));
    }

    // Held piece in the jaws
    if (gm.held >= 0) drawUpright(gm.p[gm.held], front + (int)(REACH_IN * S), gm.liftIn);
    if (gm.rider >= 0) drawUpright(gm.p[gm.rider], front + (int)(REACH_IN * S), gm.liftIn + HALF_IN);
    SDL_SetRenderDrawBlendMode(ren, prevBlend);
}

void Renderer::drawCursorOverlay(const SimState& state) {
    int mx = state.mouse_x.load();
    int my = state.mouse_y.load();
    if (mx < 0 || my < 0 || mx >= fw_ || my >= fh_) return;

    // Convert SDL pixel → field inches, origin at field center
    const double S  = fw_ / 140.41;
    double field_x  =  (mx / S) - 70.205;
    double field_y  = -(my / S) + 70.205;  // flip Y

    char buf[32];
    std::snprintf(buf, sizeof(buf), "X:%.1f\nY:%.1f", field_x, field_y);

    // Background box (top-left corner of field)
    constexpr int PAD = 3;
    constexpr int BOX_W = 54, BOX_H = 27;
    setColor(ren_, 0, 0, 0, 180);
    SDL_Rect bg = { PAD, PAD, BOX_W, BOX_H };
    SDL_RenderFillRect(ren_, &bg);

    setColor(ren_, 200, 200, 200);
    drawText(ren_, PAD + 3, PAD + 3, buf);
}

void Renderer::drawRobot(double x, double y, double heading) {
    const Game& g = game();
    setLiftType(g.set.liftBot[3]);
    drawRobotTop(x, y, heading, g.liftAngle, g.wheelRotL, g.wheelRotR, g.clawDeg, C_RED);
}

// The robot from above (this one, or one of the other robots, which are the same machine)
void Renderer::drawRobotTop(double x, double y, double heading, double liftAngle, double wheelRotL, double wheelRotR,
                            double clawDeg, int team) {
    double hw = cfg_.robot_half_w;
    double hh = cfg_.robot_half_h;

    // Four corners of robot in local space
    double cx[4] = {-hw, hw,  hw, -hw};
    double cy[4] = {-hh, -hh, hh,  hh};

    double cosH = std::cos(heading);
    double sinH = std::sin(heading);

    // Rotate and translate to world space
    SDL_Point pts[5];
    for (int i = 0; i < 4; ++i) {
        pts[i].x = static_cast<int>(x + cx[i] * cosH - cy[i] * sinH);
        pts[i].y = static_cast<int>(y + cx[i] * sinH + cy[i] * cosH);
    }
    pts[4] = pts[0]; // close the rectangle

    // Robot from above, at true size: base plate, C-channel rails and cross members, six omni
    // wheels, drive motors, battery and brain, and the DR4B towers and bars (bar length shrinks
    // as the lift rises). Positions match the 3D model.
    {
        const double hl = hw / PX_IN, hs = hh / PX_IN;
        auto part = [&](double f0, double f1, double s0, double s1, SDL_Color col) {
            double cf[4] = {f0, f1, f1, f0}, cs[4] = {s0, s0, s1, s1};
            SDL_Vertex v[4];
            for (int i = 0; i < 4; ++i) {
                double fx = cf[i] * PX_IN, sy = cs[i] * PX_IN;
                v[i].position = {(float)(x + fx * cosH - sy * sinH), (float)(y + fx * sinH + sy * cosH)};
                v[i].color = col;
                v[i].tex_coord = {0, 0};
            }
            int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(ren_, nullptr, v, 4, idx, 6);
        };
        const SDL_Color plate = {48, 48, 54, 255}, alu = {190, 193, 198, 255}, tire = {18, 18, 20, 255},
                        hub = {120, 122, 128, 255}, motorC = {28, 28, 30, 255}, screen = {60, 120, 190, 255},
                        lift = {165, 168, 175, 255};
        const double wheelY = std::min(6.5, hs - 1.0);
        // Base plate, with the front opening left open
        part(-hl, hl - OPEN_DEPTH_IN, -hs, hs, plate);
        part(hl - OPEN_DEPTH_IN, hl, OPEN_HALF_IN, hs, plate);
        part(hl - OPEN_DEPTH_IN, hl, -hs, -OPEN_HALF_IN, plate);
        for (double sd : {-1.0, 1.0}) {
            for (double off : {0.75, -0.75}) part(-hl, hl, sd * (wheelY + off) - 0.25, sd * (wheelY + off) + 0.25, alu);
            for (double f : WHEEL_X_IN) {
                part(f - 1.625, f + 1.625, sd * wheelY - 0.55, sd * wheelY + 0.55, tire);
                {   // tread marks sliding along as the wheel turns
                    const double spin = sd > 0 ? wheelRotR : wheelRotL, pitch = 3.25 / 3;
                    double off = std::fmod(-spin * 1.625, pitch);
                    if (off < 0) off += pitch;
                    for (double t = off; t < 3.25; t += pitch)
                        part(f - 1.625 + t - 0.08, f - 1.625 + t + 0.08, sd * wheelY - 0.5, sd * wheelY + 0.5, hub);
                }
                part(f - 0.9, f + 0.9, sd * wheelY + sd * 0.45, sd * wheelY + sd * 0.6, hub);
                {   // drive motor beside its pinion (front motor behind the wheel, the others ahead)
                    double mxp = f + 2.1 * M_SQRT1_2;
                    part(mxp - 0.65, mxp + 0.65, sd * (wheelY - 1.9) - 0.65, sd * (wheelY - 1.9) + 0.65, motorC);
                }
            }
        }
        for (double f : {-1.0, 0.0})
            part(f * (hl - 0.25) - 0.25, f * (hl - 0.25) + 0.25, -wheelY - 0.75, wheelY + 0.75, alu);
        for (double sd : {-1.0, 1.0})  // front pieces either side of the opening
            part(hl - 0.5, hl, sd > 0 ? OPEN_HALF_IN : -wheelY - 0.75, sd > 0 ? wheelY + 0.75 : -OPEN_HALF_IN, alu);
        part(hl - OPEN_DEPTH_IN - 0.5, hl - OPEN_DEPTH_IN, -wheelY - 0.75, wheelY + 0.75, alu);  // behind the opening
        part(-hl + 1.2, -hl + 5.2, -2.9, 2.9, motorC);                     // brain
        part(-hl + 2.1, -hl + 4.7, -2.0, 2.0, screen);                     // its screen
        // DR6B from above: the first stage's two sections go back from the main post, the meshing
        // gear pair between the stages, then the second stage's two sections come forward
        const double yS = OPEN_HALF_IN + 0.5, xt = hl - DR4B_TOWER_BACK_IN, g2 = 2 * DR4B_MID_GEAR_R;
        const double lcos = std::cos(liftAngle);
        const SDL_Color gearC = {165, 32, 48, 255};
        const int NSEC = g_liftSections;
        const bool barLift = g_liftType == 1 || g_liftType == 2;
        double xs0 = xt;
        if (!barLift) {  // cascade: a slide beside each post; clawbot: one arm from a low pivot back of the post up to the claw
            for (double sd : {-1.0, 1.0}) {
                if (g_liftType == 0) part(xt - 0.5, xt + 0.5, sd * (yS + 0.5) - 0.3, sd * (yS + 0.5) + 0.3, lift);
                else part(xt, xt + 7.0, sd * (yS + 0.5) - 0.3, sd * (yS + 0.5) + 0.3, lift);
            }
        }
        for (int k = 0; k < NSEC && barLift; ++k) {
            const int half = NSEC / 2;
            const double lc = secLen(k) * lcos;
            double xe0 = xs0 + (k < half ? -lc : lc);
            for (double sd : {-1.0, 1.0}) {
                double yk = sd * (yS + (k < half ? -0.75 : 0.75));
                part(std::min(xs0, xe0), std::max(xs0, xe0), yk - 0.22, yk + 0.22, lift);
                if (k + 1 == half) {
                    part(xe0 - 0.5, xe0 + g2 + 0.5, sd * yS - 0.12, sd * yS + 0.12, motorC);  // middle tower
                    part(xe0 - 0.75, xe0 + g2 + 0.75, sd * (yS - 0.3) - 0.15, sd * (yS - 0.3) + 0.15, gearC); // its gears
                }
            }
            xs0 = xe0 + (k + 1 == half ? g2 : 0.0);
        }
        for (double sd : {-1.0, 1.0}) {
            part(xt - 0.6, xt + 0.6, sd * yS - 0.5, sd * yS + 0.5, alu);                         // main post
            part(xt - 1.75, xt + 1.75, sd * (yS + 0.7) - 0.18, sd * (yS + 0.7) + 0.18, gearC);   // 84-tooth lift gear
        }
        {   // claw plate hanging back from the carriage over the opening
            const double gxp = hl + REACH_IN;
            const double xc = barLift ? xs0 : xt;
            part(std::min(gxp - 1.4, xc - 0.4), xc + 0.2, -3.2, 3.2, lift);
        }
        // Goal alignment guide (the red V in front)
        auto bar = [&](double f0, double s0, double f1, double s1, double wIn, SDL_Color col) {
            double df = f1 - f0, ds = s1 - s0, l = std::max(1e-6, std::hypot(df, ds));
            double nf = -ds / l * wIn / 2, ns = df / l * wIn / 2;
            double cf[4] = {f0 + nf, f1 + nf, f1 - nf, f0 - nf}, cs[4] = {s0 + ns, s1 + ns, s1 - ns, s0 - ns};
            SDL_Vertex v[4];
            for (int i = 0; i < 4; ++i) {
                double fxp = cf[i] * PX_IN, syp = cs[i] * PX_IN;
                v[i] = {{(float)(x + fxp * cosH - syp * sinH), (float)(y + fxp * sinH + syp * cosH)}, col, {0, 0}};
            }
            int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(ren_, nullptr, v, 4, idx, 6);
        };
        const SDL_Color guide = {215, 60, 65, 255};
        for (double sd : {-1.0, 1.0})
            bar(hl + ALIGN_BACK_X, sd * ALIGN_BACK_HALF, hl + ALIGN_MOUTH_X, sd * ALIGN_MOUTH_HALF, 0.35, guide);
        bar(hl + ALIGN_BACK_X, -ALIGN_BACK_HALF, hl + ALIGN_BACK_X, ALIGN_BACK_HALF, 0.35, guide);
        // Toggle arms on the back of the DR6B: out behind the middle tower, rubber pad on the end
        {
            double padX, padZ;
            ramPadPose(liftAngle, hl, padX, padZ);
            const double xm = padX + RAM_ARM_BACK_IN;
            for (double sd : {-1.0, 1.0}) {
                part(padX + 0.3, xm + 0.2, sd * RAM_Y_IN - 0.3, sd * RAM_Y_IN + 0.3, alu);
                part(padX - 0.15, padX + 0.45, sd * RAM_Y_IN - 0.9, sd * RAM_Y_IN + 0.9, SDL_Color{28, 28, 30, 255});
            }
        }
    }
    if (team == C_RED) setColor(ren_, 215, 45, 50);  // alliance outline
    else               setColor(ren_, 55, 115, 235);
    SDL_RenderDrawLines(ren_, pts, 5);

    // Ratcheting claw: two jaws on pivots just past the front bumper that swing in to grip
    {
        auto toWorld = [&](double fwdIn, double sideIn, int& ox, int& oy) {
            double fx = hw + fwdIn * PX_IN, sy = sideIn * PX_IN;
            ox = (int)(x + cosH * fx - sinH * sy);
            oy = (int)(y + sinH * fx + cosH * sy);
        };
        // Ratcheting roller claw from above: the two wheel stacks beside the grip point
        const double gxF = REACH_IN, sy = 2.35, wR = 1.35;
        for (int side = -1; side <= 1; side += 2) {
            int cxp, cyp;
            toWorld(gxF, side * sy, cxp, cyp);
            setColor(ren_, 150, 154, 160);
            fillCircle(ren_, cxp, cyp, (int)(wR * PX_IN + 0.5));
            setColor(ren_, 60, 62, 68);
            double a = clawDeg * M_PI / 180.0 * 1.5 * -side;
            for (int k = 0; k < 2; ++k) {  // a spoke line so the spin shows
                double aa = a + k * M_PI / 2;
                int ex, ey, fx2, fy2;
                toWorld(gxF + std::cos(aa) * wR, side * sy + std::sin(aa) * wR, ex, ey);
                toWorld(gxF - std::cos(aa) * wR, side * sy - std::sin(aa) * wR, fx2, fy2);
                SDL_RenderDrawLine(ren_, ex, ey, fx2, fy2);
            }
            setColor(ren_, 25, 25, 27);
            fillCircle(ren_, cxp, cyp, 1);
        }
    }

}

void Renderer::drawHUD(const SimState& state, double x, double y, double heading) {
    // We can't render fonts without SDL_ttf, so we draw simple colored bars
    // representing motor voltages instead.

    // HUD background
    setColor(ren_, 20, 20, 20);
    SDL_Rect hudBg = {0, fh_, fw_, 80};
    SDL_RenderFillRect(ren_, &hudBg);

    // Motor voltage bars (ports 1–10)
    for (int port = 1; port <= 10; ++port) {
        int voltage = state.motors[port].voltage.load(); // -127..127
        int barMaxH = 34;
        int barW    = fw_ / 10 - 4;
        int barX    = (port - 1) * (fw_ / 10) + 2;

        // Background bar
        setColor(ren_, 50, 50, 50);
        SDL_Rect bg = {barX, fh_ + 4, barW, barMaxH * 2};
        SDL_RenderFillRect(ren_, &bg);

        int center = fh_ + 4 + barMaxH;
        int barH   = static_cast<int>((std::abs(voltage) / 127.0) * barMaxH);

        if (voltage > 0) {
            setColor(ren_, 80, 200, 80);
            SDL_Rect bar = {barX, center - barH, barW, barH};
            SDL_RenderFillRect(ren_, &bar);
        } else if (voltage < 0) {
            setColor(ren_, 200, 80, 80);
            SDL_Rect bar = {barX, center, barW, barH};
            SDL_RenderFillRect(ren_, &bar);
        }

        // Centerline
        setColor(ren_, 120, 120, 120);
        SDL_RenderDrawLine(ren_, barX, center, barX + barW, center);
    }

    // Comp mode indicator
    CompMode mode = state.comp_mode.load();
    if (mode == CompMode::OPCONTROL) {
        setColor(ren_, 80, 200, 80);
    } else {
        setColor(ren_, 150, 150, 150);
    }
    SDL_Rect modeRect = {fw_ - 14, fh_ + 4, 10, 10};
    SDL_RenderFillRect(ren_, &modeRect);
}

// ── Controls sidebar (right edge of the window) ───────────────────────────────
// 5×7 capital-letter font, drawn at 2× (10×14 px, 12 px advance). Each row = 5 bits, bit4 = left.
static const struct { char ch; uint8_t rows[7]; } FONT57[] = {
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}, {'D', {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}}, {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}}, {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}}, {'J', {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}}, {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}}, {'N', {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}}, {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}}, {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}}, {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04}}, {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {'/', {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x00}}, {'(', {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}},
    {')', {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}}, {'[', {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}},
    {']', {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E}}, {';', {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x04, 0x08}},
    {':', {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}}, {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}},
    {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}}, {'+', {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}},
    {'%', {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03}}, {'>', {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}},
    {'<', {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}},
};

static constexpr int SB_SCALE  = 2;
static constexpr int SB_TEXT_H = 7 * SB_SCALE;  // 14 px

static void drawText57(SDL_Renderer* ren, int x, int y, const char* text, int scale = SB_SCALE) {
    for (const char* p = text; *p; ++p, x += 6 * scale) {
        char c = (*p >= 'a' && *p <= 'z') ? (char)(*p - 32) : *p;
        for (const auto& g : FONT57) {
            if (g.ch != c) continue;
            for (int row = 0; row < 7; ++row)
                for (int col = 0; col < 5; ++col)
                    if (g.rows[row] & (0x10 >> col)) {
                        SDL_Rect px = {x + col * scale, y + row * scale, scale, scale};
                        SDL_RenderFillRect(ren, &px);
                    }
            break;
        }
    }
}

static int textW57(const char* text, int scale = SB_SCALE) {
    int n = (int)std::strlen(text);
    return n > 0 ? n * 6 * scale - scale : 0;
}

// The start screen's labels are written in capitals; show them in a quieter case (words with digits, one letters and acronyms stay as they are)
static std::string uiCase(const std::string& s) {
    static const char* keep[] = {"VEX", "RPM", "WP", "CPU", "AM", "PM", "PTS", "DR4B", "DR6B", "ID"};
    static const char* small[] = {"OF", "TO", "AND", "THE", "IN", "ON", "VS", "FOR", "A", "AT", "OR"};
    int words = 0;
    for (size_t i = 0; i < s.size();) {
        if (!std::isalnum((unsigned char)s[i])) { ++i; continue; }
        size_t j = i; bool up = true, alpha = true;
        while (j < s.size() && std::isalnum((unsigned char)s[j])) { if (std::islower((unsigned char)s[j])) up = false; if (std::isdigit((unsigned char)s[j])) alpha = false; ++j; }
        if (up && alpha && j - i > 1) ++words;
        i = j;
    }
    const bool sentence = words >= 5;
    std::string out = s;
    int idx = 0;
    for (size_t i = 0; i < s.size();) {
        if (!std::isalnum((unsigned char)s[i])) { ++i; continue; }
        size_t j = i; bool up = true, alpha = true;
        while (j < s.size() && std::isalnum((unsigned char)s[j])) { if (std::islower((unsigned char)s[j])) up = false; if (std::isdigit((unsigned char)s[j])) alpha = false; ++j; }
        const std::string w = s.substr(i, j - i);
        bool kept = !up || !alpha || (w.size() <= 1 && w != "A");
        for (const char* k : keep) if (w == k) kept = true;
        if (!kept) {
            bool isSmall = false;
            for (const char* k : small) if (w == k) isSmall = true;
            for (size_t c = i; c < j; ++c) out[c] = (char)std::tolower((unsigned char)s[c]);
            if (idx == 0 || (!sentence && !isSmall)) out[i] = (char)std::toupper((unsigned char)s[i]);
        }
        ++idx;
        i = j;
    }
    return out;
}

// Start screen text: smooth Segoe UI where available (scale 2 is about 18 px), else the pixel font
static int dashPx(int scale) { return scale <= 1 ? 11 : (int)std::lround(9.2 * scale); }
[[maybe_unused]] static int dashTextW(const std::string& t, int scale, bool bold) {
    return uiTextOk() ? uiTextWidth(t.c_str(), dashPx(scale), bold) : textW57(t.c_str(), scale);
}
[[maybe_unused]] static void dashText(SDL_Renderer* ren, int x, int yTop, const std::string& t, int scale, SDL_Color c, bool bold) {
    if (uiTextOk()) {
        const int px = dashPx(scale);
        uiDrawText(ren, x, yTop - (int)std::lround(0.34 * px), t.c_str(), px, c, bold);
    } else {
        SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, c.a);
        drawText57(ren, x, yTop, t.c_str(), scale);
    }
}

// One sidebar row: a key cap, the controller button it stands for, and what it does.
// The cap lights up while that key is held (or, for the mode keys, while that mode runs).
struct SidebarRow { const char* key; const char* button; const char* action; bool lit; };

static void drawSidebar(SDL_Renderer* ren, const SimState& state, int w, int h) {
    const int PAD = 14, CAP_W = 58, CAP_H = 22, BTN_X = PAD + CAP_W + 8, ACT_X = BTN_X + 32, ROW_H = 27;

    SDL_SetRenderDrawColor(ren, 24, 28, 34, 255);
    SDL_Rect bg = {0, 0, w, h};
    SDL_RenderFillRect(ren, &bg);
    SDL_SetRenderDrawColor(ren, 60, 66, 76, 255);
    SDL_RenderDrawLine(ren, 0, 0, 0, h);

    SDL_SetRenderDrawColor(ren, 235, 70, 70, 255);
    drawText57(ren, PAD, 18, "CONTROLS", 3);

    const int ly = state.controller.leftY.load(), ry = state.controller.rightY.load();
    const uint32_t btn = state.controller.buttons.load();
    auto held = [&](controller_digital_e_t b) { return (btn & (1u << (int)b)) != 0; };
    const CompMode mode = state.comp_mode.load();

    // Split arcade (sim keys): forward = both sides together, turn = the difference
    const int fwd = ly + ry, turn = ly - ry;
    const SidebarRow drive[] = {
        {"W",     "", "FORWARD",    fwd > 0},  {"S",     "", "BACKWARD",   fwd < 0},
        {"A",     "", "TURN LEFT",  turn < 0}, {"D",     "", "TURN RIGHT", turn > 0},
    };
    const SidebarRow buttons[] = {
        {"UP",    "L1", "LIFT UP",       held(E_CONTROLLER_DIGITAL_L1)},
        {"DOWN",  "L2", "LIFT DOWN",     held(E_CONTROLLER_DIGITAL_L2)},
        {"RIGHT", "R1", "CLAW CLOSE",    held(E_CONTROLLER_DIGITAL_R1)},
        {"LEFT",  "R2", "CLAW OPEN",     held(E_CONTROLLER_DIGITAL_R2)},
        {"Z",     "",   "LOAD PIN",      false},
        {"X",     "",   "LOAD CUP",      false},
    };
    const SidebarRow match[] = {
        {"F6",  "", "DRIVER",      mode == CompMode::OPCONTROL},
        {"F7",  "", "DISABLE",     mode == CompMode::DISABLED},
        {"F8",  "", "END MATCH",   false}, {"F9", "", "15 S LEFT", false},
        {"R",   "", "RESET FIELD", false}, {"1-9", "", "CAMERA C/V",  false},
        {"TAB", "", "SETTINGS",    game().menuOpen},
        {"F11", "", "FULL SCREEN", false}, {"ESC", "", "QUIT",        false},
    };

    int y = 66;
    auto section = [&](const char* title, const SidebarRow* rows, int n) {
        SDL_SetRenderDrawColor(ren, 130, 140, 155, 255);
        drawText57(ren, PAD, y, title);
        SDL_SetRenderDrawColor(ren, 60, 66, 76, 255);
        SDL_RenderDrawLine(ren, PAD + textW57(title) + 8, y + SB_TEXT_H / 2, w - PAD, y + SB_TEXT_H / 2);
        y += SB_TEXT_H + 12;
        for (int i = 0; i < n; ++i, y += ROW_H) {
            const SidebarRow& r = rows[i];
            SDL_Rect cap = {PAD, y - 4, CAP_W, CAP_H};
            if (r.lit) SDL_SetRenderDrawColor(ren, 245, 190, 60, 255);
            else       SDL_SetRenderDrawColor(ren, 44, 50, 60, 255);
            SDL_RenderFillRect(ren, &cap);
            if (r.lit) SDL_SetRenderDrawColor(ren, 245, 190, 60, 255);
            else       SDL_SetRenderDrawColor(ren, 90, 98, 112, 255);
            SDL_RenderDrawRect(ren, &cap);
            if (r.lit) SDL_SetRenderDrawColor(ren, 25, 25, 25, 255);
            else       SDL_SetRenderDrawColor(ren, 235, 235, 235, 255);
            drawText57(ren, PAD + (CAP_W - textW57(r.key)) / 2, y, r.key);
            SDL_SetRenderDrawColor(ren, 120, 170, 230, 255);
            drawText57(ren, BTN_X, y, r.button);
            if (r.lit) SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
            else       SDL_SetRenderDrawColor(ren, 185, 192, 202, 255);
            drawText57(ren, ACT_X, y, r.action);
        }
        y += 14;
    };
    section("DRIVE", drive, 4);
    section("BUTTONS", buttons, 6);
    section("MATCH", match, (int)(sizeof(match) / sizeof(match[0])));
    SDL_SetRenderDrawColor(ren, 130, 140, 155, 255);
    drawText57(ren, PAD, y, "VIEW:");
    SDL_SetRenderDrawColor(ren, 245, 190, 60, 255);
    drawText57(ren, PAD + 72, y, camName(game().camMode));

    SDL_SetRenderDrawColor(ren, 130, 140, 155, 255);
    drawText57(ren, PAD, h - 50, "ALSO E Q SPACE F");
    drawText57(ren, PAD, h - 30, "CLAW GRABS ON CONTACT");
}

void Renderer::draw(const SimState& state) {
    double px, py, heading;
    {
        std::lock_guard<std::mutex> lock(state.pose_mutex);
        px      = state.pose.x;
        py      = state.pose.y;
        heading = state.pose.heading;
    }

    SDL_SetRenderDrawColor(ren_, 0, 0, 0, 255);  // black bars around the scaled picture
    SDL_RenderClear(ren_);
    drawBackground();
    drawField();
    drawGameLayer(ren_, fw_, fh_);
    // Path trail (settings menu): the robot centre over the last minute, fading out
    {
        const auto& tr = game().trail;
        if (game().set.trail && tr.size() > 1) {
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
            for (size_t k = 1; k < tr.size(); ++k) {
                Uint8 a = (Uint8)(60 + 195 * k / tr.size());
                SDL_SetRenderDrawColor(ren_, 80, 230, 255, a);
                SDL_RenderDrawLine(ren_, (int)tr[k - 1].first, (int)tr[k - 1].second, (int)tr[k].first, (int)tr[k].second);
                SDL_RenderDrawLine(ren_, (int)tr[k - 1].first + 1, (int)tr[k - 1].second, (int)tr[k].first + 1, (int)tr[k].second);
            }
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
        }
    }
    // Other robots: the same robot from above, with their number and the piece in their claw
    for (const Bot& b : game().bots) {
        if (!b.on) continue;
        setLiftType(b.lift);
        drawRobotTop(b.x, b.y, b.h, dr4bAngleRad(b.liftDeg), b.wheelRotL, b.wheelRotR, b.clawDeg, b.team);
        if (b.carry >= 0 && b.carry < (int)game().p.size()) {
            const Piece& pc = game().p[b.carry];
            drawPiece(ren_, pc, (int)pc.x, (int)pc.y);
            if (b.rider >= 0) drawPiece(ren_, game().p[b.rider], (int)pc.x, (int)pc.y - 4);
        }
        setColor(ren_, 245, 245, 245);
        drawText57(ren_, (int)(b.x - std::cos(b.h) * 4.5 * PX_IN) - textW57(b.num, 1) / 2, (int)(b.y - std::sin(b.h) * 4.5 * PX_IN) - 4, b.num, 1);
    }
    if (!game().playerOff) drawRobot(px, py, heading);
    // Physics info: the support area (wheel contact edges) and the centre of mass from above. The
    // area goes amber, then red, as the steady braking that would tip the robot drops toward the
    // wheels' grip (0.8 g); the robot can tip once it is below that.
    if (game().set.physInfo) {
        const Game& gm = game();
        const double c = std::cos(heading), s = std::sin(heading), hl = cfg_.robot_half_w / PX_IN;
        auto toPx = [&](double fx, double ly) {  // robot-frame inches (x forward, y left) -> sim px
            return SDL_Point{(int)(px + (fx * c + ly * s) * PX_IN), (int)(py + (fx * s - ly * c) * PX_IN)};
        };
        const double xE = TIP_EDGE_X_IN, yE = TIP_EDGE_Y_IN;
        double worst = std::min({gm.tipBrakeG, gm.tipAccelG, gm.tipSideG});
        if (worst < 0.8)      setColor(ren_, 240, 70, 60);
        else if (worst < 1.0) setColor(ren_, 245, 180, 50);
        else                  setColor(ren_, 90, 220, 120);
        SDL_Point sq[5] = {toPx(xE, yE), toPx(xE, -yE), toPx(-xE, -yE), toPx(-xE, yE), toPx(xE, yE)};
        SDL_RenderDrawLines(ren_, sq, 5);
        SDL_Point cm = toPx(gm.comX, gm.comY);
        setColor(ren_, 20, 20, 20);
        fillCircle(ren_, cm.x, cm.y, 6);
        setColor(ren_, 250, 210, 40);
        fillCircle(ren_, cm.x, cm.y, 4);
        setColor(ren_, 20, 20, 20);
        SDL_RenderDrawLine(ren_, cm.x - 4, cm.y, cm.x + 4, cm.y);
        SDL_RenderDrawLine(ren_, cm.x, cm.y - 4, cm.x, cm.y + 4);
    }
    drawCursorOverlay(state);
    // Position readout (settings menu): field inches from the bottom-left corner, and compass heading
    if (game().set.pose) {
        char buf[48];
        double hdg = std::fmod(heading * 180.0 / M_PI + 90.0, 360.0);
        if (hdg < 0) hdg += 360.0;
        std::snprintf(buf, sizeof(buf), "X %.1f  Y %.1f  H %.0f", px / PX_IN, (720.0 - py) / PX_IN, hdg);
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
        setColor(ren_, 0, 0, 0, 190);
        SDL_Rect bx = {3, 34, textW57(buf) + 12, 22};
        SDL_RenderFillRect(ren_, &bx);
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
        setColor(ren_, 120, 230, 255);
        drawText57(ren_, 9, 38, buf);
    }
    drawHUD(state, px, py, heading);
    // Right half: 3D view on top, side view below
    {
        const int h3 = fh_ + 80 - SIDE_VIEW_H;
        SDL_Rect vp3 = {fw_, 0, fw_, h3};
        SDL_RenderSetViewport(ren_, &vp3);
        if (game().set.dim) setColor(ren_, 16, 18, 24);
        else                setColor(ren_, 38, 44, 54);
        SDL_Rect sky = {0, 0, fw_, h3};
        SDL_RenderFillRect(ren_, &sky);
        v3d::draw(ren_, fw_, h3, px, py, heading, cfg_.robot_half_w, cfg_.robot_half_h);
        // Physics readout over the 3D view: mass, centre of mass, the steady accelerations that would
        // tip the robot (red below the 0.8 g the wheels can grip), and its lean right now
        const Game& gmI = game();
        if (gmI.set.physInfo && !gmI.menuOpen) {
            char l1[64], l2[64], l3[64];
            std::snprintf(l1, sizeof(l1), "MASS %.1f LB  COM %.1f FWD %.1f UP", gmI.massLb, gmI.comX, gmI.comZ);
            std::snprintf(l2, sizeof(l2), "TIPS AT  BRAKE %.2fG  ACCEL %.2fG  SIDE %.2fG", gmI.tipBrakeG, gmI.tipAccelG, gmI.tipSideG);
            if (gmI.set.tipping)
                std::snprintf(l3, sizeof(l3), "PITCH %.1f  ROLL %.1f DEG", gmI.pitch * 180.0 / M_PI, gmI.roll * 180.0 / M_PI);
            else
                std::snprintf(l3, sizeof(l3), "TIPPING OFF (TAB SETTINGS)");
            int wBox = std::max({textW57(l1), textW57(l2), textW57(l3)}) + 16;
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
            setColor(ren_, 0, 0, 0, 170);
            SDL_Rect bx = {6, 6, wBox, 66};
            SDL_RenderFillRect(ren_, &bx);
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
            setColor(ren_, 250, 210, 40);
            drawText57(ren_, 14, 12, l1);
            double worst = std::min({gmI.tipBrakeG, gmI.tipAccelG, gmI.tipSideG});
            if (worst < 0.8)      setColor(ren_, 240, 80, 70);
            else if (worst < 1.0) setColor(ren_, 245, 180, 50);
            else                  setColor(ren_, 140, 225, 150);
            drawText57(ren_, 14, 32, l2);
            setColor(ren_, 200, 205, 215);
            drawText57(ren_, 14, 52, l3);
        }
        if (gmI.tipped) {
            const char* msg = "TIPPED OVER";
            const char* sub = "PRESS R TO RESET";
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
            setColor(ren_, 0, 0, 0, 190);
            SDL_Rect bx = {fw_ / 2 - 200, h3 / 2 - 50, 400, 100};
            SDL_RenderFillRect(ren_, &bx);
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
            setColor(ren_, 240, 70, 60);
            drawText57(ren_, fw_ / 2 - textW57(msg, 4) / 2, h3 / 2 - 34, msg, 4);
            setColor(ren_, 230, 230, 230);
            drawText57(ren_, fw_ / 2 - textW57(sub) / 2, h3 / 2 + 14, sub);
        }
        // The referee's last call (Load Zone violations), across the top of the 3D view
        if (gmI.violMsgT > 0.0 && !gmI.violMsg.empty()) {
            const int tw = textW57(gmI.violMsg.c_str(), 2);
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
            setColor(ren_, 0, 0, 0, 200);
            SDL_Rect vb = {fw_ / 2 - tw / 2 - 14, 84, tw + 28, 34};
            SDL_RenderFillRect(ren_, &vb);
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
            setColor(ren_, 250, 200, 60);
            drawText57(ren_, fw_ / 2 - tw / 2, 94, gmI.violMsg.c_str(), 2);
        }
        // Live scoreboard and match timer (SCOREBOARD setting), along the bottom of the 3D view
        if (gmI.set.hud != 0 || gmI.matchOver) {  // (the result banner shows even with the scoreboard and timer off)
            const bool showScore = gmI.set.hud & 1, showTime = gmI.set.hud & 2;
            const int bh = 64, by = h3 - bh - 8, boxW = 150, timeW = 150;
            const int total = (showScore ? 2 * boxW : 0) + (showTime ? timeW : 0) + (showScore && showTime ? 12 : 0);
            int x = fw_ / 2 - total / 2;
            char buf[32];
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
            auto scoreBox = [&](int bx, const char* name, int pts, SDL_Color col) {
                setColor(ren_, col.r, col.g, col.b, 225);
                SDL_Rect r = {bx, by, boxW, bh};
                SDL_RenderFillRect(ren_, &r);
                setColor(ren_, 255, 255, 255);
                drawText57(ren_, bx + 10, by + 8, name, 2);
                std::snprintf(buf, sizeof(buf), "%d", pts);
                drawText57(ren_, bx + boxW - 12 - textW57(buf, 4), by + 18, buf, 4);
                const int a = name[0] == 'R' ? 0 : 1;  // warnings and DQ under the name
                if (gmI.dq[a]) drawText57(ren_, bx + 10, by + 40, "DQ", 2);
                else if (gmI.minors[a] > 0) { std::snprintf(buf, sizeof(buf), "WARN %d", gmI.minors[a]); drawText57(ren_, bx + 10, by + 44, buf, 1); }
            };
            if (showScore) { scoreBox(x, "RED", gmI.red, {200, 35, 40, 255}); x += boxW; }
            if (showTime) {
                if (showScore) x += 6;
                setColor(ren_, 12, 14, 18, 230);
                SDL_Rect r = {x, by, timeW, bh};
                SDL_RenderFillRect(ren_, &r);
                int secs = (int)std::ceil(gmI.matchT);
                std::snprintf(buf, sizeof(buf), "%d:%02d", secs / 60, secs % 60);
                if (gmI.matchT <= 15.0) setColor(ren_, 245, 190, 60);
                else                    setColor(ren_, 240, 240, 240);
                drawText57(ren_, x + timeW / 2 - textW57(buf, 4) / 2, by + 10, buf, 4);
                setColor(ren_, 150, 155, 165);
                const char* lab = gmI.matchOver ? "MATCH OVER" : "DRIVER";
                drawText57(ren_, x + timeW / 2 - textW57(lab, 1) / 2, by + bh - 14, lab, 1);
                x += timeW;
                if (showScore) x += 6;
            }
            if (showScore) scoreBox(x, "BLUE", gmI.blue, {35, 85, 210, 255});
            SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
            if (gmI.matchOver) {  // result banner
                const char* msg = gmI.red > gmI.blue ? "RED WINS" : gmI.blue > gmI.red ? "BLUE WINS" : "TIE";
                std::snprintf(buf, sizeof(buf), "%d - %d", gmI.red, gmI.blue);
                SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
                setColor(ren_, 0, 0, 0, 190);
                SDL_Rect bx = {fw_ / 2 - 190, h3 / 2 - 70, 380, 120};
                SDL_RenderFillRect(ren_, &bx);
                SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
                if (gmI.red > gmI.blue) setColor(ren_, 240, 80, 80);
                else if (gmI.blue > gmI.red) setColor(ren_, 90, 140, 245);
                else setColor(ren_, 230, 230, 230);
                drawText57(ren_, fw_ / 2 - textW57(msg, 4) / 2, h3 / 2 - 56, msg, 4);
                setColor(ren_, 230, 230, 230);
                drawText57(ren_, fw_ / 2 - textW57(buf, 3) / 2, h3 / 2 - 8, buf, 3);
                const char* sub = "PRESS ENTER OR CLICK FOR NEXT";
                drawText57(ren_, fw_ / 2 - textW57(sub, 2) / 2, h3 / 2 + 24, sub, 2);
            }
        }
        SDL_Rect vps = {fw_, h3, fw_, SIDE_VIEW_H};
        SDL_RenderSetViewport(ren_, &vps);
        setLiftType(game().set.liftBot[3]);
        drawSideView(ren_, fw_, 0, SIDE_VIEW_H, px, py, heading, cfg_.robot_half_w, cfg_.robot_half_h);
        SDL_Rect vpSide = {fw_ * 2, 0, SIDEBAR_W, fh_ + 80};
        SDL_RenderSetViewport(ren_, &vpSide);
        drawSidebar(ren_, state, SIDEBAR_W, fh_ + 80);
        SDL_RenderSetViewport(ren_, nullptr);
    }
    // Settings menu (Tab), over the 3D view
    if (game().menuOpen) {
        const Game& gm = game();
        const int rowH = 24, w = fw_ - 100, h = MENU_N * rowH + 96, x0 = fw_ + 50, y0 = 12;
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
        setColor(ren_, 14, 17, 23, 238);
        SDL_Rect panel = {x0, y0, w, h};
        SDL_RenderFillRect(ren_, &panel);
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
        setColor(ren_, 90, 98, 112);
        SDL_RenderDrawRect(ren_, &panel);
        setColor(ren_, 235, 70, 70);
        drawText57(ren_, x0 + 24, y0 + 20, "SETTINGS", 3);
        for (int i = 0; i < MENU_N; ++i) {
            int ry = y0 + 60 + i * rowH;
            bool sel = i == gm.menuSel;
            if (sel) {
                setColor(ren_, 50, 46, 30);
                SDL_Rect hl = {x0 + 12, ry - 8, w - 24, rowH - 4};
                SDL_RenderFillRect(ren_, &hl);
                setColor(ren_, 245, 190, 60);
                drawText57(ren_, x0 + 18, ry, ">");
            }
            if (sel) setColor(ren_, 255, 255, 255);
            else     setColor(ren_, 185, 192, 202);
            drawText57(ren_, x0 + 40, ry, menuLabel(i));
            const char* val = menuValue(i);
            setColor(ren_, 245, 190, 60);
            int vx = x0 + w - 24 - textW57(val);
            if (sel && i < 17) {
                drawText57(ren_, vx - 22, ry, "<");
                drawText57(ren_, x0 + w - 20, ry, ">");
                vx -= 4;
            }
            drawText57(ren_, vx, ry, val);
        }
        setColor(ren_, 130, 140, 155);
        drawText57(ren_, x0 + 24, y0 + h - 30, "UP DN PICK  LT RT CHANGE  TAB CLOSE");
    }
    // Start screen (MATCH SETUP) over everything
    if (game().dash) {
        SDL_RenderSetViewport(ren_, nullptr);
        const int mx = state.mouse_x.load(), my = state.mouse_y.load();
        // The start screen is drawn at the window's real pixel size (not scaled up or down from the 1740 x 800 layout)
        // so edges and text stay sharp: layout positions are mapped to pixels here.
        int lw = 0, lh = 0, ow = 0, oh = 0;
        SDL_RenderGetLogicalSize(ren_, &lw, &lh);
        SDL_GetRendererOutputSize(ren_, &ow, &oh);
        const double sc = (lw > 0 && lh > 0) ? std::min((double)ow / lw, (double)oh / lh) : 1.0;
        const int ox = lw > 0 ? (ow - (int)std::lround(lw * sc)) / 2 : 0, oy = lh > 0 ? (oh - (int)std::lround(lh * sc)) / 2 : 0;
        SDL_RenderSetLogicalSize(ren_, 0, 0);
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_BLEND);
        auto PX = [&](int v) { return ox + (int)std::lround(v * sc); };
        auto PY = [&](int v) { return oy + (int)std::lround(v * sc); };
        auto fillP = [&](SDL_Rect r, SDL_Color c) { setColor(ren_, c.r, c.g, c.b, c.a); SDL_RenderFillRect(ren_, &r); };
        auto rectL = [&](int x, int y, int w, int h) { return SDL_Rect{PX(x), PY(y), PX(x + w) - PX(x), PY(y + h) - PY(y)}; };
        auto fillL = [&](int x, int y, int w, int h, SDL_Color c) { fillP(rectL(x, y, w, h), c); };
        auto boxL = [&](int x, int y, int w, int h, SDL_Color fill, SDL_Color border, int bt = 1) {  // square corners, a crisp border one or more pixels wide
            const SDL_Rect r = rectL(x, y, w, h);
            fillP(r, border);
            const int b = std::max(1, (int)std::lround(bt * sc));
            const SDL_Rect in = {r.x + b, r.y + b, r.w - 2 * b, r.h - 2 * b};
            if (in.w > 0 && in.h > 0) fillP(in, fill);
        };
        auto shade = [](SDL_Color c, int d) { return SDL_Color{(Uint8)std::clamp(c.r + d, 0, 255), (Uint8)std::clamp(c.g + d, 0, 255), (Uint8)std::clamp(c.b + d, 0, 255), c.a}; };
        auto pxPhys = [&](int scale) { return std::max(8, (int)std::lround(dashPx(scale) * sc)); };
        auto textL = [&](const std::string& s0, int scale, bool bold) {  // width in layout units
            const std::string s = uiCase(s0);
            if (!uiTextOk()) return textW57(s.c_str(), scale);
            return (int)std::lround(uiTextWidth(s.c_str(), pxPhys(scale), bold) / sc);
        };
        auto drawT = [&](int x, int yTop, const std::string& s0, int scale, SDL_Color c, bool bold) {
            const std::string s = uiCase(s0);
            if (uiTextOk()) { const int p = pxPhys(scale); uiDrawText(ren_, PX(x), PY(yTop) - (int)std::lround(0.34 * p), s.c_str(), p, c, bold); }
            else { setColor(ren_, c.r, c.g, c.b, c.a); drawText57(ren_, PX(x), PY(yTop), s.c_str(), std::max(1, (int)std::lround(scale * sc))); }
        };
        // Palette: neutral greys, one blue accent, soft red and blue for the alliances
        const SDL_Color WHITE = {255, 255, 255, 255}, PANEL = {248, 250, 252, 255}, LINE = {226, 232, 240, 255}, EDGE = {203, 213, 225, 255};
        const SDL_Color INK = {15, 23, 42, 255}, ACC = {79, 70, 229, 255};
        const SDL_Color RED = {229, 72, 77, 255}, BLUE = {62, 99, 221, 255}, AMBER = {245, 158, 11, 255};

        setColor(ren_, 15, 23, 42, 150);  // dim what is behind
        SDL_RenderFillRect(ren_, nullptr);
        const SDL_Rect panel = {220, 40, 1300, 720};
        boxL(panel.x, panel.y, panel.w, panel.h, PANEL, EDGE);
        fillL(panel.x, panel.y, panel.w, 3, ACC);
        const std::vector<DashWidget> dws = dashWidgets(game());
        for (const DashWidget& d : dws) {
            if (d.kind == 3) {  // an alliance badge: its number on the colour it plays (washed out once it has lost)
                static const SDL_Color bg[8] = {{148, 163, 184, 255}, {225, 70, 80, 255}, {59, 105, 230, 255}, {148, 163, 184, 255},
                                                {226, 232, 240, 255}, {246, 205, 210, 255}, {204, 216, 247, 255}, {226, 232, 240, 255}};
                fillL(d.x, d.y, d.w, d.h, bg[std::clamp(d.col, 0, 7)]);
                const bool lost = d.col >= 4;
                drawT(d.x + d.w / 2 - textL(d.text, d.scale, true) / 2, d.y + d.h / 2 - 7 * d.scale / 2, d.text, d.scale, lost ? SDL_Color{110, 116, 130, 255} : WHITE, true);
                continue;
            }
            if (d.kind == 2 && d.col >= 9) {  // tables: 9 white card, 10 pink and 11 blue winner cells, 12 row line, 13 pale yellow tie, 14 amber line
                if (d.col == 9) boxL(d.x, d.y, d.w, d.h, WHITE, LINE);
                else {
                    static const SDL_Color lc[5] = {{253, 181, 181, 255}, {181, 212, 255, 255}, {229, 231, 235, 255}, {254, 240, 138, 255}, {245, 158, 11, 255}};
                    fillL(d.x, d.y, d.w, d.h, lc[std::clamp(d.col - 10, 0, 4)]);
                }
                continue;
            }
            if (d.kind == 2) {  // a box: a panel, a match, a result
                if (d.col == 5 || d.col == 6) fillL(d.x, d.y, d.w, d.h, d.col == 5 ? SDL_Color{253, 181, 181, 255} : SDL_Color{181, 212, 255, 255});
                else if (d.col == 7 || d.col == 8) boxL(d.x, d.y, d.w, d.h, WHITE, d.col == 7 ? SDL_Color{240, 190, 192, 255} : SDL_Color{190, 208, 242, 255});
                else {
                    boxL(d.x, d.y, d.w, d.h, d.col == 1 ? SDL_Color{254, 247, 247, 255} : d.col == 2 ? SDL_Color{246, 249, 255, 255} : d.col == 4 ? SDL_Color{255, 251, 240, 255} : WHITE,
                         d.sel ? AMBER : LINE, d.sel ? 2 : 1);
                    if (d.col == 1 || d.col == 2 || d.col == 4) fillL(d.x, d.y, d.w, 4, d.col == 1 ? RED : d.col == 2 ? BLUE : AMBER);  // an alliance's panel: its colour along the top
                }
                continue;
            }
            if (d.kind == 1) {
                static const SDL_Color cols[10] = {{15, 23, 42, 255}, {205, 50, 60, 255}, {51, 90, 210, 255}, {100, 116, 139, 255}, {180, 100, 10, 255},
                                                   {15, 23, 42, 255}, {100, 116, 139, 255}, {185, 28, 28, 255}, {40, 70, 180, 255}, {180, 83, 9, 255}};
                const SDL_Color c = cols[std::clamp(d.col, 0, 9)];
                const bool bold = d.scale >= 3;
                drawT(d.w > 0 ? d.x + d.w / 2 - textL(d.text, d.scale, bold) / 2 : d.x, d.y, d.text, d.scale, c, bold);
                continue;
            }
            // a button
            static const SDL_Color bar[13] = {{0, 0, 0, 0},  // a hardware option keeps white with a bar in its own colour
                {59, 130, 246, 255}, {16, 185, 129, 255}, {245, 158, 11, 255}, {239, 68, 68, 255},
                {139, 92, 246, 255}, {20, 184, 166, 255}, {249, 115, 22, 255}, {236, 72, 153, 255},
                {100, 116, 139, 255}, {234, 179, 8, 255}, {16, 185, 129, 255}, {234, 179, 8, 255}};
            SDL_Color barCol = {0, 0, 0, 0};
            const bool hover = d.on && d.hit(mx, my);
            SDL_Color fill = WHITE, border = EDGE, fg = INK;
            if (!d.on) { fill = {249, 250, 251, 255}; border = {243, 244, 246, 255}; fg = {170, 175, 186, 255}; }
            else if (d.id == D_PLAY) { fill = {22, 163, 112, 255}; border = {15, 130, 90, 255}; fg = WHITE; }
            else if (d.col > 0) barCol = bar[std::clamp(d.col, 0, 12)];
            else if (d.sel) { fill = ACC; border = {67, 56, 202, 255}; fg = WHITE; }
            if (hover) { fill = shade(fill, -9); border = shade(border, -9); }
            const int di = (int)(&d - &dws[0]);
            if (di == game().dashSel || (game().dashSel < 0 && d.id == D_PLAY)) boxL(d.x - 4, d.y - 4, d.w + 8, d.h + 8, PANEL, ACC, 2);  // picked with the arrow keys
            boxL(d.x, d.y, d.w, d.h, fill, border);
            if (barCol.a) fillL(d.x, d.y, 5, d.h, barCol);
            drawT(d.x + d.w / 2 + (barCol.a ? 3 : 0) - textL(d.text, d.scale, true) / 2, d.y + d.h / 2 - 7 * d.scale / 2, d.text, d.scale, fg, true);
        }
        SDL_RenderSetLogicalSize(ren_, lw, lh);
        SDL_SetRenderDrawBlendMode(ren_, SDL_BLENDMODE_NONE);
    }
    {   // testing aid: VEXSIM_SHOT=<file.bmp> saves the window once VEXSIM_SHOT_T seconds of the match have passed, then quits
        static const char* shot = std::getenv("VEXSIM_SHOT");
        static const double at = std::getenv("VEXSIM_SHOT_T") ? std::atof(std::getenv("VEXSIM_SHOT_T")) : 5.0;
        if (shot && (std::getenv("VEXSIM_SHOT_DASH") || !game().dash) && (std::getenv("VEXSIM_SHOT_DASH") || g_matchLen - game().matchT >= at)) {
            int w = 0, h = 0;
            SDL_GetRendererOutputSize(ren_, &w, &h);
            SDL_Surface* sf = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_ARGB8888);
            if (sf && SDL_RenderReadPixels(ren_, nullptr, SDL_PIXELFORMAT_ARGB8888, sf->pixels, sf->pitch) == 0) SDL_SaveBMP(sf, shot);
            if (sf) SDL_FreeSurface(sf);
            std::exit(0);
        }
    }
    SDL_RenderPresent(ren_);
}

} // namespace sim
