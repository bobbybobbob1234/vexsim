#pragma once
// Perspective 3D view of the field, drawn with SDL_RenderGeometry.
// World units are inches: x right, y up the field (toward the top of the 2D map), z up.
// Faces are clipped at the near plane, back faces of solids are skipped, and the rest are
// drawn far to near (painter's algorithm).
#include <cstring>
#include <SDL2/SDL.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "Game.hpp"

namespace sim {
namespace v3d {

struct V3 { double x, y, z; };
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline V3 norm(V3 a) { double l = std::sqrt(dot(a, a)); return l > 1e-9 ? a * (1.0 / l) : V3{0, 0, 1}; }

struct Face {
    std::vector<SDL_Vertex> v;
    double depth;
    int layer;
};

// Draw layers, used while the camera is inside the field: everything outside the perimeter is
// drawn first, then the perimeter's inner faces, then the field and robot. Seen from inside the
// field, nothing inside can be hidden by a wall, and a wall can't be hidden by anything outside,
// so this ordering is exact where sorting whole faces by their centre depth isn't.
enum { L_OUTSIDE = 0, L_WALL = 1, L_FIELD = 2 };

struct Scene {
    V3 eye, r, u, f;
    double focal, cx, cy;
    double lightScale = 1.0;   // arena lighting (dim setting)
    int layer = L_FIELD;       // layer for the faces added next
    bool layered = false;      // sort by layer first (camera inside the field)
    double maxCell = 0.0;      // > 0: split quads into cells no bigger than this (inches) so they sort well
    bool lowDetail = false;    // draw a robot without its small details (gear teeth, rollers, spokes, holes): the other robots
    std::vector<Face> floor;   // drawn first, in order
    std::vector<Face> solids;  // sorted far to near

    // Optional rigid transform for the faces added next (the robot leaning or lying on its side):
    // up to four rotations in turn, each by a about the line through p along k
    struct XStep { V3 p, k; double a; };
    struct Xf {
        bool on = false;
        int n = 0;
        XStep s[4];
        void add(V3 p, V3 k, double a) { if (n < 4) s[n++] = {p, k, a}; }
    };
    Xf xf;
    static V3 rotAbout(V3 v, V3 k, double a) {
        double ca = std::cos(a), sa = std::sin(a);
        return v * ca + cross(k, v) * sa + k * (dot(k, v) * (1 - ca));
    }
    V3 xfPoint(V3 p) const {
        for (int i = 0; i < xf.n; ++i) p = xf.s[i].p + rotAbout(p - xf.s[i].p, xf.s[i].k, xf.s[i].a);
        return p;
    }
    V3 xfDir(V3 n) const {
        for (int i = 0; i < xf.n; ++i) n = rotAbout(n, xf.s[i].k, xf.s[i].a);
        return n;
    }

    void lookAt(V3 e, V3 target, int w, int h, double fovDeg) {
        eye = e;
        f = norm(target - e);
        r = norm(cross(f, V3{0, 0, 1}));
        u = cross(r, f);
        focal = (w / 2.0) / std::tan(fovDeg * M_PI / 360.0);
        cx = w / 2.0;
        cy = h / 2.0;
    }

    // pts: convex polygon. n: outward normal (used for lighting and, if cull, skipping back faces).
    void poly(const std::vector<V3>& ptsIn, SDL_Color col, V3 n, bool cull, bool isFloor = false) {
        if (xf.on) {  // transform once, then draw as ordinary faces
            std::vector<V3> moved(ptsIn.size());
            for (size_t i = 0; i < ptsIn.size(); ++i) moved[i] = xfPoint(ptsIn[i]);
            Xf saved = xf;
            xf.on = false;
            poly(moved, col, xfDir(n), cull, isFloor);
            xf = saved;
            return;
        }
        const std::vector<V3>& pts = ptsIn;
        if (cull && dot(n, eye - pts[0]) <= 0) return;
        // Large quads sort badly against small objects beside them (a 140 in wall face is sorted by
        // its middle), so split them into a grid of small cells first
        if (maxCell > 0 && !isFloor && pts.size() == 4) {
            auto len = [](V3 a) { return std::sqrt(dot(a, a)); };
            double lu = std::max(len(pts[1] - pts[0]), len(pts[2] - pts[3]));
            double lv = std::max(len(pts[3] - pts[0]), len(pts[2] - pts[1]));
            int nu = std::min(64, (int)std::ceil(lu / maxCell)), nv = std::min(64, (int)std::ceil(lv / maxCell));
            if (nu > 1 || nv > 1) {
                auto at = [&](double s, double t) {
                    V3 a = pts[0] + (pts[1] - pts[0]) * s, b = pts[3] + (pts[2] - pts[3]) * s;
                    return a + (b - a) * t;
                };
                const double saved = maxCell;
                maxCell = 0;
                for (int i = 0; i < nu; ++i)
                    for (int j = 0; j < nv; ++j) {
                        double s0 = (double)i / nu, s1 = (double)(i + 1) / nu, t0 = (double)j / nv, t1 = (double)(j + 1) / nv;
                        poly({at(s0, t0), at(s1, t0), at(s1, t1), at(s0, t1)}, col, n, false, false);
                    }
                maxCell = saved;
                return;
            }
        }
        const V3 light = norm(V3{-0.35, -0.55, 1.0});
        double shade = (isFloor ? 1.0 : 0.6 + 0.4 * std::max(0.0, dot(norm(n), light))) * lightScale;
        SDL_Color c = {(Uint8)std::min(255.0, col.r * shade), (Uint8)std::min(255.0, col.g * shade),
                       (Uint8)std::min(255.0, col.b * shade), col.a};

        // To camera space, then clip against the near plane
        const double NEAR = 0.5;
        std::vector<V3> in, out;
        for (const V3& p : pts) { V3 d = p - eye; in.push_back({dot(d, r), dot(d, u), dot(d, f)}); }
        for (size_t i = 0; i < in.size(); ++i) {
            V3 a = in[i], b = in[(i + 1) % in.size()];
            bool ain = a.z >= NEAR, bin = b.z >= NEAR;
            if (ain) out.push_back(a);
            if (ain != bin) {
                double t = (NEAR - a.z) / (b.z - a.z);
                out.push_back(a + (b - a) * t);
            }
        }
        if (out.size() < 3) return;

        Face fc;
        double depth = 0;
        std::vector<SDL_Vertex> ring;
        for (const V3& p : out) {
            SDL_Vertex v;
            v.position = {(float)(cx + focal * p.x / p.z), (float)(cy - focal * p.y / p.z)};
            v.color = c;
            v.tex_coord = {0, 0};
            ring.push_back(v);
            depth += p.z;
        }
        for (size_t i = 1; i + 1 < ring.size(); ++i) {
            fc.v.push_back(ring[0]);
            fc.v.push_back(ring[i]);
            fc.v.push_back(ring[i + 1]);
        }
        fc.depth = depth / out.size();
        fc.layer = layer;
        (isFloor ? floor : solids).push_back(std::move(fc));
    }

    void quadFloor(double x0, double y0, double x1, double y1, double z, SDL_Color col) {
        poly({{x0, y0, z}, {x1, y0, z}, {x1, y1, z}, {x0, y1, z}}, col, {0, 0, 1}, false, true);
    }

    // A strip of tape from a to b on the floor
    void tape(double ax, double ay, double bx, double by, double width, SDL_Color col, double z = 0.03) {
        V3 d = norm(V3{bx - ax, by - ay, 0});
        V3 s = V3{-d.y, d.x, 0} * (width / 2);
        V3 a{ax, ay, z}, b{bx, by, z};
        poly({a - s, b - s, b + s, a + s}, col, {0, 0, 1}, false, true);
    }

    // Prism with a regular n-gon cross-section standing on the floor (or on z0)
    void prism(double x, double y, double z0, double z1, double rad, int n, double rot, SDL_Color col, bool cap = true) {
        std::vector<V3> b(n), t(n);
        for (int i = 0; i < n; ++i) {
            double a = rot + 2 * M_PI * i / n;
            b[i] = {x + rad * std::cos(a), y + rad * std::sin(a), z0};
            t[i] = {b[i].x, b[i].y, z1};
        }
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            double am = rot + 2 * M_PI * (i + 0.5) / n;
            poly({b[i], b[j], t[j], t[i]}, col, {std::cos(am), std::sin(am), 0}, true);
        }
        if (cap) poly(t, col, {0, 0, 1}, true);
    }

    // Cylinder between points a and b with two colored halves (a-half, b-half)
    void cylinder2(V3 a, V3 b, double rad, SDL_Color ca, SDL_Color cb, int n = 8) {
        V3 w = norm(b - a);
        V3 helper = std::fabs(w.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
        V3 uu = norm(cross(helper, w));
        V3 vv = cross(w, uu);
        V3 m = (a + b) * 0.5;
        auto ring = [&](V3 c) {
            std::vector<V3> r(n);
            for (int i = 0; i < n; ++i) {
                double t = 2 * M_PI * i / n + M_PI / n;
                r[i] = c + uu * (rad * std::cos(t)) + vv * (rad * std::sin(t));
            }
            return r;
        };
        auto ra = ring(a), rm = ring(m), rb = ring(b);
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            double t = 2 * M_PI * (i + 0.5) / n + M_PI / n;
            V3 nn = uu * std::cos(t) + vv * std::sin(t);
            poly({ra[i], ra[j], rm[j], rm[i]}, ca, nn, true);
            poly({rm[i], rm[j], rb[j], rb[i]}, cb, nn, true);
        }
        poly(rb, cb, w, true);
        std::vector<V3> raRev(ra.rbegin(), ra.rend());
        poly(raRev, ca, w * -1.0, true);
    }

    // Solid of revolution with n flat sides around the axis from base along unit w.
    // prof: (distance along the axis, radius) points; col[i] colors the segment from point i to i+1.
    // capStart / capEnd close the ends. inner > 0 also draws an inside wall that much thinner
    // (an open tube, like a Cup), in innerCol per segment, plus the rims at both ends.
    void lathe(V3 base, V3 w, const ProfPt* prof, int np, const SDL_Color* col, int n, double rot,
               bool capStart, bool capEnd, double inner = 0.0, const SDL_Color* innerCol = nullptr) {
        V3 helper = std::fabs(w.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
        V3 uu = norm(cross(helper, w));
        V3 vv = cross(w, uu);
        auto radial = [&](int i) {
            double t = rot + 2 * M_PI * i / n;
            return uu * std::cos(t) + vv * std::sin(t);
        };
        auto pt = [&](int k, int i, double dr) { return base + w * prof[k].z + radial(i) * (prof[k].r - dr); };
        for (int k = 0; k + 1 < np; ++k) {
            double dz = prof[k + 1].z - prof[k].z, drr = prof[k].r - prof[k + 1].r;
            for (int i = 0; i < n; ++i) {
                int j = (i + 1) % n;
                double tm = rot + 2 * M_PI * (i + 0.5) / n;
                V3 rm = uu * std::cos(tm) + vv * std::sin(tm);
                V3 nn = norm(rm * dz + w * drr);
                poly({pt(k, i, 0), pt(k, j, 0), pt(k + 1, j, 0), pt(k + 1, i, 0)}, col[k], nn, true);
                if (inner > 0)
                    poly({pt(k, j, inner), pt(k, i, inner), pt(k + 1, i, inner), pt(k + 1, j, inner)}, innerCol[k],
                         nn * -1.0, true);
            }
        }
        auto ring = [&](int k, double dr) {
            std::vector<V3> r(n);
            for (int i = 0; i < n; ++i) r[i] = pt(k, i, dr);
            return r;
        };
        if (inner > 0) {
            for (int e = 0; e < 2; ++e) {
                int k = e == 0 ? 0 : np - 1;
                V3 nn = e == 0 ? w * -1.0 : w;
                for (int i = 0; i < n; ++i) {
                    int j = (i + 1) % n;
                    poly({pt(k, i, 0), pt(k, j, 0), pt(k, j, inner), pt(k, i, inner)}, col[e == 0 ? 0 : np - 2], nn, false);
                }
            }
            return;
        }
        if (capEnd) poly(ring(np - 1, 0), col[np - 2], w, true);
        if (capStart) {
            auto r = ring(0, 0);
            std::vector<V3> rev(r.rbegin(), r.rend());
            poly(rev, col[0], w * -1.0, true);
        }
    }

    // A bar from a to b: halfW across (horizontal, perpendicular to the bar), halfT the other way
    void beam(V3 a, V3 b, double halfW, double halfT, SDL_Color col) {
        V3 u = norm(b - a);
        V3 vw = cross(u, V3{0, 0, 1});
        vw = dot(vw, vw) < 1e-6 ? V3{1, 0, 0} : norm(vw);
        V3 vt = cross(vw, u);
        V3 o[4] = {vw * halfW + vt * halfT, vw * -halfW + vt * halfT, vw * -halfW + vt * -halfT, vw * halfW + vt * -halfT};
        V3 nrm[4] = {vt, vw * -1.0, vt * -1.0, vw};
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            poly({a + o[i], a + o[j], b + o[j], b + o[i]}, col, nrm[i], true);
        }
        poly({b + o[0], b + o[1], b + o[2], b + o[3]}, col, u, true);
        poly({a + o[3], a + o[2], a + o[1], a + o[0]}, col, u * -1.0, true);
    }

    // Spur gear: a disc in the plane through c perpendicular to axis, with square teeth around
    // the rim, turned by ang (radians). thick = face width along the axis.
    void gear(V3 c, V3 axis, double r, int teeth, double ang, SDL_Color col, double thick = 0.25) {
        axis = norm(axis);
        if (lowDetail) {  // a plain disc
            ProfPt prof[2] = {{-thick / 2, r * 0.95}, {thick / 2, r * 0.95}};
            SDL_Color cc[1] = {col};
            lathe(c, axis, prof, 2, cc, 10, ang, true, true);
            return;
        }
        ProfPt prof[2] = {{-thick / 2, r * 0.9}, {thick / 2, r * 0.9}};
        SDL_Color cc[1] = {col};
        lathe(c, axis, prof, 2, cc, std::min(teeth, 24), ang, true, true);
        V3 helper = std::fabs(axis.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
        V3 uu = norm(cross(helper, axis)), vv = cross(axis, uu);
        double toothW = std::min(0.12, M_PI * r / teeth * 0.5);
        for (int t = 0; t < teeth; ++t) {
            double a = ang + 2 * M_PI * t / teeth;
            V3 radial = uu * std::cos(a) + vv * std::sin(a);
            beam(c + radial * (r * 0.86), c + radial * r, toothW, thick / 2, col);
        }
        ProfPt hub[2] = {{-thick / 2 - 0.05, std::max(0.12, r * 0.18)}, {thick / 2 + 0.05, std::max(0.12, r * 0.18)}};
        SDL_Color hc[1] = {{60, 62, 66, 255}};
        lathe(c, axis, hub, 2, hc, 6, ang, true, true);
        // Lightening holes and a recessed rim ring on both faces, like the plastic VEX gears
        if (r > 0.6) {
            SDL_Color dark = {(Uint8)(col.r * 0.45), (Uint8)(col.g * 0.45), (Uint8)(col.b * 0.45), 255};
            int holes = r > 1.4 ? 8 : 6;
            for (int e = -1; e <= 1; e += 2) {
                V3 face = c + axis * (e * (thick / 2 + 0.01));
                for (int hI = 0; hI < holes; ++hI) {
                    double a = ang + 2 * M_PI * (hI + 0.5) / holes;
                    V3 hcen = face + (uu * std::cos(a) + vv * std::sin(a)) * (r * 0.55);
                    double hr = r * 0.13;
                    std::vector<V3> ring(8);
                    for (int k = 0; k < 8; ++k) {
                        double t = 2 * M_PI * k / 8 * e;
                        ring[k] = hcen + uu * (hr * std::cos(t)) + vv * (hr * std::sin(t));
                    }
                    poly(ring, dark, axis * e, true);
                }
                for (int k = 0; k < 24; ++k) {  // rim ring just inside the teeth
                    double a0 = 2 * M_PI * k / 24, a1 = 2 * M_PI * (k + 1) / 24;
                    V3 p0 = uu * std::cos(a0) + vv * std::sin(a0), p1 = uu * std::cos(a1) + vv * std::sin(a1);
                    poly({face + p0 * (r * 0.74), face + p1 * (r * 0.74), face + p1 * (r * 0.8), face + p0 * (r * 0.8)}, dark, axis * e, false);
                }
            }
        }
    }

    // Box with its own heading: center (x, y), local axes fwd / side, from z0 to z1
    void box(V3 c, V3 fwd, double lenHalf, double wideHalf, double z0, double z1, SDL_Color col) {
        V3 side{-fwd.y, fwd.x, 0};
        V3 p[4] = {c + fwd * lenHalf - side * wideHalf, c + fwd * lenHalf + side * wideHalf,
                   c - fwd * lenHalf + side * wideHalf, c - fwd * lenHalf - side * wideHalf};
        V3 b[4], t[4];
        for (int i = 0; i < 4; ++i) { b[i] = {p[i].x, p[i].y, z0}; t[i] = {p[i].x, p[i].y, z1}; }
        // edge i -> j: 0-1 front, 1-2 one side, 2-3 back, 3-0 other side
        V3 normals[4] = {fwd, side, fwd * -1.0, side * -1.0};
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            poly({b[i], b[j], t[j], t[i]}, col, normals[i], true);
        }
        poly({t[0], t[1], t[2], t[3]}, col, {0, 0, 1}, true);
    }

    void render(SDL_Renderer* ren) {
        // Blending on, so the clear half of a Cup shows what is nested inside it
        SDL_BlendMode prevBlend;
        SDL_GetRenderDrawBlendMode(ren, &prevBlend);
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
        struct Restore { SDL_Renderer* r; SDL_BlendMode m; ~Restore() { SDL_SetRenderDrawBlendMode(r, m); } } restore{ren, prevBlend};
        // All faces go out in two batched calls (floor in order, then the sorted solids)
        std::vector<SDL_Vertex> batch;
        for (const Face& fc : floor) batch.insert(batch.end(), fc.v.begin(), fc.v.end());
        if (!batch.empty()) SDL_RenderGeometry(ren, nullptr, batch.data(), (int)batch.size(), nullptr, 0);
        const bool byLayer = layered;
        std::stable_sort(solids.begin(), solids.end(), [byLayer](const Face& a, const Face& b) {
            if (byLayer && a.layer != b.layer) return a.layer < b.layer;
            return a.depth > b.depth;
        });
        batch.clear();
        for (const Face& fc : solids) batch.insert(batch.end(), fc.v.begin(), fc.v.end());
        if (!batch.empty()) SDL_RenderGeometry(ren, nullptr, batch.data(), (int)batch.size(), nullptr, 0);
    }
};

inline SDL_Color colOf(Col c) {
    if (c == C_RED) return {215, 45, 45, 255};
    if (c == C_BLUE) return {30, 150, 225, 255};  // VEX's light blue
    return {240, 210, 40, 255};
}

// Sim pixels (y down) -> world inches (y up)
inline double wx(double px) { return px / PX_IN; }
inline double wy(double py) { return (720.0 - py) / PX_IN; }

// Full pin profile, tip to tip (the bottom half and its mirror image)
inline const ProfPt* pinProfile() {
    static ProfPt prof[2 * PIN_HALF_N - 1];
    static bool made = false;
    if (!made) {
        for (int i = 0; i < PIN_HALF_N; ++i) prof[i] = PIN_HALF[i];
        for (int i = 0; i < PIN_HALF_N - 1; ++i)
            prof[PIN_HALF_N + i] = {PIECE_H_IN - PIN_HALF[PIN_HALF_N - 2 - i].z, PIN_HALF[PIN_HALF_N - 2 - i].r};
        made = true;
    }
    return prof;
}

inline void addPiece(Scene& sc, const Piece& pc, double xIn, double yIn, double bottomIn) {
    // Where the piece's bottom end is and which way its axis runs (bottom -> top):
    //   lying: on its side, resting on the collar (pin) or both rims (cup), so the axis is 1.58 in up
    //   leaning: turned by `tilt` about the edge of its base on the side it leans toward
    V3 bottom, axis;
    if (pc.lying) {
        axis = {std::cos(pc.ang), -std::sin(pc.ang), 0};
        bottom = V3{xIn, yIn, PIECE_MAX_R_IN} - axis * HALF_IN;
    } else if (pc.tilt > 0.0 && loose(pc)) {
        V3 d{std::cos(pc.tiltDir), -std::sin(pc.tiltDir), 0};  // sim y points down
        double r = pc.tiltR, ct = std::cos(pc.tilt), st = std::sin(pc.tilt);
        V3 pivot = V3{xIn, yIn, 0} + d * r;
        V3 baseC = pivot - d * (r * ct) + V3{0, 0, r * st};
        axis = d * st + V3{0, 0, ct};
        bottom = baseC + axis * bottomIn;
    } else {
        axis = {0, 0, 1};
        bottom = {xIn, yIn, bottomIn};
    }
    if (pc.isPin) {
        // Octagonal pin: tapered halves with a collar in the middle, each half its own color
        constexpr int NP = 2 * PIN_HALF_N - 1;
        SDL_Color col[NP - 1];
        for (int k = 0; k < NP - 1; ++k) col[k] = colOf(k < PIN_HALF_N - 1 ? pc.bot : pc.top);
        V3 base = bottom;
        sc.lathe(base, axis, pinProfile(), NP, col, PIN_SIDES, M_PI / PIN_SIDES + (pc.lying ? pc.rollAng : 0.0), true, true);  // a lying pin turns as it rolls
        // Hollow tips: a dark opening inside each end
        V3 hu = norm(cross(std::fabs(axis.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0}, axis)), hv = cross(axis, hu);
        for (int e = 0; e < 2; ++e) {
            V3 c = e == 0 ? base - axis * 0.01 : base + axis * (PIECE_H_IN + 0.01);
            V3 nrm = e == 0 ? axis * -1.0 : axis;
            SDL_Color tc = colOf(e == 0 ? pc.bot : pc.top);
            SDL_Color dk = {(Uint8)(tc.r * 0.45), (Uint8)(tc.g * 0.45), (Uint8)(tc.b * 0.45), 255};
            std::vector<V3> ring(PIN_SIDES);
            for (int i = 0; i < PIN_SIDES; ++i) {
                double t = 2 * M_PI * i / PIN_SIDES * (e == 0 ? -1 : 1);
                ring[i] = c + hu * (0.5 * std::cos(t)) + hv * (0.5 * std::sin(t));
            }
            sc.poly(ring, dk, nrm, true);
        }
    } else {
        // Open-ended cup: opaque half with the foot, clear half with the rim. Drawn from the
        // opaque end, so a cup with its opaque half up is drawn downward from its top.
        const SDL_Color opaque = {74, 76, 82, 255}, clear = {205, 226, 238, 100};
        const SDL_Color opaqueIn = {42, 44, 48, 255}, clearIn = {160, 188, 205, 70};
        SDL_Color col[CUP_PROF_N - 1], colIn[CUP_PROF_N - 1];
        for (int k = 0; k < CUP_PROF_N - 1; ++k) {
            bool opq = k < CUP_WAIST_I;
            col[k] = opq ? opaque : clear;
            colIn[k] = opq ? opaqueIn : clearIn;
        }
        V3 base = pc.grayUp ? bottom + axis * PIECE_H_IN : bottom;
        V3 cAxis = pc.grayUp ? axis * -1.0 : axis;
        sc.lathe(base, cAxis, CUP_PROF, CUP_PROF_N, col, CUP_SIDES, pc.lying ? (pc.grayUp ? -pc.rollAng : pc.rollAng) : 0.0,
                 false, false, CUP_WALL_IN, colIn);  // a lying cup turns as it rolls
    }
}

// License plates: plates in the alliance colour on standoffs at both sides of a robot, with its team
// number in white, readable from that side. centre / dir: robot centre (floor) and forward;
// plateY: plate distance out from the centre line; halfLen: half the plate's length; standY: where
// the standoffs start.
inline void drawPlates(Scene& sc, V3 centre, V3 dir, double plateY, double halfLen, double standY,
                       const char* team, SDL_Color plateC) {
    static const char* GLYPHS = "0123456789FABXCD";
    static const unsigned char FONT[16][7] = {
        {14, 17, 19, 21, 25, 17, 14}, {4, 12, 4, 4, 4, 4, 14},     {14, 17, 1, 2, 4, 8, 31},    {31, 2, 4, 2, 1, 17, 14},
        {2, 6, 10, 18, 31, 2, 2},     {31, 16, 30, 1, 1, 17, 14},  {6, 8, 16, 30, 17, 17, 14},  {31, 1, 2, 4, 8, 8, 8},
        {14, 17, 17, 14, 17, 17, 14}, {14, 17, 17, 15, 1, 2, 12},  {31, 16, 16, 30, 16, 16, 16}, {14, 17, 17, 31, 17, 17, 17},
        {30, 17, 17, 30, 17, 17, 30}, {17, 17, 10, 4, 10, 17, 17}, {14, 17, 16, 16, 16, 17, 14},
        {30, 17, 17, 17, 17, 17, 30}};
    const SDL_Color textC = {245, 245, 245, 255}, standC = {55, 57, 62, 255};
    const V3 side{-dir.y, dir.x, 0};
    auto at = [&](double f, double s, double z) { return centre + dir * f + side * s + V3{0, 0, z}; };
    const double pz0 = 3.3, pz1 = 5.5, px = halfLen;  // 2.2 in tall, its bottom 3.3 in up
    const int n = (int)std::strlen(team);
    const double cell = (pz1 - pz0 - 0.6) / 7.0;               // one font pixel, inches
    const double adv = 6.0 * cell, textLen = n * adv - cell;   // 5 wide plus a 1-pixel gap
    for (double sd : {-1.0, 1.0}) {
        V3 out = side * sd;
        V3 tdir = dir * -sd;  // reading direction for someone standing on this side
        for (double fx : {px - 1.5, -px + 1.5})
            sc.beam(at(fx, sd * standY, (pz0 + pz1) / 2), at(fx, sd * plateY, (pz0 + pz1) / 2), 0.2, 0.2, standC);  // standoffs
        V3 c0 = at(0, sd * plateY, 0);
        auto rect = [&](double s0, double s1, double z0, double z1, SDL_Color col) {  // on the outer face
            sc.poly({c0 + tdir * s0 + V3{0, 0, z0}, c0 + tdir * s1 + V3{0, 0, z0}, c0 + tdir * s1 + V3{0, 0, z1},
                     c0 + tdir * s0 + V3{0, 0, z1}}, col, out, true);
        };
        sc.poly({c0 + tdir * px + V3{0, 0, pz0}, c0 - tdir * px + V3{0, 0, pz0}, c0 - tdir * px + V3{0, 0, pz1},
                 c0 + tdir * px + V3{0, 0, pz1}}, plateC, out * -1.0, true);  // back of the plate
        // Outer face as tiles that never overlap (overlapping faces sort unreliably): the border
        // around the number, then every font pixel of the number in white or the plate colour
        const double t0 = -textLen / 2, t1 = textLen / 2, zt1 = pz1 - 0.3, zt0 = zt1 - 7 * cell;
        rect(-px, t0, pz0, pz1, plateC);
        rect(t1, px, pz0, pz1, plateC);
        rect(t0, t1, zt1, pz1, plateC);
        rect(t0, t1, pz0, zt0, plateC);
        for (int ch = 0; ch < n; ++ch) {
            const char* g = std::strchr(GLYPHS, team[ch]);
            const unsigned char* rows = g ? FONT[g - GLYPHS] : nullptr;
            for (int ry = 0; ry < 7; ++ry)
                for (int cx = 0; cx < 6; ++cx) {  // the 6th column is the gap after the character
                    double s0 = t0 + ch * adv + cx * cell, s1 = std::min(t1, s0 + cell);
                    if (s1 <= s0) continue;
                    bool on = rows && cx < 5 && (rows[ry] & (16 >> cx));
                    rect(s0, s1, zt1 - (ry + 1) * cell, zt1 - ry * cell, on ? textC : plateC);
                }
        }
    }
}

inline SDL_Color allianceColour(Col c) { return c == C_RED ? SDL_Color{205, 32, 38, 255} : SDL_Color{35, 95, 215, 255}; }

// What the robot drawing needs to know about one robot: its mechanisms, lean, held pieces and plate
struct RobotLook {
    double liftAngle, liftIn, wheelRotL, wheelRotR, clawDeg, pawlUp, pawlClick, pitch, roll;
    std::array<bool, 4> touching;
    int held, rider;
    const std::vector<Piece>* pieces;
    bool showCom;
    double comX, comY, comZ;
    const char* plate;
    Col team;
    bool tractionMid;  // middle wheels are traction wheels (else all six are omni)
};
inline RobotLook playerLook(const Game& g) {
    return {g.liftAngle, g.liftIn, g.wheelRotL, g.wheelRotR, g.clawDeg, g.pawlUp, g.pawlClick, g.pitch, g.roll,
            g.touching, g.held, g.rider, &g.p, g.set.physInfo, g.comX, g.comY, g.comZ, "41658F", C_RED, !g.set.omniBot[3]};
}
inline RobotLook botLook(const Game& g, const Bot& b) {
    return {dr4bAngleRad(b.liftDeg), dr4bLiftIn(b.liftDeg), b.wheelRotL, b.wheelRotR, b.clawDeg, 0.0, 0.0, 0.0, 0.0,
            {}, b.carry, b.rider, &g.p, false, 0, 0, 0, b.num, b.team, !b.omni};
}

// The robot: drive base, wheels, DR6B lift, roller claw, alignment guide, toggle arms, plates and
// held piece. robot / dir: centre on the floor and forward; halfLen / halfWide in inches.
inline void drawRobotModel(Scene& sc, const RobotLook& gm, V3 robot, V3 dir, double halfLen, double halfWide) {
    // Robot: drive base, wheels, DR4B lift, ratcheting claw, held piece
    SDL_Color metal = {175, 178, 185, 255}, tire = {30, 30, 32, 255};
    SDL_Color dark = {55, 57, 62, 255}, jawCol = {240, 225, 70, 255};
    V3 side{-dir.y, dir.x, 0};
    auto at = [&](double fwd, double sideIn, double z) { return robot + dir * fwd + side * sideIn + V3{0, 0, z}; };
    // Leaning / fallen robot, the same way the physics moves it: roll, then pitch, each first about
    // the wheel edge and, past the lean where the frame touches down, about that frame corner
    // (a positive turn about `side` takes the nose down; about `dir`, the top to the right)
    if (gm.pitch != 0.0 || gm.roll != 0.0) {
        sc.xf.on = true;
        sc.xf.n = 0;
        // a: lean toward u (>= 0); the turn about k is kSign * a. eW / eC: wheel edge and frame corner
        // distance out from the centre toward u; cz: corner height
        auto lean = [&](double a, V3 u, V3 k, double kSign, double eW, double eC, double cz) {
            V3 W = robot + u * eW, C = robot + u * eC + V3{0, 0, cz};
            double f = std::atan2(cz, std::max(0.01, eC - eW));
            if (a <= f) { sc.xf.add(W, k, kSign * a); return; }
            sc.xf.add(W, k, kSign * f);
            V3 Cp = W + Scene::rotAbout(C - W, k, kSign * f);  // the corner, now on the floor
            sc.xf.add(Cp, k, kSign * (a - f));
        };
        if (gm.roll != 0.0)
            lean(std::fabs(gm.roll), gm.roll > 0 ? side : side * -1.0, dir, gm.roll > 0 ? -1.0 : 1.0,
                 TIP_EDGE_Y_IN, TIP_CORNER_Y_IN, TIP_CORNER_Z_IN);
        if (gm.pitch != 0.0)
            lean(std::fabs(gm.pitch), gm.pitch > 0 ? dir : dir * -1.0, side, gm.pitch > 0 ? 1.0 : -1.0, TIP_EDGE_X_IN,
                 gm.pitch > 0 ? halfLen + ALIGN_MOUTH_X : halfLen, gm.pitch > 0 ? ALIGN_Z0 : TIP_CORNER_Z_IN);
    }
    // Drive base: aluminum C-channel rails either side of each row of three 3.25 in omni wheels
    // (13 in track), cross members front, middle and back, six drive motors, battery and brain
    const double wheelY = std::min(6.5, halfWide - 1.0), wheelR = 1.625;
    const SDL_Color alu = {190, 193, 198, 255}, motorC = {28, 28, 30, 255}, hubC = {120, 122, 128, 255};
    for (double sd : {-1.0, 1.0}) {
        for (double off : {0.75, -0.75}) {
            sc.beam(at(-halfLen, sd * (wheelY + off), 1.9), at(halfLen, sd * (wheelY + off), 1.9), 0.25, 0.5, alu);
            if (!sc.lowDetail) for (double hx = -halfLen + 0.25; hx < halfLen; hx += 0.5) {  // VEX hole pattern on top
                V3 hc = at(hx, sd * (wheelY + off), 2.41);
                sc.poly({hc + dir * 0.09 + side * 0.09, hc - dir * 0.09 + side * 0.09, hc - dir * 0.09 - side * 0.09,
                         hc + dir * 0.09 - side * 0.09}, {70, 72, 78, 255}, {0, 0, 1}, true);
            }
        }
        for (double fx : WHEEL_X_IN) {
            V3 wc = at(fx, sd * wheelY, wheelR);
            ProfPt tireP[2] = {{-0.55, wheelR}, {0.55, wheelR}};
            SDL_Color tc[1] = {tire};
            const double spin = sd > 0 ? gm.wheelRotL : gm.wheelRotR;  // this side's wheel turn, radians
            sc.lathe(wc, side, tireP, 2, tc, 14, -spin, true, true);
            // Spokes on the outside face, turning with the wheel
            if (!sc.lowDetail) for (int sp5 = 0; sp5 < 5; ++sp5) {
                double a = -spin + sp5 * 2 * M_PI / 5;
                V3 radial = dir * std::cos(a) + V3{0, 0, std::sin(a)};
                V3 face = wc + side * (sd * 0.62);
                sc.beam(face + radial * 0.25, face + radial * 1.3, 0.12, 0.06, hubC);
            }
            ProfPt hubP[2] = {{0.56, 0.9}, {0.60, 0.9}};  // hub on the outside face
            SDL_Color hc[1] = {hubC};
            sc.lathe(wc + side * (sd > 0 ? 0.0 : -1.16), side, hubP, 2, hc, 10, 0, true, true);
            // Drive gearing (0.75:1): a 48-tooth gear on the wheel axle, driven by a 36-tooth pinion on the
            // motor just ahead of it; both turn with the wheel, the pinion faster and the other way
            const double gW = 1.2, gP = 0.9, gearY = sd * (wheelY - 0.95);
            const SDL_Color gearCol = {165, 32, 48, 255};  // maroon plastic VEX gears
            V3 wheelGearC = at(fx, gearY, wheelR);
            V3 pinionC = at(fx + (gW + gP) * M_SQRT1_2, gearY, wheelR + (gW + gP) * M_SQRT1_2);  // pinion up and ahead of its wheel (stays inside 18 in)
            sc.gear(wheelGearC, side, gW, 48, -spin, gearCol, 0.35);
            sc.gear(pinionC, side, gP, 36, spin * 48.0 / 36.0 + M_PI / 36, gearCol, 0.35);
            V3 motorAt = pinionC - side * (sd * 0.95);
            sc.box(motorAt, dir, 0.65, 0.65, pinionC.z - 0.65, pinionC.z + 0.65, motorC);       // drive motor
            // V5 cartridge colour on the motor's face: blue 600 rpm on the 11 W drive motors, green on the 5.5 W one
            const SDL_Color cartC = fx < -5.0 ? SDL_Color{60, 175, 80, 255} : SDL_Color{40, 110, 225, 255};
            sc.box(motorAt - side * (sd * 0.66), dir, 0.4, 0.4, pinionC.z - 0.4, pinionC.z + 0.4, cartC);  // motor cap
            if (gm.tractionMid && fx == WHEEL_X_IN[TRACTION_WHEEL]) {  // traction wheel: solid rubber tread with cross ribs, no rollers
                if (!sc.lowDetail) for (int rr = 0; rr < 16; ++rr) {
                    double a = rr * 2 * M_PI / 16 - spin;
                    V3 radial = dir * std::cos(a) + V3{0, 0, std::sin(a)};
                    V3 pr = wc + radial * (wheelR + 0.02);
                    sc.beam(pr - side * 0.5, pr + side * 0.5, 0.12, 0.08, {18, 18, 20, 255});
                }
                continue;
            }
            // Omni wheel rollers: two staggered rows around the rim
            if (!sc.lowDetail) for (int rr = 0; rr < 10; ++rr) {
                for (double row : {-0.28, 0.28}) {
                    double a = (rr + (row > 0 ? 0.5 : 0.0)) * 2 * M_PI / 10 - spin;
                    V3 radial = dir * std::cos(a) + V3{0, 0, std::sin(a)};
                    V3 tang = dir * -std::sin(a) + V3{0, 0, std::cos(a)};
                    V3 pr = wc + radial * (wheelR - 0.05) + side * row;
                    sc.beam(pr - tang * 0.32, pr + tang * 0.32, 0.17, 0.17, {120, 122, 128, 255});
                }
            }
        }
    }
    // License plates with the team number
    drawPlates(sc, robot, dir, halfWide - 0.15, std::min(5.0, halfLen - 1.0), wheelY + 0.75, gm.plate, allianceColour(gm.team));
    for (double fx : {-1.0, 0.0})
        sc.beam(at(fx * (halfLen - 0.25), -wheelY - 0.75, 2.6), at(fx * (halfLen - 0.25), wheelY + 0.75, 2.6), 0.25, 0.5, alu);
    // Front: no cross member across the opening, just short pieces either side of it and a
    // channel across the back of the opening
    for (double sd : {-1.0, 1.0})
        sc.beam(at(halfLen - 0.25, sd * (OPEN_HALF_IN + 0.25), 2.6), at(halfLen - 0.25, sd * (wheelY + 0.75), 2.6), 0.25, 0.5, alu);
    sc.beam(at(halfLen - OPEN_DEPTH_IN - 0.25, -wheelY - 0.75, 2.6), at(halfLen - OPEN_DEPTH_IN - 0.25, wheelY + 0.75, 2.6), 0.25, 0.5, alu);
    sc.box(at(-halfLen + 3.2, 0, 0), dir, 1.6, 2.6, 1.6, 3.2, {40, 40, 44, 255});    // battery
    sc.box(at(-halfLen + 3.2, 0, 0), dir, 2.0, 2.9, 3.6, 4.4, motorC);               // V5 brain
    sc.box(at(-halfLen + 3.4, 0, 0), dir, 1.3, 2.0, 4.4, 4.45, {60, 120, 190, 255}); // brain screen

    // DR6B, built like a real one: per side, a main post; the first six-bar stage (two sections,
    // the long bottom bar running on as the second section's tower) goes back; a middle tower with
    // one pair of meshing gears ties it to the second stage, which comes forward again to the
    // carriage, so the carriage rides straight up. The main posts sit either side of the front
    // opening, back inside the robot.
    const int NS = g_liftSections;  // sections: DR6B 4, DR4B 2 (the cascade and the clawbot arm draw none)
    const bool bars = g_liftType == 1 || g_liftType == 2;
    const double L = DR4B_BAR_IN, sp = DR4B_SPACING_IN, z0 = dr4bTowerPivotIn();
    const double xt = halfLen - DR4B_TOWER_BACK_IN, cth = std::cos(gm.liftAngle), sth = std::sin(gm.liftAngle);
    const double gMid = DR4B_MID_GEAR_R, g2 = 2 * gMid;
    const double yS = OPEN_HALF_IN + 0.5;                // just outside the front opening
    const SDL_Color gearCol = {165, 32, 48, 255};        // maroon plastic VEX gears
    // Section k runs from (xs[k], zb[k]) to (xe[k], zb[k] + L sin); the next starts one gear-width forward
    double xs[4], xe[4], zb[4];
    for (int k = 0; k < NS; ++k) {
        const int half = NS / 2;  // sections per stage
        xs[k] = k == 0 ? xt : xe[k - 1] + (k == half ? g2 : 0.0);   // the gear pair sits between the stages
        zb[k] = k == 0 ? z0 : zb[k - 1] + secLen(k - 1) * sth + sp;
        xe[k] = xs[k] + (k < half ? -secLen(k) * cth : secLen(k) * cth);
    }
    if (!bars) for (int k = 0; k < NS; ++k) xs[k] = xe[k] = xt;  // cascade and clawbot: straight up
    const double xcar = xe[NS - 1];
    const double xmBrace = xe[NS / 2 - 1];  // at the middle gears
    for (double sd : {-1.0, 1.0}) {
        double y = sd * yS;
        if (g_liftType == 3) {
            // Official V5 Clawbot arm, scaled up: a short tower at the back, an arm that swings up and forward
            // from a geared pivot (two parallel bars a side, like the kit's arm), the claw on its end
            const double gxC = halfLen + REACH_IN, xcl = gxC - 2.0, xp = -halfLen + 1.0, zp = 11.5;
            const double zc = gm.liftIn + 3.7;                       // arm end: the claw's top plate
            const double ang = std::atan2(zc - zp, xcl - xp);
            sc.box(at(xp, y, 0), dir, 0.6, 0.5, 2.6, zp + 2.2, metal);                                  // tower post
            sc.box(at(xp, y + sd * 0.1, 0), dir, 1.4, 0.3, 2.6, 3.0, metal);                            // foot plate
            sc.beam(at(xp + 3.6, y, 2.6), at(xp, y, zp - 1.0), 0.2, 0.4, metal);                        // brace
            const double gBig = 1.75, gSmall = 0.3;
            sc.gear(at(xp, y + sd * 0.8, zp), side, gBig, 84, -ang, gearCol, 0.35);                      // 84-tooth gear on the arm pivot
            sc.gear(at(xp, y + sd * 0.8, zp - gBig - gSmall), side, gSmall, 12, ang * DR4B_GEAR, gearCol, 0.35);
            sc.box(at(xp, y + sd * 1.35, 0), dir, 1.05, 0.55, zp - gBig - gSmall - 1.05, zp - gBig - gSmall + 1.05, {22, 22, 24, 255});  // arm motor
            sc.box(at(xp, y + sd * 1.92, 0), dir, 0.55, 0.05, zp - gBig - gSmall - 0.55, zp - gBig - gSmall + 0.55, {60, 175, 80, 255});
            for (double dz : {0.0, 1.8})
                sc.beam(at(xp, y, zp + dz), at(xcl, y, zc + dz - 1.8), 0.28, 0.45, metal);               // arm bars
            sc.beam(at(xp + 0.9, y - sd * 0.3, zp - 0.2), at(xp + 0.9, y - sd * 0.3, zp + 2.0), 0.15, 0.2, dark);  // pivot standoffs
            sc.box(at(xcl, y - sd * 0.3, 0), dir, 0.5, 0.3, zc - 2.1, zc + 0.3, dark);                  // arm end plate
            continue;
        }
        sc.box(at(xt, y, 0), dir, 0.6, 0.5, 2.6, ROBOT_START_H_IN - 0.3, metal);           // main post, up to the 18 in start height
        if (!sc.lowDetail) for (double hz = 3.0; hz < ROBOT_START_H_IN - 0.6; hz += 0.5) {                    // VEX hole pattern on its inner face
            V3 hc = at(xt, y - sd * 0.51, hz);
            sc.poly({hc + dir * 0.1 + V3{0, 0, 0.1}, hc - dir * 0.1 + V3{0, 0, 0.1}, hc - dir * 0.1 - V3{0, 0, 0.1},
                     hc + dir * 0.1 - V3{0, 0, 0.1}}, {70, 72, 78, 255}, side * -sd, false);
        }
        // Lift gearing (1:7): an 84-tooth gear on the first section's lower pivot, driven by a
        // 12-tooth pinion on the lift motor just below it
        {
            const double gBig = 1.75, gSmall = 0.3;
            V3 bigC = at(xt, y + sd * 0.7, z0);
            V3 pinC = at(xt, y + sd * 0.7, z0 - gBig - gSmall);
            sc.gear(bigC, side, gBig, 84, -gm.liftAngle, gearCol, 0.35);
            sc.gear(pinC, side, gSmall, 12, gm.liftAngle * DR4B_GEAR, gearCol, 0.35);
            sc.box(at(xt, y + sd * 1.25, 0), dir, 1.05, 0.55, pinC.z - 1.05, pinC.z + 1.05, {22, 22, 24, 255});
            sc.box(at(xt, y + sd * 1.82, 0), dir, 0.55, 0.05, pinC.z - 0.55, pinC.z + 0.55, {60, 175, 80, 255});  // green 200 rpm cartridge
        }
        if (g_liftType == 0) {  // cascade: two nested slides ride up beside the post, the carriage and claw on the top one
            const double hC = gm.liftIn + 2.0;
            sc.box(at(xt + 1.0, y - sd * 0.4, 0), dir, 0.5, 0.3, 3.0, 3.0 + std::max(hC * 0.5, 4.0) + 10.0, dark);
            sc.box(at(xt + 1.8, y - sd * 0.4, 0), dir, 0.5, 0.3, 3.0 + std::max(hC - 6.0, 0.0), hC + 6.0, SDL_Color{120, 122, 130, 255});
        }
        if (g_liftType == 3) {  // clawbot: one arm from a pivot low on the post up to the claw
            const double hC = gm.liftIn + 2.0;
            sc.box(at(xt + 7.0, y - sd * 0.4, 0), dir, 0.5, 0.4, 4.0, 6.0, dark);
            sc.beam(at(xt + 7.0, y - sd * 0.4, 5.0), at(xt, y - sd * 0.4, hC), 0.35, 0.5, metal);
        }
        for (int k = 0; k < NS && bars; ++k) {
            double yk = y + sd * (k < NS / 2 ? -0.75 : 0.75);  // first stage on the post's inner face, second on its outer face
            double ze = zb[k] + secLen(k) * sth;
            sc.beam(at(xs[k], yk, zb[k]), at(xe[k], yk, ze), 0.22, 0.45, metal);
            sc.beam(at(xs[k], yk, zb[k] + sp), at(xe[k], yk, ze + sp), 0.22, 0.45, metal);
            if (k + 1 == NS / 2) {
                // Middle tower between the stages: a plate carrying the first stage's top pivots and
                // the second stage's bottom pivots, with the meshing gear pair that reverses them
                sc.box(at((xe[k] + xs[k + 1]) / 2, y, 0), dir, gMid + 0.5, 0.12, ze - 0.6, ze + 2 * sp + 0.6, dark);
                sc.gear(at(xe[k], y - sd * 0.3, ze + sp), side, gMid, 36, -gm.liftAngle, gearCol, 0.3);
                sc.gear(at(xs[k + 1], y - sd * 0.3, ze + sp), side, gMid, 36, gm.liftAngle + M_PI / 36, gearCol, 0.3);
            } else if (k + 1 < NS) {
                // Six-bar joint inside a stage: the long bar runs on and a short link plate carries
                // the next section's pivots (no gears)
                sc.box(at(xe[k], yk + sd * 0.3, 0), dir, 0.35, 0.1, ze - 0.4, ze + 2 * sp + 0.4, dark);
            }
        }
        double cz = !bars ? gm.liftIn + 2.0 : zb[NS - 1] + secLen(NS - 1) * sth;  // carriage lower pivot
        sc.beam(at(xcar, y + sd * 0.75, cz - 0.4), at(xcar, y + sd * 0.75, cz + sp + 0.4), 0.25, 0.45, dark);  // carriage upright
    }
    // Cross braces between the two sides: first middle tower and carriage
    if (bars) sc.beam(at(xmBrace, -yS, zb[NS / 2]), at(xmBrace, yS, zb[NS / 2]), 0.25, 0.35, dark);
    double clawZ = gm.liftIn + 2.0;  // equals the carriage's lower pivot height
    if (g_liftType != 3) sc.beam(at(xcar, -yS - 0.75, clawZ + sp), at(xcar, yS + 0.75, clawZ + sp), 0.25, 0.35, dark);
    else {
        const double xp = -halfLen + 1.0, zp = 11.5, xcl = halfLen + REACH_IN - 2.0, zc = gm.liftIn + 3.7;
        sc.beam(at(xp, -yS, zp + 2.2), at(xp, yS, zp + 2.2), 0.25, 0.35, dark);                          // tower cross bar
        sc.beam(at(xp, -yS, zp + 0.9), at(xp, yS, zp + 0.9), 0.15, 0.15, metal);                         // pivot axle
        sc.beam(at((xp + xcl) / 2, -yS, (zp + zc) / 2 + 0.9), at((xp + xcl) / 2, yS, (zp + zc) / 2 + 0.9), 0.2, 0.3, dark);  // arm cross bar
        sc.beam(at(xcl, -yS, zc - 1.0), at(xcl, yS, zc - 1.0), 0.25, 0.35, dark);                        // claw mount bar
    }

    // Passive toggle arms on the back of the DR6B: an arm out behind the middle tower, where the two
    // stages meet, with a black rubber pad on the end. The tower swings back and up with the lift, so
    // the pads rise and fall with it; backing them into a Toggle at its height rolls it one face.
    {
        double padX, padZ;
        ramPadPose(gm.liftAngle, halfLen, padX, padZ);
        const bool tall = g_liftType != 2;  // arms stand flush with the back bumper
        const double xm = tall ? -halfLen + 1.0 : xe[NS / 2 - 1];  // the middle tower
        const bool hit = gm.touching[0] || gm.touching[1] || gm.touching[2] || gm.touching[3];
        for (double sd : {-1.0, 1.0}) {
            const double yR = sd * (yS + (tall ? 1.0 : 0.0));
            sc.beam(at(xm + 0.2, yR, padZ - (tall ? RAM_TALL_RISE_IN : RAM_ARM_RISE_IN)), at(xm + 0.2, yR, padZ + 0.2), 0.3, 0.3, metal);   // riser
            sc.beam(at(xm + 0.2, yR, padZ), at(padX + 0.3, yR, padZ), 0.3, 0.35, metal);                       // arm
            sc.beam(at(xm, yR, padZ - 1.6), at(padX + 1.8, yR, padZ - 0.2), 0.12, 0.12, metal);                // brace
            sc.box(at(padX + 0.15, yR, 0), dir, 0.3, 0.9, padZ - 0.9, padZ + 0.9,
                   hit ? SDL_Color{60, 60, 64, 255} : SDL_Color{28, 28, 30, 255});                             // rubber pad
        }
    }

    // Ratcheting claw (like the reference photo): two vertical shafts either side of the grip point,
    // each stacked with grey and black wheels, hanging from a top plate on the carriage. Pushing the
    // claw onto a piece rolls it in between the wheel stacks; ratchet gears with spring pawls on top
    // of the shafts stop the wheels turning back, so it can't slide out. The claw motor (Space / F)
    // spins the wheels in or out; out lifts the pawls first.
    if (g_liftType != 3) {
        const SDL_Color shaftC = {25, 25, 27, 255}, greyW = {150, 154, 160, 255}, blackW = {32, 32, 35, 255};
        const SDL_Color plateC = {190, 193, 198, 255}, ratchetC = {165, 32, 48, 255}, pawlC = {225, 70, 60, 255};
        const double gx = halfLen + REACH_IN;            // shafts beside the held piece's center
        const double shaftY = 2.35;                      // shaft spacing: wheels squeeze a 2 in pin
        const double zBot = gm.liftIn + 0.35, zTop = gm.liftIn + 4.4;
        // Top plate from the carriage out over the shafts, and a brace back to the carriage
        const double px0 = std::min(gx - 1.4, xcar - 0.4), px1 = std::max(gx + 1.4, xcar + 0.2);
        sc.box(at((px0 + px1) / 2, 0, 0), dir, (px1 - px0) / 2, shaftY + 0.85, zTop, zTop + 0.3, plateC);
        sc.box(at(xcar - 0.2, 0, 0), dir, 0.3, shaftY + 0.85, clawZ - 0.35, zTop, plateC);  // drop plate on the carriage
        // Wheels turn with the claw motor (in on Space, out on F); the two sides turn opposite ways
        const double roll = gm.clawDeg * M_PI / 180.0 * 1.5;
        const double wz[4] = {gm.liftIn + 0.85, gm.liftIn + 1.75, gm.liftIn + 2.65, gm.liftIn + 3.55};
        const double wr[4] = {1.35, 1.0, 1.35, 1.35};
        const SDL_Color wc[4] = {greyW, blackW, greyW, greyW};
        for (double sd : {-1.0, 1.0}) {
            V3 axisBase = at(gx, sd * shaftY, zBot);
            sc.beam(axisBase, axisBase + V3{0, 0, zTop - zBot + 0.6}, 0.1, 0.1, shaftC);  // hex shaft
            double ang = roll * -sd;
            for (int k = 0; k < 4; ++k) {
                V3 c = at(gx, sd * shaftY, wz[k]);
                ProfPt tire[2] = {{-0.28, wr[k]}, {0.28, wr[k]}};
                SDL_Color tc[1] = {wc[k]};
                sc.lathe(c, V3{0, 0, 1}, tire, 2, tc, 16, ang, true, true);
                // Hub and spokes on top so the spin shows
                ProfPt hub[2] = {{0.28, 0.35}, {0.33, 0.35}};
                SDL_Color hc[1] = {{70, 72, 78, 255}};
                sc.lathe(c, V3{0, 0, 1}, hub, 2, hc, 6, ang, true, true);
                if (!sc.lowDetail) for (int s5 = 0; s5 < 4; ++s5) {
                    double a = ang + s5 * M_PI / 2;
                    V3 rad{std::cos(a), std::sin(a), 0};
                    sc.beam(c + rad * 0.35 + V3{0, 0, 0.3}, c + rad * (wr[k] - 0.15) + V3{0, 0, 0.3}, 0.07, 0.03, {90, 92, 98, 255});
                }
            }
            // Ratchet gear on top of the shaft, with its spring pawl (lifted while F lets go)
            V3 rc = at(gx, sd * shaftY, zTop + 0.5);
            sc.gear(rc, V3{0, 0, 1}, 0.75, 16, ang, ratchetC, 0.22);
            V3 pp = at(gx - 1.5, sd * (shaftY + 0.6), zTop + 0.5);
            V3 toG = norm(V3{rc.x - pp.x, rc.y - pp.y, 0});
            double lift = (gm.pawlUp * 30.0 + gm.pawlClick * 10.0) * M_PI / 180.0 * sd;
            V3 rot{toG.x * std::cos(lift) - toG.y * std::sin(lift), toG.x * std::sin(lift) + toG.y * std::cos(lift), 0};
            sc.beam(pp, pp + rot * (std::hypot(rc.x - pp.x, rc.y - pp.y) - 0.7), 0.1, 0.08, pawlC);
            sc.prism(pp.x, pp.y, zTop + 0.3, zTop + 0.7, 0.15, 6, 0, shaftC);
            sc.beam(pp + V3{0, 0, 0.1}, at(gx - 2.3, sd * (shaftY - 0.4), zTop + 0.6), 0.04, 0.04, {150, 30, 30, 255});  // rubber band
        }
        // Claw motor on the plate, driving both shafts through a chain/gear underneath
        sc.box(at(gx - 1.6, 0, 0), dir, 0.65, 0.65, zTop + 0.3, zTop + 1.3, {22, 22, 24, 255});
        sc.box(at(gx - 1.6, 0, 0), dir, 0.4, 0.4, zTop + 1.3, zTop + 1.4, {40, 110, 225, 255});  // blue 600 rpm cartridge (the claw motor is an 11 W blue motor)
    } else {
        // The kit's claw, scaled up: a plate on the arm end with the claw motor on top; two meshed gears turn the two
        // fingers (each a link out from its gear and a rubber-padded tip) together like a pincer
        const SDL_Color plateC = {190, 193, 198, 255}, gearC = {165, 32, 48, 255}, padC = {28, 28, 30, 255};
        const double gx = halfLen + REACH_IN, zb = gm.liftIn;
        const double gap = std::clamp(gm.held >= 0 ? widthIn((*gm.pieces)[gm.held]) : JAW_OPEN_IN, 1.0, JAW_OPEN_IN);
        sc.box(at(gx - 1.6, 0, 0), dir, 1.9, 4.2, zb + 3.4, zb + 3.7, plateC);                  // claw plate
        sc.box(at(gx - 1.6, 0, 0), dir, 0.65, 0.65, zb + 3.7, zb + 4.9, {22, 22, 24, 255});       // claw motor
        sc.box(at(gx - 1.6, 0, 0), dir, 0.4, 0.4, zb + 4.9, zb + 5.0, {40, 110, 225, 255});       // blue cartridge
        const double hx = gx - 2.6, hy = 0.8;
        double fAng[2];
        for (int si = 0; si < 2; ++si) {
            const double sd = si == 0 ? -1.0 : 1.0;
            const V3 H = at(hx, sd * hy, zb + 2.1), K = at(gx - 1.2, sd * (gap / 2 + 1.3), zb + 2.1), T = at(gx + 1.9, sd * (gap / 2 - 0.1), zb + 2.1);
            sc.beam(H, K, 0.14, 1.2, metal);                                                      // finger link
            sc.beam(K, T, 0.2, 1.2, padC);                                                        // rubber-padded tip
            sc.prism(H.x, H.y, zb + 0.8, zb + 3.4, 0.2, 8, 0, {25, 25, 27, 255});                 // hinge pin
            fAng[si] = std::atan2(sd * (gap / 2 + 1.3 - hy), 1.4);
        }
        sc.gear(at(hx, -hy, zb + 3.1), V3{0, 0, 1}, 0.8, 16, fAng[0], gearC, 0.3);
        sc.gear(at(hx, hy, zb + 3.1), V3{0, 0, 1}, 0.8, 16, -fAng[1] + M_PI / 16, gearC, 0.3);
    }

    // Goal alignment guide: two angled polycarbonate arms and a back plate on standoffs from the chassis
    {
        const SDL_Color poly = {200, 55, 60, 235};
        double zc = (ALIGN_Z0 + ALIGN_Z1) / 2, ht = (ALIGN_Z1 - ALIGN_Z0) / 2;
        for (double sd : {-1.0, 1.0}) {
            sc.beam(at(halfLen + ALIGN_BACK_X, sd * ALIGN_BACK_HALF, zc), at(halfLen + ALIGN_MOUTH_X, sd * ALIGN_MOUTH_HALF, zc), 0.06, ht, poly);
            sc.beam(at(halfLen + ALIGN_MOUTH_X - 0.2, sd * ALIGN_MOUTH_HALF, zc), at(halfLen + ALIGN_MOUTH_X - 0.2, sd * (OPEN_HALF_IN + 0.3), zc), 0.1, 0.1, metal);  // mount
        }
        sc.beam(at(halfLen + ALIGN_BACK_X, -ALIGN_BACK_HALF, zc), at(halfLen + ALIGN_BACK_X, ALIGN_BACK_HALF, zc), 0.06, ht, poly);
        sc.beam(at(halfLen + ALIGN_BACK_X, 0, zc), at(halfLen - OPEN_DEPTH_IN - 0.25, 0, zc), 0.1, 0.1, metal);  // mount
    }
    if (gm.held >= 0) {
        V3 hp = robot + dir * (halfLen + REACH_IN);
        addPiece(sc, (*gm.pieces)[gm.held], hp.x, hp.y, gm.liftIn);
        if (gm.rider >= 0) addPiece(sc, (*gm.pieces)[gm.rider], hp.x, hp.y, gm.liftIn + HALF_IN);
    }
    // Centre of mass (physics info setting): a yellow and black marker at the COM, drawn with the robot
    if (gm.showCom) {
        V3 cm = at(gm.comX, gm.comY, gm.comZ);
        sc.prism(cm.x, cm.y, cm.z - 0.45, cm.z + 0.45, 0.45, 8, 0, {250, 210, 40, 255});
        sc.beam(cm - dir * 1.1, cm + dir * 1.1, 0.08, 0.08, {20, 20, 20, 255});
        sc.beam(cm - side * 1.1, cm + side * 1.1, 0.08, 0.08, {20, 20, 20, 255});
    }
    sc.xf.on = false;
}

// rxPx, ryPx, heading: robot pose in sim units. halfW/halfH: robot half size in sim pixels.
inline void draw(SDL_Renderer* ren, int w, int h, double rxPx, double ryPx, double heading,
                 double halfWpx, double halfHpx) {
    Game& gm = game();
    Scene sc;
    sc.lightScale = gm.set.dim ? 0.62 : 1.0;
    const double F = 140.41;
    V3 robot{wx(rxPx), wy(ryPx), 0};
    V3 dir{std::cos(heading), -std::sin(heading), 0};
    double halfLen = halfWpx / PX_IN, halfWide = halfHpx / PX_IN;

    // Cameras (C / V cycle them; names in camName())
    const V3 fieldC{F / 2, F / 2, 0};
    V3 rside{dir.y, -dir.x, 0};  // the robot's right
    // Follow cameras stay 2 in inside the walls; for every inch pulled in they rise 0.7 in, so
    // backing into a wall gives a higher view instead of a camera behind the wall panels
    auto keepInside = [&](V3 e) {
        const double lo = 2.0, hi = F - 2.0;
        V3 c{std::clamp(e.x, lo, hi), std::clamp(e.y, lo, hi), e.z};
        double pulled = std::hypot(e.x - c.x, e.y - c.y);
        c.z += pulled * 0.7;
        return c;
    };
    int camSel = ((gm.camMode % CAM_COUNT) + CAM_COUNT) % CAM_COUNT;
    if (gm.playerOff && (camSel == 0 || camSel == 2 || camSel == 6 || camSel == 8 || camSel == 9 || camSel == 10 || camSel == 12))
        camSel = 5;  // the player's robot sits this match out: no cameras that follow it
    switch (camSel) {
        case 0:  // chase: behind and above the robot (kept inside the walls; it rises when it has to)
            sc.lookAt(keepInside(robot - dir * 34 + V3{0, 0, 26}), robot + dir * 22 + V3{0, 0, 3}, w, h, 62);
            break;
        case 1:  // standing at the red alliance station (left wall), eye height
            sc.lookAt({-30, F / 2, 62}, {78, F / 2, 0}, w, h, 70);
            break;
        case 2: {  // on the lift, looking past the claw
            V3 eye = robot + dir * (halfLen - 3) + V3{0, 0, gm.liftIn + 9};
            sc.lookAt(eye, robot + dir * (halfLen + 16) + V3{0, 0, gm.liftIn}, w, h, 70);
            break;
        }
        case 3:  // high above the field, looking almost straight down
            sc.lookAt({F / 2, F / 2 - 30, 190}, fieldC, w, h, 52);
            break;
        case 4:  // blue alliance station (right wall)
            sc.lookAt({F + 30, F / 2, 62}, {F - 78, F / 2, 0}, w, h, 70);
            break;
        case 5:  // audience side (bottom wall), raised like a stream camera
            sc.lookAt({F / 2, -55, 80}, {F / 2, F / 2 - 10, 0}, w, h, 62);
            break;
        case 6:  // beside the robot, from its right, to watch the lift and claw
            sc.lookAt(keepInside(robot + rside * (38 + gm.liftIn * 0.6) + dir * 4 + V3{0, 0, 14 + gm.liftIn * 0.5}),
                      robot + dir * 6 + V3{0, 0, 8 + gm.liftIn * 0.5}, w, h, 62);
            break;
        case 8:  // first person: from the top of the robot, looking where it's going
            sc.lookAt(robot + dir * (halfLen - 3) + V3{0, 0, 17.5}, robot + dir * 45 + V3{0, 0, 3}, w, h, 75);
            break;
        case 9:  // lift cam: from above the carriage, looking down into the claw
            sc.lookAt(robot + dir * (halfLen + 2) + V3{0, 0, gm.liftIn + 9}, robot + dir * (halfLen + REACH_IN) + V3{0, 0, gm.liftIn},
                      w, h, 70);
            break;
        case 10:  // high above the robot, following it, north up
            sc.lookAt(robot + V3{0, -10, 95}, robot, w, h, 55);
            break;
        case 11:  // from the top row of the bleachers
            sc.lookAt({F / 2, -140, 62}, {F / 2, F / 2 - 15, 0}, w, h, 55);
            break;
        case 12:  // the driver's view: standing at the red alliance station, eyes on the robot
            sc.lookAt({-46, F / 2 - 34, -STAGE_H_IN + 64}, robot + V3{0, 0, 6}, w, h, 60);
            break;
        case 13: {  // close up of the busiest Loader (the one whose tube is highest; else the first red one)
            int li = 0;
            for (int k = 1; k < 4; ++k) if (gm.loaderGate[k] > gm.loaderGate[li] + 1e-6) li = k;
            const LoaderDef& ld = LOADERS[li];
            const double sx = ld.x < F / 2 ? 1.0 : -1.0, sy = ld.y < F / 2 ? 1.0 : -1.0;
            sc.lookAt({ld.x + sx * 30.0, ld.y + sy * 20.0, 15.0}, {ld.x + sx * 2.0, ld.y, 9.0}, w, h, 58);
            break;
        }
        default: {  // slow orbit around the field
            double a = SDL_GetTicks() * 0.00015;
            sc.lookAt(fieldC + V3{std::cos(a) * 125, std::sin(a) * 125, 75}, fieldC, w, h, 62);
            break;
        }
    }
    sc.layered = sc.eye.x > 0 && sc.eye.x < F && sc.eye.y > 0 && sc.eye.y < F;

    // ── Outside the field ──
    // Event carpet, the alliance stations' tape lines, match-load racks, driver tables with
    // controllers, the scoring table on the audience side, and a backdrop behind the far wall.
    if (gm.set.outside) {
        const double GZ = -STAGE_H_IN;
        const SDL_Color carpet = {38, 44, 62, 255};
        sc.quadFloor(-160, -180, F + 160, F + 160, GZ, carpet);
        for (int side2 = 0; side2 < 2; ++side2) {
            SDL_Color lc = side2 == 0 ? SDL_Color{215, 35, 45, 255} : SDL_Color{30, 150, 220, 255};
            double xLine = side2 == 0 ? -36.0 : F + 36.0, xWall = side2 == 0 ? -7.0 : F + 7.0;
            sc.tape(xLine, 22, xLine, F - 22, 2.0, lc, GZ + 0.03);
            sc.tape(xLine, 22, xWall, 22, 2.0, lc, GZ + 0.03);
            sc.tape(xLine, F - 22, xWall, F - 22, 2.0, lc, GZ + 0.03);
        }
    }

    // Floor: 6 x 6 foam tiles
    const double T = F / 6;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            Uint8 g = ((i + j) % 2) ? 96 : 104;
            sc.quadFloor(i * T, j * T, (i + 1) * T, (j + 1) * T, 0, {g, g, (Uint8)(g + 4), 255});
        }
    // Foam tile seams (the interlocking edges show as darker lines)
    for (int i = 1; i < 6; ++i) {
        sc.tape(i * T, 0, i * T, F, 0.18, {78, 78, 84, 255});
        sc.tape(0, i * T, F, i * T, 0.18, {78, 78, 84, 255});
    }

    // Tape (official drawing): the Midfield diamond; diagonals from the load zones to the diamond,
    // doubled (the Autonomous Line) on the top-left and bottom-right; red / blue load zone Ls
    SDL_Color white = {235, 235, 235, 255};
    double m = MIDFIELD_IN, c = F / 2, lz = LOADZONE_X, dEnd = c - m / 2;
    sc.tape(c, c + m, c + m, c, TAPE_IN, white);
    sc.tape(c + m, c, c, c - m, TAPE_IN, white);
    sc.tape(c, c - m, c - m, c, TAPE_IN, white);
    sc.tape(c - m, c, c, c + m, TAPE_IN, white);
    sc.tape(lz, lz, dEnd, dEnd, TAPE_IN, white);                     // bottom-left
    sc.tape(F - lz, F - lz, F - dEnd, F - dEnd, TAPE_IN, white);     // top-right
    for (double o : {-0.875, 0.875}) {
        double ox = o * M_SQRT1_2, oy = o * M_SQRT1_2;
        sc.tape(lz + ox, F - lz + oy, dEnd + ox, F - dEnd + oy, 0.75, white);   // top-left (doubled)
        sc.tape(F - lz + ox, lz + oy, F - dEnd + ox, dEnd + oy, 0.75, white);   // bottom-right (doubled)
    }
    for (int side = 0; side < 2; ++side) {
        SDL_Color lc = side == 0 ? SDL_Color{215, 35, 45, 255} : SDL_Color{30, 150, 220, 255};
        double x = side == 0 ? lz : F - lz, wallX = side == 0 ? 0.0 : F;
        for (double yEdge : {TILE_IN, F - TILE_IN}) {
            double yCorner = yEdge < c ? 0.0 : F;
            sc.tape(wallX, yEdge, x, yEdge, TAPE_IN * 1.4, lc);
            sc.tape(x, yEdge, x, yCorner, TAPE_IN * 1.4, lc);
        }
    }

    // Loaders: a clear tube standing right on the floor against the wall (alliance-colored rims), held
    // between two uprights on a dark back plate. When a robot comes up to its foot the tube rides up
    // (LOADER_RISE_IN) to open the bottom, and when the robot leaves it drops all the way back to the floor.
    for (const LoaderDef& ld : LOADERS) {
        V3 inward = ld.x < c ? V3{1, 0, 0} : V3{-1, 0, 0};
        SDL_Color lc = ld.c == C_RED ? SDL_Color{220, 60, 75, 255} : SDL_Color{60, 170, 225, 255};
        V3 base{ld.x, ld.y, 0};
        const double rise = gm.loaderGate[&ld - LOADERS] * LOADER_RISE_IN, topZ = LOADER_H + LOADER_RISE_IN + 1.0;
        const SDL_Color darkC = {32, 33, 36, 255};
        sc.box(base - inward * (LOADER_D_IN / 2 - 0.35), inward, 0.35, LOADER_W_IN / 2, 0, topZ, darkC);   // back plate on the wall
        for (double sd : {-1.0, 1.0})                                                                         // the two uprights the tube slides on
            sc.box(base + V3{inward.x * 1.2, sd * (LOADER_W_IN / 2 - 0.15), 0}, inward, 0.15, 0.15, 0, topZ, darkC);
        const double tr = LOADER_TUBE_D / 2;
        ProfPt tube[2] = {{0, tr}, {LOADER_H, tr}};
        SDL_Color tcol[1] = {{215, 228, 236, 90}}, tin[1] = {{170, 190, 205, 60}};
        sc.maxCell = 2.0;  // the tall clear tube sorts in short rings against the pieces inside it
        sc.lathe({ld.x, ld.y, rise}, {0, 0, 1}, tube, 2, tcol, 16, 0, false, false, 0.06, tin);
        sc.maxCell = 0;
        SDL_Color rc[1] = {lc};
        ProfPt rimTop[2] = {{LOADER_H - 1.0, tr + 0.08}, {LOADER_H, tr + 0.08}};
        ProfPt rimBot[2] = {{0.0, tr + 0.08}, {0.8, tr + 0.08}};
        sc.lathe({ld.x, ld.y, rise}, {0, 0, 1}, rimTop, 2, rc, 16, 0, false, false, 0.1, rc);
        sc.lathe({ld.x, ld.y, rise}, {0, 0, 1}, rimBot, 2, rc, 16, 0, false, false, 0.1, rc);
    }

    // Perimeter (inner faces only, so they vanish when the camera is outside them): 11.5 in
    // clear panels with a black-and-white checkered rail along the top
    const double WH = 11.5, RAIL = 2.4;
    SDL_Color wall = {205, 210, 214, 255}, railW = {235, 235, 235, 255}, railB = {25, 25, 28, 255};
    struct WallDef { V3 a, b, n; };
    const WallDef walls[4] = {{{0, 0, 0}, {0, F, 0}, {1, 0, 0}}, {{F, F, 0}, {F, 0, 0}, {-1, 0, 0}},
                              {{F, 0, 0}, {0, 0, 0}, {0, 1, 0}}, {{0, F, 0}, {F, F, 0}, {0, -1, 0}}};
    // Drawn as a grid of small cells (64 along each wall), so each cell sorts correctly against
    // the pieces, Loaders and robot right beside it. The rail's black squares are cells of the
    // grid rather than squares laid over a white strip, so they can't flicker against it.
    sc.layer = L_WALL;
    for (const WallDef& wd : walls) {
        V3 up{0, 0, 1};
        if (dot(wd.n, sc.eye - wd.a) <= 0) continue;  // inner face turned away from the camera
        const int cells = 64, panelRows = 3;
        V3 step = (wd.b - wd.a) * (1.0 / cells);
        for (int k = 0; k < cells; ++k) {
            V3 p = wd.a + step * k;
            for (int row = 0; row < panelRows; ++row) {
                double z0 = (WH - RAIL) * row / panelRows, z1 = (WH - RAIL) * (row + 1) / panelRows;
                sc.poly({p + up * z0, p + step + up * z0, p + step + up * z1, p + up * z1}, wall, wd.n, false);
            }
            for (int row = 0; row < 2; ++row) {
                bool black = (k * 7 + row * 3 + (k / 5)) % 3 == 0;  // irregular, like the real rail
                double z0 = WH - RAIL + row * RAIL / 2, z1 = z0 + RAIL / 2;
                sc.poly({p + up * z0, p + step + up * z0, p + step + up * z1, p + up * z1}, black ? railB : railW, wd.n, false);
            }
        }
    }

    // Outside of the perimeter: white panels, a dark base kick plate, and support posts (part of the field)
    sc.maxCell = 3.0;
    {
        const SDL_Color outer = {226, 228, 232, 255}, kick = {60, 62, 68, 255}, postC = {45, 47, 52, 255};
        const double TH = 1.2;  // wall thickness
        for (const WallDef& wd : walls) {
            V3 up{0, 0, 1}, out = wd.n * -TH, on = wd.n * -1.0;
            sc.layer = L_WALL;
            sc.poly({wd.a + up * WH, wd.b + up * WH, wd.b + out + up * WH, wd.a + out + up * WH}, {200, 202, 206, 255}, up, true);  // top of the wall
            sc.layer = L_OUTSIDE;
            // Outer panel in two bands (the dark kick plate at the bottom, white above), not one
            // laid over the other
            sc.poly({wd.b + out, wd.a + out, wd.a + out + up * 1.5, wd.b + out + up * 1.5}, kick, on, true);
            sc.poly({wd.b + out + up * 1.5, wd.a + out + up * 1.5, wd.a + out + up * WH, wd.b + out + up * WH}, outer, on, true);
            for (int k = 0; k <= 3; ++k) {  // posts at the panel joints
                V3 pp = wd.a + (wd.b - wd.a) * (k / 3.0) + wd.n * -(TH + 0.6);
                sc.box(pp, norm(wd.b - wd.a), 0.7, 0.7, 0, WH + 0.6, postC);
            }
        }
    }
    if (gm.set.outside) {
        const double GZ = -STAGE_H_IN;  // the arena floor: the field sits on a stage this high
        // Stage skirt: black curtain all round the stage, with a lighter trim band at the top
        const double E = 6.0;  // the stage runs this far out past the walls
        const V3 sc0{-E, -E, 0}, sc1{F + E, -E, 0}, sc2{F + E, F + E, 0}, sc3{-E, F + E, 0};
        const V3 corners[5] = {sc0, sc1, sc2, sc3, sc0};
        for (int k = 0; k < 4; ++k) {
            V3 a = corners[k], b = corners[k + 1];
            V3 n = norm(cross(b - a, V3{0, 0, 1}));
            sc.poly({a + V3{0, 0, GZ}, b + V3{0, 0, GZ}, b + V3{0, 0, -0.6}, a + V3{0, 0, -0.6}}, {18, 18, 22, 255}, n, true);
            sc.poly({a + V3{0, 0, -0.6}, b + V3{0, 0, -0.6}, b, a}, {90, 92, 100, 255}, n, true);
        }
        // Stage deck between the field walls and the skirt
        sc.quadFloor(-E, -E, F + E, 0, -0.01, {70, 72, 80, 255});
        sc.quadFloor(-E, F, F + E, F + E, -0.01, {70, 72, 80, 255});
        sc.quadFloor(-E, 0, 0, F, -0.01, {70, 72, 80, 255});
        sc.quadFloor(F, 0, F + E, F, -0.01, {70, 72, 80, 255});

        // Match-load racks beside each alliance station: standing pins and cups waiting to go in the Loaders
        for (int side2 = 0; side2 < 2; ++side2) {
            double xr = side2 == 0 ? -20.0 : F + 20.0;
            Col allianceCol = side2 == 0 ? C_RED : C_BLUE;
            sc.box({xr, F / 2, 0}, {0, 1, 0}, 22.0, 3.0, GZ, GZ + 30.0, {64, 66, 74, 255});  // rack stand, level with the field
            for (int k = 0; k < 10; ++k) {
                double y = F / 2 - 19.0 + k * 4.2;
                Piece pc{k % 2 == 0, 0, 0, allianceCol, C_YEL};
                pc.grayUp = (k % 4 == 1);
                addPiece(sc, pc, xr, y, GZ + 30.0);
            }
            // Driver tables with a V5 controller each
            for (double ty : {F / 2 - 34.0, F / 2 + 34.0}) {
                V3 tc{side2 == 0 ? -44.0 : F + 44.0, ty, 0};
                sc.box(tc, {0, 1, 0}, 8.0, 5.0, GZ + 28.0, GZ + 29.0, {120, 92, 64, 255});  // table top
                for (double lx : {-7.0, 7.0})
                    for (double ly : {-4.0, 4.0})
                        sc.box(tc + V3{ly, lx, 0}, {0, 1, 0}, 0.5, 0.5, GZ, GZ + 28.0, {50, 50, 54, 255});
                sc.box(tc, {0, 1, 0}, 3.2, 1.6, GZ + 29.0, GZ + 30.0, {30, 30, 33, 255});           // controller
                sc.box(tc, {0, 1, 0}, 1.2, 0.9, GZ + 30.0, GZ + 30.05, {70, 130, 200, 255});        // its screen
            }
        }
        // Scoring table on the audience side, with a monitor showing the match
        {
            sc.maxCell = 0;  // the screen sits on the monitor's face; keep both whole so it stays in front
            V3 tc{F / 2, -40.0, 0};
            sc.box(tc, {1, 0, 0}, 30.0, 8.0, GZ + 28.0, GZ + 29.2, {90, 92, 100, 255});
            for (double lx : {-28.0, 28.0})
                for (double ly : {-6.5, 6.5})
                    sc.box(tc + V3{lx, ly, 0}, {1, 0, 0}, 0.6, 0.6, GZ, GZ + 28.0, {50, 50, 54, 255});
            sc.box(tc + V3{0, 3.0, 0}, {1, 0, 0}, 11.0, 0.6, GZ + 31.0, GZ + 44.0, {20, 20, 24, 255});  // monitor
            sc.poly({tc + V3{-10.2, 3.65, GZ + 32}, tc + V3{10.2, 3.65, GZ + 32}, tc + V3{10.2, 3.65, GZ + 43}, tc + V3{-10.2, 3.65, GZ + 43}},
                    {40, 90, 160, 255}, {0, 1, 0}, true);                                                 // its screen
            sc.box(tc + V3{0, 3.0, 0}, {1, 0, 0}, 0.6, 0.6, GZ + 29.2, GZ + 31.0, {50, 50, 54, 255});   // stand
            sc.maxCell = 6.0;
        }
        // Bleachers behind the scoring table: five rising rows of grey steps with blue seat boards
        for (int row = 0; row < 5; ++row) {
            double y0 = -70.0 - row * 16.0, zTop = GZ + 10.0 + row * 10.0;
            sc.box({F / 2, y0 - 8.0, 0}, {1, 0, 0}, F / 2 + 50.0, 8.0, GZ, zTop, {96, 100, 108, 255});
            sc.box({F / 2, y0 - 3.0, 0}, {1, 0, 0}, F / 2 + 50.0, 3.0, zTop, zTop + 1.0, {40, 80, 160, 255});
        }
        // Backdrop behind the far wall: a dark event banner with a yellow stripe
        {
            sc.maxCell = 0;  // stripes lie on the banner; keep them whole
            const double yB = F + 70.0;
            sc.poly({{-60, yB, GZ}, {F + 60, yB, GZ}, {F + 60, yB, 60}, {-60, yB, 60}}, {28, 34, 58, 255}, {0, -1, 0}, true);
            sc.poly({{-60, yB - 0.1, 38}, {F + 60, yB - 0.1, 38}, {F + 60, yB - 0.1, 42}, {-60, yB - 0.1, 42}}, {235, 200, 40, 255}, {0, -1, 0}, true);
            sc.poly({{-60, yB - 0.1, 30}, {F / 2 - 10, yB - 0.1, 30}, {F / 2 - 10, yB - 0.1, 33}, {-60, yB - 0.1, 33}}, {215, 35, 45, 255}, {0, -1, 0}, true);
            sc.poly({{F / 2 + 10, yB - 0.1, 30}, {F + 60, yB - 0.1, 30}, {F + 60, yB - 0.1, 33}, {F / 2 + 10, yB - 0.1, 33}}, {30, 150, 220, 255}, {0, -1, 0}, true);
        }
    }
    sc.layer = L_FIELD;
    sc.maxCell = 0;

    // Toggles (official drawing): 26 in rounded-triangle bars, 2.03 in tall and 2.19 in wide, one yellow, one red and one
    // blue face, pivoting 13.2 in above the floor on mounts on top of the middle of each wall.
    // The face of the color it is set to points straight up.
    for (int k = 0; k < 4; ++k) {
        const double half = 13.0, R = 1.19, axisZ = 13.2, inset = 0.9;
        V3 ctr = k == 0 ? V3{inset, c, axisZ} : k == 1 ? V3{F - inset, c, axisZ} : k == 2 ? V3{c, inset, axisZ} : V3{c, F - inset, axisZ};
        V3 along = (k <= 1) ? V3{0, 1, 0} : V3{1, 0, 0};
        V3 inward = k == 0 ? V3{1, 0, 0} : k == 1 ? V3{-1, 0, 0} : k == 2 ? V3{0, 1, 0} : V3{0, -1, 0};
        V3 up{0, 0, 1};
        const Col order[3] = {C_YEL, C_RED, C_BLUE};
        int setI = gm.toggle[k] == C_RED ? 1 : gm.toggle[k] == C_BLUE ? 2 : 0;
        const double turn = gm.toggleTurn[k] * 2 * M_PI / 3;  // part-way through rolling to the next face
        auto dirAt = [&](double ang) { return inward * std::cos(ang) + up * std::sin(ang); };
        const double faceUp = 90.0 * M_PI / 180.0;  // the set color faces straight up
        // Cross-section from the official drawing: a rounded triangle 2.03 in tall and about 2.19 in wide, the pivot 1.19 in
        // from the tip. Three flat faces 0.84 in from the pivot, joined by corner arcs of radius 0.49 whose centres are 0.70 in
        // out, so every corner reaches 1.19 in from the pivot.
        const double rc = 0.49, rcen = 0.70;
        V3 cen[3];
        for (int f = 0; f < 3; ++f) cen[f] = dirAt(faceUp - turn + (f - setI) * 2 * M_PI / 3 + M_PI / 3) * rcen;
        std::vector<V3> outline;  // the rounded outline, for the end caps
        for (int f = 0; f < 3; ++f) {
            const double fa = faceUp - turn + (f - setI) * 2 * M_PI / 3;  // this face's normal
            V3 n = dirAt(fa);
            V3 a = cen[(f + 2) % 3] + n * rc, b = cen[f] + n * rc;
            sc.maxCell = 3.0;  // the 26 in faces sort in short pieces
            sc.poly({ctr + a - along * half, ctr + b - along * half, ctr + b + along * half, ctr + a + along * half}, colOf(order[f]), n, true);
            for (int s = 0; s < 4; ++s) {  // the corner after this face turns 120 degrees to the next face's normal
                const double a0 = fa + s * (2 * M_PI / 3) / 4, a1 = fa + (s + 1) * (2 * M_PI / 3) / 4;
                V3 p0 = cen[f] + dirAt(a0) * rc, p1 = cen[f] + dirAt(a1) * rc;
                sc.poly({ctr + p0 - along * half, ctr + p1 - along * half, ctr + p1 + along * half, ctr + p0 + along * half},
                        {40, 40, 44, 255}, dirAt((a0 + a1) / 2), true);
            }
            sc.maxCell = 0;
            outline.push_back(a);
            outline.push_back(b);
        }
        SDL_Color mount = {30, 30, 32, 255};
        for (double e : {-1.0, 1.0}) {
            V3 endC = ctr + along * (e * (half + 0.1));
            std::vector<V3> cap;
            for (const V3& q : outline) cap.push_back(endC + q);
            sc.poly(cap, mount, along * e, false);  // end cap
            V3 m = endC + along * (e * 0.25) - inward * 0.3;
            sc.box({m.x, m.y, 0}, inward, 0.8, 0.3, WH - 1.0, axisZ + 0.4, mount);  // mounting bracket
        }
    }

    // Goals (official drawing): octagonal, 5.61 in across the base, narrowing to 3.49 in across the
    // top with a 2.37 in opening. Alliance: 0.39 in flange then the taper to 3.25 in. Neutral:
    // a 2.52 in block then the taper to 5.77 in. Center: a 5.52 in block then the taper to 8.77 in.
    // AprilTags on the four sloped faces; black goals have yellow trim on the block.
    for (int i = 0; i < 9; ++i) {
        SDL_Color gc = i < 2 ? SDL_Color{200, 38, 45, 255} : i < 4 ? SDL_Color{40, 150, 215, 255} : SDL_Color{34, 35, 38, 255};
        double gxIn = GOAL_POS_IN[i][0], gyIn = GOAL_POS_IN[i][1];
        const double k8 = 1.0 / std::cos(M_PI / 8);  // across-flats half width -> corner radius
        const double baseH = i < 4 ? 0.39 : (i < 8 ? 2.52 : 5.52), H = GOAL_H_IN[i];
        const double fb = 2.805, fTaper = i < 4 ? 2.60 : 2.55, ft = 1.745;
        ProfPt prof[4] = {{0, fb * k8}, {baseH, fb * k8}, {baseH, fTaper * k8}, {H, ft * k8}};
        SDL_Color gcol[3] = {gc, gc, gc};
        sc.lathe({gxIn, gyIn, 0}, {0, 0, 1}, prof, 4, gcol, 8, M_PI / 8, false, true);
        sc.prism(gxIn, gyIn, H, H + 0.03, 1.185, 16, 0, {12, 12, 12, 255});  // the opening
        if (i >= 4) {
            for (double zb : {0.18 * baseH, 0.82 * baseH}) {
                ProfPt band[2] = {{zb, fb * k8 + 0.02}, {zb + 0.12, fb * k8 + 0.02}};
                SDL_Color yc[1] = {{225, 195, 40, 255}};
                sc.lathe({gxIn, gyIn, 0}, {0, 0, 1}, band, 2, yc, 8, M_PI / 8, false, false);
            }
        }
        for (int d = 0; d < 4; ++d) {
            V3 nd = d == 0 ? V3{1, 0, 0} : d == 1 ? V3{0, 1, 0} : d == 2 ? V3{-1, 0, 0} : V3{0, -1, 0};
            V3 tg{-nd.y, nd.x, 0};
            auto onFace = [&](double z, double lat, double out) {
                double f = fTaper + (ft - fTaper) * (z - baseH) / (H - baseH);
                return V3{gxIn, gyIn, z} + nd * (f + out) + tg * lat;
            };
            V3 fn = norm(nd * (H - baseH) + V3{0, 0, 1} * (fTaper - ft));
            double zc = baseH + 0.42 * (H - baseH), hs = std::min(0.7, 0.3 * (H - baseH));
            sc.poly({onFace(zc - hs, -hs, 0.03), onFace(zc - hs, hs, 0.03), onFace(zc + hs, hs, 0.03), onFace(zc + hs, -hs, 0.03)},
                    {235, 235, 235, 255}, fn, true);
            double hi = hs * 0.55;
            sc.poly({onFace(zc - hi, -hi, 0.05), onFace(zc - hi, hi, 0.05), onFace(zc + hi, hi, 0.05), onFace(zc + hi, -hi, 0.05)},
                    {20, 20, 20, 255}, fn, true);
        }
        const auto& st = gm.stack[i];
        for (size_t k = 0; k < st.size(); ++k) addPiece(sc, gm.p[st[k]], gxIn, gyIn, stackBottomIn(i, (int)k) + gm.p[st[k]].fz);
    }

    // Loose pieces
    for (int i = 0; i < (int)gm.p.size(); ++i) {
        const Piece& pc = gm.p[i];
        if (loose(pc)) addPiece(sc, pc, wx(pc.x), wy(pc.y), floorBottomIn(gm, i) + pc.fz);
    }
    // Pieces in the Loaders: the bottom one waits in the opening at the foot, the rest stack in the
    // tube (still falling if just dropped in)
    for (int li = 0; li < 4; ++li)
        for (int k = 0; k < (int)gm.loaderStack[li].size(); ++k) {
            const Piece& pc = gm.p[gm.loaderStack[li][k]];
            addPiece(sc, pc, LOADERS[li].x + (loaderOutX(li) - LOADERS[li].x) * pc.slide, LOADERS[li].y, pc.lz);
        }

    // Other robots (settings menu): the same robot, driven by the computer
    for (const Bot& b : gm.bots)
        if (b.on) {
            sc.lowDetail = true;
            setLiftType(b.lift);  // its own lift type for the drawing
            drawRobotModel(sc, botLook(gm, b), V3{wx(b.x), wy(b.y), 0}, V3{std::cos(b.h), -std::sin(b.h), 0}, halfLen, halfWide);
            sc.lowDetail = false;
        }
    // This robot
    setLiftType(gm.set.liftBot[3]);
    if (!gm.playerOff) drawRobotModel(sc, playerLook(gm), robot, dir, halfLen, halfWide);

    sc.render(ren);
}

}  // namespace v3d
}  // namespace sim
