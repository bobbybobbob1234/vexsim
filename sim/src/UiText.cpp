#include "sim/UiText.hpp"
#include <string>
#include <cstring>
#include <cwchar>
#include <cctype>
#include <algorithm>
#include <unordered_map>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace sim {
namespace {
struct Entry { SDL_Texture* tex = nullptr; int w = 0, h = 0; };
std::unordered_map<std::string, Entry> g_cache;
std::unordered_map<int, HFONT> g_fonts;
HDC g_dc = nullptr;
bool g_ok = true;

HDC dc() {
    if (!g_dc) g_dc = CreateCompatibleDC(nullptr);
    return g_dc;
}
HFONT fontFor(int px, bool bold) {
    const int key = px * 2 + (bold ? 1 : 0);
    auto it = g_fonts.find(key);
    if (it != g_fonts.end()) return it->second;
    static const wchar_t* faces[2][2] = {{L"Segoe UI Variable Text", L"Segoe UI"}, {L"Segoe UI Variable Text Semibold", L"Segoe UI Semibold"}};
    HFONT f = nullptr;
    for (int k = 0; k < 2 && !f; ++k) {  // the first face that is really installed (GDI otherwise swaps in some other font silently)
        HFONT c = CreateFontW(-px, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_TT_PRECIS,
                              CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, faces[bold ? 1 : 0][k]);
        HGDIOBJ old = SelectObject(dc(), c);
        wchar_t got[64] = {0};
        GetTextFaceW(dc(), 64, got);
        SelectObject(dc(), old);
        if (c && wcscmp(got, faces[bold ? 1 : 0][k]) == 0) f = c;
        else if (c) DeleteObject(c);
    }
    if (!f) f = CreateFontW(-px, 0, 0, 0, bold ? FW_SEMIBOLD : FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_TT_PRECIS,
                            CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
    g_fonts[key] = f;
    return f;
}
std::wstring widen(const char* s) {
    std::wstring w;
    for (; *s; ++s) w.push_back((wchar_t)(unsigned char)*s);
    return w;
}
}  // namespace

bool uiTextOk() { return g_ok; }

int uiTextWidth(const char* text, int px, bool bold) {
    if (!*text) return 0;
    HDC d = dc();
    HGDIOBJ old = SelectObject(d, fontFor(px, bold));
    const std::wstring w = widen(text);
    SIZE sz = {0, 0};
    GetTextExtentPoint32W(d, w.c_str(), (int)w.size(), &sz);
    SelectObject(d, old);
    return (int)sz.cx;
}

void uiDrawText(SDL_Renderer* ren, int x, int y, const char* text, int px, SDL_Color c, bool bold) {
    if (!*text) return;
    const std::string key = std::string(text) + "|" + std::to_string(px) + (bold ? "b" : "n");
    auto it = g_cache.find(key);
    if (it == g_cache.end()) {
        Entry e;
        HDC d = dc();
        HGDIOBJ old = SelectObject(d, fontFor(px, bold));
        const std::wstring w = widen(text);
        SIZE sz = {0, 0};
        GetTextExtentPoint32W(d, w.c_str(), (int)w.size(), &sz);
        e.w = sz.cx + 2; e.h = sz.cy + 2;
        BITMAPINFO bi = {};
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = e.w;
        bi.bmiHeader.biHeight = -e.h;  // top-down
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        bi.bmiHeader.biCompression = BI_RGB;
        void* bits = nullptr;
        HBITMAP bmp = CreateDIBSection(d, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        if (bmp && bits) {
            HGDIOBJ oldBmp = SelectObject(d, bmp);
            std::memset(bits, 0, (size_t)e.w * e.h * 4);
            SetBkMode(d, TRANSPARENT);
            SetTextColor(d, RGB(255, 255, 255));
            TextOutW(d, 1, 1, w.c_str(), (int)w.size());
            GdiFlush();
            SDL_Surface* sf = SDL_CreateRGBSurfaceWithFormat(0, e.w, e.h, 32, SDL_PIXELFORMAT_ARGB8888);
            if (sf) {
                const Uint8* src = (const Uint8*)bits;
                Uint8* dst = (Uint8*)sf->pixels;
                for (int row = 0; row < e.h; ++row)
                    for (int col = 0; col < e.w; ++col) {
                        const Uint8* s = src + ((size_t)row * e.w + col) * 4;
                        Uint8* o = dst + (size_t)row * sf->pitch + col * 4;
                        const Uint8 a = std::max(s[0], std::max(s[1], s[2]));
                        o[0] = 255; o[1] = 255; o[2] = 255; o[3] = a;
                    }
                e.tex = SDL_CreateTextureFromSurface(ren, sf);
                if (e.tex) SDL_SetTextureBlendMode(e.tex, SDL_BLENDMODE_BLEND);
                SDL_FreeSurface(sf);
            }
            SelectObject(d, oldBmp);
        }
        if (bmp) DeleteObject(bmp);
        SelectObject(d, old);
        if (g_cache.size() > 3000) { for (auto& kv : g_cache) if (kv.second.tex) SDL_DestroyTexture(kv.second.tex); g_cache.clear(); }
        it = g_cache.emplace(key, e).first;
    }
    const Entry& e = it->second;
    if (!e.tex) return;
    SDL_SetTextureColorMod(e.tex, c.r, c.g, c.b);
    SDL_SetTextureAlphaMod(e.tex, c.a);
    SDL_Rect r = {x - 1, y - 1, e.w, e.h};
    SDL_RenderCopy(ren, e.tex, nullptr, &r);
}
}  // namespace sim
#else
namespace sim {
bool uiTextOk() { return false; }
int uiTextWidth(const char*, int, bool) { return 0; }
void uiDrawText(SDL_Renderer*, int, int, const char*, int, SDL_Color, bool) {}
}  // namespace sim
#endif
