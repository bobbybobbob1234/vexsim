#pragma once
#include <SDL2/SDL.h>

namespace sim {

// Smooth anti-aliased text for the start screen and tournament pages (Windows: Segoe UI through GDI).
// uiTextOk() is false where it is not available; the caller then draws the pixel font instead.
bool uiTextOk();
int  uiTextWidth(const char* text, int px, bool bold);
void uiDrawText(SDL_Renderer* ren, int x, int y, const char* text, int px, SDL_Color c, bool bold);

}  // namespace sim
