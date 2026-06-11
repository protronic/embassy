#pragma once

#include <cstdint>

// Minimal Graphics4D API for Embassy LVGL scan-out glue (display_gfx4d.cpp).
// The prebuilt lib + full headers live under vendor/graphics4d-rp2350/.

#ifndef LANDSCAPE
#define LANDSCAPE 1
#endif

class Graphics4D {
  public:
    void Initialize();
    void ScreenMode(unsigned char mode);
    void Contrast(unsigned char level);
    void WriteToFrameBuffer(unsigned long offset, std::uint16_t *pixels, unsigned long count);
    void Refresh();
};

extern Graphics4D gfx;
