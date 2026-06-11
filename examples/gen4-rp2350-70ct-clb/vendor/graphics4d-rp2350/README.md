# graphics4d-rp2350

Prebuilt Graphics4D static library for gen4-RP2350-70CT-CLB (Embassy / OxivGL).

## Layout

```
lib/libgraphics4d_rp2350.a   # merged static archive (~6.8 MB)
include/                     # public headers
board/gen4_rp2350_70ct.h     # Pico board definition
```

## Link

```
-Llib -lgraphics4d_rp2350 -lstdc++ -lm -lc
```

## Compile includes

```
-Iinclude -Iboard
-DPICO_BOARD=gen4_rp2350_70ct -DGEN4_RP2350_70CT -DGEN4_RP2350_RGB
```

Also requires Raspberry Pi Pico SDK 2.1+ headers on the include path when compiling glue code.

## Embassy OxivGL minimal API

```cpp
#include "Graphics4D.h"
extern Graphics4D gfx;

gfx.Initialize();
gfx.ScreenMode(LANDSCAPE);
gfx.WriteToFrameBuffer(0, pixels, width * height);
gfx.Refresh();
```

## Rebuild from source

See https://github.com/protronic/Graphics4D-pico
