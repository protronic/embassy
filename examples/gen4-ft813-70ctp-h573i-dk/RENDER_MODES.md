# Framebuffer vs EVE GPU — Gegenüberstellung

Zwei Build-Varianten desselben UI (`canboss_touch`, `oxivgl_widget_demo`, …):

| | **Framebuffer** (Default) | **EVE GPU** (`eve` Feature) |
|---|---------------------------|-----------------------------|
| **Cargo** | `--features oxivgl-demo` | `--features oxivgl-demo,eve` |
| **LVGL** | `LV_USE_DRAW_SW` — Software-Renderer | `LV_USE_DRAW_EVE` — External GPU Renderer |
| **Pixelweg** | Host rendert RGB565 → `Ft81x::blit()` → `RAM_G` | Co-Prozessor zeichnet Widgets (DL-Befehle) |
| **Display-Liste** | Einmal `co_show_framebuffer()` (Bitmap-Scan) | Pro Frame neu (RENDER_START … SWAP) |
| **SRAM** | 2× ~62 KiB Stripe-Buffer + 128 KiB LVGL-Heap | Keine Stripe-Buffer |
| **SPI-Last** | Hoch (volle Pixel bei jedem Flush) | Niedriger (nur Zeichenbefehle + Assets) |
| **Scrollen** | Kann Stripe-Artefakte zeigen | Soll flüssiger wirken |
| **Boot-Log** | `render=framebuffer` | `render=eve_gpu` |
| **Doku** | [LVGL EVE Framebuffer](https://lvgl.io/docs/open/integration/external_display_controllers/eve/frame_buffer_mode) | [LVGL EVE GPU](https://lvgl.io/docs/open/integration/external_display_controllers/eve/gpu) |

## A/B-Test auf der Hardware

Gleiche Szene, gleiche Aktionen, Logs vergleichen.

### 1. Framebuffer flashen

```bash
cargo run --release --bin canboss_touch --features oxivgl-demo
```

### 2. EVE GPU flashen

```bash
cargo run --release --bin canboss_touch --features oxivgl-demo,eve
```

### 3. Vergleichs-Checkliste

| Kriterium | Framebuffer | EVE GPU |
|-----------|-------------|---------|
| Boot-Zeile `render=` | `framebuffer` | `eve_gpu` |
| Scrollen in langer Liste | | |
| Touch-Reaktion | | |
| Erster Paint nach Screen-Wechsel | | |
| `flushes=` in Heartbeat (bei Scroll) | steigt stark | weniger / anderes Muster |

Heartbeat (~2 s):

```text
oxivgl mode=framebuffer queued=… fed=… read_cb=… clicks=… flushes=…
oxivgl mode=eve_gpu     queued=… fed=… read_cb=… clicks=… flushes=…
```

`flushes` zählt im Framebuffer-Modus SPI-Blit-Aufrufe; im EVE-Modus Display-List-Swaps (andere Skala — subjektiv Scroll/Flackern zählt mehr).

### 4. Subjektiv notieren

- Scroll flüssig / ruckelnd / zeilenweise?
- Texte vollständig? (EVE: nur 4-bpp-Fonts mit stride 1)
- Erster Screen nach Menü-Wechsel: Verzögerung?

## Wann welcher Modus?

- **Framebuffer**: Bewährt, volle LVGL-Font/Widget-Kompatibilität, höhere SPI-Last.
- **EVE GPU**: Weniger CPU/SPI für Pixel, besseres Scrollen — Font-/Layer-Limits beachten.

Beide nutzen dieselbe Touch-Pipeline und Panel-Timings (gen4-70CTP).
