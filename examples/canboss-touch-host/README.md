# CANbossTouch host (SDL2)

Host-Port der **CANbossTouch-Rust-Firmware** (Binary `canboss_touch` im
Beispiel [`gen4-ft813-70ctp-h573i-dk`](../gen4-ft813-70ctp-h573i-dk)) — im Stil
von [`oxivgl-host`](../oxivgl-host). Rendert dieselben EDS-Screens am
Entwicklungs-PC, um Layout, Widget-Auswahl und SDO-Wertdarstellung ohne
Hardware zu prüfen.

Der SDO-Client (`canboss/sdo.rs`) und die Views (`canboss/views/`) werden per
`#[path]` **direkt aus der Firmware geteilt** (nicht kopiert) — der Host-Build
rendert also exakt denselben Code wie das Gerät. Host-spezifisch sind nur das
CAN-Backend (`src/canboss/canbus.rs`, ein In-Memory-SDO-Server-Mock) und
`src/main.rs` (SDL-Fenster + Screenshot-Harness). Auch die EDS→Rust-Tabellen
entstehen aus **denselben** EDS-Dateien wie die Firmware (der `build.rs`-Codegen
`build_codegen.rs` wird geteilt).

Der Mock-Server beantwortet die echten Client-Requests wie ein CANopen-Knoten:
Upload-Requests liefern simulierte, datentyp-gerechte Werte (Dreieckskurven im
Limitbereich, BOOL-Toggle, REAL32, „Host …"-Strings), Downloads werden
gespeichert und zurückgelesen. Damit exerziert der Host-Build den **echten**
SDO-Protokollcode — expedited **und** segmented (die Strings > 4 Byte gehen als
segmentierter Upload).

Die PoC-Hallenlichtsteuerung ist im Host-Port ausgeblendet (sie hängt an
`touch_can`/FDCAN); dafür gibt es `examples/oxivgl-host`.

## Voraussetzungen

- **Nightly Rust** (siehe `rust-toolchain.toml`)
- **SDL2** + pkg-config: `sudo apt-get install libsdl2-dev pkg-config`
- `arm-none-eabi`-GCC + newlib (LVGL wird von `oxivgl-sys` aus Quellen gebaut)

## Starten

```bash
cd examples/canboss-touch-host
cargo run --release           # SDL-Fenster, Maus bedienbar
```

Headless-Vorschau/CI (rendert Menü + alle Knoten-Screens als BMP und beendet
sich; braucht einen X-Server, z. B. `xvfb-run`):

```bash
CANBOSS_HOST_SCREENSHOT=/tmp/canboss \
  xvfb-run -a -s "-screen 0 800x480x24" cargo run --release
```

## Netzwerk anpassen

Knoten und EDS-Dateien liegen bei der Firmware in
[`../gen4-ft813-70ctp-h573i-dk/eds/`](../gen4-ft813-70ctp-h573i-dk/eds); Host
und Target teilen sie sich. Nach Änderungen einfach neu bauen.
