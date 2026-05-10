# LightWare SF30/D LiDAR — STM32G431 firmware

Bare-metal firmware for a custom PCB based on the **STM32G431** that talks
to a **LightWare SF30/D microLiDAR** over UART using its native binary
protocol

The firmware:

1. Performs the §10 handshake with the sensor (`Product Name`, CMD 0).
2. Reads the configured **measurement update rate** from CMD 76 (the
   value persisted in the SF30/D's flash by the LightWare Studio
   configurator, or by the included Python recorder under `misc/`). This
   is the rate that actually governs CMD 40 streaming — see "Which rate
   matters?" below for why it's CMD 76 and not CMD 79.
3. Starts a CMD 40 full-speed distance stream.
4. Once per second prints the **configured rate**, the **measured rate**
   (computed from the actual incoming readings), and the **most recent
   distance**, so you can change settings on the sensor and watch the
   firmware report the new value live.

```
=== SF30/D LiDAR live readout ===
Debug : USART1 PB6/PB7 @ 115200 baud
Sensor: USART2 PA2/PA3 @ 921600 baud
Handshake OK: "SF30"
CMD 76 measurement update rate (configured) : 20010 Hz
Streaming CMD 40 (full-speed distance). Reporting every 1 s.
--------------------------------------------------
[   1.000s] cfg=20010 Hz  meas=19998 Hz  d=  342 cm  pkts=2502
[   2.000s] cfg=20010 Hz  meas=19996 Hz  d=  339 cm  pkts=2500
[   3.000s] cfg=20010 Hz  meas=19999 Hz  d=  341 cm  pkts=2501
```

## Hardware notes

- **USART1** (PB6 / PB7, 115200 8N1) — debug serial, where `printf` lands.
- **USART2** (PA2 / PA3, 921600 8N1) — SF30/D binary protocol.
- **PA4** — LiDAR power-enable. Drives the on-board boost converter that
  steps up to 5 V for the SF30/D, so it is asserted **HIGH** in
  `BSP_Init()` (before `MODER` flips to output, to avoid a 0 V glitch on
  reset) and is never toggled afterwards.
- **PA7** — heartbeat LED, toggled at 2 Hz from the main loop. A solid /
  unlit PA7 means the firmware has hung or wedged in an error loop.

## Toolchain

Required:

```bash
sudo apt install gcc-arm-none-eabi cmake ninja-build openocd
```

Tested with `arm-none-eabi-gcc 10.3.1` and `cmake ≥ 3.22`.

---

## Build

```bash
# One-time: configure
cmake --preset debug

# Build (Debug)
cmake --build build/debug

# Build (Release, optional)
cmake --preset release
cmake --build build/release
```

Artifacts land in `build/<preset>/`:

- `stm32g431_nucleo.elf` — full ELF for debugging
- `stm32g431_nucleo.hex` — Intel HEX for flashing
- `stm32g431_nucleo.bin` — raw binary for flashing
- `stm32g431_nucleo.map` — link map

A typical Debug build is ~12 KB flash and ~4 KB RAM.

---

## Flash

Using **OpenOCD** with an ST-Link connected over SWD:

```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
        -c "program build/debug/stm32g431_nucleo.hex verify reset exit"
```

---

## What you can do with this program

This firmware is a **live cross-check tool** for the SF30/D. The intended
workflow:

1. Connect the SF30/D to the board, power both, flash the firmware.
2. Open the debug serial; confirm the handshake succeeds and the
   configured rate matches what you set in LightWare Studio.
3. Compare the **`meas=` Hz** reported each second against the **`cfg=`**
   value. They should track within ~1 % (printf to USART1 at 115200 baud
   briefly stalls USART2 RX once per second; the SF30/D's `OVRDIS` mode
   drops the oldest byte during that window — enough to dent the measured
   value by about 1 %, but nowhere near enough to blur the >2× separation
   between the SF30/D's discrete update-rate steps).
4. Aim the sensor at a known target; verify the **`d=` cm** field matches
   reality.

### Which rate matters?

The SF30/D has **two independent rate settings** and they look almost
identical in the spec. It's important to set the right one:

| Cmd | Name                       | What it controls                                    | Affects CMD 40 stream? |
|----:|----------------------------|-----------------------------------------------------|:----------------------:|
| 76  | **Update rate**            | Internal laser-firing / measurement rate            | **Yes**                |
| 79  | Serial port output rate    | Throttle for the legacy ASCII serial mode (CMD 70 = 2) | No                  |

Per §10.1.6, CMD 40 ("Full speed distance in cm") streams "at the
**measurement update rate**" — i.e. CMD 76. CMD 79 only matters if you've
configured the sensor for the legacy `Distance over serial` output type
(CMD 70 = 2), which this firmware does not use.

So: to change how many readings/sec this firmware sees, edit **CMD 76** in
Studio (Update rate), not CMD 79.

### Cross-checking a rate change

The `cfg` value comes from CMD 76 read at boot, which reflects the
sensor's persisted measurement update rate. To confirm a configurator
change took effect:

1. Set **Update rate** in LightWare Studio (CMD 76) and **save to flash**.
2. Power-cycle the SF30/D (and the board, to re-handshake).
3. Watch the firmware boot banner — `cfg=` should report the new rate.
4. Watch the per-second lines — `meas=` should converge to that rate
   within the first couple of seconds.

CMD 76 / CMD 79 / CMD 78 (USB) all share the same rate-code table
(§10.1.6):

| Code | Samples/sec |
|-----:|------------:|
| 0    | 20010       |
| 1    | 10005       |
| 2    | 5002        |
| 3    | 2501        |
| 4    | 1250        |
| 5    | 625         |
| 6    | 312         |
| 7    | 156         |
| 8    | 78          |
| 9    | 39          |

### Things this program intentionally does **not** do

- It never writes to the sensor's flash. CMD 30 (start stream) is RAM-only
  per the spec, so no Token (CMD 10) + Save Parameters (CMD 12) dance is
  performed. The sensor's persisted settings are untouched.
- It does not reconfigure the baud rate, return mode, filters, or update
  rate. Use the LightWare Studio configurator (or
  `misc/sf30d_record_data.py`) for those changes.
- It does not record data to non-volatile storage. For data logging, use
  `misc/sf30d_record_data.py` from a host PC.

---

## SF30/D protocol notes

Implemented per **SF30/D Product Guide rev 3.3, §10.1** (Binary protocol):

- **Framing** (§10.1.1): `[0xAA] [flags lo] [flags hi] [ID] [data…] [CRC lo] [CRC hi]`,
  with `flags = (payload_len << 6) | write_bit` and payload length 1–1023.
- **Checksum** (§10.1.2): CRC-16-CCITT (LightWare variant, `0x1021`),
  computed over every byte except the CRC itself.
- **Resync** (§10.1.3): on invalid length or bad CRC the parser slides the
  byte stream past the offending start byte to the next `0xAA` and
  re-attempts — implemented in `sf_parser_feed` / `sf_resync_after_error`.
- **Handshake** (§10): the very first command after powerup gets no
  reply, so `lidar_handshake()` sends `Product Name` (CMD 0) twice.
- **Saving** (§10.1.5): for parameters marked `Persists: Yes`, the value
  in the sensor's RAM is changed by a normal write, but it must be
  committed to flash via Token (CMD 10) → Save Parameters (CMD 12). The
  driver provides `sf_build_save_parameters_request()` and
  `sf_decode_token()` for this; this firmware does not invoke them.

---

## Recorder script (`misc/sf30d_record_data.py`)

Standalone Python tool for capturing data from a host PC over a USB-UART
adapter. Same wire protocol as the firmware. Writes two CSVs:

```bash
pip install pyserial
python3 misc/sf30d_record_data.py
# Ctrl-C to stop. Files in ~/lidar_collect/sf30d_{fast,rich}_<timestamp>.csv
```
