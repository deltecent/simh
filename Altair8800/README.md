# MITS Altair 8800 Simulator (GEN2)

This is a SIMH-based simulator for the **MITS Altair 8800**, the iconic S-100
bus microcomputer introduced in January 1975.  The simulator is named
*Altair 8800 (GEN2)* and was written by Patrick A. Linstruth (copyright 2025–2026).

---

## Table of Contents

1. [Overview](#overview)
2. [Building the Simulator](#building-the-simulator)
3. [Running the Simulator](#running-the-simulator)
4. [Devices](#devices)
   - [BUS – S-100 Bus](#bus--s-100-bus)
   - [CPU – Central Processing Unit](#cpu--central-processing-unit)
   - [SSW – Front Panel Sense Switches](#ssw--front-panel-sense-switches)
   - [PO – Front Panel Programmed Output](#po--front-panel-programmed-output)
   - [RAM – Random Access Memory](#ram--random-access-memory)
   - [BRAM – Banked Random Access Memory](#bram--banked-random-access-memory)
   - [ROM – Read Only Memory](#rom--read-only-memory)
   - [SIMH – SIMH Pseudo Device](#simh--simh-pseudo-device)
   - [MDSK – MITS 88-DCDD Floppy Disk Controller](#mdsk--mits-88-dcdd-floppy-disk-controller)
   - [MHDSK – MITS 88-HDSK Hard Disk Controller](#mhdsk--mits-88-hdsk-hard-disk-controller)
   - [M2SIO – MITS 88-2SIO Serial I/O](#m2sio--mits-88-2sio-serial-io)
   - [ACR – MITS 88-ACR Audio Cassette Recorder](#acr--mits-88-acr-audio-cassette-recorder)
   - [SIO – Generic Serial I/O](#sio--generic-serial-io)
   - [DAZ – Cromemco Dazzler](#daz--cromemco-dazzler)
   - [VDM1 – Processor Technology VDM-1](#vdm1--processor-technology-vdm-1)
   - [TARBELL – Tarbell 1011/2022 FDC](#tarbell--tarbell-10112022-fdc)
   - [FDCP – FarmTek FDC+](#fdcp--farmtek-fdc)
   - [ICOM – iCOM FD3712/FD3812](#icom--icom-fd3712fd3812)
   - [VFII – SD Systems VersaFloppy II](#vfii--sd-systems-versafloppy-ii)
   - [SBC200 – SD Systems SBC-200](#sbc200--sd-systems-sbc-200)
   - [PMMI – PMMI MM-103 Modem](#pmmi--pmmi-mm-103-modem)
5. [Booting the Simulator](#booting-the-simulator)
6. [SIMH Commands Quick Reference](#simh-commands-quick-reference)

---

## Overview

The Altair 8800 was designed around the Intel 8080 CPU and used the S-100 bus
to connect peripheral cards.  This simulator faithfully recreates that bus
architecture: each physical card is represented by a discrete SIMH device, and
the memory map is managed by a page-based bus controller exactly as on real
hardware.

**Key facts:**

| Item | Value |
|------|-------|
| CPU options | Intel 8080 / Zilog Z80 |
| Address space | 16-bit (64 KB) |
| Data bus | 8-bit |
| Bus | S-100 (IEEE-696) |

---

## Building the Simulator

The Altair 8800 simulator is built as part of the standard SIMH source tree.
From the top-level SIMH directory run:

```
make Altair8800
```

or, on Windows with Visual Studio, open the provided solution file and build
the `Altair8800` project.  The resulting executable is typically named
`altair8800` (Linux/macOS) or `altair8800.exe` (Windows).

---

## Running the Simulator

Start the simulator from a terminal:

```
./altair8800
```

You are dropped into the SIMH command prompt (`sim>`).  From here you can
examine and set device parameters, attach disk images, load programs, and
start execution.

To exit the simulator type:

```
sim> EXIT
```

---

## Devices

### BUS – S-100 Bus

The `BUS` device models the S-100 backplane.  It manages the memory map
(via a 256-page table), dispatches I/O port reads and writes to the
appropriate device handlers, and provides a 100 Hz clock tick used by
devices that require periodic service.

**Useful commands:**

```
sim> SHOW BUS CONFIG       ; display the current memory map
sim> SHOW BUS CONSOLE      ; show which device is the console
sim> SET BUS VERBOSE       ; enable verbose bus messages
sim> SET BUS QUIET         ; suppress verbose bus messages
```

The bus also provides `HEXLOAD` and `HEXSAVE` commands to load or save Intel
HEX files directly into or out of simulated memory:

```
sim> HEXLOAD myfile.hex          ; load a HEX file at the address embedded in the file
sim> HEXLOAD myfile.hex 0100     ; load with a 0x0100 address bias
sim> HEXSAVE myfile.hex 0000 00ff ; save addresses 0x0000–0x00FF to a HEX file
```

---

### CPU – Central Processing Unit

The `CPU` device selects and drives the simulated processor.  Two CPU types
are supported:

| Type | Description |
|------|-------------|
| `8080` | Intel 8080 (default) |
| `Z80`  | Zilog Z80 |

Switch between CPU types:

```
sim> SET CPU 8080     ; select Intel 8080
sim> SET CPU Z80      ; select Zilog Z80
```

> **Note:** Changing the CPU type at runtime re-initialises the register set
> and instruction decoder.  It is best to select the CPU type before loading
> any software.

---

### SSW – Front Panel Sense Switches

The `SSW` device simulates the eight sense switches on the Altair 8800 front
panel.  Programs can read the switch state via I/O port `0xFF`.

```
sim> SET SSW SSWVAL=0xAB    ; set sense switches to 0xAB
sim> SHOW SSW               ; display current switch value
```

The device is disabled by default.  Enable it with:

```
sim> SET SSW ENABLE
```

---

### PO – Front Panel Programmed Output

The `PO` device simulates the front panel LED display (Programmed Output port,
address `0x08`).  Writes to this port illuminate the front panel LEDs.

```
sim> SET PO ENABLE          ; enable the device (disabled by default)
sim> SET PO VERBOSE         ; print a message each time the LEDs change
sim> SHOW PO                ; display the current LED state
```

---

### RAM – Random Access Memory

The `RAM` device provides up to 64 KB of flat (non-banked) RAM beginning at
address `0x0000`.  By default, RAM is selected as the *default* memory
device, meaning it backs any page of the address space not explicitly claimed
by another device.

**Common options:**

```
sim> SET RAM SIZE=<n>       ; set memory size in bytes (e.g., SET RAM SIZE=65536)
sim> SET RAM CLEAR          ; fill all RAM with zeros
sim> SET RAM RANDOMIZE      ; fill RAM with random data (simulates cold start)
sim> SET RAM PROT           ; write-protect all pages
sim> SET RAM DEFAULT        ; make RAM the default (background) memory device
sim> SET RAM VERBOSE        ; enable verbose messages
```

---

### BRAM – Banked Random Access Memory

The `BRAM` device implements banked memory boards commonly used with Altair
systems.  Several popular bank-switching schemes are supported:

| Type | Description |
|------|-------------|
| `NONE` | No banked RAM (default) |
| `ERAM` | Extended RAM (bank select port `0xFF`) |
| `VRAM` | Vector RAM |
| `CRAM` | CompuPro RAM |
| `HRAM` | HRAM-64 (up to 64 banks) |
| `B810` | B810 Banked RAM (up to 64 banks) |

```
sim> SET BRAM TYPE=ERAM     ; configure as Extended RAM
sim> SET BRAM BANKS=8       ; set number of banks
sim> SET BRAM CLEAR         ; zero all banks
sim> SET BRAM RANDOMIZE     ; fill all banks with random data
```

---

### ROM – Read Only Memory

The `ROM` device provides a set of built-in ROMs that are mapped into the
upper address space.  Multiple ROMs can be active simultaneously; each
occupies a distinct address range.

| ROM Name | Address    | Size    | Description |
|----------|-----------|---------|-------------|
| `DBL`    | `0xFF00`  | 256 B   | MITS Disk Boot Loader 4.1 (default) |
| `HDSK`   | `0xFC00`  | 512 B   | MITS Hard Disk Boot ROM |
| `TURMON` | `0xFD00`  | 256 B   | MITS Turnkey Monitor |
| `ALTMON` | `0xF800`  | 1 KB    | Altmon 1.3 |
| `CDBL`   | `0xFF00`  | 256 B   | Combined Disk Boot Loader |
| `AZ80DBL`| `0xFF00`  | 256 B   | AltairZ80 Disk Boot Loader |

```
sim> SET ROM DBL            ; enable the MITS Disk Boot Loader (default)
sim> SET ROM ALTMON         ; enable Altmon 1.3
sim> SET ROM NODBL          ; disable the MITS Disk Boot Loader
sim> SHOW ROM LIST          ; list all available ROMs and their state
```

---

### SIMH – SIMH Pseudo Device

The `SIMH` device provides a synthetic I/O interface (ports `0xFE`/`0xFF`)
that lets software running inside the simulator communicate with the host
environment.  This is used by CP/M-aware tools to access the host file system,
retrieve the simulator version, and perform other meta-operations.

```
sim> SET SIMH ENABLE        ; enable the pseudo device
sim> SHOW SIMH              ; show device state
```

When attached to a file, the SIMH device can redirect simulated console I/O
to that file:

```
sim> ATTACH SIMH myfile.txt
sim> DETACH SIMH
```

---

### MDSK – MITS 88-DCDD Floppy Disk Controller

The `MDSK` device simulates the **MITS 88-DCDD** 8-inch floppy disk
controller.  Up to 16 drives can be daisy-chained.  Each diskette image
contains 77 tracks of 32 sectors at 137 bytes per sector.

I/O addresses (octal): `010` (select/status), `011` (control/sector), `012`
(read/write data).

**Attaching a disk image:**

```
sim> ATTACH MDSK0 cpm22.dsk    ; attach an image to drive 0
sim> ATTACH MDSK1 disk1.dsk    ; attach an image to drive 1
sim> DETACH MDSK0              ; eject drive 0
```

**Booting from the floppy:**

```
sim> BOOT MDSK0
```

---

### MHDSK – MITS 88-HDSK Hard Disk Controller

The `MHDSK` device simulates the **MITS 88-HDSK** hard disk system, which
comprises a 5 MB removable platter and a 5 MB fixed platter (each double-sided).

| Parameter | Value |
|-----------|-------|
| Cylinders | 406 |
| Heads per platter | 2 |
| Sectors per track | 24 |
| Bytes per sector | 256 |
| Capacity per platter | ~5 MB |

The controller uses four-PIO I/O addresses `0xA0`–`0xA7`.  Up to 8 drives
are supported.

```
sim> ATTACH MHDSK0 hdsk0.img   ; attach a hard disk image
sim> BOOT MHDSK0               ; boot from the hard disk
sim> SET MHDSK0 LOCKED         ; write-protect the image
```

---

### M2SIO – MITS 88-2SIO Serial I/O

The `M2SIO0` and `M2SIO1` devices simulate the two ports of the **MITS
88-2SIO** serial card, built around the Motorola MC6850 ACIA.  Each port
provides a status register and a data register.

By default:
- `M2SIO0` is connected to the SIMH console (keyboard/screen).
- `M2SIO1` is available for attachment to a file or a network socket.

```
sim> SET M2SIO0 CONSOLE        ; route port 0 to the simulator console
sim> ATTACH M2SIO1 <port>      ; connect port 1 to a TCP socket
sim> SHOW M2SIO0               ; display current configuration
```

---

### ACR – MITS 88-ACR Audio Cassette Recorder

The `ACR` device simulates the **MITS 88-ACR** audio cassette interface, which
was used for loading and saving programs on audio cassette tape.  In the
simulator the cassette is represented by a binary file.

```
sim> ATTACH ACR mytape.bin     ; load a tape image
sim> SET ACR REWIND            ; rewind to the beginning of the tape
sim> DETACH ACR                ; eject the tape
sim> SET ACR VERBOSE           ; enable verbose messages
```

---

### SIO – Generic Serial I/O

The `SIO` device is a flexible serial I/O driver that can emulate several
popular UART/ACIA chips and card configurations.

Supported chip types:

| Type | Chip | Description |
|------|------|-------------|
| `2502` | SC/MP 2502 | Used on the MITS 88-SIO board |
| `2651` | SCN2651 | Used on the CompuPro System Support 1 (SS-1) board |
| `6850` | MC6850 ACIA | Generic 6850 |
| `8250` | INS8250 | Generic 8250 UART |
| `8251` | Intel 8251 | Generic 8251 USART |

Pre-configured board types:

| Board | Chip | Default I/O base |
|-------|------|-----------------|
| `SIO` | 2502 | MITS 88-SIO (0x00) |
| `SS1` | 2651 | CompuPro System Support 1 (0x5C) |

```
sim> SET SIO TYPE=6850         ; select 6850 ACIA
sim> SET SIO BOARD=SS1         ; configure as CompuPro SS-1
sim> SHOW SIO                  ; display current configuration
```

---

### DAZ – Cromemco Dazzler

The `DAZ` device simulates the **Cromemco Dazzler** colour graphics board and
the optional D+7A joystick interface (JS-1).  A graphical window is opened on
the host desktop when the device is active.

**Display characteristics:**

| Mode | Resolution | Colours |
|------|-----------|---------|
| Normal B/W | 32×32 | Monochrome |
| Normal colour | 32×32 | 16 colours |
| X4 B/W | 64×64 (4×zoom) | Monochrome |
| X4 colour | 64×64 (4×zoom) | 16 colours |
| 2K | 64×64 | 4 colours |

The Dazzler memory window defaults to 512 bytes (32×32 pixels) or 2 KB
(64×64 pixels) in 2K mode.

```
sim> SET DAZ ENABLE            ; enable the Dazzler
sim> SET DAZ VIDEO=ON          ; open the display window
sim> SET DAZ VIDEO=OFF         ; close the display window
sim> BOOT DAZ                  ; run the Dazzler demo ROM
sim> SHOW DAZ                  ; display current settings
```

---

### VDM1 – Processor Technology VDM-1

The `VDM1` device simulates the **Processor Technology VDM-1** video display
card, which maps a character buffer into S-100 memory and renders it in a
host window.

| Parameter | Value |
|-----------|-------|
| Columns | 64 |
| Rows | 16 |
| Memory base | `0xCC00` |
| Memory size | 1 KB |
| I/O port | `0xFE` |

```
sim> SET VDM1 ENABLE           ; enable the VDM-1
sim> SET VDM1 NORMAL           ; normal (non-reversed) video
sim> SET VDM1 REVERSE          ; reversed video
sim> SET VDM1 BLINK            ; enable cursor blink
sim> SET VDM1 NOBLINK          ; disable cursor blink
```

---

### TARBELL – Tarbell 1011/2022 FDC

The `TARBELL` device simulates the **Tarbell Electronics 1011/2022** floppy
disk controller, built around the Western Digital WD1771/WD1791 FDC chip.
A 32-byte boot PROM is included and is enabled by default.

Supported drive types: 8-inch and 5.25-inch single/double density.

```
sim> ATTACH TARBELL0 cpm.dsk   ; attach an image to drive 0
sim> BOOT TARBELL0             ; boot from the Tarbell PROM
sim> SET TARBELL MODEL=1011    ; select the 1011 (single density) model
sim> SET TARBELL MODEL=2022    ; select the 2022 (double density) model
sim> SET TARBELL PROM=ENABLE   ; enable the on-board boot PROM
sim> SET TARBELL PROM=DISABLE  ; disable the on-board boot PROM
```

---

### FDCP – FarmTek FDC+

The `FDCP` device simulates the **FarmTek FDC+**, a modern 100%-compatible
replacement for the original two-board MITS floppy disk controller.  The
FDC+ adds support for 5.25-inch drives and 1.5 MB high-capacity media.

Supported drive types:

| Type | Description |
|------|-------------|
| `5` | 5.25-inch drive (Altair 8-inch compatible or 1.5 MB) |

```
sim> ATTACH FDCP0 disk.dsk     ; attach a disk image to drive 0
sim> SET FDCP0 TYPE=5          ; configure as a 5.25-inch drive
sim> BOOT FDCP0                ; boot from the FDC+
```

---

### ICOM – iCOM FD3712/FD3812

The `ICOM` device simulates the **iCOM FD3712/FD3812 Flexible Disk System**.

| Model | Density | Format |
|-------|---------|--------|
| FD3712 | Single density | IBM Diskette type 1 |
| FD3812 | Single and double density | IBM Diskette types 1 & 2D |

The density is determined automatically from the size of the attached disk
image file.

I/O ports: `0xC0` (command/data-in) and `0xC1` (data-out).

```
sim> ATTACH ICOM0 disk.img     ; attach a disk image to drive 0
sim> SET ICOM MODEL=FD3812     ; select the double-density model
```

---

### VFII – SD Systems VersaFloppy II

The `VFII` device simulates the **SD Systems VersaFloppy II** floppy
controller, based on the Western Digital WD1771 chip.  Up to 4 drives are
supported.

```
sim> ATTACH VFII0 disk.dsk     ; attach an image to drive 0
sim> BOOT VFII0                ; boot from the VersaFloppy II
```

---

### SBC200 – SD Systems SBC-200

The `SBC200` device simulates the **SD Systems SBC-200** single-board
computer card.  It integrates a serial port (I/O base `0x78`) with an
on-board monitor ROM mapped at `0xE000` and a diagnostic/debug block at
`0xF000`.

```
sim> SET SBC200 ENABLE         ; enable the SBC-200
sim> SHOW SBC200               ; display current state
```

---

### PMMI – PMMI MM-103 Modem

The `PMMI` device simulates the **PMMI Communications MM-103** modem and
communications adapter, using the Motorola MC6860L digital modem chip.  All
modem control signals (switch-hook, dial tone, dialling) are emulated so that
standard Altair modem software operates correctly.

I/O addresses: `0xE0`–`0xE3` (default).

To make use of the modem, attach it to a TCP socket or serial port on the host:

```
sim> ATTACH PMMI <port>        ; attach to a TCP port number
sim> DETACH PMMI               ; disconnect
sim> SHOW PMMI                 ; display modem status
```

---

## Booting the Simulator

### Booting CP/M from the MITS 88-DCDD floppy controller

```
sim> SET ROM DBL               ; enable the MITS Disk Boot Loader
sim> ATTACH MDSK0 cpm22.dsk    ; attach a CP/M disk image
sim> BOOT MDSK0                ; execute the boot loader
```

### Booting CP/M from the Tarbell FDC

```
sim> ATTACH TARBELL0 cpm.dsk   ; attach a CP/M disk image
sim> SET TARBELL PROM=ENABLE   ; enable the Tarbell boot PROM
sim> BOOT TARBELL0
```

### Booting the Altmon monitor

```
sim> SET ROM NODBLL            ; disable the Disk Boot Loader
sim> SET ROM ALTMON            ; enable Altmon 1.3
sim> RUN F800                  ; jump to the Altmon entry point
```

### Booting the MITS Turnkey Monitor

```
sim> SET ROM TURMON            ; enable the Turnkey Monitor
sim> RUN FD00                  ; jump to the Turnkey Monitor entry point
```

---

## SIMH Commands Quick Reference

The following SIMH commands are most commonly used with this simulator.
For full documentation enter `HELP` at the `sim>` prompt.

| Command | Description |
|---------|-------------|
| `SHOW DEVICES` | List all available devices and their status |
| `SHOW <dev>` | Show configuration and status of a device |
| `SET <dev> ENABLE` | Enable a device |
| `SET <dev> DISABLE` | Disable a device |
| `ATTACH <dev><n> <file>` | Attach a file (disk/tape image) to a unit |
| `DETACH <dev><n>` | Detach a file from a unit |
| `BOOT <dev><n>` | Boot from the specified device/unit |
| `RUN <addr>` | Start execution at the given address |
| `GO` | Continue execution |
| `STEP` | Execute a single instruction |
| `EXAMINE <addr>` | Examine memory at address |
| `DEPOSIT <addr> <val>` | Write a value to memory |
| `RESET` | Reset all devices |
| `HEXLOAD <file>` | Load an Intel HEX file into memory |
| `HEXSAVE <file> <start> <end>` | Save a memory range to an Intel HEX file |
| `EXIT` | Exit the simulator |

---

*Altair 8800 Simulator (GEN2) — Copyright © 2025–2026 Patrick A. Linstruth*
