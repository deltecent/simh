# Altair 8800 (GEN2) Simulator

This is a second-generation SIMH simulator for the [MITS Altair 8800](https://en.wikipedia.org/wiki/Altair_8800)
S-100 bus computer.

## Building

```
make altair8800
```

The simulator binary will be written to `BIN/altair8800`.

Video support (required for the Cromemco Dazzler and Processor Technology VDM-1 devices) is
enabled automatically when SDL2 is available on the build system.

## Devices

### CPU and System

| Device   | Description                        |
|----------|------------------------------------|
| `BUS`    | S100 Bus                           |
| `CPU`    | Central Processing Unit            |
| `Z80`    | Intel 8080 / Zilog Z80 CPU         |
| `SIMH`   | SIMH Pseudo Device                 |
| `SSW`    | Front Panel Sense Switches         |
| `PO`     | Front Panel                        |
| `RAM`    | Random Access Memory               |
| `BRAM`   | Banked Random Access Memory        |
| `ROM`    | Read Only Memory                   |

The `CPU` device supports both the **Intel 8080** and **Zilog Z80** instruction sets, selectable at run time.

### Floppy Disk Controllers

| Device   | Description                                           |
|----------|-------------------------------------------------------|
| `DSK`    | MITS 88-DCDD Floppy Disk Controller                   |
| `FDCP`   | FarmTek FDC+ Floppy Disk Controller (1.5 MB)          |
| `ICOM`   | iCOM 3712/3812 Floppy Disk System                     |
| `PMMI`   | PMMI MM-103 Modem / Floppy Disk Controller            |
| `TARBELL`| Tarbell 2022 Double-Density FDC                       |
| `VFII`   | SD Systems VersaFloppy II                             |

### Hard Disk

| Device   | Description                        |
|----------|------------------------------------|
| `MHDSK`  | MITS Hard Disk                     |

### Serial / Communications

| Device    | Description                              |
|-----------|------------------------------------------|
| `M2SIO0`  | MITS 88-2SIO Serial Adapter (port 0)     |
| `M2SIO1`  | MITS 88-2SIO Serial Adapter (port 1)     |
| `ACR`     | MITS 88-ACR                              |
| `SBC200`  | SD Systems SBC-200                       |
| `SIO`     | Generic Serial IO                        |

### Video

| Device  | Description                              |
|---------|------------------------------------------|
| `DAZ`   | Cromemco Dazzler                         |
| `VDM1`  | Processor Technology VDM-1 Display       |

## ROM Images

The following built-in ROM images are available and can be selected with the `ROM` device:

| Name       | Description                                    |
|------------|------------------------------------------------|
| `DBL`      | MITS Disk Boot Loader                          |
| `TURMON`   | MITS TURMON Monitor                            |
| `HDSK`     | MITS Hard Disk Boot ROM                        |
| `ALTMON`   | Altair Monitor                                 |
| `CDBL`     | ME Disk Boot Loader                            |
| `AZ80DBL`  | AltairZ80 Disk Boot Loader                     |

## Notes

- The simulator name reported to SIMH is **"Altair 8800 (GEN2)"** to distinguish it from
  the original `altair` and `altairz80` simulators in the SIMH distribution.
- This simulator was developed by Patrick A. Linstruth.
