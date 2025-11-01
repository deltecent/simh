/* altair_bus.c: S-100 Bus

   Copyright (c) 2025, Patrick A. Linstruth

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Charles E. Owen shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Charles E. Owen.


   4. Adding I/O devices.  These modules must be modified:

        altair_cpu.c    add I/O service routines to idev_table
        altair_sys.c    add pointer to data structures in sim_devices
*/


#include <stdio.h>

#include "s100_sys.h"
#include "s100_defs.h"
#include "s100_bus.h"
#include "s100_z80.h"
#include "s100_ram.h"

static t_stat bus_reset               (DEVICE *dptr);
static t_stat bus_dep                 (t_value val, t_addr addr, UNIT *uptr, int32 sw);
static t_stat bus_ex                  (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw);
static t_stat bus_cmd_memory          (int32 flag, const char *cptr);
static t_stat bus_show                (FILE *st, UNIT *uptr, int32 val, CONST void *desc);


static ChipType chiptype = CHIP_TYPE_Z80;

static MDEV mdev_table[MAXPAGE];      /* Active memory table  */
static MDEV mdev_dflt;                /* Default memory table */

static uint32 bus_addr = 0x0000;
uint32 PCX = 0x0000;             /* AltairZ80 Compatibility */

static int32 bus_poc = TRUE; /* Power On Clear */

/* Interrupts */
uint32 nmiInterrupt = 0x00;      /* NMI                     */
uint32 vectorInterrupt = 0x00;   /* Vector Interrupt bits   */
uint8 dataBus[MAX_INT_VECTORS];  /* Data bus value          */

/*  This is the I/O configuration table. There are 255 possible
    device addresses, if a device is plugged to a port it's routine
    address is here, 'nulldev' means no device is available
*/
IDEV idev_table[MAXPAGE];

int32 nulldev(const int32 addr, const int32 io, const int32 data) { return 0xff; }

static const char* bus_description(DEVICE *dptr) {
    return "S100 Bus";
}

static UNIT bus_unit = {
    UDATA (NULL, 0, 0)
};

static REG bus_reg[] = {
    { FLDATAD (POC,     bus_poc,       0x01,         "Power on Clear flag"), },
    { HRDATAD(VECINT,vectorInterrupt,       8, "Vector Interrupt pseudo register"), },
    { BRDATAD (DATABUS, dataBus, 16, 8,     MAX_INT_VECTORS, "Data bus pseudo register"), REG_RO + REG_CIRC },
    { HRDATAD(NMI,       nmiInterrupt,       1, "NMI Interrupt pseudo register"), },
    { NULL }
};

static MTAB bus_mod[] = {
    { UNIT_BUS_VERBOSE,     UNIT_BUS_VERBOSE,   "VERBOSE",      "VERBOSE",      NULL, &bus_show,
        NULL, "Enable verbose messages"     },
    { UNIT_BUS_VERBOSE,     0,                  "QUIET",        "QUIET",        NULL, NULL,
        NULL, "Disable verbose messages"                },
    { 0 }
};

/* Debug flags */
#define IN_MSG          (1 << 0)
#define OUT_MSG         (1 << 1)
#define MEMR_MSG        (1 << 2)
#define MEMW_MSG        (1 << 3)
#define INT_MSG         (1 << 4)

static DEBTAB bus_dt[] = {
    { "IN",     IN_MSG,     "Log IN operations"     },
    { "OUT",    OUT_MSG,    "Log OUT operations"    },
    { "MEMR",   MEMR_MSG,   "Log MEMR operations"   },
    { "MEMW",   MEMW_MSG,   "Log MEMW operations"   },
    { "INT",    INT_MSG,    "Log interrupts"        },
    { NULL,     0                                   }
};

DEVICE bus_dev = {
    "BUS", &bus_unit, bus_reg, bus_mod,
    1, 16, 16, 1, 16, 8,
    &bus_ex, &bus_dep, &bus_reset,
    NULL, NULL, NULL,
    NULL, DEV_DEBUG, 0,
    bus_dt, NULL, NULL, NULL, NULL, NULL, &bus_description
};

/* Simulator-specific commands */
static CTAB bus_cmd_tbl[] = {
    { "REG", &cpu_cmd_reg,    0, "REG              Display registers\n" },
    { "MEM", &bus_cmd_memory, 0, "MEM <address>    Dump a block of memory\n" },
    { NULL, NULL, 0, NULL }
};

/* bus reset */
static t_stat bus_reset(DEVICE *dptr) {
    int i;

    if (bus_poc) {
        sim_vm_cmd = bus_cmd_tbl;

    /* Clear MEM and IO table */
        for (i = 0; i < MAXPAGE; i++) {
            mdev_table[i].routine = &nulldev;
            mdev_table[i].name = "nulldev";

            mdev_dflt.routine = &nulldev;
            mdev_dflt.name = "nulldev";

            idev_table[i].routine = &nulldev;
            idev_table[i].name = "nulldev";
        }

	bus_poc = FALSE;
    }

    return SCPE_OK;
}

/* memory examine */
static t_stat bus_ex(t_value *vptr, t_addr addr, UNIT *uptr, int32 sw)
{
    *vptr = s100_bus_memr(addr & ADDRMASK);

    return SCPE_OK;
}

/* memory deposit */
static t_stat bus_dep(t_value val, t_addr addr, UNIT *uptr, int32 sw)
{
    s100_bus_memw(addr & ADDRMASK, val);
 
    return SCPE_OK;
}

static t_stat bus_show(FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
    const char *last = NULL;
    int i, spage, epage;

    fprintf(st, "VERBOSE\n");

    /* show memory */
    fprintf(st, "\nMEMORY:\n");

    for (i = 0; i < MAXPAGE; i++) {
        if (mdev_table[i].name != last) {
            if (last != NULL) {
                fprintf(st, "%04X-%04X: %s\n", spage << LOG2PAGESIZE, (epage << LOG2PAGESIZE) | 0xff, mdev_table[epage].routine != &nulldev ? sys_strupr(last) : "");
            }

	    last = mdev_table[i].name;
            spage = i;
        }

        epage = i;
    }

    fprintf(st, "%04X-%04X: %s\n", spage << LOG2PAGESIZE, (epage << LOG2PAGESIZE) | 0xff, mdev_table[epage].routine != &nulldev ? sys_strupr(last) : "");

    fprintf(st, "\nDefault Memory Device: %s\n", sys_strupr(mdev_dflt.name));

    /* show which ports are assigned */
    fprintf(st, "\nIO:\n");
    for (i = 0; i < MAXPAGE; i++) {
        if (idev_table[i].routine != &nulldev) {
            fprintf(st, "%02X: %s\n", i, sys_strupr(idev_table[i].name));
        }
    }

    return SCPE_OK;
}

void s100_bus_get_idev(int32 port, IDEV *idev)
{
    idev->routine = idev_table[port & 0xff].routine;
    idev->name = idev_table[port & 0xff].name;
}

t_stat s100_bus_addio(int32 port, int32 size, int32 (*routine)(const int32, const int32, const int32), const char *name)
{
    int i;

    for (i = port; i < port + size; i++) {
        if (bus_unit.flags & UNIT_BUS_VERBOSE) {
            sim_printf("  Mapping  IO %04x, handler=%s\n", i, name);
        }
        idev_table[i & 0xff].routine = routine;
        idev_table[i & 0xff].name = name;
    }

    return SCPE_OK;
}

t_stat s100_bus_remio(int32 port, int32 size, int32 (*routine)(const int32, const int32, const int32))
{
    int i;

    for (i = port; i < port + size; i++) {
        if (idev_table[i & 0xff].routine == routine) {
            if (bus_unit.flags & UNIT_BUS_VERBOSE) {
                sim_printf("Unmapping  IO %04x, handler=%s\n", i, idev_table[i & 0xff].name);
            }
            idev_table[i & 0xff].routine = &nulldev;
            idev_table[i & 0xff].name = "nulldev";
	}
    }

    return SCPE_OK;
}

t_stat s100_bus_addmem(int32 baseaddr, uint32 size, 
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data), const char *name)
{
    int page, i;

    page = baseaddr >> LOG2PAGESIZE;

    for (i = 0; i < (size >> LOG2PAGESIZE); i++) {
            mdev_table[page + i].routine = routine;
            mdev_table[page + i].name = name;
    }

    return SCPE_OK;
}

t_stat s100_bus_setmem_dflt(int32 (*routine)(const int32 addr, const int32 rw, const int32 data), const char *name)
{
    mdev_dflt.routine = routine;
    mdev_dflt.name = name;

    return SCPE_OK;
}

t_stat s100_bus_remmem(int32 baseaddr, uint32 size, 
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data))
{
    int page, i;

    page = baseaddr >> LOG2PAGESIZE;

    for (i = 0; i < (size >> LOG2PAGESIZE); i++) {
        if (mdev_table[page + i].routine == routine) {
            mdev_table[page + i].routine = mdev_dflt.routine;
            mdev_table[page + i].name = mdev_dflt.name;
        }
    }

    return SCPE_OK;
}

t_stat s100_bus_remmem_dflt(int32 (*routine)(const int32 addr, const int32 rw, const int32 data))
{
    if (mdev_dflt.routine == routine) {
        mdev_dflt.routine = &nulldev;
        mdev_dflt.name = "nulldev";
    }

    return SCPE_OK;
}

int32 s100_bus_in(int32 port)
{
    return idev_table[port].routine(port, S100_IO_READ, 0);
}

void s100_bus_out(int32 port, int32 data)
{
    idev_table[port].routine(port, S100_IO_WRITE, data);
}

int32 s100_bus_memr(t_addr addr)
{
    int32 page;

    page = (addr & ADDRMASK) >> LOG2PAGESIZE;

    return mdev_table[page].routine(addr, S100_IO_READ, 0);
}

void s100_bus_memw(t_addr addr, int32 data)
{
    int32 page;

    page = (addr & ADDRMASK) >> LOG2PAGESIZE;

    mdev_table[page].routine(addr, S100_IO_WRITE, data);
}

ChipType s100_bus_set_chiptype(ChipType new)
{
    chiptype = new;

    return chiptype;
}

ChipType s100_bus_get_chiptype(void)
{
    return chiptype;
}

uint32 s100_bus_set_addr(uint32 new)
{
    bus_addr = new;
    PCX = new;

    return bus_addr;
}

uint32 s100_bus_get_addr(void)
{
    return bus_addr;
}

uint32 s100_bus_int(int32 vector, int32 data)
{
    vectorInterrupt |= vector;
    dataBus[vector] = data;

    return vectorInterrupt;
}

uint32 s100_bus_get_int(void)
{
    return vectorInterrupt;
}

uint32 s100_bus_get_int_data(int32 vector)
{
    return dataBus[vector];
}

uint32 s100_bus_clr_int(int32 vector)
{
    vectorInterrupt &= ~(1 << vector);

    return vectorInterrupt;
}

void s100_bus_nmi()
{
    nmiInterrupt = TRUE;
}

int32 s100_bus_get_nmi()
{
    return nmiInterrupt;
}

void s100_bus_clr_nmi()
{
    nmiInterrupt = FALSE;
}

static t_stat bus_cmd_memory(int32 flag, const char *cptr)
{
    char abuf[16];
    t_addr lo, hi, last;
    t_value byte;
    static t_addr disp_addr = 0;

    if (get_range(NULL, cptr, &lo, &hi, 16, ADDRMASK, 0) == NULL) {
        lo = hi = disp_addr;
    }
    else {
        disp_addr = lo & ~(0x0f);
    }

    if (hi == lo) {
        hi = (lo & ~(0x0f)) + 0xff;
    }

    last = hi | 0x00000f;

    while (disp_addr <= last && disp_addr <= ADDRMASK) {

        if (!(disp_addr & 0x0f)) {
            if (ADDRMASK+1 <= 0x10000) {
                sim_printf("%04X ", disp_addr);
            }
            else {
                sim_printf("%02X:%04X ", disp_addr >> 16, disp_addr & 0xffff);
            }
        }

        if (disp_addr < lo || disp_addr > hi) {
            sim_printf("   ");
            abuf[disp_addr & 0x0f] = ' ';
        }
        else {
            byte = s100_bus_memr(disp_addr);
            sim_printf("%02X ", byte);
            abuf[disp_addr & 0x0f] = sim_isprint(byte) ? byte : '.';
        }

        if ((disp_addr & 0x000f) == 0x000f) {
            sim_printf("%16.16s\n", abuf);
        }

        disp_addr++;
    }

    if (disp_addr > ADDRMASK) {
        disp_addr = 0;
    }

    return SCPE_OK | SCPE_NOMESSAGE;
}

