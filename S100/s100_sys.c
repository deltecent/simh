/* altair_sys.c: MITS Altair system interface

   Copyright (c) 1997-2005, Charles E. Owen

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
*/

#include <ctype.h>
#include "s100_sys.h"
#include "s100_defs.h"
#include "s100_z80.h"
#include "s100_ram.h"

extern REG z80_reg[];

/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
   sim_load             binary loader
*/

char sim_name[] = "S100";

REG *sim_PC = NULL;

int32 sim_emax = 4;

DEVICE *sim_devices[] = {
    &bus_dev,
    &fp_dev,
    &simh_dev,
    &z80_dev,
    &ram_dev,
    &bram_dev,
    &rom_dev,
    &dsk_dev,
    &m2sio0_dev,
    &m2sio1_dev,
    &tuart0_dev,
    &tuart1_dev,
    &tuart2_dev,
    &cromfdc_dev,
    &disk1a_dev,
    &icom_dev,
    &dj2d_dev,
    &tarbell_dev,
    &sbc200_dev,
    &vfii_dev,
    &hayes_dev,
    &pmmi_dev,
    &jair_dev,
    &jairs0_dev,
    &jairs1_dev,
    &jairp_dev,
    &vdm1_dev,
    &i8272_dev,
    &wd179x_dev,
    NULL
};

char memoryAccessMessage[256];
char instructionMessage[256];

const char *sim_stop_messages[SCPE_BASE] = {
    "Unknown error",            /* 0 is reserved/unknown */
    "Breakpoint",
    memoryAccessMessage,
    instructionMessage,
    "Invalid Opcode",
    "HALT instruction"
};

static t_stat (*sys_cpu_instr)(void) = NULL;
static t_stat (*sys_cpu_parse_sym)(CONST char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw) = NULL;
static int32  (*sys_cpu_dasm)(char *S, const uint32 *val, const int32 addr) = NULL;

t_stat sim_instr()
{
    t_stat reason = SCPE_NXDEV;

    if (sys_cpu_instr != NULL) {
        reason = (*sys_cpu_instr)();
    }

    return reason;
}

void sys_set_cpu_instr(t_stat (*routine)(void))
{
    sys_cpu_instr = routine;
}

void sys_set_cpu_pc(REG *reg)
{
    sim_PC = reg;
}

void sys_set_cpu_pc_value(t_value (*routine)(void))
{
    sim_vm_pc_value = routine;
}

void sys_set_cpu_parse_sym(t_stat (*routine)(CONST char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw))
{
    sys_cpu_parse_sym = routine;
}

void sys_set_cpu_dasm(int32 (*routine)(char *S, const uint32 *val, const int32 addr))
{
    sys_cpu_dasm = routine;
}

void sys_set_cpu_is_subroutine_call(t_bool (*routine)(t_addr **ret_addrs))
{
}

/*  This is the binary loader. The input file is considered to be a string of
    literal bytes with no special format. The load starts at the current value
    of the PC if no start address is given. If the input string ends with ROM
    (not case sensitive) the memory area is made read only.
    ALTAIRROM/NOALTAIRROM settings are ignored.
*/


/* cpu_hex_load will load an Intel hex file into RAM.
   Based on HEX2BIN by Mike Douglas
   https://deramp.com/downloads/misc_software/hex-binary utilities for the PC/
*/
static t_stat sim_hex_load(FILE *fileref, CONST char *cptr, CONST char *fnam, int flag)
{
    return (SCPE_OK);
}

t_stat sim_load (FILE *fileref, CONST char *cptr, CONST char *fnam, int flag)
{
    return (SCPE_OK);
}

t_stat fprint_sym(FILE *of, t_addr addr, t_value *val, UNIT *uptr, int32 sw)
{
    char disasm_result[128];
    int32 ch = val[0] & 0x7f;
    long r = 1;
    unsigned char vals[SIM_EMAX];
    int32 i;
    if (sw & (SWMASK('A') | SWMASK('C'))) {
        fprintf(of, ((0x20 <= ch) && (ch < 0x7f)) ? "'%c'" : "%02x", ch);
        return SCPE_OK;
    }
    if (!(sw & SWMASK('M'))) {
        return SCPE_ARG;
    }

    if (sys_cpu_dasm != NULL) {
        r = sys_cpu_dasm(disasm_result, val, addr);

        fprintf(of, "%s", disasm_result);
    }

    return 1 - r;
}

/*  Symbolic input

    Inputs:
        *cptr   =   pointer to input string
        addr    =   current PC
        *uptr   =   pointer to unit
        *val    =   pointer to output values
        sw      =   switches
    Outputs:
        status  =   error status
*/
t_stat parse_sym(CONST char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw) {
    while (isspace(*cptr)) {
        cptr++;                 /* absorb spaces            */
    }

    if ((sw & (SWMASK('A') | SWMASK('C'))) || ((*cptr == '\'') && cptr++)) { /* ASCII char? */
        if (cptr[0] == 0) {
            return SCPE_ARG;    /* must have one char       */
        }
        val[0] = (uint32) cptr[0];
        return SCPE_OK;
    }
    if (sys_cpu_parse_sym != NULL) {
        return sys_cpu_parse_sym(cptr, addr, uptr, val, sw);
    }

    return SCPE_OK;
}

char *sys_strupr(const char *str)
{
    static char s[128];
    int i;

    for (i = 0; i < sizeof(s) && str[i] != '\0'; i++) {
        s[i] = str[i];
    }

    s[i] = '\0';

    return s;
}
