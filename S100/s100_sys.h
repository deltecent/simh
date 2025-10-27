#ifndef _S100_SYS_H
#define _S100_SYS_H

#include "sim_defs.h"

#define SIM_EMAX 6

extern DEVICE bus_dev;
extern DEVICE fp_dev;
extern DEVICE simh_dev;
extern DEVICE z80_dev;
extern DEVICE ram_dev;
extern DEVICE bram_dev;
extern DEVICE dsk_dev;
extern DEVICE rom_dev;
extern DEVICE m2sio0_dev;
extern DEVICE m2sio1_dev;
extern DEVICE sbc200_dev;
extern DEVICE vfii_dev;
extern DEVICE wd179x_dev;

extern char memoryAccessMessage[256];
extern char instructionMessage[256];

extern void sys_set_cpu_instr(t_stat (*routine)(void));
extern void sys_set_cpu_pc(REG *reg);
extern void sys_set_cpu_parse_sym(t_stat (*routine)(CONST char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw));
extern void sys_set_cpu_dasm(int32 (*routine)(char *S, const uint32 *val, const int32 addr));

#endif
