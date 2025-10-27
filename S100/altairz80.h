#ifndef _ALTAIRZ80_H
#define _ALTAIRZ80_H

#include "sim_defs.h"
#include "s100_bus.h"

extern int32 find_unit_index(UNIT* uptr);
extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern t_stat set_membase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_membase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern void cpu_raise_interrupt(uint32 irq);


#endif

