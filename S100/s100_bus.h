#ifndef _SIM_BUS_H
#define _SIM_BUS_H

#include "sim_defs.h"

#define UNIT_BUS_V_VERBOSE      (UNIT_V_UF+0)               /* warn if ROM is written to                    */
#define UNIT_BUS_VERBOSE        (1 << UNIT_BUS_V_VERBOSE)

/* S100 Bus Architecture */

#define ADDRWIDTH           16
#define DATAWIDTH           8

#define MAXADDR             (1 << ADDRWIDTH)
#define MAXDATA             (1 << DATAWIDTH)
#define ADDRMASK            (MAXADDR - 1)
#define DATAMASK            (MAXDATA - 1)

#define LOG2PAGESIZE        8
#define PAGESIZE            (1 << LOG2PAGESIZE)

#define MAXMEMORY           MAXADDR
#define MAXBANKSIZE         MAXADDR
#define MAXPAGE             (MAXADDR >> LOG2PAGESIZE)
#define PAGEMASK            (MAXPAGE - 1)

#define MAXBANK             8
#define MAXBANKS2LOG        4

#define ADDRESS_FORMAT      "[0x%08x]"

#define KB                  1024                        /* kilo byte                                    */
#define KBLOG2              10                          /* log2 of KB                                   */

/* This is the I/O configuration table.  There are 255 possible
device addresses, if a device is plugged to a port it's routine
address is here, 'nulldev' means no device is available
*/

#define S100_IO_READ  0
#define S100_IO_WRITE 1

/* Interrupt Vectors */
#define MAX_INT_VECTORS         32      /* maximum number of interrupt vectors */

extern uint32 nmiInterrupt;             /* NMI                     */
extern uint32 vectorInterrupt;          /* Vector Interrupt bits */
extern uint8 dataBus[MAX_INT_VECTORS];  /* Data bus value        */

/* CPU chip types */
typedef enum {
    CHIP_TYPE_8080 = 0,
    CHIP_TYPE_Z80,
    NUM_CHIP_TYPE,      /* must be last */
} ChipType;


/* data structure for IN/OUT instructions */
typedef struct idev {
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data);
    const char *name;
} IDEV;

typedef struct { /* Structure to describe memory device address space */
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data);
    const char *name; /* name of handler routine */
} MDEV;

extern t_stat s100_bus_addio(int32 port, int32 size, int32 (*routine)(const int32, const int32, const int32), const char* name);
extern t_stat s100_bus_remio(int32 port, int32 size, int32 (*routine)(const int32, const int32, const int32));
extern t_stat s100_bus_addmem(int32 baseaddr, uint32 size, 
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data), const char *name);
extern t_stat s100_bus_remmem(int32 baseaddr, uint32 size, 
    int32 (*routine)(const int32 addr, const int32 rw, const int32 data));
extern t_stat s100_bus_setmem_dflt(int32 (*routine)(const int32 addr, const int32 rw, const int32 data), const char *name);
extern t_stat s100_bus_remmem_dflt(int32 (*routine)(const int32 addr, const int32 rw, const int32 data));

extern void s100_bus_get_idev(int32 port, IDEV *idev);
extern int32 nulldev(const int32 addr, const int32 io, const int32 data);

extern ChipType s100_bus_set_chiptype(ChipType chiptype);
extern ChipType s100_bus_get_chiptype(void);

extern uint32 s100_bus_set_addr(uint32 pc);
extern uint32 s100_bus_get_addr(void);

extern int32 s100_bus_in(int32 port);
extern void s100_bus_out(int32 port, int32 data);
extern int32 s100_bus_memr(t_addr addr);
extern void s100_bus_memw(t_addr addr, int32 data);
extern uint32 s100_bus_int(int32 vector, int32 data);
extern uint32 s100_bus_get_int(void);
extern uint32 s100_bus_get_int_data(int32 vector);
extern uint32 s100_bus_clr_int(int32 vector);
extern void s100_bus_nmi(void);
extern int32 s100_bus_get_nmi(void);
extern void s100_bus_clr_nmi(void);

#define S100_BUS_MEMR    0x01
#define S100_BUS_MEMW    0x02
#define S100_BUS_IN      0x04
#define S100_BUS_OUT     0x08

extern uint32 s100_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                        int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);

#endif
