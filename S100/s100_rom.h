#ifndef _S100_ROM_H
#define _S100_ROM_H

#include "sim_defs.h"

#define UNIT_ROM_V_VERBOSE      (UNIT_V_UF+0)               /* warn if ROM is written to                    */
#define UNIT_ROM_VERBOSE        (1 << UNIT_ROM_V_VERBOSE)
#define UNIT_ROM_V_DBL          (UNIT_V_UF+1)               /* Enable/Disable Disk Boot Loader              */
#define UNIT_ROM_DBL            (1 << UNIT_ROM_V_DBL    )
#define UNIT_ROM_V_ALTMON       (UNIT_V_UF+2)               /* Enable/Disable Altmon                        */
#define UNIT_ROM_ALTMON         (1 << UNIT_ROM_V_ALTMON )

typedef struct {
	uint32 flag;
	int32 *rom;
	int32 baseaddr;
	int32 size;
	char *name;
	char *desc;
} ROM;

#endif

