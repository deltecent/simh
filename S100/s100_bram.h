#ifndef _S100_BRAM_H
#define _S100_BRAM_H

#include "sim_defs.h"

#define UNIT_BRAM_V_VERBOSE     (UNIT_V_UF+0)               /* Enable verbose messagesto */
#define UNIT_BRAM_VERBOSE       (1 << UNIT_BRAM_V_VERBOSE)

/* Supported Memory Boards */

#define BRAM_TYPE_NONE          0    /* No type selected      */
#define BRAM_TYPE_ERAM          1    /* SD Systems ExpandoRAM */
#define BRAM_TYPE_AZ80          2    /* AltairZ80 RAM         */
#define BRAM_TYPE_MAX           2    /* Maximum board number  */

typedef struct {
    int32 baseport;                  /* Base IO address       */
    int32 size;                      /* Number of addresses   */
    int32 banks;                     /* Number of banks       */
    char *name;                      /* Short name            */
} BRAM;

#endif

