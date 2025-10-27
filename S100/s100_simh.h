#ifndef _S100_SIMH_H
#define _S100_SIMH_H

#define UNIT_V_SIMH_VERBOSE  (UNIT_V_UF + 0)     /* verbose mode, i.e. show error messages       */
#define UNIT_SIMH_VERBOSE    (1 << UNIT_V_SIMH_VERBOSE)
#define UNIT_V_SIMH_TIMERON  (UNIT_V_UF + 1)     /* SIMH pseudo device timer generate interrupts */
#define UNIT_SIMH_TIMERON    (1 << UNIT_V_SIMH_TIMERON)

#endif
