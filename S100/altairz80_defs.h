/*  altairz80_defs.h: MITS Altair simulator definitions

    Copyright (c) 2002-2024, Peter Schorn

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
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    PETER SCHORN BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    Except as contained in this notice, the name of Peter Schorn shall not
    be used in advertising or otherwise to promote the sale, use or other dealings
    in this Software without prior written authorization from Peter Schorn.

    Based on work by Charles E Owen (c) 1997
*/

/* AltairZ80 compatibility */

#ifndef ALTAIRZ80_DEFS_H_
#define ALTAIRZ80_DEFS_H_

#include "sim_defs.h"                                       /* simulator definitions                        */
#include "altairz80.h"
#include "s100_sys.h"
#include "s100_bus.h"
#include "mits_dsk.h"

extern uint32 PCX;

typedef struct {
    uint32 mem_base;    /* Memory Base Address */
    uint32 mem_size;    /* Memory Address space requirement */
    uint32 io_base;     /* I/O Base Address */
    uint32 io_size;     /* I/O Address Space requirement */
} PNP_INFO;

#define RESOURCE_TYPE_MEMORY (S100_BUS_MEMR | S100_BUS_MEMW)
#define RESOURCE_TYPE_IO     (S100_BUS_IN | S100_BUS_OUT)

#define sim_map_resource(a,b,c,d,e,f) s100_map_resource(a,b,c,d,e,f)

#endif
