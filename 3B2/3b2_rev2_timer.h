/* 3b2_rev2_timer.h: 8253 Interval Timer

   Copyright (c) 2017, Seth J. Morabito

   Permission is hereby granted, free of charge, to any person
   obtaining a copy of this software and associated documentation
   files (the "Software"), to deal in the Software without
   restriction, including without limitation the rights to use, copy,
   modify, merge, publish, distribute, sublicense, and/or sell copies
   of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   Except as contained in this notice, the name of the author shall
   not be used in advertising or otherwise to promote the sale, use or
   other dealings in this Software without prior written authorization
   from the author.
*/

#ifndef _3B2_REV2_TIMER_H_
#define _3B2_REV2_TIMER_H_

#include "sim_defs.h"

#define TIMER_STP_US      1
#define tmrnum            u3
#define tmr               up7

#define TIMER_REG_DIVA    0x03
#define TIMER_REG_DIVB    0x07
#define TIMER_REG_DIVC    0x0b
#define TIMER_REG_CTRL    0x0f
#define TIMER_CLR_LATCH   0x13

#define CLK_RW            0x30
#define CLK_LSB           0x10
#define CLK_MSB           0x20
#define CLK_LMB           0x30

struct timer_ctr {
    uint16 divider;
    uint16 val;
    uint8  mode;
    t_bool lmb;
    t_bool enabled;
    t_bool gate;
    double stime;     /* Most recent start time of counter */
};

t_stat timer_reset(DEVICE *dptr);
uint32 timer_read(uint32 pa, size_t size);
void timer_write(uint32 pa, uint32 val, size_t size);
void timer_tick();
t_stat timer0_svc(UNIT *uptr);
t_stat timer1_svc(UNIT *uptr);
t_stat timer2_svc(UNIT *uptr);
t_stat timer_set_shutdown(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
void timer_disable(uint8 ctrnum);
void timer_enable(uint8 ctrnum);

#endif /* _3B2_REV2_TIMER_H_ */
