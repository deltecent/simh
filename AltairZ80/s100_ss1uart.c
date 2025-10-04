/* s100_ss1uart.c: CompuPro SS1 UART

   Copyright (c) 2024, Patrick Linstruth

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

#include "altairz80_defs.h"
#include "sim_tmxr.h"

#define SS1UART_NAME  "COMPUPRO SS1 UART"
#define SS1UART_SNAME "SS1UART"

#define SS1UART_WAIT       1000           /* Service Wait Interval */

#define SS1UART_IOBASE     0x5c
#define SS1UART_IOSIZE     4

/* Status Register */
#define SS1UART_TBE         0x01           /* Transmit Buffer Empty        */
#define SS1UART_RDA         0x02           /* Receive Data Available       */
#define SS1UART_PE          0x08           /* Parity Error                 */
#define SS1UART_ORE         0x10           /* Overrun                      */
#define SS1UART_FME         0x20           /* Framing Error                */
#define SS1UART_DCD         0x40           /* Data Carrier Detect          */
#define SS1UART_DSR         0x80           /* Data Set Ready               */

/* Command Register */
#define SS1UART_TXENA       0x01           /* Tx Enable                    */
#define SS1UART_DTR         0x02           /* DTR Enable                   */
#define SS1UART_RXENA       0x04           /* Rx Enable                    */
#define SS1UART_RESET       0x10           /* Reset                        */
#define SS1UART_RTS         0x20           /* RTS Enable                   */
#define SS1UART_LLB         0x80           /* Interrupt Enable             */
#define SS1UART_RLB         0xc0           /* Interrupt Enable             */

#define SS1UART_50          0x00
#define SS1UART_75          0x01
#define SS1UART_110         0x02
#define SS1UART_150         0x04
#define SS1UART_300         0x05
#define SS1UART_1200        0x07
#define SS1UART_2400        0x0a
#define SS1UART_4800        0x0c
#define SS1UART_9600        0x0e
#define SS1UART_19200       0x0f
#define SS1UART_7BITS       0x08
#define SS1UART_8BITS       0x0c
#define SS1UART_NPAR        0x00
#define SS1UART_OPAR        0x10
#define SS1UART_EPAR        0x30
#define SS1UART_1STOP       0x40
#define SS1UART_2STOP       0xc0

#define SS1UART_RDAIE       0x10
#define SS1UART_TBEIE       0x20

#define SS1UART_RDAIA       0xe7
#define SS1UART_TBSIA       0xef

/* Debug flags */
#define STATUS_MSG        (1 << 0)
#define IRQ_MSG           (1 << 1)
#define ERROR_MSG         (1 << 2)
#define VERBOSE_MSG       (1 << 3)

/* IO Read/Write */
#define IO_RD            0x00     /* IO Read  */
#define IO_WR            0x01     /* IO Write */

typedef struct {
    PNP_INFO pnp;        /* Must be first     */
    t_bool conn;         /* Connected Status  */
    TMLN *tmln;          /* TMLN pointer      */
    TMXR *tmxr;          /* TMXR pointer      */
    t_bool txena;        /* Tx Enable         */
    t_bool rxena;        /* Rx Enable         */
    int32 baud;          /* Baud rate         */
    uint8 sbits;         /* Stop bits         */
    t_bool dtr;          /* DTR Enable        */
    t_bool rts;          /* RTS Enable        */
    t_bool llb;          /* Local Loopback    */
    t_bool rlb;          /* Remote Loopback   */
    uint8 rxb;           /* Receive Buffer    */
    uint8 txb;           /* Transmit Buffer   */
    t_bool txp;          /* Transmit Pending  */
    uint8 mr1;           /* Mode Register 1   */
    uint8 mr2;           /* Mode Register 2   */
    uint8 mrrc;          /* Mode Register Read Counter  */
    uint8 mrwc;          /* Mode Register Write Counter  */
    uint8 cr;            /* Command Register  */
    uint8 sr;            /* Status Register   */
    t_bool inta;         /* Interrupt Ack Ena */
    uint8 intmask;       /* Int Enable Mask   */
    uint8 intadr;        /* Interrupt Address */
    uint8 intvector;     /* Interrupt Vector  */
} SS1UART_CTX;

extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);


static const char* ss1uart_description(DEVICE *dptr);
static t_stat ss1uart_svc(UNIT *uptr);
static t_stat ss1uart_reset(DEVICE *dptr);
static t_stat ss1uart_attach(UNIT *uptr, CONST char *cptr);
static t_stat ss1uart_detach(UNIT *uptr);
static t_stat ss1uart_set_baud(UNIT *uptr, int32 value, const char *cptr, void *desc);
static t_stat ss1uart_show_baud(FILE *st, UNIT *uptr, int32 value, const void *desc);
static t_stat ss1uart_config_line(UNIT *uptr);
static int32 ss1uart_io(int32 addr, int32 io, int32 data);
static int32 ss1uart_data(DEVICE *dptr, int32 io, int32 data);
static int32 ss1uart_stat(DEVICE *dptr, int32 io, int32 data);
static int32 ss1uart_mode(DEVICE *dptr, int32 io, int32 data);
static int32 ss1uart_command(DEVICE *dptr, int32 io, int32 data);
static int32 ss1uart_intadrmsk(DEVICE *dptr, int32 io, int32 data);
static void ss1uart_int(UNIT *uptr);

extern uint32 vectorInterrupt;          /* Vector Interrupt bits */
extern uint8 dataBus[MAX_INT_VECTORS];  /* Data bus value        */

/* Debug Flags */
static DEBTAB ss1uart_dt[] = {
    { "STATUS",         STATUS_MSG,         "Status messages"  },
    { "IRQ",            IRQ_MSG,            "Interrupt messages"  },
    { "ERROR",          ERROR_MSG,          "Error messages"  },
    { "VERBOSE",        VERBOSE_MSG,        "Verbose messages"  },
    { NULL,             0                   }
};

/* Terminal multiplexer library descriptors */

static TMLN ss1uart_tmln[] = {         /* line descriptors */
    { 0 }
};

static TMXR ss1uart_tmxr = {           /* multiplexer descriptor */
    1,                                /* number of terminal lines */
    0,                                /* listening port (reserved) */
    0,                                /* master socket  (reserved) */
    ss1uart_tmln,                      /* line descriptor array */
    NULL,                             /* line connection order */
    NULL                              /* multiplexer device (derived internally) */
};

#define UNIT_V_SS1UART_CONSOLE  (UNIT_V_UF + 0)     /* Port checks console for input */
#define UNIT_SS1UART_CONSOLE    (1 << UNIT_V_SS1UART_CONSOLE)
#define UNIT_V_SS1UART_DSR      (UNIT_V_UF + 1)     /* Force DSR active low          */
#define UNIT_SS1UART_DSR        (1 << UNIT_V_SS1UART_DSR)
#define UNIT_V_SS1UART_DCD      (UNIT_V_UF + 2)     /* Force DCD active low          */
#define UNIT_SS1UART_DCD        (1 << UNIT_V_SS1UART_DCD)

static MTAB ss1uart_mod[] = {
    { MTAB_XTD|MTAB_VDV|MTAB_VALR,    0,                      "IOBASE",  "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets TU-ART base I/O address"   },
    { UNIT_SS1UART_CONSOLE,   UNIT_SS1UART_CONSOLE, "CONSOLE",   "CONSOLE",   NULL, NULL, NULL,
        "Port checks for console input" },
    { UNIT_SS1UART_CONSOLE,   0,                  "NOCONSOLE", "NOCONSOLE", NULL, NULL, NULL,
        "Port does not check for console input" },
    { UNIT_SS1UART_DSR,       UNIT_SS1UART_DSR,     "DSR",       "DSR",       NULL, NULL, NULL,
        "Force DSR active" },
    { UNIT_SS1UART_DSR,       0,                  "NODSR",     "NODSR",     NULL, NULL, NULL,
        "DSR follows status line (default)" },
    { UNIT_SS1UART_DCD,       UNIT_SS1UART_DCD,     "DCD",       "DCD",       NULL, NULL, NULL,
        "Force DCD active" },
    { UNIT_SS1UART_DCD,       0,                  "NODCD",     "NODCD",     NULL, NULL, NULL,
        "DCD follows status line (default)" },
    { MTAB_XTD|MTAB_VDV|MTAB_VALR,  0,    "BAUD",  "BAUD",  &ss1uart_set_baud, &ss1uart_show_baud,
        NULL, "Set baud rate (default=9600)" },
    { 0 }
};

static SS1UART_CTX ss1uart_ctx = {{0, 0, SS1UART_IOBASE, SS1UART_IOSIZE}, 0, ss1uart_tmln, &ss1uart_tmxr};

static UNIT ss1uart_unit[] = {
        { UDATA (&ss1uart_svc, UNIT_ATTABLE | UNIT_DISABLE | UNIT_SS1UART_CONSOLE, 0), SS1UART_WAIT },
};

static REG ss1uart_reg[] = {
    { HRDATAD (TXP, ss1uart_ctx.txp, 1, "SS1 transmit pending"), },
    { HRDATAD (STB, ss1uart_ctx.sr, 8, "SS1 status register"), },
    { HRDATAD (MR1, ss1uart_ctx.mr1, 8, "SS1 mode register 1"), },
    { HRDATAD (MR2, ss1uart_ctx.mr2, 8, "SS1 mode register 2"), },
    { HRDATAD (CR, ss1uart_ctx.cr, 8, "SS1 command register"), },
    { NULL }
};

DEVICE ss1uart_dev = {
    SS1UART_SNAME,       /* name */
    ss1uart_unit,        /* unit */
    ss1uart_reg,         /* registers */
    ss1uart_mod,          /* modifiers */
    1,                  /* # units */
    10,                 /* address radix */
    31,                 /* address width */
    1,                  /* address increment */
    8,                  /* data radix */
    8,                  /* data width */
    NULL,               /* examine routine */
    NULL,               /* deposit routine */
    &ss1uart_reset,      /* reset routine */
    NULL,               /* boot routine */
    &ss1uart_attach,      /* attach routine */
    &ss1uart_detach,      /* detach routine */
    &ss1uart_ctx,        /* context */
    (DEV_DISABLE | DEV_DIS | DEV_DEBUG | DEV_MUX),  /* flags */
    0,                  /* debug control */
    ss1uart_dt,           /* debug flags */
    NULL,               /* mem size routine */
    NULL,               /* logical name */
    NULL,               /* help */
    NULL,               /* attach help */
    NULL,               /* context for help */
    &ss1uart_description  /* description */
};

static const char* ss1uart_description(DEVICE *dptr)
{
    return SS1UART_NAME;
}

static t_stat ss1uart_reset(DEVICE *dptr)
{
    SS1UART_CTX *xptr;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    /* Connect/Disconnect I/O Ports at base address */
    if (sim_map_resource(xptr->pnp.io_base, xptr->pnp.io_size, RESOURCE_TYPE_IO, &ss1uart_io, dptr->name, dptr->flags & DEV_DIS) != 0) {
        sim_debug(ERROR_MSG, dptr, "error mapping I/O resource at 0x%02x.\n", xptr->pnp.io_base);
        return SCPE_ARG;
    }

    /* Set DEVICE for this UNIT */
    dptr->units[0].dptr = dptr;

    /* Reset registers */
    xptr->sr = SS1UART_TBE | SS1UART_DSR | SS1UART_DCD;
    xptr->mr1 = 0;
    xptr->mr2 = 0;
    xptr->cr = 0;
    xptr->txp = FALSE;
    xptr->txena = FALSE;
    xptr->rxena = FALSE;
    xptr->llb = FALSE;
    xptr->rlb = FALSE;
    xptr->baud = 9600;
    xptr->sbits = 1;
    xptr->mrrc = 0;
    xptr->mrwc = 0;

    ss1uart_config_line(&dptr->units[0]);

    if (!(dptr->flags & DEV_DIS)) {
        sim_activate_abs(&dptr->units[0], dptr->units[0].wait);
    } else {
        sim_cancel(&dptr->units[0]);
    }

    sim_debug(STATUS_MSG, dptr, "reset adapter.\n");

    return SCPE_OK;
}


static t_stat ss1uart_svc(UNIT *uptr)
{
    SS1UART_CTX *xptr;
    int32 c,s,sr;
    t_stat r;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    /* Check for new incoming connection */
    if (uptr->flags & UNIT_ATT) {
        if (tmxr_poll_conn(xptr->tmxr) >= 0) {      /* poll connection */

            xptr->conn = TRUE;          /* set connected   */
            xptr->tmln->rcve = 1;

            sim_debug(STATUS_MSG, uptr->dptr, "new connection.\n");
        }
    }

    /* Update incoming modem status bits */
    if (uptr->flags & UNIT_ATT) {
        tmxr_set_get_modem_bits(xptr->tmln, 0, 0, &s);
        sr = xptr->sr;
        xptr->sr &= ~SS1UART_DSR;
        xptr->sr |= (s & TMXR_MDM_DSR) ? SS1UART_DSR : 0;
        if ((sr ^ xptr->sr) & SS1UART_DSR) {
            sim_debug(STATUS_MSG, uptr->dptr, "DSR state changed to %s.\n", (xptr->sr & SS1UART_DSR) ? "HIGH" : "LOW");
        }

        /* Enable transmitter if DSR is active */
        xptr->tmln->xmte = (xptr->sr & SS1UART_DSR);

        xptr->sr &= ~SS1UART_DCD;
        xptr->sr |= ((s & TMXR_MDM_DCD) || (uptr->flags & UNIT_SS1UART_DCD)) ? SS1UART_DCD : 0;
        if ((sr ^ xptr->sr) & SS1UART_DCD) {
            sim_debug(STATUS_MSG, uptr->dptr, "DCD state changed to %s.\n", (xptr->sr & SS1UART_DCD) ? "HIGH" : "LOW");
        }

        /* Enable receiver if DCD is active */
        xptr->tmln->rcve = (xptr->sr & SS1UART_DCD);
    }

    /* TX data */
    if (xptr->txp && xptr->txena) {
        if (uptr->flags & UNIT_ATT) {
            r = tmxr_putc_ln(xptr->tmln, xptr->txb);
            xptr->txp = FALSE;             /* Reset TX Pending */

            if (r == SCPE_LOST) {
                xptr->conn = FALSE;        /* Connection was lost */
                sim_debug(STATUS_MSG, uptr->dptr, "lost connection.\n");
            }
        } else {
            r = sim_putchar(xptr->txb);
            xptr->txp = FALSE;             /* Reset TX Pending */
            xptr->sr |= SS1UART_TBE;       /* Xmit buffer empty */
        }

        /* If TX buffer now empty, send interrupt */
        if ((!xptr->txp) && (xptr->intmask & SS1UART_TBEIE)) {
            xptr->intadr = SS1UART_TBSIA;
            ss1uart_int(uptr);
        }
    }

    /* Update TBE if not set and no character pending */
    if (!xptr->txp && !(xptr->sr & SS1UART_TBE) && uptr->flags & UNIT_ATT) {
        tmxr_poll_tx(xptr->tmxr);
        xptr->sr |= (tmxr_txdone_ln(xptr->tmln) && xptr->conn) ? SS1UART_TBE : 0;
    }

    /* Check for Data if RX buffer empty */
    if (!(xptr->sr & SS1UART_RDA) && xptr->rxena) {
        if (uptr->flags & UNIT_ATT) {
            tmxr_poll_rx(xptr->tmxr);

            c = tmxr_getc_ln(xptr->tmln);
        } else if (uptr->flags & UNIT_SS1UART_CONSOLE) {
            c = sim_poll_kbd();
        } else {
            c = 0;
        }

        if (c & (TMXR_VALID | SCPE_KFLAG)) {
            xptr->rxb = c & 0xff;
            xptr->sr |= SS1UART_RDA;
            xptr->sr &= ~(SS1UART_FME | SS1UART_ORE);

            if (xptr->intmask & SS1UART_RDAIE) {
                xptr->intadr = SS1UART_RDAIA;
                ss1uart_int(uptr);
            }
        }
    }

    sim_activate_abs(uptr, uptr->wait);

    return SCPE_OK;
}

/* Attach routine */
static t_stat ss1uart_attach(UNIT *uptr, CONST char *cptr)
{
    SS1UART_CTX *xptr;
    t_stat r = SCPE_OK;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    sim_debug(VERBOSE_MSG, uptr->dptr, "attach (%s).\n", cptr);

    if ((r = tmxr_attach(xptr->tmxr, uptr, cptr)) == SCPE_OK) {
        xptr->tmln->rcve = 1;

        r = ss1uart_config_line(uptr);
    }

    return r;
}


/* Detach routine */
static t_stat ss1uart_detach(UNIT *uptr)
{
    SS1UART_CTX *xptr;

    if (uptr->dptr == NULL) {
        return SCPE_IERR;
    }

    sim_debug(VERBOSE_MSG, uptr->dptr, "detach.\n");

    if (uptr->flags & UNIT_ATT) {
        xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

        sim_cancel(uptr);

        return (tmxr_detach(xptr->tmxr, uptr));
    }

    return SCPE_UNATT;
}

static t_stat ss1uart_set_baud(UNIT *uptr, int32 value, const char *cptr, void *desc)
{
    SS1UART_CTX *xptr;
    int32 baud;
    t_stat r = SCPE_ARG;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    if (cptr != NULL) {
        if (sscanf(cptr, "%d", &baud)) {
            switch (baud) {
                case 110:
                case 150:
                case 300:
                case 1200:
                case 2400:
                case 4800:
                case 9600:
                case 19200:
                    xptr->baud = baud;
                    r = ss1uart_config_line(uptr);
                    return r;

                default:
                    sim_printf("invalid baud rate\n");
                    break;
            }
        }
    }

    return r;
}

static t_stat ss1uart_show_baud(FILE *st, UNIT *uptr, int32 value, const void *desc)
{
    SS1UART_CTX *xptr;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    fprintf(st, "%d (wait=%d)", xptr->baud, uptr->wait);

    return SCPE_OK;
}

static t_stat ss1uart_config_line(UNIT *uptr)
{
    SS1UART_CTX *xptr;
    char config[20];
    t_stat r = SCPE_OK;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    if (xptr != NULL) {
        sprintf(config, "%d-8N%d", xptr->baud, xptr->sbits);

        if ((uptr->flags & UNIT_ATT) && (xptr->tmln->serport)) {
            r = tmxr_set_config_line(xptr->tmln, config);
        }

        sim_debug(STATUS_MSG, uptr->dptr, "Port configuration set to '%s'.\n", config);
    }

    return r;
}

static int32 ss1uart_io(int32 addr, int32 io, int32 data)
{
    int32 r;

    sim_debug(VERBOSE_MSG, &ss1uart_dev, "IO port %02X %s %02X\n", addr, (io == IO_RD) ? "IN" : "OUT", data);

    if ((addr & 0x03) == 0x00) {
        r = ss1uart_data(&ss1uart_dev, io, data);
    }
    else if ((addr & 0x03) == 0x01) {
        r = ss1uart_stat(&ss1uart_dev, io, data);
    }
    else if ((addr & 0x03) == 0x02) {
        r = ss1uart_mode(&ss1uart_dev, io, data);
    } else {
        r = ss1uart_command(&ss1uart_dev, io, data);
    }

    return(r);
}

static int32 ss1uart_stat(DEVICE *dptr, int32 io, int32 data)
{
    SS1UART_CTX *xptr;
    int32 r = 0xff;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    if (io == IO_RD) {
        r = xptr->sr;
        sim_debug(VERBOSE_MSG, dptr, "Status Register Read %02X\n", r);
    } else {
        sim_debug(VERBOSE_MSG, dptr, "Status Register Write %02X Ignored\n", data);
    }

    return r;
}

static int32 ss1uart_mode(DEVICE *dptr, int32 io, int32 data)
{
    SS1UART_CTX *xptr;
    int32 r = 0xff;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    if (io == IO_RD) {
        r = (xptr->mrrc) ? xptr->mr1 : xptr->mr2;
	xptr->mrrc = ++xptr->mrrc & 0x01;
    } else {
        if (xptr->mrwc == 0) {
            xptr->mr1 = data;

            sim_debug(VERBOSE_MSG, dptr, "Mode Register 1 Write %02X\n", data);

            xptr->sbits = ((data & SS1UART_2STOP) == SS1UART_2STOP) ? 2 : 1;

            ss1uart_config_line(&dptr->units[0]);
        } else {
            xptr->mr2 = data;

            sim_debug(VERBOSE_MSG, dptr, "Mode Register 2 Write %02X\n", data);

            switch (data & 0x0f) {
                case SS1UART_110:
                    xptr->baud = 110;
                    break;

                case SS1UART_150:
                    xptr->baud = 150;
                    break;

                case SS1UART_300:
                    xptr->baud = 300;
                    break;

                case SS1UART_1200:
                    xptr->baud = 1200;
                    break;

                case SS1UART_2400:
                    xptr->baud = 2400;
                    break;

                case SS1UART_4800:
                    xptr->baud = 4800;
                    break;

                case SS1UART_9600:
                default:
                    xptr->baud = 9600;
                    break;
            }

            ss1uart_config_line(&dptr->units[0]);
        }

	xptr->mrwc = ++xptr->mrwc & 0x01;
    }

    return r;
}

static int32 ss1uart_data(DEVICE *dptr, int32 io, int32 data)
{
    SS1UART_CTX *xptr;
    int32 r = 0xff;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    if (io == IO_RD) {
        r = xptr->rxb;
        xptr->sr &= ~(SS1UART_RDA | SS1UART_FME | SS1UART_ORE);
        sim_debug(VERBOSE_MSG, dptr, "Data read %02X.\n", r);
    } else {
        xptr->txb = data;
        xptr->txp = TRUE;
        xptr->sr &= ~(SS1UART_TBE);
        sim_debug(VERBOSE_MSG, dptr, "Data write %02X.\n", r);
    }

    return r;
}

static int32 ss1uart_command(DEVICE *dptr, int32 io, int32 data)
{
    SS1UART_CTX *xptr;
    int32 r = 0xff;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    if (io == IO_RD) {
        r = xptr->cr;
        sim_debug(VERBOSE_MSG, dptr, "Command Register Read %02X\n", r);
    } else {
        xptr->txena = data & SS1UART_TXENA;
        xptr->rxena = data & SS1UART_RXENA;
        xptr->llb = data & SS1UART_LLB;
        xptr->rlb = data & SS1UART_RLB;
        xptr->dtr = data & SS1UART_DTR;
        xptr->rts = data & SS1UART_RTS;

        if (data & SS1UART_RESET) {
            sim_debug(STATUS_MSG, dptr, "Reset Error Register\n");
        }
        sim_debug(VERBOSE_MSG, dptr, "Command Register Write %02X\n", data);
        ss1uart_config_line(&dptr->units[0]);
    }

    return r;
}

static int32 ss1uart_intadrmsk(DEVICE *dptr, int32 io, int32 data)
{
    SS1UART_CTX *xptr;

    xptr = (SS1UART_CTX *) dptr->ctxt;

    if (io == IO_RD) {
        return xptr->intadr;
    } else {
        xptr->intmask = data;
    }

    return 0xff;
}

static void ss1uart_int(UNIT *uptr)
{
    SS1UART_CTX *xptr;

    xptr = (SS1UART_CTX *) uptr->dptr->ctxt;

    if (!xptr->inta) {
        return;
    }

    vectorInterrupt |= (1 << xptr->intvector);
    dataBus[xptr->intvector] = xptr->intadr;

    sim_debug(IRQ_MSG, uptr->dptr, "Vector=%d Data bus=%02X\n", xptr->intvector, dataBus[xptr->intvector]);
}

