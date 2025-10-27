#include "sim_defs.h"
#include "s100_sys.h"
#include "s100_bus.h"
#include "s100_fp.h"

static int32 SR = 0;               /* switch register */

static t_stat fp_reset             (DEVICE *dptr);
static int32 fp_io                 (const int32 addr, const int32 rw, const int32 data);
static t_stat fp_show              (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static const char* fp_description  (DEVICE *dptr);

static const char* fp_description(DEVICE *dptr) {
    return "Front Panel";
}

static UNIT fp_unit = {
    UDATA (NULL, 0, 0)
};

static REG fp_reg[] = {
    { HRDATAD (SR, SR, 8, "Front panel switches pseudo register") },
    { NULL }
};

static MTAB fp_mod[] = {
    { UNIT_FP_VERBOSE,     UNIT_FP_VERBOSE,   "VERBOSE",      "VERBOSE",      NULL, &fp_show,
        NULL, "Enable verbose messages"     },
    { UNIT_FP_VERBOSE,     0,                  "QUIET",        "QUIET",        NULL, NULL,
        NULL, "Disable verbose messages"                },
    { 0 }
};

/* Debug Flags */
static DEBTAB fp_dt[] = {
    { NULL,         0                                   }
};

DEVICE fp_dev = {
    "FP", &fp_unit, fp_reg, fp_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &fp_reset,
    NULL, NULL, NULL,
    NULL, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), 0,
    fp_dt, NULL, NULL, NULL, NULL, NULL, &fp_description
};

/* memory reset */
static t_stat fp_reset(DEVICE *dptr) {
    if (dptr->flags & DEV_DIS) {    /* Disable Device */
        s100_bus_remio(0xff, 1, &fp_io);
    }
    else {
        s100_bus_addio(0xff, 1, &fp_io, "FP");
    }

    return SCPE_OK;
}

static int32 fp_io(const int32 addr, const int32 rw, const int32 data)
{
    if (rw == S100_IO_READ) {
        return SR;
    }

    sim_printf("\n[PO %02X]\n", data & DATAMASK);

    return 0x0ff;
}

static t_stat fp_show(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    return SCPE_OK;
}

