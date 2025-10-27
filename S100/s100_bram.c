#include "s100_sys.h"
#include "s100_bus.h"
#include "s100_bram.h"

static t_stat bram_reset             (DEVICE *dptr);
static t_stat bram_dep               (t_value val, t_addr addr, UNIT *uptr, int32 sw);
static t_stat bram_ex                (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw);
static int32 bram_io                 (const int32 addr, const int32 rw, const int32 data);
static int32 bram_memio              (const int32 addr, const int32 rw, const int32 data);
static t_stat bram_set_banks         (int32 banks);
static t_stat bram_clear_command     (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat bram_enable_command    (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat bram_randomize_command (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat bram_banks_command     (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static void bram_addio               (int32 type);
static void bram_remio               (int32 type);
static t_stat bram_set_type          (int32 type);
static t_stat bram_type_command      (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static void bram_clear               (void);
static void bram_randomize           (void);
static const char* bram_description  (DEVICE *dptr);

static void PutBYTE(register uint32 Addr, const register uint32 Value);
static void PutWORD(register uint32 Addr, const register uint32 Value);
static uint32 GetBYTE(register uint32 Addr);

static int32 bram_poc = TRUE; /* Power On Clear */

static int32 *M = NULL;   /* RAM */
static int32 memsize = MAXBANKSIZE;
static int32 bram_banks = 0;
static int32 bram_bank = 0;
static int32 bram_type = BRAM_TYPE_NONE;

static BRAM B[BRAM_TYPE_MAX + 1] = {
    { 0x00, 0, 0, "NONE" },
    { 0xff, 1, 8, "ERAM" },
    { 0x00, 0, 1, "AZ80" }
};

#define DEV_NAME "BRAM"

static const char* bram_description(DEVICE *dptr) {
    return "Banked Random Access Memory";
}

static UNIT bram_unit = {
    UDATA (NULL, UNIT_FIX | UNIT_BINK, MAXBANKSIZE)
};

static REG bram_reg[] = {
    { FLDATAD (POC,     bram_poc,       0x01,         "Power on Clear flag"), },
    { HRDATAD (BANK,    bram_bank,      MAXBANKS2LOG, "Selected bank"), },
    { DRDATAD (BANKS,   bram_banks,     8,            "Number of banks"), },
    { DRDATAD (TYPE,    bram_type,      8,            "RAM type"), },
    { NULL }
};

static MTAB bram_mod[] = {
    { UNIT_BRAM_VERBOSE,     UNIT_BRAM_VERBOSE,   "VERBOSE",    "VERBOSE",      NULL, NULL,
        NULL, "Enable verbose messages"  },
    { UNIT_BRAM_VERBOSE,     0,                  "QUIET",       "QUIET",        NULL, NULL,
        NULL, "Disable verbose messages"                },
    { MTAB_XTD | MTAB_VDV,  BRAM_TYPE_NONE      , NULL,           "NONE",         &bram_type_command,
        NULL, NULL, "Sets the RAM type to NONE"  },
    { MTAB_XTD | MTAB_VDV,  BRAM_TYPE_AZ80,       NULL,           "AZ80",         &bram_type_command,
        NULL, NULL, "Sets the RAM type to AltairZ80"  },
    { MTAB_XTD | MTAB_VDV,  BRAM_TYPE_ERAM      , NULL,           "ERAM",         &bram_type_command,
        NULL, NULL, "Sets the RAM type to SD Systems ExpandoRAM"  },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  0,      NULL,         "BANKS={1-8}",    &bram_banks_command,
        NULL, NULL, "Sets the RAM size" },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  1,      NULL,      "PENABLE={PAGE | START-END | ALL}",  &bram_enable_command,
        NULL, NULL, "Enable RAM page(s)" },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  0,      NULL,     "PDISABLE={PAGE | START-END | ALL}", &bram_enable_command,
        NULL, NULL, "Disable RAM page(s)" },
    { MTAB_VDV,             0,                  NULL,           "CLEAR",        &bram_clear_command,
        NULL, NULL, "Sets RAM to 0x00"  },
    { MTAB_VDV,             0,                  NULL,           "RANDOMIZE",    &bram_randomize_command,
        NULL, NULL, "Sets RAM to random values"  },
    { 0 }
};

/* Debug Flags */
static DEBTAB bram_dt[] = {
    { NULL,         0                                   }
};

DEVICE bram_dev = {
    DEV_NAME,                   /* name */
    &bram_unit,                 /* units */
    bram_reg,                   /* registers */
    bram_mod,                   /* modifiers */
    1,                         /* # units */
    ADDRWIDTH,                 /* address radix */
    ADDRWIDTH,                 /* address width */
    1,                         /* addr increment */
    DATAWIDTH,                 /* data radix */
    DATAWIDTH,                 /* data width */
    &bram_ex,                   /* examine routine */
    &bram_dep,                  /* deposit routine */
    &bram_reset,                /* reset routine */
    NULL,                      /* boot routine */
    NULL,                      /* attach routine */
    NULL,                      /* detach routine */
    NULL,                      /* context */
    (DEV_DISABLE | DEV_DIS | DEV_DEBUG), /* flags */
    0,                         /* debug control */
    bram_dt,                    /* debug flags */
    NULL,                      /* mem size routine */
    NULL,                      /* logical name */
    NULL,                      /* help */
    NULL,                      /* attach help */
    NULL,                      /* context available to help routines */
    &bram_description           /* device description */
};

/* memory reset */
static t_stat bram_reset(DEVICE *dptr)
{
    if (dptr->flags & DEV_DIS) {    /* Disable Device */
        bram_set_type(BRAM_TYPE_NONE);
    }
    else {
	if (bram_poc) {
            bram_poc = FALSE;
        }
	else {
            bram_bank = 0;
	}
    }

    return SCPE_OK;
}

/* memory examine */
static t_stat bram_ex(t_value *vptr, t_addr addr, UNIT *uptr, int32 sw)
{
    *vptr = GetBYTE(addr & ADDRMASK) & DATAMASK;

    return SCPE_OK;
}

/* memory deposit */
static t_stat bram_dep(t_value val, t_addr addr, UNIT *uptr, int32 sw)
{
    PutBYTE(addr & ADDRMASK, val & DATAMASK);
 
    return SCPE_OK;
}

static int32 bram_io(const int32 addr, const int32 rw, const int32 data)
{
    if (rw == S100_IO_WRITE) {

        switch (bram_type) {
            case BRAM_TYPE_ERAM:
                if (data >= 0 && data < B[bram_type].banks) {
                    bram_bank = data & DATAMASK;
                } else {
                    sim_printf("Invalid bank select 0x%02x for EXPANDORAM\n", data);
                }
                break;

            case BRAM_TYPE_AZ80:
            default:
                break;
        }
    }

    return DATAMASK;
}

static int32 bram_memio(const int32 addr, const int32 rw, const int32 data)
{
    if (rw == S100_IO_READ) {
        return GetBYTE(addr);
    }

    PutBYTE(addr, data);

    return DATAMASK;
}

static uint32 GetBYTE(register uint32 Addr)
{
    t_addr bankAddr;

    if (M != NULL) {
        Addr &= ADDRMASK;

        bankAddr = Addr + (bram_bank * MAXBANKSIZE);

        return M[bankAddr] & DATAMASK;
    }

    return DATAMASK;
}

static void PutBYTE(register uint32 Addr, const register uint32 Value)
{
    t_addr bankAddr;

    if (M != NULL) {
        Addr &= ADDRMASK;

        bankAddr = Addr + (bram_bank * MAXBANKSIZE);

        M[bankAddr] = Value & DATAMASK;
    }
}

static void PutWORD(register uint32 Addr, const register uint32 Value)
{
    PutBYTE(Addr, Value);
    PutBYTE(Addr + 1, Value >> 8);
}

static void bram_addio(int32 type)
{
    if (type > BRAM_TYPE_NONE && type <= BRAM_TYPE_MAX) {
        if (B[type].size) {
            s100_bus_addio(B[type].baseport, B[type].size, &bram_io, B[type].name);
        }
    }
}

static void bram_remio(int32 type)
{
    if (type > BRAM_TYPE_NONE && type <= BRAM_TYPE_MAX) {
        s100_bus_remio(B[type].baseport, B[type].size, &bram_io);
    }
}

static t_stat bram_set_type(int32 type)
{
    if (bram_type == type) {  /* No change */
        return SCPE_OK;
    }

    bram_remio(bram_type);    /* Changing type - remove previous IO */

    bram_type = type;
    bram_bank = 0;

    bram_set_banks(B[bram_type].banks);
    bram_addio(bram_type);

    return SCPE_OK;
}

static t_stat bram_type_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    return bram_set_type(value);
}

/* set memory to 'size' kilo byte */
static t_stat bram_set_banks(int32 banks) {
    if (banks > 0 && banks <= MAXBANK) {
        M = realloc(M, banks * MAXBANKSIZE);
    }
    else if (M != NULL) {
        free(M);

	M = NULL;

        s100_bus_remmem(0x0000, MAXBANKSIZE, &bram_memio);         /* Remove enabled pages */
    }

    bram_banks = banks;

    return SCPE_OK;
}

static t_stat bram_banks_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc) {
    int32 result, banks;

    if (cptr == NULL) {
        sim_printf("Banks must be provided as SET %s BANKS=1-%d\n", DEV_NAME, MAXBANK);
        return SCPE_ARG | SCPE_NOMESSAGE;
    }

    result = sscanf(cptr, "%i", &banks);

    if (result == 1 && banks && banks <= MAXBANK) {
        return bram_set_banks(banks);
    }

    return SCPE_ARG | SCPE_NOMESSAGE;
}

static t_stat bram_enable_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc) {
    int32 size;
    t_addr start, end;

    if (cptr == NULL) {
        sim_printf("Memory page(s) must be provided as SET %s PENABLE=E0-EF\n", DEV_NAME);
        return SCPE_ARG | SCPE_NOMESSAGE;
    }

    if (get_range(NULL, cptr, &start, &end, 16, PAGEMASK, 0) == NULL) {
        return SCPE_ARG;
    }

    if (start < MAXPAGE) {
        start = start << LOG2PAGESIZE;
    }
    if (end < MAXPAGE) {
        end = end << LOG2PAGESIZE;
    }

    start &= 0xff00;
    end &= 0xff00;

    size = end - start + PAGESIZE;

    if (value) {
        s100_bus_addmem(start, size, &bram_memio, DEV_NAME); /* Add pages */
    }
    else {
        s100_bus_remmem(start, size, &bram_memio);         /* Remove pages */
    }

    return SCPE_OK;
}

static t_stat bram_clear_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    bram_clear();

    return SCPE_OK;
}

static t_stat bram_randomize_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    bram_randomize();

    return SCPE_OK;
}

static void bram_clear()
{
    uint32 i;

    for (i = 0; i < MAXBANKSIZE; i++) {
        M[i] = 0;
    }
}

static void bram_randomize()
{
    uint32 i;

    for (i = 0; i < bram_banks * MAXBANKSIZE; i++) {
        if (M != NULL) {
            M[i] = sim_rand() & DATAMASK;
        }
    }
}

