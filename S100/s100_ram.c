#include "s100_sys.h"
#include "s100_bus.h"
#include "s100_ram.h"

static t_stat ram_reset             (DEVICE *dptr);
static t_stat ram_dep               (t_value val, t_addr addr, UNIT *uptr, int32 sw);
static t_stat ram_ex                (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw);
static int32 ram_memio              (const int32 addr, const int32 rw, const int32 data);
static t_stat ram_default_ena       (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat ram_default_dis       (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat ram_set_memsize       (int32 value);
static t_stat ram_clear_command     (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat ram_enable_command    (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat ram_randomize_command (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat ram_size_command      (UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static void ram_clear               (void);
static void ram_randomize           (void);
static const char* ram_description  (DEVICE *dptr);

static void PutBYTE(register uint32 Addr, const register uint32 Value);
static void PutWORD(register uint32 Addr, const register uint32 Value);
static uint32 GetBYTE(register uint32 Addr);

static int32 ram_poc = TRUE; /* Power On Clear */

static int32 M[MAXBANKSIZE];                 /* RAM */
static int32 P[MAXBANKSIZE >> LOG2PAGESIZE]; /* Active pages */
static int32 memsize = MAXBANKSIZE;

static const char* ram_description(DEVICE *dptr) {
    return "Random Access Memory";
}

static UNIT ram_unit = {
    UDATA (NULL, UNIT_FIX | UNIT_BINK | UNIT_RAM_DEFAULT, MAXBANKSIZE)
};

static REG ram_reg[] = {
    { FLDATAD (POC,     ram_poc,       0x01,         "Power on Clear flag"), },
    { NULL }
};

static MTAB ram_mod[] = {
    { UNIT_RAM_VERBOSE,     UNIT_RAM_VERBOSE,   "VERBOSE",      "VERBOSE",      NULL, NULL,
        NULL, "Enable verbose messages"  },
    { UNIT_RAM_VERBOSE,     0,                  "QUIET",        "QUIET",        NULL, NULL,
        NULL, "Disable verbose messages"                },
    { UNIT_RAM_DEFAULT,     UNIT_RAM_DEFAULT,   "DEFAULT",      "DEFAULT",      &ram_default_ena, NULL,
        NULL, "Enable RAM as default memory"    },
    { UNIT_RAM_DEFAULT,     0,                  "NODEFAULT",    "NODEFAULT",    &ram_default_dis, NULL,
        NULL, "Disable RAM as default memory"   },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  0,      NULL,         "SIZE={1-64}",    &ram_size_command,
        NULL, NULL, "Sets the RAM size" },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  1,      NULL,      "PENABLE={PAGE | START-END | ALL}",  &ram_enable_command,
        NULL, NULL, "Enable RAM page(s)" },
    { MTAB_XTD | MTAB_VDV | MTAB_VALR,  0,      NULL,     "PDISABLE={PAGE | START-END | ALL}", &ram_enable_command,
        NULL, NULL, "Disable RAM page(s)" },
    { MTAB_VDV,             0,                  NULL,           "CLEAR",        &ram_clear_command,
        NULL, NULL, "Sets RAM to 0x00"  },
    { MTAB_VDV,             0,                  NULL,           "RANDOMIZE",    &ram_randomize_command,
        NULL, NULL, "Sets RAM to random values"  },
    { 0 }
};

/* Debug Flags */
static DEBTAB ram_dt[] = {
    { NULL,         0                                   }
};

DEVICE ram_dev = {
    "RAM",                     /* name */
    &ram_unit,                 /* units */
    ram_reg,                   /* registers */
    ram_mod,                   /* modifiers */
    1,                         /* # units */
    ADDRWIDTH,                 /* address radix */
    ADDRWIDTH,                 /* address width */
    1,                         /* addr increment */
    DATAWIDTH,                 /* data radix */
    DATAWIDTH,                 /* data width */
    &ram_ex,                   /* examine routine */
    &ram_dep,                  /* deposit routine */
    &ram_reset,                /* reset routine */
    NULL,                      /* boot routine */
    NULL,                      /* attach routine */
    NULL,                      /* detach routine */
    NULL,                      /* context */
    (DEV_DISABLE | DEV_DEBUG), /* flags */
    0,                         /* debug control */
    ram_dt,                    /* debug flags */
    NULL,                      /* mem size routine */
    NULL,                      /* logical name */
    NULL,                      /* help */
    NULL,                      /* attach help */
    NULL,                      /* context available to help routines */
    &ram_description           /* device description */
};

/* memory reset */
static t_stat ram_reset(DEVICE *dptr)
{
    if (dptr->flags & DEV_DIS) {    /* Disable Device */
        s100_bus_remmem(0x0000, MAXBANKSIZE, &ram_memio);
        ram_default_dis(NULL, 0, NULL, NULL);
    }
    else {
        if (ram_poc) {
            ram_set_memsize(memsize);

            if (ram_unit.flags & UNIT_RAM_DEFAULT) {
                ram_default_ena(NULL, 0, NULL, NULL);
            }

            ram_poc = FALSE;
        }
    }

    return SCPE_OK;
}

/* memory examine */
static t_stat ram_ex(t_value *vptr, t_addr addr, UNIT *uptr, int32 sw) {
    *vptr = GetBYTE(addr & ADDRMASK) & 0xff;

    return SCPE_OK;
}

/* memory deposit */
static t_stat ram_dep(t_value val, t_addr addr, UNIT *uptr, int32 sw) {
    PutBYTE(addr & ADDRMASK, val & 0xff);
 
    return SCPE_OK;
}

static int32 ram_memio(const int32 addr, const int32 rw, const int32 data)
{
    if (rw == S100_IO_READ) {
        return GetBYTE(addr);
    }

    PutBYTE(addr, data);

    return 0x0ff;
}

static void PutBYTE(register uint32 Addr, const register uint32 Value)
{
//    sim_printf("PutBYTE: %04X %02X\n", Addr, Value);
    M[Addr & ADDRMASK] = Value & 0xff;
}

static void PutWORD(register uint32 Addr, const register uint32 Value)
{
    PutBYTE(Addr, Value);
    PutBYTE(Addr + 1, Value >> 8);
}

static uint32 GetBYTE(register uint32 Addr)
{
//    sim_printf("GetBYTE: %04X\n", Addr);
    return M[Addr & ADDRMASK] & 0xff; /* RAM */
}

static t_stat ram_default_ena(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    s100_bus_setmem_dflt(&ram_memio, "RAM"); /* Set RAM as default memory device */

    return SCPE_OK;
}

static t_stat ram_default_dis(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    s100_bus_remmem_dflt(&ram_memio);       /* Remove RAM as default memory device */

    return SCPE_OK;
}

/* set memory to 'size' kilo byte */
static t_stat ram_set_memsize(int32 size) {
    int32 page;

    size <<= KBLOG2;

    if (size < KB) {
        memsize = KB;
    }
    else if (size > MAXBANKSIZE) {
        memsize = MAXBANKSIZE;
    }
    else {
        memsize = size;
    }

    s100_bus_remmem(0x0000, MAXBANKSIZE, &ram_memio);     /* Remove all pages */
    s100_bus_addmem(0x0000, memsize, &ram_memio, "RAM");  /* Add memsize pages */

    /* Keep track of active pages for SHOW */
    for (page = 0; page < (MAXBANKSIZE >> LOG2PAGESIZE); page++) {
        P[page] = (page << LOG2PAGESIZE) <= memsize;
    }

    ram_unit.capac = memsize;

    return SCPE_OK;
}

static void ram_clear()
{
    uint32 i;

    for (i = 0; i < MAXBANKSIZE; i++) {
        M[i] = 0;
    }
}

static void ram_randomize()
{
    uint32 i;

    for (i = 0; i < MAXBANKSIZE; i++) {
        M[i] = sim_rand() & DATAMASK;
    }
}

static t_stat ram_size_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc) {
    int32 size, result;

    if (cptr == NULL) {
        sim_printf("Memory size must be provided as SET RAM SIZE=1-64\n");
        return SCPE_ARG | SCPE_NOMESSAGE;
    }

    result = sscanf(cptr, "%i", &size);

    if (result == 1) {
        return ram_set_memsize(size); /* Set size in KB */
    }

    return SCPE_ARG | SCPE_NOMESSAGE;
}

static t_stat ram_enable_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc) {
    int32 size;
    t_addr start, end;

    if (cptr == NULL) {
        sim_printf("Memory page(s) must be provided as SET RAM ENABLE=E0-EF\n");
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
        s100_bus_addmem(start, size, &ram_memio, "RAM");  /* Add pages */
    }
    else {
        s100_bus_remmem(start, size, &ram_memio);         /* Remove pages */
    }

    return SCPE_OK;
}

static t_stat ram_clear_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    ram_clear();

    return SCPE_OK;
}

static t_stat ram_randomize_command(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
    ram_randomize();

    return SCPE_OK;
}

