#include "sim_defs.h"
#include "s100_sys.h"
#include "s100_bus.h"
#include "s100_ram.h"
#include "s100_rom.h"
#include "s100_roms.h"

static t_stat rom_reset             (DEVICE *dptr);
static int32 rom_memio              (const int32 addr, const int32 rw, const int32 data);

static int32 rom_poc = TRUE; /* Power On Clear */

static const char* rom_description  (DEVICE *dptr);

static uint32 GetBYTE(register uint32 Addr);

static t_stat rom_enadis(int32 value, int32 ena);
static t_stat rom_ena_dbl(UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat rom_dis_dbl(UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat rom_ena_altmon(UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat rom_dis_altmon(UNIT *uptr, int32 value, CONST char *cptr, void *desc);
static t_stat rom_show(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static t_stat rom_show_roms(FILE *st, UNIT *uptr, int32 val, CONST void *desc);

static int32 M[MAXBANKSIZE];

static ROM rom_table[] = {
    { UNIT_ROM_ALTMON,    rom_altmon,    ROM_ALTMON_BASEADDR,    ROM_ALTMON_SIZE,    ROM_ALTMON_NAME,    ROM_ALTMON_DESC },
    { UNIT_ROM_DBL,       rom_mits_dbl,  ROM_MITS_DBL_BASEADDR,  ROM_MITS_DBL_SIZE,  ROM_MITS_DBL_NAME,  ROM_MITS_DBL_DESC },
    { 0,                  NULL,          0x0000,                 0,                  "",                 "" }
};

static const char* rom_description(DEVICE *dptr) {
    return "Read Only Memory";
}

static UNIT rom_unit = {
    UDATA (NULL, UNIT_FIX | UNIT_BINK | UNIT_ROM_DBL, MAXBANKSIZE)
};

static REG rom_reg[] = {
    { FLDATAD (POC,     rom_poc,       0x01,         "Power on Clear flag"), },
    { NULL }
};

static MTAB rom_mod[] = {
    { UNIT_ROM_VERBOSE,     UNIT_ROM_VERBOSE,   "VERBOSE",                 "VERBOSE",           NULL, NULL,
        NULL, "Enable verbose messages"    },
    { UNIT_ROM_VERBOSE,     0,                  "QUIET",                   "QUIET",             NULL, NULL,
        NULL, "Disable verbose messages"   },

    { UNIT_ROM_DBL,         UNIT_ROM_DBL,       ROM_MITS_DBL_NAME,      ROM_MITS_DBL_NAME,      &rom_ena_dbl,  NULL,
        NULL, "Enable "  ROM_MITS_DBL_DESC },
    { UNIT_ROM_DBL,         0,                  "NO" ROM_MITS_DBL_NAME, "NO" ROM_MITS_DBL_NAME, &rom_dis_dbl,  NULL,
        NULL, "Disable " ROM_MITS_DBL_DESC },

    { UNIT_ROM_ALTMON,      UNIT_ROM_ALTMON,    ROM_ALTMON_NAME,        ROM_ALTMON_NAME,        &rom_ena_altmon,  NULL,
        NULL, "Enable "  ROM_ALTMON_DESC   },
    { UNIT_ROM_ALTMON,      0,                  "NO" ROM_ALTMON_NAME,   "NO" ROM_ALTMON_NAME,   &rom_dis_altmon,  NULL,
        NULL, "Disable " ROM_ALTMON_DESC   },

    { MTAB_XTD|MTAB_VDV|MTAB_NMO, 1, "ROMS", NULL,
        NULL, &rom_show_roms, (void *) "Hmmmm", "Show available ROMs" },

    { 0 }
};

/* Debug Flags */
static DEBTAB rom_dt[] = {
    { NULL,         0 }
};

DEVICE rom_dev = {
    "ROM", &rom_unit, rom_reg, rom_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &rom_reset,
    NULL, NULL, NULL,
    NULL, (DEV_DISABLE | DEV_DEBUG), 0,
    rom_dt, NULL, NULL, NULL, NULL, NULL, &rom_description
};

#if 0
static t_stat chip_show(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    fprintf(st, rom_unit.flags & UNIT_RAM_OPSTOP ? "ITRAP, " : "NOITRAP, ");
    if ((chiptype >= 0) && (chiptype < NUM_CHIP_TYPE)) {
        fprintf(st, "%s", cpu_mod[chiptype].mstring);
    }
    fprintf(st, ", ");
    if (ramtype <= MAX_RAM_TYPE)
        fprintf(st, "%s", ramTypeToString[ramtype]);

    return SCPE_OK;
}
#endif

/* memory reset */
static t_stat rom_reset(DEVICE *dptr) {
    if (dptr->flags & DEV_DIS) {    /* Disable Device */
        rom_enadis(rom_unit.flags, FALSE);
    }
    else {
        if (rom_poc) {
            rom_enadis(rom_unit.flags, TRUE);

            rom_poc = FALSE;
        }
    }

    return SCPE_OK;
}

static int32 rom_memio(const int32 addr, const int32 rw, const int32 data)
{
    if (rw == S100_IO_READ) {
        return GetBYTE(addr);
    }

    return 0x0ff;
}

uint32 GetBYTE(register uint32 Addr)
{
//    sim_printf("GetBYTE: %04X\n", Addr);
    return M[Addr & ADDRMASK]; /* ROM */
}

static t_stat rom_enadis(int32 value, int32 ena)
{
    ROM *r = rom_table;
    int i;

    while (r->flag != 0) {
        if (value & r->flag) {
            if (ena) {
                for (i = 0; i < r->size; i++) {
                    M[r->baseaddr + i] = r->rom[i];
                }

                s100_bus_addmem(r->baseaddr, r->size, &rom_memio, r->name);

	        if (rom_unit.flags & UNIT_ROM_VERBOSE) {
                    sim_printf("Installed ROM %s @ %04X\n", r->name, r->baseaddr);
                }
            }
	    else {
                s100_bus_remmem(r->baseaddr, r->size, &rom_memio);

	        if (rom_unit.flags & UNIT_ROM_VERBOSE) {
                    sim_printf("Removed ROM %s @ %04X\n", r->name, r->baseaddr);
                }
            }
        }

        r++;
    }

    return SCPE_OK;
}

static t_stat rom_ena_dbl(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
	return rom_enadis(value, TRUE);
}

static t_stat rom_dis_dbl(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
	return rom_enadis(UNIT_ROM_DBL, FALSE);
}

static t_stat rom_ena_altmon(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
	return rom_enadis(value, TRUE);
}

static t_stat rom_dis_altmon(UNIT *uptr, int32 value, CONST char *cptr, void *desc)
{
	return rom_enadis(UNIT_ROM_ALTMON, FALSE);
}

static t_stat rom_show_roms(FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
    ROM *r = rom_table;

    fprintf(st, "\n");

    while (r->rom != NULL) {
        fprintf(st, "%c %-8.8s: %-25.25s @ %04X-%04X\n", 
            rom_unit.flags & r->flag ? '*' : ' ', r->name, r->desc, r->baseaddr, r->baseaddr + r->size - 1);
	r++;
    }

    fprintf(st, "\n* = enabled\n");

    return SCPE_OK;
}

