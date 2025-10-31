#include "sim_defs.h"
#include "s100_sys.h"
#include "s100_bus.h"
#include "s100_simh.h"
#include "altairz80_defs.h"

static t_stat simh_dev_reset(DEVICE *dptr);
static t_stat simh_svc(UNIT *uptr);
static int32 simh_io_status(const int32 port, const int32 io, const int32 data);
static int32 simh_io_data(const int32 port, const int32 io, const int32 data);
static int32 simh_io_cmd(const int32 port, const int32 io, const int32 data);
static int32 simh_cmd_in(const int32 port);
static int32 simh_cmd_out(const int32 port, const int32 data);
static void createCPMCommandLine(void);
static void attachCPM(UNIT *uptr, int32 readOnly);
static void detachCPM(UNIT *uptr);

extern uint32 PCX;      /* external view of PC */

static IDEV idev[2] = { {NULL, NULL}, {NULL, NULL} };    /* Save IO devices on 0x12 and 0x13 */

/* Debug flags */
#define IN_MSG              (1 << 0)
#define OUT_MSG             (1 << 1)
#define CMD_MSG             (1 << 2)
#define VERBOSE_MSG         (1 << 3)
#define BUFFER_EMPTY_MSG    (1 << 4)

/* Debug Flags */
static DEBTAB generic_dt[] = {
    { "IN",             IN_MSG,             "IN messages"           },
    { "OUT",            OUT_MSG,            "OUT messages"          },
    { "CMD",            CMD_MSG,            "Commands"              },
    { "VERBOSE",        VERBOSE_MSG,        "Verbose messages"      },
    { "BUFFEREMPTY",    BUFFER_EMPTY_MSG,   "IN for empty buffer"   },
    { NULL,             0                                           }
};

#define PORT_TABLE_SIZE     256                 /* size of port mapping table                   */
#define SLEEP_ALLOWED_START_DEFAULT 100         /* default initial value for sleepAllowedCounter*/
#define DEFAULT_TIMER_DELTA         100         /* default value for timer delta in ms          */
#define CPM_COMMAND_LINE_LENGTH     128
#define CPM_FCB_ADDRESS             0x0080      /* Default FCB address for CP/M.                */

#define SIMH_CAN_READ        0x01               /* bit 0 is set iff character available         */
#define SIMH_CAN_WRITE       0x02               /* bit 1 is set iff character can be sent       */
#define SIMH_RESET           0x03               /* Command to reset SIMH                        */
#define CONTROLZ_CHAR        0x1a               /* control Z character                          */

/* SIMH pseudo device status registers                                                                          */
/* miscellaneous                                                                                                */
static int32 versionPos             = 0;        /* determines state for sending device identifier               */
static int32 lastCPMStatus          = 0;        /* result of last attachCPM command                             */
static int32 lastCommand            = 0;        /* most recent command processed on port 0xfeh                  */

/* Set FCB Address (needed for MS-DOS READ and WRITE commands. */
static int32 FCBAddress = CPM_FCB_ADDRESS;      /* FCB Address                                                  */

static int32 warnLevelSIMH          = 3;        /* display at most 'warnLevelSIMH' times the same warning       */
static int32 warnUnattachedSIMH     = 0;        /* display a warning message if < warnLevel and SIMH set to
                                                VERBOSE and output to SIMH without an attached file             */
static int32 warnSIMHEOF            = 0;        /* display a warning message if < warnLevel and SIMH set to
                                                VERBOSE and attempt to read from SIMH past EOF                  */
/* support for wild card file expansion */

#if defined (_WIN32)
const static char hostPathSeparator     = '\\'; /* back slash in Windows    */
const static char hostPathSeparatorAlt  = '/';  /* '/' is an alternative    */
#else
const static char hostPathSeparator     = '/';  /* slash in UNIX            */
const static char hostPathSeparatorAlt  = '/';  /* no alternative           */
#endif

/*  Synthetic device SIMH for communication
    between Altair and SIMH environment using port 0xfe */
static UNIT simh_unit = {
    UDATA (&simh_svc, UNIT_ATTABLE | UNIT_ROABLE, 0)
};

static REG simh_reg[] = {
    { DRDATAD (VPOS,     versionPos,             8,
               "Status register for sending version information"), REG_RO                           },
    { DRDATAD (LCPMS,    lastCPMStatus,          8,
               "Result of last attachCPM command"), REG_RO                                          },
    { DRDATAD (LCMD,     lastCommand,            8,
               "Last command processed on SIMH port"), REG_RO                                       },
    { HRDATAD (FCBA,     FCBAddress,  16,
               "Address of the FCB for file operations")                                            },
    { NULL }
};

static MTAB simh_mod[] = {
    { 0 }
};

const char* simh_description(DEVICE *dptr) {
    return "SIMH Pseudo Device";
}

DEVICE simh_dev = {
    "SIMH", &simh_unit, simh_reg, simh_mod,
    1, 10, 31, 1, 16, 4,
    NULL, NULL, &simh_dev_reset,
    NULL, NULL, NULL,
    NULL, (DEV_DISABLE | DEV_DEBUG), 0,
    generic_dt, NULL, NULL, NULL, NULL, NULL, &simh_description
};

static t_stat simh_svc(UNIT *uptr) {
    return SCPE_OK;
}

static char cpmCommandLine[CPM_COMMAND_LINE_LENGTH];

/*  Z80 or 8080 programs communicate with the SIMH pseudo device via port 0xfe.
        The following principles apply:

    1)  For commands that do not require parameters and do not return results
        ld  a,<cmd>
        out (0feh),a
        Special case is the reset command which needs to be send 128 times to make
        sure that the internal state is properly reset.

    2)  For commands that require parameters and do not return results
        ld  a,<cmd>
        out (0feh),a
        ld  a,<p1>
        out (0feh),a
        ld  a,<p2>
        out (0feh),a
        ...
        Note: The calling program must send all parameter bytes. Otherwise
        the pseudo device is left in an undefined state.

    3)  For commands that do not require parameters and return results
        ld  a,<cmd>
        out (0feh),a
        in  a,(0feh)    ; <A> contains first byte of result
        in  a,(0feh)    ; <A> contains second byte of result
        ...
        Note: The calling program must request all bytes of the result. Otherwise
        the pseudo device is left in an undefined state.

    4)  For commands that do require parameters and return results
        ld  a,<cmd>
        out (0feh),a
        ld  a,<p1>
        out (0feh),a
        ld  a,<p2>
        out (0feh),a
        ...             ; send all parameters
        in  a,(0feh)    ; <A> contains first byte of result
        in  a,(0feh)    ; <A> contains second byte of result
        ...

*/

/* simhPseudoDeviceCommands - do not change command numbers                                                   */
#define resetPTRCmd               3   /*  3 reset the PTR device                                              */
#define attachPTRCmd              4   /*  4 attach the PTR device                                             */
#define detachPTRCmd              5   /*  5 detach the PTR device                                             */
#define getSIMHVersionCmd         6   /*  6 get the current version of the SIMH pseudo device                 */
#define resetSIMHInterfaceCmd     14  /* 14 reset the SIMH pseudo device                                      */
#define attachPTPCmd              16  /* 16 attach PTP to the file with name at beginning of CP/M command line*/
#define detachPTPCmd              17  /* 17 detach PTP                                                        */
#define setZ80CPUCmd              19  /* 19 set the CPU to a Z80                                              */
#define set8080CPUCmd             20  /* 20 set the CPU to an 8080                                            */
#define getHostOSPathSeparatorCmd 28  /* 28 obtain the file path separator of the OS under which SIMH runs    */
#define kSimhPseudoDeviceCommands 35  /* Highest AltairZ80 SIMH command                                       */

static const char *cmdNames[kSimhPseudoDeviceCommands] = {
    "Undefined",
    "Undefined",
    "Undefined",
    "resetPTR",
    "attachPTR",
    "detachPTR",
    "getSIMHVersion",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "resetSIMHInterface",
    "Undefined",
    "attachPTP",
    "detachPTP",
    "Undefined",
    "setZ80CPU",
    "set8080CPU",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "getHostOSPathSeparator",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
};

static char version[] = "SIMH005";

static t_stat simh_dev_reset(DEVICE *dptr) {
    sim_map_resource(0xfe, 1, RESOURCE_TYPE_IO, &simh_io_cmd,    "SIMH", dptr->flags & DEV_DIS); /* Command Port */
    versionPos              = 0;
    lastCommand             = 0;
    lastCPMStatus           = SCPE_OK;
    FCBAddress              = CPM_FCB_ADDRESS;

    if (simh_unit.flags & UNIT_ATT) {             /* not attached                             */
        detachCPM(&simh_unit);
    }

    return SCPE_OK;
}

static void createCPMCommandLine(void) {
    int32 i, len = (s100_bus_memr(FCBAddress) & 0x7f); /* 0x80 contains length of command line, discard first char   */
    for (i = 0; i < len - 1; i++) {
        cpmCommandLine[i] = (char) s100_bus_memr(FCBAddress + 0x02 + i); /* the first char, typically ' ', is discarded      */
    }
    cpmCommandLine[i] = 0; /* make C string */
}

/* The CP/M command line is used as the name of a file and UNIT* uptr is attached to it. */
static void attachCPM(UNIT *uptr, int32 readOnly)
{
    createCPMCommandLine();

    if (readOnly) {
        sim_switches = SWMASK('R') | SWMASK('Q');
    }
    else {
        sim_switches = SWMASK('W') | SWMASK('N') | SWMASK('Q');
    }

    /* 'N' option makes sure that file is properly truncated if it had existed before   */
    sim_quiet = sim_switches & SWMASK('Q');    /* -q means quiet                       */

    lastCPMStatus = attach_unit(uptr, cpmCommandLine);

    if (lastCPMStatus != SCPE_OK) {
        sim_debug(VERBOSE_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                  " Cannot open '%s' (%s).\n", PCX, cpmCommandLine,
                  sim_error_text(lastCPMStatus));
    }

    /* Save any devices attached to IO Port 0x12 and 0x13 */
    s100_bus_get_idev(0x12, &idev[0]);
    s100_bus_get_idev(0x13, &idev[1]);

    s100_bus_addio(0x12, 1, &simh_io_status, "SIMHS"); /* Status Port  */
    s100_bus_addio(0x13, 1, &simh_io_data,   "SIMHD"); /* Data Port    */
}

static void detachCPM(UNIT *uptr)
{
    detach_unit(&simh_unit);

    if (idev[0].routine != NULL) {
        s100_bus_addio(0x12, 1, idev[0].routine, idev[0].name); /* Status Port  */
	idev[0].routine = NULL;
    }
    if (idev[1].routine != NULL) {
        s100_bus_addio(0x13, 1, idev[1].routine, idev[1].name); /* Data Port    */
	idev[1].routine = NULL;
    }
}

static int32 simh_cmd_in(const int32 port)
{
    int32 result = 0;

    switch(lastCommand) {
        case attachPTRCmd:
        case attachPTPCmd:
            result = lastCPMStatus;
            lastCommand = 0;
            break;

        case getSIMHVersionCmd:
            result = version[versionPos++];
            if (result == 0)
                versionPos = lastCommand = 0;
            break;

        case getHostOSPathSeparatorCmd:
            result = hostPathSeparator;
            break;

        default:
            sim_debug(VERBOSE_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                      " Undefined IN from SIMH pseudo device on port %03xh ignored.\n",
                      PCX, port);
            result = lastCommand = 0;
    }

    return result;
}

static int32 simh_cmd_out(const int32 port, const int32 data) {

    switch(lastCommand) {
        default: /* lastCommand not yet set */
            sim_debug(CMD_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                      " CMD(0x%02x) <- %i (0x%02x, '%s')\n",
                      PCX, port, data, data,
                      (0 <= data) && (data < kSimhPseudoDeviceCommands) ?
                      cmdNames[data] : "Unknown command");

            lastCommand = data;

            switch(data) {
                case getSIMHVersionCmd:
                    versionPos = 0;
                    break;

                case resetPTRCmd:   /* reset ptr device */
                    break;

                case attachPTRCmd:  /* attach ptr to the file with name at beginning of CP/M command line */
                    attachCPM(&simh_unit, TRUE);
                    break;

                case detachPTRCmd:  /* detach ptr */
                    detachCPM(&simh_unit);
                    break;

                case attachPTPCmd:  /* attach ptp to the file with name at beginning of CP/M command line */
                    attachCPM(&simh_unit, FALSE);
                    break;

                case detachPTPCmd:  /* detach ptp */
                    detachCPM(&simh_unit);
                    break;

                case resetSIMHInterfaceCmd:
                    lastCommand = 0;
                    FCBAddress = CPM_FCB_ADDRESS;
                    break;

                case setZ80CPUCmd:
                    s100_bus_set_chiptype(CHIP_TYPE_Z80);
                    break;

                case set8080CPUCmd:
                    s100_bus_set_chiptype(CHIP_TYPE_8080);
                    break;

                default:
                    sim_debug(CMD_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                              " Unknown command (%i) to SIMH pseudo device on port %03xh ignored.\n",
                              PCX, data, port);
            }
    }

    return 0xff; /* ignored, since OUT */
}

/* port 0xfc is a device for communication SIMH <--> Altair machine */
static int32 simh_io_status(const int32 port, const int32 io, const int32 data)
{
    int32 ch;

    if (io == S100_IO_READ) {                                          /* IN                                       */
        if ((simh_unit.flags & UNIT_ATT) == 0) {             /* SIMH is not attached                      */
            if ((simh_dev.dctrl & VERBOSE_MSG) && (warnUnattachedSIMH < warnLevelSIMH)) {
                warnUnattachedSIMH++;
/*06*/          sim_debug(VERBOSE_MSG, &simh_dev, "PTR: " ADDRESS_FORMAT
                          " Attempt to test status of unattached SIMH[0x%02x]. 0x02 returned.\n", PCX, port);
            }
            return SIMH_CAN_WRITE;
        }
                                                            /* if EOF then SIMH_CAN_WRITE else
                                                                (SIMH_CAN_WRITE and SIMH_CAN_READ)        */
        return simh_unit.u3 ? SIMH_CAN_WRITE : (SIMH_CAN_READ | SIMH_CAN_WRITE);
    }                                                       /* OUT follows                              */
    if (data == SIMH_RESET) {
        simh_unit.u3 = FALSE;                                /* reset EOF indicator                      */
        sim_debug(CMD_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                  " Command OUT(0x%03x) = 0x%02x\n", PCX, port, data);
    }
    return 0x00;                                            /* ignored since OUT                        */
}

/* port 0xfd is a device for communication SIMH <--> Altair machine */
static int32 simh_io_data(const int32 port, const int32 io, const int32 data)
{
    int32 ch;

    if (io == S100_IO_READ) {                                /* IN                                       */
        if (simh_unit.u3) {                                  /* EOF reached, no more data available      */
            if ((simh_dev.dctrl & VERBOSE_MSG) && (warnSIMHEOF < warnLevelSIMH)) {
                warnSIMHEOF++;
/*07*/          sim_debug(VERBOSE_MSG, &simh_dev, "PTR: " ADDRESS_FORMAT
                          " SIMH[0x%02x] attempted to read past EOF. 0x00 returned.\n", PCX, port);
            }
            return 0x00;
        }
        if ((simh_unit.flags & UNIT_ATT) == 0) {             /* not attached                             */
            if ((simh_dev.dctrl & VERBOSE_MSG) && (warnUnattachedSIMH < warnLevelSIMH)) {
                warnUnattachedSIMH++;
/*08*/          sim_debug(VERBOSE_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                          " Attempt to read from unattached SIMH[0x%02x]. 0x00 returned.\n", PCX, port);
            }
            return 0x00;
        }
        if ((ch = getc(simh_unit.fileref)) == EOF) {         /* end of file?                             */
            simh_unit.u3 = TRUE;                             /* remember EOF reached                     */
            return CONTROLZ_CHAR;                           /* ^Z denotes end of text file in CP/M      */
        }
        return ch & 0xff;
    }                                                       /* OUT follows                              */
    if (simh_unit.flags & UNIT_ATT)                          /* unit must be attached                    */
        putc(data, simh_unit.fileref);
                                                            /* else ignore data                         */
    else if ((simh_dev.dctrl & VERBOSE_MSG) && (warnUnattachedSIMH < warnLevelSIMH)) {
        warnUnattachedSIMH++;
/*09*/  sim_debug(VERBOSE_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                  " Attempt to output '0x%02x' to unattached SIMH[0x%02x] - ignored.\n", PCX, data, port);
    }
    return 0x00;                                            /* ignored since OUT                        */
}

/* port 0xfe is a device for communication SIMH <--> Altair machine */
static int32 simh_io_cmd(const int32 port, const int32 io, const int32 data)
{
    int32 result = 0;

    if (io == S100_IO_READ) {
        result = simh_cmd_in(port);
        sim_debug(IN_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                  " IN(0x%02x) -> %i (0x%02x, '%c')\n", PCX,
                  port, result, result,
                  (32 <= (result & 0xff)) && ((result & 0xff) <= 127) ? (result & 0xff) : '?');

    } else {
        sim_debug(OUT_MSG, &simh_dev, "SIMH: " ADDRESS_FORMAT
                  " OUT(0x%02x) <- %i (0x%02x, '%c')\n", PCX,
                  port, data, data,
                  (32 <= (data & 0xff)) && ((data & 0xff) <= 127) ? (data & 0xff) : '?');
        simh_cmd_out(port, data);
    }

    return result;
}
