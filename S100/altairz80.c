#include "altairz80_defs.h"

t_stat set_membase(UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
    DEVICE *dptr;
    PNP_INFO *pnp;
    uint32 newba;
    t_stat r;

    if (cptr == NULL)
        return SCPE_ARG;
    if (uptr == NULL)
        return SCPE_IERR;
    dptr = find_dev_from_unit (uptr);
    if (dptr == NULL)
        return SCPE_IERR;
    pnp = (PNP_INFO *) dptr->ctxt;
    if (pnp == NULL)
        return SCPE_IERR;

    newba = get_uint (cptr, 16, 0xFFFF, &r);
    if (r != SCPE_OK)
        return r;

    if ((newba > 0xFFFF) || (newba % pnp->mem_size))
        return SCPE_ARG;

    if (dptr->flags & DEV_DIS) {
        sim_printf("device not enabled yet.\n");
        pnp->mem_base = newba & ~(pnp->mem_size-1);
    } else {
        dptr->flags |= DEV_DIS;
        dptr->reset(dptr);
        pnp->mem_base = newba & ~(pnp->mem_size-1);
        dptr->flags &= ~DEV_DIS;
        dptr->reset(dptr);
    }

    return SCPE_OK;
}

/* Show Base Address routine */
t_stat show_membase(FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
    DEVICE *dptr;
    PNP_INFO *pnp;

    if (uptr == NULL)
        return SCPE_IERR;
    dptr = find_dev_from_unit (uptr);
    if (dptr == NULL)
        return SCPE_IERR;
    pnp = (PNP_INFO *) dptr->ctxt;
    if (pnp == NULL)
        return SCPE_IERR;

    fprintf(st, "MEM=0x%04X-0x%04X", pnp->mem_base, pnp->mem_base+pnp->mem_size-1);
    return SCPE_OK;
}

/* Set Memory Base Address routine */
t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
    DEVICE *dptr;
    PNP_INFO *pnp;
    uint32 newba;
    t_stat r;

    if (cptr == NULL)
        return SCPE_ARG;
    if (uptr == NULL)
        return SCPE_IERR;
    dptr = find_dev_from_unit (uptr);
    if (dptr == NULL)
        return SCPE_IERR;
    pnp = (PNP_INFO *) dptr->ctxt;
    if (pnp == NULL)
        return SCPE_IERR;

    newba = get_uint (cptr, 16, 0xFF, &r);
    if (r != SCPE_OK)
        return r;

    if ((newba > 0xFF) ||
        (newba % pnp->io_size))
        return SCPE_ARG;

    if (dptr->flags & DEV_DIS) {
        sim_printf("device not enabled yet.\n");
        pnp->io_base = newba & ~(pnp->io_size-1);
    } else {
        dptr->flags |= DEV_DIS;
        dptr->reset(dptr);
        pnp->io_base = newba & ~(pnp->io_size-1);
        dptr->flags &= ~DEV_DIS;
        dptr->reset(dptr);
    }

    return SCPE_OK;
}

/* Show I/O Base Address routine */
t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
    DEVICE *dptr;
    PNP_INFO *pnp;

    if (uptr == NULL)
        return SCPE_IERR;
    dptr = find_dev_from_unit (uptr);
    if (dptr == NULL)
        return SCPE_IERR;
    pnp = (PNP_INFO *) dptr->ctxt;
    if (pnp == NULL)
        return SCPE_IERR;

    fprintf(st, "I/O=0x%02X-0x%02X", pnp->io_base, pnp->io_base+pnp->io_size-1);
    return SCPE_OK;
}

/* find_unit_index   find index of a unit

   Inputs:
        uptr    =       pointer to unit
   Outputs:
        result  =       index of device
*/
int32 find_unit_index(UNIT* uptr)
{
    DEVICE *dptr = find_dev_from_unit(uptr);

    if (dptr == NULL)
        return -1;

    return (uptr - dptr->units);
}

/*
** This is for compatibility with AltairZ80 simulator
*/
uint32 s100_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                        int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap)
{
    int i;

    if (resource_type == RESOURCE_TYPE_IO) {
        if (unmap) {
            s100_bus_remio(baseaddr, size, routine);
	}
	else {
            s100_bus_addio(baseaddr, size, routine, name);
        }
    }
    else if (RESOURCE_TYPE_MEMORY) {
        if (unmap) {
            s100_bus_remmem(baseaddr, size, routine);
	}
	else {
            s100_bus_addmem(baseaddr, size, routine, name);
	}
    }
    else {
        sim_printf("%s: cannot map unknown resource type %d\n", __FUNCTION__, resource_type);
        return -1;
    }

    return 0;
}

void cpu_raise_interrupt(uint32 irq) {
    s100_bus_int(irq, 0x00);
}

const char * handlerNameForPort(const int32 port)
{
    IDEV idev;

    s100_bus_get_idev(port, &idev);

    return idev.name;
}

