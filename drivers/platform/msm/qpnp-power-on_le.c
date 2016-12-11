/* add by LETV */
int qpnp_pon_spare_reg_masked_read(u8 off, u8 mask)
{
    struct qpnp_pon *pon = sys_reset_dev;
    int rc = 0;
    u8 reg;

    if (!pon)
        return -ENODEV;

    /* only 4 regs available */
    if ((off > 0x8F) || (off < 0x8C))
        return -EINVAL;

    rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
            (pon->base + off), &reg, 1);
    if (rc) {
        dev_err(&pon->spmi->dev,
                "Unable to read addr=%x, rc(%d)\n",
                (pon->base + off), rc);
        return rc;
    }
    reg &= mask;

    return reg;
}

int qpnp_pon_spare_reg_masked_write(u8 off, u8 mask, u8 reg)
{
    struct qpnp_pon *pon = sys_reset_dev;

    if (!pon)
        return -ENODEV;

    /* only 4 regs available */
    if ((off > 0x8F) || (off < 0x8C))
        return -EINVAL;

    return qpnp_pon_masked_write(pon, (pon->base + off), mask, reg);
}

