/*
 * Intel Snowridge Memory Controller iMC SMBus Driver to DIMMs
 *
 * Ported from:
 * https://lkml.org/lkml/2020/2/23/245
 *
 * Copyright (c) 2013-2016 Andrew Lutomirski <luto@kernel.org>
 * Copyright (c) 2020 Stefan Schaeckeler <sschaeck@cisco.com>, Cisco Systems
 * Copyright (c) 2020 Kuncoro Irawan <kuangik@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/i2c.h>

#define PCI_DEVICE_ID_INTEL_SNOWRIDGE_IMC                (0x3448)
#define PCI_DEVICE_ID_INTEL_SNOWRIDGE_IMC_PCU            (0x3459)

#define PCI_SNOWRIDGE_IMC_SPD_DEV_NO                    (0x0B)
#define PCI_SNOWRIDGE_IMC_SPD_FUNC_NO                   (0x0)

#define PCI_SNOWRIDGE_PCU_BUS_NO                        (0xFF)
#define PCI_SNOWRIDGE_PCU_DEV_NO                        (0x1E)

#define PCU_TSOD_CONTROL_CFG                            (0xE0)

#define PCU_TSOD_WAIT                                   (10000)

#define IMC_SMB_STAT_IS_BUSY_RETRY                      (20)

#define IMC_SMB_CMD_CFG                     (0x80)
#define IMC_SMB_STAT_CFG                    (0x84)
#define IMC_SMB_DATA_CFG                    (0x88)
#define IMC_SMB_TSOD_POLL_RATE_CFG          (0x98)
#define IMC_SMB_TSOD_CONFIG_CFG             (0x9C)

/* SMBSTAT fields */
#define SMBSTAT_RDO                (1U << 2)      /* Read Data Valid */
#define SMBSTAT_WOD                (1U << 3)      /* Write Operation Done */
#define SMBSTAT_SBE                (1U << 1)      /* SMBus Error */
#define SMBSTAT_SMB_BUSY           (1U << 0)      /* SMBus Busy State */

/* SMBCMD fields */
#define SMBCMD_TRIGGER             (1U << 19)      /* CMD Trigger */
#define SMBCMD_WORD_ACCESS         (1U << 17)      /* Word (vs byte) access */
#define SMBCMD_TYPE_READ           (0U << 15)      /* Read */
#define SMBCMD_TYPE_WRITE          (1U << 15)      /* Write */
#define SMBCMD_SA_SHIFT            8
#define SMBCMD_BA_SHIFT            0
#define SMBCMD_BA_MASK             (0xFF)
#define SMBCMD_SA_MASK             (0x700)

/* SMBCNTL fields */
#define SMBCNTL_DTI_MASK           0x7800      /* Slave Address low bits */
#define SMBCNTL_DTI_SHIFT          11              /* Slave Address low bits */
#define SMBCNTL_DIS_WRT            (1U << 28)      /* Disable Write */

/* SMBDATA fields */
#define SMB_DATA_CFG_RD_MASK       (0x0000FFFF)
#define SMB_DATA_CFG_WR_MASK       (0xFFFF0000) 

/* SMBTSOD fields */
#define SMBTSOD_CFG_G0_DIMM_PRESENT_MASK                (0xFF)

/* DIMMs hold jc42 thermal sensors starting at i2c address 0x18 */
#define DIMM_SENSOR_DRV            "jc42"
#define DIMM_SENSOR_BASE_ADR       0x18

struct imc_channelpair {
    struct i2c_adapter adapter;
    bool can_write, cltt;
};

struct imc_pcu {
    struct pci_dev *pci_dev;
    u32 tsod_polling_interval;
    struct mutex mutex; /*see imc_channelpair_claim() */
};

struct imc_priv {
    struct pci_dev *pci_dev;
    struct imc_channelpair channelpair;
    struct imc_pcu pcu;
    bool suspended;
};

static u32 imc_func(struct i2c_adapter *);
static s32 imc_smbus_xfer(struct i2c_adapter *, u16, unsigned short, char, u8,
                          int, union i2c_smbus_data *);
static int imc_resume(struct pci_dev *);
static int imc_suspend(struct pci_dev *, pm_message_t);
static void imc_remove(struct pci_dev *);
static int imc_probe(struct pci_dev *, const struct pci_device_id *);
static void imc_free_channelpair(struct imc_priv *);
static int imc_init_channelpair(struct imc_priv *);
static void imc_instantiate_sensors(struct i2c_adapter *, u8);
static bool imc_wait_for_transaction(struct imc_priv *, u32 *);
static int imc_channelpair_claim(struct imc_priv *);
static void imc_channelpair_release(struct imc_priv *);

static const struct i2c_algorithm imc_smbus_algorithm = {
    .smbus_xfer        = imc_smbus_xfer,
    .functionality    = imc_func,
};

static const struct pci_device_id imc_ids[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_INTEL,
                 PCI_DEVICE_ID_INTEL_SNOWRIDGE_IMC) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, imc_ids);

static struct pci_driver imc_pci_driver = {
    .name          = "imc_smbus",
    .id_table      = imc_ids,
    .probe         = imc_probe,
    .remove        = imc_remove,
    .suspend       = imc_suspend,
    .resume        = imc_resume,
};


static void imc_channelpair_release (struct imc_priv *priv)
{
	if (priv->channelpair.cltt) {
		/* set tosd_control.tsod_polling_interval to previous value */
		pci_write_config_dword(priv->pcu.pci_dev, PCU_TSOD_CONTROL_CFG,
				               priv->pcu.tsod_polling_interval);
	}
	mutex_unlock(&priv->pcu.mutex);
}


static int imc_channelpair_claim (struct imc_priv *priv)
{
    if (priv->suspended) {
        printk("%s: Suspended\n", __func__);
        return -EIO;
    }

    /*
     * i2c controllers need exclusive access to a psu register and wait
     * then for 10ms before starting their transaction.
     *
     * Possible optimization: Once an i2c controller modified the psu
     * register and waits, the other controller does not need to wait for
     * the whole 10ms, but then only this other controller has to clean up
     * the psu register.
     */
    mutex_lock(&priv->pcu.mutex);

    if (priv->channelpair.cltt) {
        pci_write_config_dword(priv->pcu.pci_dev, PCU_TSOD_CONTROL_CFG, 0);
        usleep_range(PCU_TSOD_WAIT, PCU_TSOD_WAIT + 500);
    }
    return 0;
}


static bool imc_wait_for_transaction (struct imc_priv *priv, u32 *stat)
{
    int j;
    static int busywaits = 1;

    /*
     * Distribution of transaction time from 10000 collected samples:
     *
     * 70us: 1, 80us: 12, 90us: 34, 100us: 132, 110us: 424, 120us: 1138,
     * 130us: 5224, 140us: 3035.
     *
     */
    usleep_range(131, 140);

    /* Don't give up, yet */
    for (j = 0; j < IMC_SMB_STAT_IS_BUSY_RETRY; j++) {
        pci_read_config_dword(priv->pci_dev, IMC_SMB_STAT_CFG, stat);
        if (!(*stat & SMBSTAT_SMB_BUSY)) {
            if (j > busywaits) {
                busywaits = j;
                dev_warn(&priv->pci_dev->dev, "Discovered surprisingly long transaction time (%d)\n",
                          busywaits);
            }
            return true;
        }
        udelay(9);
    }
    return false;
}


static s32 imc_smbus_xfer (struct i2c_adapter *adap, u16 addr,
                           unsigned short flags, char read_write, u8 command,
                           int size, union i2c_smbus_data *data)
{
    int ret;
    u32 cmd = 0, stat;
    struct imc_channelpair *cp;
    struct imc_priv *priv = i2c_get_adapdata(adap);

    cp = &priv->channelpair;
    pci_read_config_dword(priv->pci_dev, IMC_SMB_CMD_CFG, &cmd);

    cmd &= ~(SMBCMD_BA_MASK | SMBCMD_SA_MASK);

    /* Encode CMD part of addresses and access size */
    cmd |= ((u32)addr & 0x7) << SMBCMD_SA_SHIFT;
    cmd |= ((u32)command) << SMBCMD_BA_SHIFT;
    if (size == I2C_SMBUS_WORD_DATA) {
        cmd |= SMBCMD_WORD_ACCESS;
    }

    /* Encode read/write and data to write */
    if (read_write == I2C_SMBUS_READ) {
        cmd |= SMBCMD_TYPE_READ;
    } else {
        cmd |= SMBCMD_TYPE_WRITE;
        cmd |= (size == I2C_SMBUS_WORD_DATA ? swab16(data->word) : data->byte);
    }

    ret = imc_channelpair_claim(priv);
    if (ret)
        return ret;

    /* Write DTI (Device Type Identifier)
     * 0011 -> TSOD
     * 1010 -> EEPROM
     * 0110 -> Write-protect operation for an EEPROM
     * 1110 -> 4-channel I2C bus multiplexer
     */
    cmd &= ~(SMBCNTL_DTI_MASK);
    cmd |= (addr >> 3) << SMBCNTL_DTI_SHIFT;

    /* Issue SMBus command */
    cmd |= SMBCMD_TRIGGER;
    pci_write_config_dword(priv->pci_dev, IMC_SMB_CMD_CFG, cmd);

    if (!imc_wait_for_transaction(priv, &stat)) {
        dev_warn(&priv->pci_dev->dev, "smbus transaction did not complete.\n");
        ret = -ETIMEDOUT;
        goto xfer_out_release;
    }

    if (stat & SMBSTAT_SBE) {
        /*
         * While SBE is set hardware TSOD polling is disabled. This is
         * very bad as this bit is RO-V and will only be cleared after
         * a further software initiated transaction finishes
         * successfully.
         */
        dev_err(&priv->pci_dev->dev,
                "smbus error: sbe is set 0x%x\n", stat);
        ret = -ENXIO;
        goto xfer_out_release;
    }

    if (read_write == I2C_SMBUS_READ) {
        if (!(stat & SMBSTAT_RDO)) {
            dev_warn(&priv->pci_dev->dev,
                     "Unexpected read status 0x%08X\n", stat);
            ret = -EIO;
            goto xfer_out_release;
        }
        pci_read_config_dword(priv->pci_dev, IMC_SMB_DATA_CFG, &stat);
        /*
         * The iMC SMBus controller thinks of SMBus words as being
         * big-endian (MSB first). Linux treats them as little-endian,
         * so we need to swap them.
         */
        if (size == I2C_SMBUS_WORD_DATA) {
            data->word = swab16(stat & SMB_DATA_CFG_RD_MASK);
        } else {
            data->byte = stat & 0xFF;
        }
    } else {
        if (!(stat & SMBSTAT_WOD)) {
            dev_warn(&priv->pci_dev->dev,
                     "Unexpected write status 0x%08X\n", stat);
            ret = -EIO;
        }
    }

xfer_out_release:
    imc_channelpair_release(priv);

    return ret;
}

/*
 * The iMC supports five access types. The terminology is rather inconsistent.
 * These are the types:
 *
 * "Write to pointer register SMBus": I2C_SMBUS_WRITE, I2C_SMBUS_BYTE
 *
 * Read byte/word: I2C_SMBUS_READ, I2C_SMBUS_{BYTE|WORD}_DATA
 *
 * Write byte/word: I2C_SMBUS_WRITE, I2C_SMBUS_{BYTE|WORD}_DATA
 */
static u32 imc_func (struct i2c_adapter *adapter)
{
    struct imc_channelpair *cp;
    struct imc_priv *priv = i2c_get_adapdata(adapter);

    cp = &priv->channelpair;

    if (cp->can_write) {
        return (I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA);
    } else {
        return (I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_READ_WORD_DATA);
    }
}

static void imc_instantiate_sensors (struct i2c_adapter *adapter, u8 presence)
{
    struct i2c_board_info info = {};

    strcpy(info.type, DIMM_SENSOR_DRV);
    info.addr = DIMM_SENSOR_BASE_ADR;

    /*
     * Presence is a bit vector. Bits from right to left map into i2c slave
     * addresses starting 0x18.
     */
    while (presence) {
        if (presence & 0x1)
            i2c_new_device(adapter, &info);
        info.addr++;
        presence >>= 1;
    }
}


static int imc_init_channelpair (struct imc_priv *priv)
{
    int err;
    u32 val;
    struct imc_channelpair *cp = &priv->channelpair;

    i2c_set_adapdata(&cp->adapter, priv);
    cp->adapter.owner = THIS_MODULE;
    cp->adapter.algo = &imc_smbus_algorithm;
    cp->adapter.dev.parent = &priv->pci_dev->dev;

    pci_read_config_dword(priv->pci_dev, IMC_SMB_CMD_CFG, &val);
    cp->can_write = !(val & SMBCNTL_DIS_WRT);

    /*
     * A TSOD polling interval of > 0 tells us if CLTT mode is enabled on
     * some channel pair.
     * (CLTT: Closed-Loop Thermal Throttling)
     *
     * Is there a better way to check for CLTT mode? In particular, is
     * there a way to distingush the mode on a channel pair basis?
     */
    cp->cltt = (priv->pcu.tsod_polling_interval  > 0);

    snprintf(cp->adapter.name, sizeof(cp->adapter.name), "Snowridge IMC SPD SMBus");
    err = i2c_add_adapter(&cp->adapter);
    if (err) {
        printk("%s: Failed in adding I2C adapter\n", __func__);
        return err;
    }

    /* For reasons unknown, TSOD_PRES_MASK is only set in CLTT mode. */
    if (cp->cltt) {
        dev_info(&priv->pci_dev->dev,
                 "CLTT is enabled. Thermal sensors will be automatically enabled\n");
    } else {
        dev_info(&priv->pci_dev->dev,
                 "CLTT is disabled. Thermal sensors need to be manually enabled\n");
    }

    pci_read_config_dword(priv->pci_dev, IMC_SMB_TSOD_CONFIG_CFG, &val);

    imc_instantiate_sensors(&cp->adapter, val & SMBTSOD_CFG_G0_DIMM_PRESENT_MASK);

    return 0;
}


static void imc_free_channelpair (struct imc_priv *priv)
{
    struct imc_channelpair *cp = &priv->channelpair;

    i2c_del_adapter(&cp->adapter);
}


static int imc_probe (struct pci_dev *dev, const struct pci_device_id *id)
{
    int err;
    struct imc_priv *priv;

    /* Sanity check. This device is always at 0x0b.0 */
    if (dev->devfn != PCI_DEVFN(PCI_SNOWRIDGE_IMC_SPD_DEV_NO, 
                      PCI_SNOWRIDGE_IMC_SPD_FUNC_NO)) {
        printk("%s: Can't find PCI IMC device\n", __func__);
        return -ENODEV;
    }

    priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        printk("%s: Alloc failed\n", __func__);
        return -ENOMEM;
    }

    priv->pci_dev = dev;
    pci_set_drvdata(dev, priv);

    /*
     * From pcu, we access the CLTT polling interval.
     *
     * The polling interval is set by BIOS. We assume it will not change at
     * runtime and cache the initial value.
     */
    priv->pcu.pci_dev = pci_get_domain_bus_and_slot(0, PCI_SNOWRIDGE_PCU_BUS_NO, 
                                                    PCI_DEVFN(PCI_SNOWRIDGE_PCU_DEV_NO, 1));
    if (!priv->pcu.pci_dev) {
        printk("%s: Can't get PCI device\n", __func__);
        err = -ENODEV;
        goto probe_out_free;
    }

    /* Sanity check if we get the right PCI device (PCU) */
    if (priv->pcu.pci_dev->device != PCI_DEVICE_ID_INTEL_SNOWRIDGE_IMC_PCU) {
        printk("%s: Device is not PCU (%#x)\n", __func__, priv->pcu.pci_dev->device);
        err = -ENODEV;
        goto probe_out_free;
    }

    pci_read_config_dword(priv->pcu.pci_dev, PCU_TSOD_CONTROL_CFG,
                         &priv->pcu.tsod_polling_interval);

    mutex_init(&priv->pcu.mutex);

    err = imc_init_channelpair(priv);

    if (err) {
        goto probe_out_free_channelpair;
    }

    return 0;

probe_out_free_channelpair:
    imc_free_channelpair(priv);

    mutex_destroy(&priv->pcu.mutex);

probe_out_free:
    kfree(priv);
    return err;
}

static void imc_remove (struct pci_dev *dev)
{
    struct imc_priv *priv = pci_get_drvdata(dev);

    imc_free_channelpair(priv);

    /* set tosd_control.tsod_polling_interval to initial value */
    pci_write_config_dword(priv->pcu.pci_dev, PCU_TSOD_CONTROL_CFG,
                           priv->pcu.tsod_polling_interval);

    mutex_destroy(&priv->pcu.mutex);
}

static int imc_suspend (struct pci_dev *dev, pm_message_t mesg)
{
    struct imc_priv *priv = pci_get_drvdata(dev);

    /* BIOS is in charge. We should finish any pending transaction */
    priv->suspended = true;

    return 0;
}

static int imc_resume (struct pci_dev *dev)
{
    struct imc_priv *priv = pci_get_drvdata(dev);

    priv->suspended = false;

    return 0;
}


static int __init i2c_snow_imc_init (void)
{
    return pci_register_driver(&imc_pci_driver);
}
module_init(i2c_snow_imc_init);

static void __exit i2c_snow_imc_exit (void)
{
    pci_unregister_driver(&imc_pci_driver);
}
module_exit(i2c_snow_imc_exit);

MODULE_AUTHOR("Kuncoro Irawan <kuangik@gmail.com>");
MODULE_DESCRIPTION("Snowridge IMC SMBus driver");
MODULE_LICENSE("GPL v2");
