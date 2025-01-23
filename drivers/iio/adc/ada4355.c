// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD4080 SPI ADC driver
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>


/** Register Definition */

#define ADA4355_REG_CHIP_CONFIGURATION      0x00
#define ADA4355_REG_CHIP_ID                 0x01
#define ADA4355_REG_DEVICE_INDEX            0X05
#define ADA4355_REG_TRANFER                 0XFF
#define ADA4355_REG_POWER_MODES             0X08
#define ADA4355_REG_CLOCK                   0X09
#define ADA4355_REG_CLOCK_DIVIDE            0X0B
#define ADA4355_REG_TEST_MODE               0X0D
#define ADA4355_REG_OUTPUT_MODE             0X14
#define ADA4355_REG_OUTPUT_ADJUST           0X15
#define ADA4355_REG_OUTPUT_PHASE            0X16
#define ADA4355_REG_USER_PATT1_LSB          0X19
#define ADA4355_REG_USER_PATT1_MSB          0X1A
#define ADA4355_REG_USER_PATT2_LSB          0X1B
#define ADA4355_REG_USER_PATT2_MSB          0X1C
#define ADA4355_REG_SERIAL_OUT_DATA_CNTRL   0X21
#define ADA4355_REG_SERIAL_CHANNEL_STATUS   0X22
#define ADA4355_REG_RESOLUTION_SAMPLE_RATE  0X100
#define ADA4355_REG_USER_IN_OUT_CNTRL       0X101

//#define ADA4355_CHIP_ID                     0x8B
#define AXI_ADA4355_SELF_SYNC_BIT	    	BIT(1) //???

/*ADA4355_REG_POWER_MODES 0x08*/
#define ADA4355_DIGITAL_RESET               GENMASK(1, 0)
#define ADA4355_NORMAL_OPERATION            0xFC

/*CHIP ID*/
#define ADA4355_CHIP_ID                     0x8B

/*REG TRANSFER 0xFF*/
#define ADA4355_OVERRIDE                    BIT(0)

/*SAMPLE RATE OVERRIDE*/
#define ADA4355_125_RATE                    0x06

static const int ada4355_scale_table[][2] = {
	    {6000, 0},
};

struct ada4355_state {
        struct spi_device           *spi;
        struct regmap               *regmap;
        struct clk                  *clk;
        /* Protect against concurrent accesses to the device and data content */
        struct mutex	            lock;
        unsigned int	            num_lanes;
       // enum ad4080_filter_sel		filter_mode;
       // bool                        filter_enable;
};

static const struct regmap_config ada4355_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	//.max_register = 0x29,
};

static struct ada4355_state *ada4355_get_data(struct iio_dev *indio_dev)
{
         struct axiadc_converter *conv;
         conv = iio_device_get_drvdata(indio_dev);

         return conv->phy;
};

static int ada4355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
                              unsigned int writeval, unsigned int *readval)
{
         struct ada4355_state *st = ada4355_get_data(indio_dev);

         if(readval)
                return regmap_read(st->regmap, reg, readval); //regmap_read() - Read a value from a single register

         return regmap_write(st->regmap, reg, writeval);
}

static int ada4355_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned int tmp;

	tmp = (conv->chip_info->scale_table[0][0] * 1000000ULL) >>
		    conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ada4355_read_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int *val, int *val2, long m)
{
    struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
    struct ada4355_state *st = ada4355_get_data(indio_dev);

    switch (m) {
    case IIO_CHAN_INFO_SCALE:
            return ada4355_get_scale(conv, val, val2);
    case IIO_CHAN_INFO_SAMP_FREQ:
            *val = clk_get_rate(st->clk);
            return IIO_VAL_INT;
    default:
            return -EINVAL;
    }
}

static int ada4355_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct ada4355_state *st = ada4355_get_data(indio_dev);
	//unsigned long s_clk;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return -EINVAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -EINVAL;
	default:
		return -EINVAL;
	}

	return 0;
}

//static ssize_t ada4355_lvds_sync_read(struct iio_dev *indio_dev,
//				     uintptr_t private,
//				     const struct iio_chan_spec *chan,
//				     char *buf)
//{
//	return sprintf(buf, "enable\n");
//}
//
//static ssize_t ada4355_lvds_sync_write(struct iio_dev *indio_dev,
//				      uintptr_t private,
//				      const struct iio_chan_spec *chan,
//				      const char *buf, size_t len)
//{
//	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
//	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
//	struct ada4355_state *st = ada4355_get_data(indio_dev);
//	unsigned int reg_cntrl, reg_cntrl_3;
//	unsigned int timeout = 10000;
//	int ret;
//
//	mutex_lock(&st->lock);
//	/*if (st->num_lanes == 1)
//		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//				   AD4080_RESERVED_CONFIG_A_MSK |
//				   AD4080_INTF_CHK_EN_MSK);
//	else 
//		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//				   AD4080_RESERVED_CONFIG_A_MSK |
//				   AD4080_INTF_CHK_EN_MSK |
//				   AD4080_SPI_LVDS_LANES_MSK);
//	if (ret)
//		return ret; */
//
//	// set bit 2 of ADI_REG_CNTRL_3 to let the HDL know that the CNV is not used
//	reg_cntrl_3 = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
//	reg_cntrl_3 |= AXI_ADA4355_SELF_SYNC_BIT;
//	axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl_3);
//
//	// enable the sync
//	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
//	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
//	reg_cntrl |= ADI_SYNC;
//	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);
//
//// last part
//	do {
//		if (axiadc_read(axi_adc_st, ADI_REG_SYNC_STATUS) == 0)
//			dev_info(&st->spi->dev, "Not Locked: Running Bit Slip\n");
//		else
//			break;
//	} while (--timeout);
//
//// Do we need that ???? Set reg 15 from ad4080, in ada4355 not present a such reg
//	/*if (timeout) {
//		dev_info(&st->spi->dev, "Success: Pattern correct and Locked!\n");
//		if (st->num_lanes == 1)
//			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//					   AD4080_RESERVED_CONFIG_A_MSK);
//		else
//			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//					   AD4080_RESERVED_CONFIG_A_MSK |
//					   AD4080_SPI_LVDS_LANES_MSK);
//	} else {
//		dev_info(&conv->spi->dev, "LVDS Sync Timeout.\n");
//		if (st->num_lanes == 1)
//			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//					   AD4080_RESERVED_CONFIG_A_MSK);
//		else
//			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
//					   AD4080_RESERVED_CONFIG_A_MSK |
//					   AD4080_SPI_LVDS_LANES_MSK);
//		ret = -ETIME;
//	}
//*/
//	mutex_unlock(&st->lock);
//
//	return ret ? ret : len;
//}

/*#define ADA4355_CHAN(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),		\
	  .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	 /* .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	}
*/
#define ADA4355_CHAN(_chan, _si, _bits, _sign, _shift)		\
    { .type = IIO_VOLTAGE,						\
      .indexed = 1,							\
      .channel = _chan,						\
      .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),		\
      .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
      .scan_index = _si,						\
      .scan_type = {						\
    		.sign = _sign,					\
    		.realbits = _bits,				\
    		.storagebits = 16,				\
    		.shift = _shift,				\
      },								\
    }

static const struct axiadc_chip_info ada4355_chip_info = {
	.name = "ADA4355",
	.id = ADA4355_CHIP_ID,
	.max_rate = 125000000UL,
	.scale_table = ada4355_scale_table, //???
	.num_scales = ARRAY_SIZE(ada4355_scale_table),//???
	.num_channels = 2,
	.channel[0] = ADA4355_CHAN(0, 0, 16, 'S', 0),
    .channel[1] = ADA4355_CHAN(1, 0, 16, 'S', 0),
};

static void ada4355_clk_disable(void *data)
{
	struct axiadc_converter *st = data;

	clk_disable_unprepare(st->clk);
}

static int ada4355_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct ada4355_state *st = ada4355_get_data(indio_dev);
	unsigned int reg_cntrl, reg_cntrl_3;

    // Set the numbers of lanes
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);


	//reg_cntrl_3 = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
	//reg_cntrl_3 |= AXI_ADA4355_SELF_SYNC_BIT;
	//axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl_3); // in the default mode the CNV is not used as a start of transfer flag

    // enable the sync
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	reg_cntrl |= ADI_SYNC;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);

	return 0;
}

static int ada4355_setup(struct ada4355_state *st)
{
    unsigned int reg, id;
    int ret;

   // ret = regmap_write(st->regmap,  ADA4355_REG_POWER_MODES , ADA4355_DIGITAL_RESET);
   // if (ret)
	//	return ret;
//
   // dev_err(&st->spi->dev, "ADA4355_REG_POWER_MODES_DIG_RESET 0x%X\n",  ret);
//
   // ret = regmap_read(st->regmap, ADA4355_REG_POWER_MODES, &reg);
//
   // dev_err(&st->spi->dev, "ADA4355_REG_POWER_MODES_NORMAL_OPERATION 0x%X\n", ret);
	//if (ret)
	//	return ret;
//
   // ret = regmap_write(st->regmap,  ADA4355_REG_POWER_MODES , (reg & ADA4355_NORMAL_OPERATION));
   //  dev_err(&st->spi->dev, "ADA4355_REG_POWER_MODES_NORMAL_OPERATION_WR 0x%X\n", ret);
   // if (ret)
	//	return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_CHIP_CONFIGURATION, 0x00);
	dev_err(&st->spi->dev, "ADA4355_REG_CHIP_CONFIGURATION 0x%X\n", ret);
    if (ret)
		return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_DEVICE_INDEX, 0x02);
	    dev_err(&st->spi->dev, "ADA4355_REG_DEVICE_INDEX 0x%X\n", ret);
    if (ret)
		return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_SERIAL_CHANNEL_STATUS, 0x03);
    dev_err(&st->spi->dev, "ADA4355_REG_SERIAL_CHANNEL_STATUS 0x%X\n", ret);
	if (ret)
		return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_DEVICE_INDEX, 0x31);
	dev_err(&st->spi->dev, "ADA4355_REG_DEVICE_INDEX 0x%X\n", ret);
    if (ret)
		return ret;

    ret = regmap_read(st->regmap, ADA4355_REG_CHIP_ID, &id);
	if (ret)
		return ret;

	if (id != ADA4355_CHIP_ID) {
		dev_err(&st->spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
	//	return -EINVAL;
	}

    ret = regmap_write(st->regmap, ADA4355_REG_SERIAL_OUT_DATA_CNTRL, 0x30);
	if (ret)
		return ret;

    ret = regmap_read(st->regmap, ADA4355_REG_TRANFER, &reg);
	if (ret)
		return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_TRANFER, (reg | ADA4355_OVERRIDE));
	if (ret)
		return ret;

    ret = regmap_write(st->regmap, ADA4355_REG_RESOLUTION_SAMPLE_RATE, ADA4355_125_RATE);
	//if (ret)
	return ret;
}

static int ada4355_properties_parse(struct ada4355_state *st)
{
	struct spi_device *spi = st->spi;
	unsigned int val;
	int ret;

	st->clk = devm_clk_get(&spi->dev, "adc_clk"); // verify with devicetree
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = of_property_read_u32(spi->dev.of_node, "num_lanes", &val); // used to read a 32-bit integer value from a property in a devicetree node -?
                                                                     // ar trebui 16?? of_property_read_u16_array
	if (!ret)
		st->num_lanes = val;
	else
		st->num_lanes = 1;

	return 0;
}

static int ada4355_probe(struct spi_device *spi)
{
    struct iio_dev *indio_dev;
    struct regmap *regmap;
    struct axiadc_converter *conv;
    struct ada4355_state *st;
    int ret;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
    if (!indio_dev)
            return -ENOMEM;

    regmap = devm_regmap_init_spi(spi, &ada4355_regmap_config);
    if (IS_ERR(regmap))
            return PTR_ERR(regmap);

    st = iio_priv(indio_dev);
    st->regmap = regmap;
    st->spi = spi;

    mutex_init(&st->lock);

    conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

    ret = ada4355_properties_parse(st);
	if (ret)
		return ret;

    ret = clk_prepare_enable(st->clk);
	if (ret)
		return ret;

    ret = devm_add_action_or_reset(&spi->dev, ada4355_clk_disable, st);
	if (ret)
		return ret;

    conv->spi = st->spi;
    conv->clk = st->clk;
    conv->chip_info = &ada4355_chip_info;
    conv->reg_access = ada4355_reg_access;
    conv->read_raw = ada4355_read_raw;
    conv->write_raw = ada4355_write_raw;
    conv->post_setup = ada4355_post_setup;
    conv->phy = st;

    /* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

    return ada4355_setup(st);
}

static const struct spi_device_id ada4355_id[] = {
	{ "ada4355", 0 },
	{}
};

MODULE_DEVICE_TABLE(spi, ada4355_id);

static const struct of_device_id ada4355_of_match[] = {
	{ .compatible = "adi,ada4355" },
	{},
};
MODULE_DEVICE_TABLE(of, ada4355_of_match);

static struct spi_driver ada4355_driver = {
	.driver = {
		.name = "ada4355",
		.of_match_table = ada4355_of_match,
	},
	.probe = ada4355_probe,
	.id_table = ada4355_id,
};
module_spi_driver(ada4355_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices AD4080");
MODULE_LICENSE("GPL");
