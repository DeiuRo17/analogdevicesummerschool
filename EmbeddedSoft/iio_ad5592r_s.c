// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO ADC: analog to digital converter for weight sensor module
 *
 * Copyright (c) 2025 Andreas Klinger <ak@it-klinger.de>
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/bitfield.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>


#define AD5592R_S_GEN_CTRL_REG              0x3
#define AD5592R_S_BUFF_EN                BIT(8)

#define AD5592R_S_REG_READBACK               0x7
#define AD5592R_S_RD_EN_MSK                 BIT(6)
#define AD5592R_S_RD_REG_MSK                GENMASK(5, 2)

#define AD5592R_S_ADDR_MSK                  GENMASK(14, 11)
#define AD5592R_S_DATA_MSK                  GENMASK(10, 0)

#define AD5592R_S_RESULT_DATA               GENMASK(11,0)
#define AD5592R_S_REG_PD_REF                0xB
#define AD5592R_S_EN_REF_MASK                  BIT(9)

#define AD5592R_S_REG_ADC_CONF              0x4
#define ADD592R_S_EN_CHANS                  GENMASK(5, 0)

#define AD5592R_S_REG_SEQ                   0x2
#define AD5592R_S_SEQ_CHAN(x)               BIT(x)

struct iio_adc_st {
	struct spi_device *spi;
	bool en;
	int chan1;
	int chan2;
	int chan3;
	int chan4;
	int chan5;
	int chan6;
};

static int ad5592r_s_spi_write(struct iio_adc_st *st, u8 addr, u16 data)
{
	u16 msg = 0;
	u16 tx;

	struct spi_transfer t = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg = FIELD_PREP(AD5592R_S_ADDR_MSK, addr) |
	      FIELD_PREP(AD5592R_S_DATA_MSK, data);
	put_unaligned_be16(msg, &tx);

	dev_info(&st->spi->dev, "SPI WR msg = 0x%X", msg);
	dev_info(&st->spi->dev, "SPI WR tx = 0x%X", tx);
	return spi_sync_transfer(st->spi, &t, 1);
}

static int ad5592r_s_nop(struct iio_adc_st *st, u16 *data)
{
	u16 rx;
	int ret;
	struct spi_transfer t = {
		.tx_buf = NULL,
		.rx_buf = &rx,
		.len = 2,
	};

	ret = spi_sync_transfer(st->spi, &t, 1);
	if (ret) {
		dev_err(&st->spi->dev, "SPI NOP fail %d", ret);
		return ret;
	}
	*data = get_unaligned_be16(&rx);
	return 0;
}

static int ad5592r_s_spi_read_reg(struct iio_adc_st *st, u8 addr, u16 *data)
{
	u16 msg = 0;
	u16 tx;
	u16 rx;
	int ret;

	struct spi_transfer t = {
		.tx_buf = &tx,
		.len = 2,
	};

	msg = FIELD_PREP(AD5592R_S_ADDR_MSK, AD5592R_S_REG_READBACK) |
	      AD5592R_S_RD_EN_MSK | FIELD_PREP(AD5592R_S_RD_REG_MSK, addr);

	put_unaligned_be16(msg, &tx);

	ret = spi_sync_transfer(st->spi, &t, 1);
	if (ret) {
		dev_err(&st->spi->dev, "SPI RD fail %d", ret);
		return ret;
	}
	ret = ad5592r_s_nop(st, &rx);
	if (ret) {
		dev_info(&st->spi->dev, "SPI NOP FAIL %d", ret);
	}
	dev_info(&st->spi->dev, "SPI WR tx = 0x%X", tx);
	dev_info(&st->spi->dev, "SPI WR rx = 0x%X", rx);
	*data = rx;
	return 0;
}

static int iio_adc_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct iio_adc_st *st = iio_priv(indio_dev);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (st->en) {
			switch (chan->channel) {
			case 0:
				st->chan1 = val;
				break;
			case 1:
				st->chan2 = val;
				break;
			case 2:
				st->chan3 = val;
				break;
			case 3:
				st->chan4 = val;
				break;
			case 4:
				st->chan5 = val;
				break;
			case 5:
				st->chan6 = val;
				break;
			default:
				return -EINVAL;
			}
			return 0;
		}
		return -EIO;
	case IIO_CHAN_INFO_ENABLE:
		st->en = val;
		return 0;
	default:
		return -EINVAL;
	}
}


static irqreturn_t ad5592r_s_trigger_handler(int irq, void *p)
{
        struct iio_poll_func *pf = p;
        struct iio_dev *indio_dev = pf->indio_dev;
        struct iio_adc_st *st = iio_priv(indio_dev);
        u16 buf[6];
        u16 rx;
        int i = 0;
        int bit;
        int ret;
        for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->num_channels){
                ret = ad5592r_s_spi_write(st, AD5592R_S_REG_SEQ, AD5592R_S_SEQ_CHAN(bit));
                if (ret) {
                        dev_err(&st->spi->dev, "Failed to write sequence %d", ret);
                        return IRQ_HANDLED;
                }
                ret = ad5592r_s_nop(st, &rx);
                if (ret) {
                        dev_err(&st->spi->dev, "Failed NOP %d", ret);
                        return IRQ_HANDLED;
                }    
                ret = ad5592r_s_nop(st, &rx);
                if (ret) {
                        dev_err(&st->spi->dev, "Failed getting data %d", ret);
                        return IRQ_HANDLED;
                }
                buf[i++] = FIELD_GET(AD5592R_S_RESULT_DATA, rx);
        }
        iio_push_to_buffers(indio_dev, buf);
        iio_trigger_notify_done(indio_dev->trig);
        return IRQ_HANDLED;
}


static int ad5592r_s_read_chan(struct iio_adc_st *st,int channel, u16 *data){
    int ret;
    u16  rx;
    ret = ad5592r_s_spi_write(st,AD5592R_S_REG_SEQ,AD5592R_S_SEQ_CHAN(channel));
    if (ret){
        dev_info(&st->spi->dev, "Failed sequence %d", ret);
    return ret;
    }

    ret = ad5592r_s_nop(st,&rx);
    if (ret){
        dev_info(&st->spi->dev, "Failed nop %d", ret);
    return ret;
    }

    *data = FIELD_GET(AD5592R_S_RESULT_DATA,rx);
    return 0;
}

static int iio_adc_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct iio_adc_st *st = iio_priv(indio_dev);
int ret;
u16 tmp;
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (st->en) {
			ret = ad5592r_s_read_chan(st,chan->channel,&tmp);
        if (ret){
        dev_info(&st->spi->dev, "Read  channel fail %d", ret);
        return ret;
    }
    *val= tmp;
			return IIO_VAL_INT;
		} else {
			return -EIO;
		}
	case IIO_CHAN_INFO_ENABLE:
		*val = st->en;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int iio_adc_emu_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				  unsigned int writeval, unsigned int *readval)
{
	struct iio_adc_st *st = iio_priv(indio_dev);
	if (readval)
		return ad5592r_s_spi_read_reg(st, reg, (u16 *)readval);
	return ad5592r_s_spi_write(st, reg, writeval);
}

static const struct iio_info iio_adc_info = {
	.read_raw = iio_adc_read_raw,
	.write_raw = iio_adc_write_raw,
	.debugfs_reg_access = &iio_adc_emu_reg_access,
};

static struct iio_chan_spec iio_adc_chans[] = {
    {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 0,
                .scan_index = 0,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
        {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 1,
                .scan_index = 1,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
        {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 2,
                .scan_index = 2,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
        {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 3,
                .scan_index = 3,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
        {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 4,
                .scan_index = 4,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
        {
                .type = IIO_VOLTAGE,
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
                .output = 0,
                .indexed = 1,
                .channel = 5,
                .scan_index = 5,
                .scan_type = {
                        .sign = 'u',
                        .realbits = 12,
                        .storagebits = 16
                }
        },
};

static int iio_ad5592r_s_probe(struct spi_device *spi)
{

	
	struct iio_dev *indio_dev;
	struct iio_adc_st *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
    int ret;
	st = iio_priv(indio_dev);
	st->en = false;
	st->chan1 = 1;
	st->chan2 = 2;
	st->chan3 = 3;
	st->chan4 = 4;
	st->chan5 = 5;
	st->chan6 = 6;
	st->spi = spi;

    ret =ad5592r_s_spi_write(st,AD5592R_S_REG_PD_REF,FIELD_PREP(AD5592R_S_EN_REF_MASK,1));

    if (ret)
    {
    dev_info(&spi->dev, "FAILED ENABLING REFERENCE %d", ret);
    return ret;
    }
    
    ret =ad5592r_s_spi_write(st,AD5592R_S_REG_ADC_CONF,ADD592R_S_EN_CHANS);

    if (ret)
    {
    dev_info(&spi->dev, "FAILED ENABLING REFERENCE %d", ret);
    return ret;
    }

     ret =ad5592r_s_spi_write(st,AD5592R_S_GEN_CTRL_REG,AD5592R_S_BUFF_EN);

    if (ret)
    {
    dev_info(&spi->dev, "FAILED ENABLING BUFFER %d", ret);
    return ret;
    }

	indio_dev->name = "iio_ad5592r_s";
	indio_dev->info = &iio_adc_info;
	indio_dev->channels = iio_adc_chans;
	indio_dev->num_channels = ARRAY_SIZE(iio_adc_chans);

        ret = devm_iio_triggered_buffer_setup(
		&spi->dev, indio_dev, NULL, &ad5592r_s_trigger_handler, NULL);
	if (ret) {
		dev_err(&st->spi->dev, "buffer setup fail %d", ret);
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver iio_ad5592_s_driver = {
    .driver = {
        .name = "iio_ad5592r_s",
    },
    .probe = iio_ad5592r_s_probe,
};

module_spi_driver(iio_ad5592_s_driver);

MODULE_AUTHOR("Bota Andrei-Daniel");
MODULE_DESCRIPTION("Analog Devices AD5592R ADC driver");
MODULE_LICENSE("GPL v2");