 /*
 * TI TVP5158 V4L2 subdevice driver
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 * Sathishkumar S <sathish.omap@gmail.com>
 * Nikhil Devshatwar, <nikhil.nd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include <linux/pm_runtime.h>

#include <linux/of_gpio.h>
#include <linux/of_device.h>

/* Debug functions */
static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* VBUS Register Address (24-bit), VBUS Register value (8-bit) */
struct vbus_addr_value {
        unsigned int addr;
        unsigned char val;
};

struct vbus_addr_value vbus_addr_value_set[] = {
        {0x00403E50, 0x40},
        {0x00403E51, 0x00},
        {0x00403E52, 0x40},
        {0x00403E53, 0x00},
        {0x00403E54, 0x44},
        {0x00403E55, 0x23},
        {0x00403E56, 0x38},
        {0x00403E57, 0x10},
        {0x00403E58, 0x53},
        {0x00403E59, 0x23},
        {0x00403E5A, 0x02},
        {0x00403E5B, 0x00},
        {0x00403E5C, 0x20},
        {0x00403E5D, 0x00},
        {0x00403E5E, 0x04},
        {0x00403E5F, 0x04},
        {0x00403E60, 0x04},
        {0x00403E61, 0x04},
        {0x00403E62, 0x10},
        {0x00403E63, 0xF8},
        {0x00403E64, 0x30},
        {0x00403E65, 0x20},
        {0x00403E66, 0x30},
        {0x00403E67, 0x3F},
        {0x00403E68, 0x3F},
        {0x00403E69, 0x3F},
        {0x00403E6A, 0x0C},
        {0x00403E6B, 0x0C},
        {0x00403E6C, 0x80},
        {0x00403E6D, 0x80},
        {0x00403E6E, 0x08},
        {0x00403E6F, 0x08},
        {0x00403E70, 0x08},
        {0x00403E71, 0x30},
        {0x00403E72, 0x08},
        {0x00403E73, 0x04},
        {0x00403E74, 0x00},
        {0x00403E75, 0x10},
        {0x00403E76, 0x00},
        {0x00403E77, 0x00},
        {0x00403E78, 0x38},
        {0x00403E79, 0x00},
        {0x00403E7A, 0x55},
        {0x00403E7B, 0x03},
        {0x00403E7C, 0x03},
        {0x00403E7D, 0x00},
        {0x00403E7E, 0x03},
        {0x00403E7F, 0x00},
        {0x00403E80, 0x30},
        {0x00403E81, 0x30},
        {0x00403E82, 0x18},
        {0x00403E83, 0x0C},
        {0x00403E95, 0x00},
        {0x00403E96, 0x00},
        {0x00403E9B, 0x3F},
        {0x00403EA2, 0x0C},
        {0x00403EA3, 0x10},
        {0x00403EA6, 0x0C},
        {0x00403EA7, 0x10},
        {0x00403EA8, 0x44},
        {0x00403EA9, 0x00},
};
static bool vbus_prog = 1;
static bool stream_enable = 0;
static int
tvp5158_of_probe(struct i2c_client *client, struct device_node *node);
static int
tvp5158_init_gpios(struct i2c_client *client);
static int
tvp5158_set_default(struct i2c_client *client, unsigned char core);
static enum tvp5158_std
tvp5158_get_video_std(struct i2c_client *client, unsigned char core);
static void
tvp5158_start_streaming(struct i2c_client *client, unsigned char core);
static int
tvp5158_set_int_regs(struct i2c_client *client, struct vbus_addr_value *reg,
                        int cnt);

#define REG_STAUS_1    0x00
#define REG_STAUS_2    0x01
#define REG_VID_STAND  0x0C
#define REG_CHIPID_MSB 0x08
#define REG_CHIPID_LSB 0x09
#define REG_AVD_CTRL_1 0xB0
#define REG_AVD_CTRL_2 0xB1
#define REG_OFM_CTRL   0xB2
#define REG_DEC_RD_EN  0xFF
#define REG_DEC_WR_EN  0xFE
#define REG_VBUS_1     0xE8
#define REG_VBUS_2     0xE9
#define REG_VBUS_3     0xEA
#define REG_VBUS_DATA  0xE0

#define TVP_CORE_ALL            0x0F
#define TVP_DECODER_1           (1<<0)
#define TVP_DECODER_2           (1<<1)
#define TVP_DECODER_3           (1<<2)
#define TVP_DECODER_4           (1<<3)
#define TVP_DECODER_NUM		4

/* Non interleaved */
#define TVP_INTERLEAVE_MODE_NON         (0<<6)
#define TVP_INTERLEAVE_MODE_PIX         (1<<6)
#define TVP_INTERLEAVE_MODE_LINE        (2<<6)
/* Number of time multiplexed channels = 0 */
#define TVP_CH_MUX_NUMBER       (2<<4)
/* ITU BT 656 8 bit */
#define TVP_OUTPUT_TYPE         (0<<3)
/* Single stage */
#define TVP_VCS_ID              (0<<2)
/* D1 */
#define TVP_VID_RES             (0<<0)

#define TVP_ENABLE_DITHERING    (1<<4)
#define TVP_CHAN_ID_SAVEAV_EN   (1<<2)
#define TVP_CHAN_ID_BLANK_EN    (1<<1)
#define TVP_VID_DET_SAVEAV_EN   (1<<0)

#define TVP_VIDEO_PORT_ENABLE   (1<<0)
#define TVP_OUT_CLK_P_EN        (1<<2)
#define TVP_OSC_OUT_EN          (1<<5)
#define TVP_FIELD_RATE          (1<<5)
#define TVP_SIGNAL_PRESENT      (1<<7)
#define TVP_VIDEO_STANDARD_MASK (0x07)

/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS  (720)
#define NTSC_NUM_ACTIVE_LINES   (480)
#define PAL_NUM_ACTIVE_PIXELS  (720)
#define PAL_NUM_ACTIVE_LINES   (576)

#define MAX_NUM_GPIOS                   3

struct tvp5158_color_format {
        u32 code;
        enum v4l2_colorspace colorspace;
};

static const struct tvp5158_color_format tvp5158_cfmts[] = {
        {
                .code           = MEDIA_BUS_FMT_UYVY8_2X8,
                .colorspace     = V4L2_COLORSPACE_SMPTE170M,
        },
};

/* enum tvp5158_std - enum for supported standards */
enum tvp5158_std {
        STD_NTSC_MJ = 0,
        STD_PAL_BDGHIN,
        STD_INVALID
};

/**
 * struct tvp5158_std_info - Structure to store standard information
 * @width: Line width in pixels
 * @height:Number of active lines
 * @standard: v4l2 standard structure information
 */
struct tvp5158_std_info {
        unsigned long width;
        unsigned long height;
        struct v4l2_standard standard;
};

enum tvp5158_signal_present {
        TVP5158_SIGNAL_DEAD = 0,
        TVP5158_SIGNAL_PRESENT
};
struct tvp5158_priv {
        struct v4l2_subdev              subdev;
        struct v4l2_mbus_framefmt       format;
        enum tvp5158_std		current_std;
        enum tvp5158_std		ch_std[TVP_DECODER_NUM];

        struct gpio                     mux_gpios[MAX_NUM_GPIOS];
        int                             num_gpios;
};

static const struct tvp5158_std_info std_list[] = {
[STD_NTSC_MJ] = {
        .width = NTSC_NUM_ACTIVE_PIXELS,
        .height = NTSC_NUM_ACTIVE_LINES,
        .standard = {
                .index = 0,
                .id = V4L2_STD_NTSC,
                .name = "NTSC",
                .frameperiod = {1001, 30000},
                .framelines = 525
                },
        },

[STD_PAL_BDGHIN] = {
        .width = PAL_NUM_ACTIVE_PIXELS,
        .height = PAL_NUM_ACTIVE_LINES,
        .standard = {
                .index = 1,
                .id = V4L2_STD_PAL,
                .name = "PAL",
                .frameperiod = {1001, 25000},
                .framelines = 625
                },
        },
};

static struct tvp5158_priv *to_tvp5158(const struct i2c_client *client)
{
        return container_of(i2c_get_clientdata(client), struct tvp5158_priv,
                        subdev);
}


static int tvp5158_read(struct i2c_client *client, unsigned char addr)
{
        unsigned char buffer[1];
        int rc;

        buffer[0] = addr;

        rc = i2c_master_send(client, buffer, 1);
        if (rc < 0) {
                dev_err(&client->dev, "i2c i/o error: rc == %d (should be 1)\n"
                , rc);
                return rc;
        }

        rc = i2c_master_recv(client, buffer, 1);
        if (rc < 0) {
                dev_err(&client->dev, "i2c i/o error: rc == %d (should be 1)\n"
                , rc);
                return rc;
        }

        return buffer[0];
}
static inline void tvp5158_write(struct i2c_client *client, unsigned char addr,
                        unsigned char value)
{
        unsigned char buffer[2];
        int rc;

        buffer[0] = addr;
        buffer[1] = value;
        rc = i2c_master_send(client, buffer, 2);
        if (rc != 2)
                dev_err(&client->dev, "i2c i/o error: rc == %d (should be 2)\n"
                , rc);
}

/**
 * tvp5158_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable.
 */
static int tvp5158_s_stream(struct v4l2_subdev *sd, int enable)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_dbg(1, debug, sd,  "tvp5158_s_stream: enable(%d)\n", enable);

        tvp5158_init_gpios(client);

        if (enable) {
                if (vbus_prog == 1)
                        /* VBUS address value setting */
                        tvp5158_set_int_regs(client, vbus_addr_value_set,
                                        ARRAY_SIZE(vbus_addr_value_set));

		if(stream_enable == 0)
			tvp5158_start_streaming(client, TVP_CORE_ALL);

		stream_enable = 1;

        } else {
                tvp5158_write(client, REG_OFM_CTRL, 0x0);
		stream_enable = 0;
        }

        return 0;
}

/**
 * tvp5158_querystd() - V4L2 decoder interface handler for querystd
 * @sd: pointer to standard V4L2 sub-device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by TVP5146/47. If no active input is
 * detected then *std_id is set to 0 and the function returns 0.
 */
static int tvp5158_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);

        if (std_id == NULL)
                return -EINVAL;
#if 0
        *std_id = V4L2_STD_UNKNOWN;

        tvp5158_get_video_std(client, TVP_CORE_ALL);
        if (priv->current_std == STD_INVALID)
                return -ENODEV;
#endif
        *std_id = std_list[priv->current_std].standard.id;

return 0;
}

/**
 * tvp5158_g_parm() - V4L2 decoder interface handler for g_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int tvp5158_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        struct v4l2_captureparm *cparm;
        enum tvp5158_std current_std;

        if (parms == NULL || parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;

#if 0
        tvp5158_get_video_std(client, TVP_CORE_ALL);
        if (priv->current_std == STD_INVALID)
                return -ENODEV;
#endif

        /* get the current standard */
        current_std = priv->current_std;
        cparm = &parms->parm.capture;
        cparm->capability = V4L2_CAP_TIMEPERFRAME;
        cparm->timeperframe =
                std_list[current_std].standard.frameperiod;

        return 0;
}

/**
 * tvp5158_s_parm() - V4L2 decoder interface handler for s_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int tvp5158_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        struct v4l2_fract *timeperframe;
        enum tvp5158_std current_std;

        if (parms == NULL || parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;

        timeperframe = &parms->parm.capture.timeperframe;

        /* get the current standard */
        current_std = priv->current_std;
        if (current_std == STD_INVALID)
                return -ENODEV;

        *timeperframe =
            std_list[current_std].standard.frameperiod;

        return 0;
}

static int tvp5158_get_fmt(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_format *fmt)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        struct v4l2_mbus_framefmt *mf;
        enum tvp5158_std current_std;

#if 0
        tvp5158_get_video_std(client, TVP_CORE_ALL);
#endif
        current_std = priv->current_std;
        if (current_std == STD_INVALID)
                return -EINVAL;

        switch (fmt->which) {
        case V4L2_SUBDEV_FORMAT_TRY:
                mf = v4l2_subdev_get_try_format(sd, cfg, 0);
                break;
        case V4L2_SUBDEV_FORMAT_ACTIVE:
                mf = &priv->format;
                break;
        default:
                return -EINVAL;
        }

        mf->width = std_list[current_std].width;
        mf->height = std_list[current_std].height / 2;
        fmt->format = *mf;

        return 0;
}

static int tvp5158_set_fmt(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_format *fmt)

{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        int ret;

        if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {

                ret = tvp5158_get_fmt(sd, cfg, fmt);
                if (ret)
                        return ret;

                priv->format = fmt->format;
        }

        return 0;
}

static int tvp5158_enum_mbus_code(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_mbus_code_enum *code)
{
        if (code->index >= ARRAY_SIZE(tvp5158_cfmts))
                return -EINVAL;

        code->code = tvp5158_cfmts[code->index].code;

        return 0;
}

static int tvp5158_enum_frame_size(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_frame_size_enum *fse)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        enum tvp5158_std std;

        if (fse->index >= 1)
                return -EINVAL;
#if 0
        if (priv->current_std == STD_INVALID)
                tvp5158_get_video_std(client, TVP_CORE_ALL);
#endif
        if (priv->current_std == STD_INVALID)
                return -ENODEV;

        std = priv->current_std;
        fse->code = priv->format.code;

        fse->max_width  = fse->min_width  = std_list[std].width;
        fse->max_height = fse->min_height = std_list[std].height / 2;

        return 0;
}

static int tvp5158_enum_frame_interval(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_frame_interval_enum *fie)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tvp5158_priv *priv = to_tvp5158(client);
        enum tvp5158_std current_std;
        struct v4l2_fract *timeperframe;

        /* Each standard has one frame interval, so return
         * EINVAL if more are requested
         */
        if (fie->index != 0)
                return -EINVAL;

        /* Get the current standard */
        current_std = priv->current_std;

        /* Check if current standard is invalid */
        if (current_std == STD_INVALID)
                return -EINVAL;

        timeperframe = &fie->interval;
        *timeperframe =
                std_list[current_std].standard.frameperiod;

        return 0;
}


static int tvp5158_of_probe(struct i2c_client *client,
                        struct device_node *node)
{
        struct tvp5158_priv *priv = to_tvp5158(client);
        struct gpio *gpios = &priv->mux_gpios[0];
        unsigned int flags;
        int i, gpio;

        /* Iterate over all the gpios in the device tree
         * ENOENT is returned when trying to access last + 1 gpio
         */
        for (i = 0; i < MAX_NUM_GPIOS; i++) {

                gpio = of_get_named_gpio_flags(node, "mux-gpios", i, &flags);
                if (gpio_is_valid(gpio)) {
                        gpios[i].gpio   = gpio;
                        gpios[i].flags  = (flags & OF_GPIO_ACTIVE_LOW) ?
                                GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;
                        gpios[i].label  = client->name;
                } else if (gpio == -ENOENT) {
                        break;
                } else {
                        return gpio;
                }
        }
        priv->num_gpios = i;
        return 0;
}

static int tvp5158_init_gpios(struct i2c_client *client)
{
        struct tvp5158_priv *priv = to_tvp5158(client);
        int ret = 0;

        ret = gpio_request_array(priv->mux_gpios, priv->num_gpios);
        if (ret)
                goto done;
        gpio_free_array(priv->mux_gpios, priv->num_gpios);

done:
        return ret;
}

static struct v4l2_subdev_video_ops tvp5158_video_ops = {
        .querystd       = tvp5158_querystd,
        .g_parm         = tvp5158_g_parm,
        .s_parm         = tvp5158_s_parm,
        .s_stream       = tvp5158_s_stream,
};

static struct v4l2_subdev_pad_ops tvp5158_pad_ops = {
        .enum_mbus_code         = tvp5158_enum_mbus_code,
        .enum_frame_size        = tvp5158_enum_frame_size,
        .enum_frame_interval    = tvp5158_enum_frame_interval,
        .get_fmt                = tvp5158_get_fmt,
        .set_fmt                = tvp5158_set_fmt,
};

static struct v4l2_subdev_ops tvp5158_subdev_ops = {
        .video          = &tvp5158_video_ops,
        .pad            = &tvp5158_pad_ops,
};

static const struct i2c_device_id tvp5158_id[] = {
        { "ti,tvp5158", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, tvp5158_id);

static const struct of_device_id tvp5158_dt_id[] = {
        {
        .compatible   = "ti,tvp5158", .data = "tvp5158"
        },
        {
        }
};

static int tvp5158_set_int_regs(struct i2c_client *client,
                        struct vbus_addr_value *reg, int cnt)
{
        int i = 0;
        /* Core Write enable*/
        tvp5158_write(client, REG_DEC_WR_EN, TVP_CORE_ALL);
        for (i = 0; i < cnt; i++) {
                tvp5158_write(client, REG_VBUS_1, ((reg[i].addr >> 0) & 0xFF));
                tvp5158_write(client, REG_VBUS_2, ((reg[i].addr >> 8) & 0xFF));
                tvp5158_write(client, REG_VBUS_3, ((reg[i].addr >> 16) & 0xFF));
                tvp5158_write(client, REG_VBUS_DATA, reg[i].val);
        }
        vbus_prog = 0;
        return 0;
}

static int
tvp5158_set_default(struct i2c_client *client, unsigned char core)
{
        unsigned char tvp_reg_val = 0;
        /* Core Write enable*/
        tvp5158_write(client, REG_DEC_WR_EN, core);
        /* Set Video format */
        tvp_reg_val = TVP_INTERLEAVE_MODE_LINE | TVP_CH_MUX_NUMBER
                        | TVP_OUTPUT_TYPE | TVP_VCS_ID | TVP_VID_RES;
        tvp5158_write(client, REG_AVD_CTRL_1, tvp_reg_val);
        tvp_reg_val = 0;
        tvp_reg_val = TVP_ENABLE_DITHERING | TVP_CHAN_ID_SAVEAV_EN | TVP_CHAN_ID_BLANK_EN | TVP_VID_DET_SAVEAV_EN;
        tvp5158_write(client, REG_AVD_CTRL_2, tvp_reg_val);

        return 0;
}

static enum tvp5158_std
tvp5158_get_video_std(struct i2c_client *client, unsigned char core)
{
        struct tvp5158_priv *priv       = to_tvp5158(client);
        int i2c_read = 0;
	int i;

	/* Set the default video standard to NTSC video */
	priv->current_std = STD_NTSC_MJ;

	for (i = 0; i < TVP_DECODER_NUM; i++) {
	        /* Core Read Enable */
	        tvp5158_write(client, REG_DEC_RD_EN, 0x1 << i);
		mdelay(20);

	        /* Get Video Status */
	        i2c_read = tvp5158_read(client, REG_STAUS_2);
	        v4l2_dbg(1, debug, &priv->subdev, "ch %d: %s\n", i,
	                (i2c_read & TVP_SIGNAL_PRESENT) ?
	                "Signal Present" : "Signal not present");

	        if (!(i2c_read & TVP_SIGNAL_PRESENT))
	                priv->ch_std[i] = STD_NTSC_MJ;
	        else {
		        i2c_read = tvp5158_read(client, REG_VID_STAND);

		        switch(i2c_read & TVP_VIDEO_STANDARD_MASK) {
			case 0x1:
		                priv->current_std = STD_NTSC_MJ;
				priv->ch_std[i] = priv->current_std;
		                v4l2_info(&priv->subdev, "ch %d :NTSC video detected\n", i);
				break;
			case 0x2:
		                priv->current_std = STD_PAL_BDGHIN;
				priv->ch_std[i] = priv->current_std;
		                v4l2_info(&priv->subdev, "ch %d: PAL video detected\n", i);
				break;
		        default:
				priv->ch_std[i] = STD_NTSC_MJ;
		                v4l2_err(&priv->subdev, "ch %d: No video detected\n", i);
		        }
	        }
	}

	/* Core Read Enable */
        tvp5158_write(client, REG_DEC_RD_EN, core);
	mdelay(20);

        return priv->current_std;
}

static void
tvp5158_start_streaming(struct i2c_client *client, unsigned char core)
{
        unsigned char tvp_reg_val = 0;

        /* Decoder Write Enable */
        tvp5158_write(client, REG_DEC_WR_EN, core);
	msleep(20);

	/* Set Video format */
	tvp_reg_val = TVP_INTERLEAVE_MODE_LINE | TVP_CH_MUX_NUMBER
			| TVP_OUTPUT_TYPE | TVP_VCS_ID | TVP_VID_RES;
	tvp5158_write(client, REG_AVD_CTRL_1, tvp_reg_val);
	tvp_reg_val = 0;
	tvp_reg_val = TVP_ENABLE_DITHERING | TVP_CHAN_ID_SAVEAV_EN | TVP_CHAN_ID_BLANK_EN | TVP_VID_DET_SAVEAV_EN;
	tvp5158_write(client, REG_AVD_CTRL_2, tvp_reg_val);

#if 0
	/* Disable output, stream off */
	tvp_reg_val = 0x00;
	tvp5158_write(client, REG_OFM_CTRL, tvp_reg_val);
	msleep(20);
#endif

        /* Enable output, stream on */
        tvp_reg_val =  TVP_OSC_OUT_EN | TVP_VIDEO_PORT_ENABLE | TVP_OUT_CLK_P_EN;
        tvp5158_write(client, REG_OFM_CTRL, tvp_reg_val);
}

static int tvp5158_probe(struct i2c_client *client,
                         const struct i2c_device_id *did)
{
        struct tvp5158_priv *priv;
        struct v4l2_subdev *sd;
        struct device_node *node = client->dev.of_node;
        int ret = -1;
        int i2c_read;
        union {
                char buff[4];
                int buffer;
        } u_i2cbuf;

        priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
        if (!priv)
                return -ENOMEM;
        i2c_set_clientdata(client, priv);

        sd = &priv->subdev;

        v4l2_i2c_subdev_init(sd, client, &tvp5158_subdev_ops);

        ret = tvp5158_of_probe(client, node);
        if (ret) {
                dev_err(&client->dev, "Unable to of_probe\n");
                return ret;
        }

        ret = tvp5158_init_gpios(client);
        if (ret) {
                dev_err(&client->dev, "failed to init gpios ERR %d\n", ret);
                return ret;
        }

        priv->current_std = STD_NTSC_MJ;
        priv->format.code = tvp5158_cfmts[0].code;
        priv->format.colorspace = tvp5158_cfmts[0].colorspace;
        priv->format.field = V4L2_FIELD_ALTERNATE;

        /* Get Chip ID and register with v4l2*/
        i2c_read = tvp5158_read(client, REG_CHIPID_MSB);
        u_i2cbuf.buffer = i2c_read << 8;
        i2c_read = tvp5158_read(client, REG_CHIPID_LSB);
        u_i2cbuf.buffer |= i2c_read;
        if (u_i2cbuf.buffer == 0x5158) {
                v4l2_dbg(1, debug, sd, "Chip id : %x\n", u_i2cbuf.buffer);
                tvp5158_write(client, REG_DEC_WR_EN, TVP_CORE_ALL);
        } else {
                dev_err(&client->dev, "ERROR: Chip id is not TVP5158");
                return -ENODEV;
        }

        tvp5158_set_default(client, TVP_CORE_ALL);
        tvp5158_get_video_std(client, TVP_CORE_ALL);

        /* V4l2 asyn subdev register */
        sd->dev = &client->dev;
        ret = v4l2_async_register_subdev(sd);
        if (!ret)
                v4l2_info(&priv->subdev, "Camera sensor driver registered\n");
        else
                return ret;

        pm_runtime_enable(&client->dev);

        return ret;
}

static int tvp5158_remove(struct i2c_client *client)
{
        struct tvp5158_priv *priv = i2c_get_clientdata(client);

        v4l2_device_unregister_subdev(&priv->subdev);
        return 0;
}

static struct i2c_driver tvp5158_i2c_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "tvp5158",
                .of_match_table = tvp5158_dt_id,
        },
        .probe    = tvp5158_probe,
        .remove   = tvp5158_remove,
        .id_table = tvp5158_id,
};

module_i2c_driver(tvp5158_i2c_driver);

MODULE_DESCRIPTION("Video Decoder driver");
MODULE_AUTHOR("Sathishkumar S");
MODULE_LICENSE("GPL v2");
