 /*
 * GV7601 device registration.
 *
 * Copyright (C) 2016-2017 TronLong
 * Author: vefone <lwf@tronlong.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

/* Debug functions */
static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

#define REG_STATUS                      0x06
#define REG_WORDS_PER_ACT_LINE          0x1F
#define REG_WORDS_PER_LINE              0x20
#define REG_LINES_PER_FRAME             0x21
#define REG_ACT_LINES_PER_FRAME 	0x22
#define REG_H_V_LOCK                    0x23

#define MASK_H_LOCK                     (1 << 0)
#define MASK_V_LOCK                     (1 << 1)
#define MASK_INT_PROG                   (1 << 11)
#define MASK_STD_LOCK                   (1 << 12)
#define MASK_VD_STD_STATUS              0x3F00  // 0x06 [13:8]
#define MASK_WORDS_PER_ACTLINE          0x3FFF  // 0x1f [13:0]
#define MASK_WORDS_PER_LINE             0x3FFF  // 0x20 [13:0]
#define MASK_LINES_PER_FRAME            0x7FF   // 0x21 [10:0]
#define MASK_ACTLINE_PER_FIELD          0x7FF   // 0x22 [10:0]

#define GV7601_WIDTH_MIN                720
#define GV7601_WIDTH_MAX                1920
#define GV7601_HEIGHT_MIN               480
#define GV7601_HEIGHT_MAX               1080
#define GV7601_PIXELCLOCK_MIN           10519200
#define GV7601_PIXELCLOCK_MAX           74250000

#define WIDTH_LINE_1920      1920
#define HEIGHT_LINE_1080     1080
#define WIDTH_LINE_1280      1280
#define HEIGHT_LINE_720      720

/* enum gv7601_std - enum for supported standards */
enum gv7601_std {
        STD_1080P60 = 0,
        STD_1080P30,
	STD_1080I60,
	STD_720P60,
        STD_INVALID
};

struct gv7601_priv {
	struct spi_device *pdev;
	struct v4l2_subdev subdev;
	enum   gv7601_std current_std;
	struct v4l2_mbus_framefmt       format;
	int enabled;
};

/*
 * struct gv7601_platform_data - GV7601 platform data
 * @ext_freq: Input clock frequency
 * @target_freq: Pixel clock frequency
 */
struct gv7601_platform_data {
	int ext_freq;
	int target_freq;
};

struct gv7601_color_format {
        u32 code;
        enum v4l2_colorspace colorspace;
};

static const struct gv7601_color_format gv7601_cfmts[] = {
        {
                .code           = MEDIA_BUS_FMT_UYVY8_2X8,
                .colorspace     = V4L2_COLORSPACE_SMPTE170M,
        },
};

struct gv7601_std_info {
        unsigned long width;
        unsigned long height;
	struct v4l2_standard standard;
};


static const struct gv7601_std_info std_list[] = {
[STD_1080P60] = {
        .width = WIDTH_LINE_1920,
        .height = HEIGHT_LINE_1080,
        .standard = {
                .index = 0,
                .id = -1,
                .name = "1080P60",
                .frameperiod = {1001, 60000},
                .framelines = 1125
                },
        },

[STD_1080P30] = {
        .width = WIDTH_LINE_1920,
        .height = HEIGHT_LINE_1080,
        .standard = {
                .index = 1,
                .id = -1,
                .name = "1080P30",
                .frameperiod = {1001, 30000},
                .framelines = 1125
                },
        },

[STD_1080I60] = {
        .width = WIDTH_LINE_1920,
        .height = HEIGHT_LINE_1080,
        .standard = {
                .index = 2,
                .id = -1,
                .name = "1080I60",
                .frameperiod = {1001, 60000},
                .framelines = 1125
                },
        },

[STD_720P60] = {
        .width = WIDTH_LINE_1280,
        .height = HEIGHT_LINE_720,
        .standard = {
                .index = 3,
                .id = -1,
                .name = "720P60",
                .frameperiod = {1001, 60000},
                .framelines = 750
                },
        },
};

static int gv7601_read_register(struct spi_device *spi, u16 addr, u16 *value)
{
        int ret;
        u16 buf_addr = (0x8000 | (0x0FFF & addr));
        u16 buf_value = 0;
        struct spi_message msg;
        struct spi_transfer tx[] = {
                {
                        .tx_buf = &buf_addr,
                        .len = 2,
                        .delay_usecs = 1,
                }, {
                        .rx_buf = &buf_value,
                        .len = 2,
                        .delay_usecs = 1,
                },
        };

        spi_message_init(&msg);
        spi_message_add_tail(&tx[0], &msg);
        spi_message_add_tail(&tx[1], &msg);
        ret = spi_sync(spi, &msg);

        *value = buf_value;

        return ret;
}

static int gv7601_write_register(struct spi_device *spi, u16 addr, u16 value)
{
        int ret;
        u16 buf_addr = addr;
        u16 buf_value = value;
        struct spi_message msg;
        struct spi_transfer tx[] = {
                {
                        .tx_buf = &buf_addr,
                        .len = 2,
                        .delay_usecs = 1,
                }, {
                        .tx_buf = &buf_value,
                        .len = 2,
                        .delay_usecs = 1,
                },
        };

        spi_message_init(&msg);
        spi_message_add_tail(&tx[0], &msg);
        spi_message_add_tail(&tx[1], &msg);
        ret = spi_sync(spi, &msg);

        return ret;
}

static struct gv7601_priv *to_gv7601(struct spi_device *spi)
{
        return container_of(spi_get_drvdata(spi), struct gv7601_priv,
                        subdev);
}

static enum gv7601_std
gv7601_get_video_std(struct spi_device *spi)
{
	u16 reg_value;
	u16 video_stand;
	struct gv7601_priv *priv  = to_gv7601(spi);

	gv7601_read_register(spi, REG_H_V_LOCK, &reg_value);

	v4l2_dbg(1, debug, &priv->subdev, "%s\n",
                (reg_value & 0x03) ?
                "Signal Present" : "Signal not present");
	if (!(reg_value & 0x03)) {
		priv->current_std = STD_1080P60;
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "Set the default video standard to 1080P60");
		goto fail;
	}

	gv7601_read_register(spi, REG_STATUS, &reg_value);
	video_stand = (reg_value & MASK_VD_STD_STATUS) >> 8;
	switch (video_stand) {
	case 0x2b:
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "the resolution of the input video is 1080P60");
		priv->current_std = STD_1080P60;
		break;
	case 0x0b:
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "the resolution of the input video is 1080P30");
		priv->current_std = STD_1080P30;
		break;
	case 0x0a:
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "the resolution of the input video is 1080I60");
		priv->current_std = STD_1080I60;
		break;
	case 0x00:
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "the resolution of the input video is 720P60");
		priv->current_std = STD_720P60;
		break;
	default:
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "the resolution of the input video is unknowed");
		v4l2_dbg(1, debug, &priv->subdev, "%s\n", "Set the default video standard to 1080P60");
		priv->current_std = STD_1080P60;
		break;
	}

fail:
	return priv->current_std;
}

static int gv7601_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{

	struct spi_device *spi = v4l2_get_subdevdata(sd);
	struct gv7601_priv *priv = to_gv7601(spi);

	if (std_id == NULL)
		return -EINVAL;
	*std_id = V4L2_STD_UNKNOWN;

	gv7601_get_video_std(spi);

	if (priv->current_std == STD_INVALID)
		return -ENODEV;
	*std_id = std_list[priv->current_std].standard.id;

	return 0;
}

static int gv7601_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
        struct gv7601_priv *priv = to_gv7601(spi);
        struct v4l2_captureparm *cparm;
        enum gv7601_std current_std;

        if (parms == NULL || parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;

        gv7601_get_video_std(spi);
        if (priv->current_std == STD_INVALID)
                return -ENODEV;

        /* get the current standard */
        current_std = priv->current_std;
        cparm = &parms->parm.capture;
        cparm->capability = V4L2_CAP_TIMEPERFRAME;
        cparm->timeperframe =
                std_list[current_std].standard.frameperiod;

	return 0;
}

static int gv7601_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
        struct gv7601_priv *priv = to_gv7601(spi);
        struct v4l2_captureparm *cparm;
        enum gv7601_std current_std;

        if (parms == NULL || parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;

        gv7601_get_video_std(spi);
        if (priv->current_std == STD_INVALID)
                return -ENODEV;

        /* get the current standard */
        current_std = priv->current_std;
        cparm = &parms->parm.capture;
        cparm->capability = V4L2_CAP_TIMEPERFRAME;
        cparm->timeperframe =
                std_list[current_std].standard.frameperiod;

	return 0;
}

static int gv7601_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int gv7601_enum_mbus_code(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(gv7601_cfmts))
                return -EINVAL;

	code->code = gv7601_cfmts[code->index].code;

	return 0;
}

static int gv7601_enum_frame_size(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_frame_size_enum *fse)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
        struct gv7601_priv *priv = to_gv7601(spi);
	enum gv7601_std std;

	if (fse->index >= 1)
                return -EINVAL;

	if (priv->current_std == STD_INVALID)
		gv7601_get_video_std(spi);
	if (priv->current_std == STD_INVALID)
		return -ENODEV;

	std = priv->current_std;
        fse->code = priv->format.code;

        fse->max_width  = fse->min_width  = std_list[std].width;
        fse->max_height = fse->min_height = std_list[std].height;

	return 0;
}

static int gv7601_enum_frame_interval(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_frame_interval_enum *fie)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
        struct gv7601_priv *priv = to_gv7601(spi);
        enum gv7601_std current_std;
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

static int gv7601_get_fmt(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf;
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	struct gv7601_priv *priv = to_gv7601(spi);
	enum gv7601_std current_std;

	gv7601_get_video_std(spi);
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

	mf->code = MEDIA_BUS_FMT_UYVY8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_ALTERNATE;
        mf->width = std_list[current_std].width;;
        mf->height = std_list[current_std].height;
        fmt->format = *mf;

	return 0;
}

static int gv7601_set_fmt(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_format *fmt)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
        struct gv7601_priv *priv = to_gv7601(spi);
        int ret;

        if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {

                ret = gv7601_get_fmt(sd, cfg, fmt);
                if (ret)
                        return ret;

                priv->format = fmt->format;
        }

	return 0;
}

static struct v4l2_subdev_video_ops gv7601_video_ops = {
        .querystd       = gv7601_querystd,
        .g_parm         = gv7601_g_parm,
        .s_parm         = gv7601_s_parm,
        .s_stream       = gv7601_s_stream,
};

static struct v4l2_subdev_pad_ops gv7601_pad_ops = {
        .enum_mbus_code         = gv7601_enum_mbus_code,
        .enum_frame_size        = gv7601_enum_frame_size,
        .enum_frame_interval    = gv7601_enum_frame_interval,
        .get_fmt                = gv7601_get_fmt,
        .set_fmt                = gv7601_set_fmt,
};

static struct v4l2_subdev_ops gv7601_subdev_ops = {
        .video          = &gv7601_video_ops,
        .pad            = &gv7601_pad_ops,
};

#ifdef CONFIG_OF
static const struct of_device_id of_gv7601_match_ids[] = {
	{ .compatible = "gennum,gv7601", .data = "gv7601" },
	{},
};
MODULE_DEVICE_TABLE(of, of_gv7601_match_ids);
#endif

static const struct spi_device_id gv7601_id_table[] = {
	{"gennum,gv7601", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, gv7601_id_table);

static int gv7601_probe(struct spi_device *spi)
{
	int ret;
	u16 vd_std_value, words_per_active_line, total_words_per_line, total_line_per_frame, reg_22_value, reg_23_value;
	struct v4l2_subdev *sd;
	struct gv7601_priv *gv7601 = to_gv7601(spi);

	gv7601 = devm_kzalloc(&spi->dev, sizeof(*gv7601), GFP_KERNEL);
	if (!gv7601)
		return -ENOMEM;

	gv7601->pdev = spi;
	sd = &gv7601->subdev;

	spi->mode = SPI_MODE_0;
        spi->irq = -1;
        spi->max_speed_hz = 15000000;
        spi->bits_per_word = 16;
        ret = spi_setup(spi);

	/*
         * TODO: detect the gv7601
         */

	gv7601_read_register(spi, REG_STATUS, &vd_std_value);
	gv7601_read_register(spi, REG_WORDS_PER_ACT_LINE, &words_per_active_line);
	gv7601_read_register(spi, REG_WORDS_PER_LINE, &total_words_per_line);
	gv7601_read_register(spi, REG_LINES_PER_FRAME, &total_line_per_frame);
	gv7601_read_register(spi, REG_ACT_LINES_PER_FRAME, &reg_22_value);
	gv7601_read_register(spi, REG_H_V_LOCK, &reg_23_value);

	v4l2_dbg(1, debug, &gv7601->subdev, "VD_STD: 0x%x, WORDS_PER_ACTLINE: 0x%x, WORDS_PER_LINE: 0x%x, LINES_PER_FRAME: 0x%x, ACTLINE_PER_FIELD: 0x%x\n",
					(vd_std_value & MASK_VD_STD_STATUS) >> 8,
					words_per_active_line & MASK_WORDS_PER_ACTLINE,
					total_words_per_line & MASK_WORDS_PER_LINE,
					total_line_per_frame & MASK_LINES_PER_FRAME,
					reg_22_value & MASK_ACTLINE_PER_FIELD);
	v4l2_dbg(1, debug, &gv7601->subdev, "%s\n", (reg_22_value & MASK_STD_LOCK) ? "Video standard is lock" : "Video standard is unlock");
	v4l2_dbg(1, debug, &gv7601->subdev, "%s\n", (reg_22_value & MASK_INT_PROG) ? "Video standard is interlaced" : "Video standard is progressive");
	v4l2_dbg(1, debug, &gv7601->subdev, "%s, %s\n", (reg_23_value & MASK_H_LOCK) ? "horizontal signal is lock" : "horizontal signal is unlock",
							(reg_23_value & MASK_V_LOCK) ? "vertical signal is lock" : "vertical signal is unlock");

	v4l2_spi_subdev_init(sd, spi, &gv7601_subdev_ops);

	sd->dev = &spi->dev;

	gv7601_get_video_std(spi);

	ret = v4l2_async_register_subdev(sd);

        if (!ret)
               v4l2_info(&gv7601->subdev, "gv7601 sdi driver registered successfully!\n");
        else {
		v4l2_info(&gv7601->subdev, "gv7601 sdi driver registered failed!\n");
                return ret;
	}

	return 0;
}

static int gv7601_remove(struct spi_device *spi)
{
	struct gv7601_priv *priv = spi_get_drvdata(spi);

        v4l2_device_unregister_subdev(&priv->subdev);

	v4l2_info(&priv->subdev, "gv7601 sdi driver unregistered successfully!\n");
	return 0;
}

static struct spi_driver gv7601_driver = {
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= "gv7601",
		.of_match_table = of_match_ptr(of_gv7601_match_ids),
	},
	.probe		= gv7601_probe,
	.remove		= gv7601_remove,
};

module_spi_driver(gv7601_driver);

MODULE_DESCRIPTION("GENNUM GV7601 driver");
MODULE_AUTHOR("Vefone");
MODULE_ALIAS("spi:gv7601");
MODULE_LICENSE("GPL v2");


