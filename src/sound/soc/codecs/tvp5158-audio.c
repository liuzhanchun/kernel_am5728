/*
 * ALSA SoC TVP5158 audio dummy driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

static const struct snd_soc_dapm_widget tvp5158_dapm_widgets[] = {
    SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route tvp5158_dapm_routes[] = {

};

static struct snd_soc_codec_driver soc_codec_tvp5158 = {
	.dapm_widgets = tvp5158_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tvp5158_dapm_widgets),
	.dapm_routes = tvp5158_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tvp5158_dapm_routes),
};

/* null function */
static int tvp5158_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int tvp5158_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	return 0;
}

static struct snd_soc_dai_ops tvp5158_dai_ops = {
	.set_sysclk	= tvp5158_set_dai_sysclk,
	.set_fmt	= tvp5158_set_dai_fmt,
};

static struct snd_soc_dai_driver tvp5158_dai = {
	.name = "tvp5158-hifi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 16,
		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
		.ops = &tvp5158_dai_ops,
};

static int tvp5158_audio_codec_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_tvp5158,
					&tvp5158_dai, 1);
	if (ret < 0)
		printk(KERN_ERR "TVP5158 Audio Codec Register Failed\n");

	printk(KERN_INFO "Registered tvp5158 audio codec\n");

	return ret;
}

static int tvp5158_audio_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_tvp5158_audio_match_ids[] = {
	{ .compatible = "ti,tvp5158-audio", .data = "tvp5158-audio" },
	{},
};
MODULE_DEVICE_TABLE(of, of_tvp5158_audio_match_ids);
#endif

static struct platform_driver tvp5158_audio_codec_driver = {
	.probe		= tvp5158_audio_codec_probe,
	.remove		= tvp5158_audio_codec_remove,
	.driver		= {
		.name	= "tvp5158-audio",
		.of_match_table	= of_tvp5158_audio_match_ids,
		.owner	= THIS_MODULE,
	},
};

static int __init tvp5158_audio_init(void)
{
	return platform_driver_register(&tvp5158_audio_codec_driver);
}

static void __exit tvp5158_audio_exit(void)
{
	platform_driver_unregister(&tvp5158_audio_codec_driver);
}

module_init(tvp5158_audio_init);
module_exit(tvp5158_audio_exit);

MODULE_DESCRIPTION("TVP5158 Dummy codec Interface");
MODULE_LICENSE("GPL");
