/*
 * linux/sound/soc/codecs/sanremo_audio.c
 * Base on linux/sound/soc/codecs/wm8753.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/sanremo.h>
#include "sanremo-audio.h"

#define SANREMO_SOC_PROC
#define AUDIO_NAME "sanremo audio codec"
#define SANREMO_AUDIO_VERSION "0.1"

/* debug */
#define SANREMO_DEBUG 0
#if SANREMO_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "sanremo-audio: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

/* codec private data */
struct sanremo_audio_priv {
        unsigned int sysclk;
        unsigned int pcmclk;
};

#define SANREMO_AUDIO_OFFSET	0xB0

static const u8 sanremo_audio_regs[] = {
	0x00, 0x00, 0x10, 0x00,		      /*0xb0 ~ 0xb3*/
	0x00, 0x00, 0x00, 0x00,		      /*0xb4 ~ 0xb7*/
	0x00, 0x00, 0x00, 0x00,		      /*0xb8 ~ 0xbb*/
	0x00, 0x10, 0xc8, 0x00,		      /*0xbc ~ 0xbf*/
	0x00, 0x00, 0x00, 0x00,		      /*0xc0 ~ 0xc3*/
	0x00, 0x3f, 0x3f, 0x00,		      /*0xc4 ~ 0xc7*/
	0x3f, 0x3f, 0x00, 0x00,		      /*0xc8 ~ 0xcb*/
	0x00, 0x00, 0x00, 0x00,		      /*0xcc ~ 0xcf*/
	0x7e, 0x84, 0x1b, 0x11,		      /*0xd0 ~ 0xd3*/
	0x70, 0x4c, 0x4c, 0x00,		      /*0xd4 ~ 0xd7*/
	0x00, 0x34, 0x05, 0x00,		      /*0xd8 ~ 0xdb*/
	0x0b, 0x05, 0x3f, 0x23,		      /*0xdc ~ 0xdf*/
	0x00, 0x39, 0x00, 0x00,		      /*0xe0 ~ 0xe3*/
	0x00, 0x00, 0x00, 0x09,		      /*0xe4 ~ 0xe7*/
	0x01, 0x00, 0x00, 0x0a,		      /*0xe8 ~ 0xeb*/
	0x00, 0x00, 0x00, 0x00,		      /*0xec ~ 0xef*/
};

/*
 * read sanremo audio register cache
 */
static unsigned int sanremo_audio_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(sanremo_audio_regs)))
		return -EIO;

	return cache[reg];
}


/*
 * write to the sanremo audio register space
 */
static int sanremo_audio_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{

	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(sanremo_audio_regs)))
		return -EIO;
		
	sanremo_write(SANREMO_AUDIO_OFFSET +reg, value);
	cache[reg] = value;
			

	return 0;
}


/*
 * Sanremo audio codec reset 
 */
static int sanremo_audio_reset(struct snd_soc_codec *codec)
{
	unsigned int reg;

	for (reg = 0; reg < ARRAY_SIZE(sanremo_audio_regs); reg++) {
		if (reg != 0x2c)
			sanremo_audio_write(codec, reg, sanremo_audio_regs[reg]);
	}

	sanremo_write(0xda, 0x05);

	return 0;
}

/*
 * sanremo audio controls
 */


static int sanremo_audio_get_dai(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int sanremo_audio_set_dai(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

/* 
 * add non dapm controls 
 */
static int sanremo_audio_add_controls(struct snd_soc_codec *codec)
{
	return 0;
}


/*
 * DAMP controls
 */


/* 
 * add dapm controls
 */
static int sanremo_audio_add_widgets(struct snd_soc_codec *codec)
{
	return 0;
}



static int sanremo_audio_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	return 0;
}

static int sanremo_audio_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

/* set PCM DAI configuration */
static int sanremo_aduio_pcm_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int sanremo_audio_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

/* set HIFI DAI configuration */
static int sanremo_aduio_hifi_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int sanremo_audio_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	int rate, value;
	rate =  params_rate(params);
	value = sanremo_audio_read(codec, 0x0e);
	value &= 0xf0; 
	switch(rate){
		case 48000:
			value |= 0x08;
			break;
		case 44100:
			value |= 0x07;
			break;
		case 32000:
			value |= 0x06;
			break;
		case 24000:
			value |= 0x05;
			break;
		case 22050:
			value |= 0x04;
			break;
		case 16000:
			value |= 0x03;
			break;
		case 12000:
			value |= 0x02;
			break;
		case 11025:
			value |= 0x01;
			break;
		case 8000:
			value |= 0x00;
			break;
		default:
			printk(KERN_ERR "unsupported rate\n");
			return -EINVAL;
			break;
	}

	return 0;
}

static int sanremo_audio_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int sanremo_audio_set_bias_level(struct snd_soc_codec *codec,  enum snd_soc_bias_level level)
{
	switch (level) {
	        case SND_SOC_BIAS_ON: /* full On */
			sanremo_write(0xb2, 0x10);
			sanremo_write(0xde, 0x35);
			sanremo_write(0xeb, 0x0a);
			sanremo_write(0xda, 0x05);
			break;

		case SND_SOC_BIAS_PREPARE: /* partial On */
			break;

		case SND_SOC_BIAS_STANDBY: /* partial On */
			break;

		case SND_SOC_BIAS_OFF: /* Off, without power */
			break;
	}

	codec->bias_level = level;

	return 0;
}

#define SANREMO_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\ 
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define SANREMO_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)


/*
 * HIFI DAI
 */
struct snd_soc_dai sanremo_audio_dai[]={
/* DAI HIFI mode*/
	{
		.name = "sanremo audio HiFi",
		.id = 1,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.ops = {
			.hw_params = sanremo_audio_hifi_hw_params,
		},
		.dai_ops = {
			.digital_mute = sanremo_audio_mute,
			.set_fmt = sanremo_aduio_hifi_set_dai_fmt,
			.set_clkdiv = NULL,
			.set_pll = NULL,
			.set_sysclk = NULL,
		},
	},

	{
		.name = "sanremo audio pcm",
		.id = 1,
		.playback = {
			.stream_name = "Pcm Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "Pcm Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.ops = {
			.hw_params = sanremo_audio_pcm_hw_params,
		},
		.dai_ops = {
			.digital_mute = sanremo_audio_mute,
			.set_fmt = sanremo_aduio_pcm_set_dai_fmt,
			.set_clkdiv = NULL,
			.set_pll = NULL,
			.set_sysclk = NULL,
		},
	},
};

static void sanremo_audio_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
			container_of(work, struct snd_soc_codec, delayed_work.work);
	sanremo_audio_set_bias_level(codec, codec->bias_level);
}


static int sanremo_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}


static int sanremo_audio_resume(struct platform_device *pdev)
{
	return 0;
}

static int sanremo_audio_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int reg, ret = 0;

	codec->name = "sanremo audio";
	codec->owner = THIS_MODULE;
	codec->read = sanremo_audio_read;
	codec->write = sanremo_audio_write;
	codec->set_bias_level = sanremo_audio_set_bias_level;
	codec->dai = sanremo_audio_dai;
	codec->num_dai = ARRAY_SIZE(sanremo_audio_dai);
	codec->reg_cache_size = sizeof(sanremo_audio_regs);
	codec->reg_cache = kmemdup(sanremo_audio_regs, sizeof(sanremo_audio_regs), GFP_KERNEL);

	if(codec->reg_cache == NULL)
		return -ENOMEM;

	printk(KERN_ERR "sanremo_audio_init :power & pll init\n" );

	/* sanremo power&pll  init for audio codec */
	sanremo_write(0xdc, 0x00);
	mdelay(10);
	sanremo_write(0x43,0x18);
	mdelay(10);
	sanremo_write(0x44,0x12);
	mdelay(10);
	sanremo_write(0x45,0x83);
	mdelay(10);
	sanremo_write(0x42,0x38);
	mdelay(10);
	sanremo_write(0xdc, 0x0b);
	mdelay(10);
	

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "sanremo audio: failed to create pcms\n");
		goto pcm_err;
	}

	codec->bias_level = SND_SOC_BIAS_STANDBY;
	schedule_delayed_work(&codec->delayed_work,msecs_to_jiffies(2));

	printk(KERN_ERR "sanremo_audio_init :register pcms\n" );
	sanremo_audio_reset(codec);
	
	sanremo_audio_add_controls(codec);
	sanremo_audio_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0)
	{
		printk(KERN_ERR "sanremo audio: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static struct snd_soc_device *sanremo_audio_socdev;


static int sanremo_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct _setup_data *setup;
	struct snd_soc_codec *codec;
	struct sanremo_audio_priv *sanremo_audio;
	int ret = 0;

	printk(KERN_INFO "Sanremo Audio Codec %s", SANREMO_AUDIO_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	sanremo_audio = kzalloc(sizeof(struct sanremo_audio_priv), GFP_KERNEL);
	if (sanremo_audio == NULL) {
		kfree(codec);
		return -ENOMEM;
	}


	codec->private_data = sanremo_audio;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	sanremo_audio_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, sanremo_audio_work);

	sanremo_audio_init(socdev);

	return ret;
}


/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int sanremo_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		sanremo_audio_set_bias_level(codec, SND_SOC_BIAS_OFF);

	run_delayed_work(&codec->delayed_work);
	
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	
	kfree(codec->private_data);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sanremo_audio = {
	.probe = 	sanremo_audio_probe,
	.remove = 	sanremo_audio_remove,
	.suspend = 	sanremo_audio_suspend,
	.resume =	sanremo_audio_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_sanremo_audio);

MODULE_DESCRIPTION("ASoC Sanremo audio driver");
MODULE_AUTHOR("bshen9@marvell.com");
MODULE_LICENSE("GPL");


