/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdio.h>
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"
#include <math.h>

typedef struct AmbisonicContext {
	const AVClass *class;
	/*Not needed, if any new variables are to be used, this struct can be populated(f)*/

} AmbisonicContext;

#define OFFSET(x) offsetof(AmbisonicContext, x)

static const AVOption ambisonic_options[] = {  //square will be an option
{NULL}
};

AVFILTER_DEFINE_CLASS(ambisonic);
static int query_formats(AVFilterContext *ctx)
{
	AVFilterFormats *formats = NULL;
	AVFilterChannelLayouts *layout = NULL;
	int ret;
	if ((ret = ff_add_format     (&formats, AV_SAMPLE_FMT_FLTP   )) < 0 ||
	(ret = ff_set_common_formats     (ctx    , formats )) < 0 ||
	(ret = ff_add_channel_layout     (&layout , AV_CH_LAYOUT_4POINT0)) < 0 ||
	(ret = ff_set_common_channel_layouts (ctx    , layout   )) < 0)
	return ret;
	formats = ff_all_samplerates();
	return ff_set_common_samplerates(ctx, formats);
}
static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
	AVFilterContext *ctx = inlink->dst;
	AVFilterLink *outlink = ctx->outputs[0];
	/*If variables used, this has to be created*/
	// AmbisonicContext *s = ctx->priv;
	// const float *src = (const float *)in->data[0];
	// float *dst;
	AVFrame *out;
	int itr;
	float *w,*x,*y;
	float root8= sqrt(8);
	float lf=0,lb=0,rb=0,rf=0;

	if (av_frame_is_writable(in))
	{
		out = in;
	}
	else
	{
		out = ff_get_audio_buffer(inlink, in->nb_samples);
		if (!out)
		{
			av_frame_free(&in);
			return AVERROR(ENOMEM);
		}
		av_frame_copy_props(out, in);
	}

	/*If planar samples are used, output must be put in dst*/
	//dst = (float *)out->data[0];

	w=(float *)in->extended_data[0];
	x=(float *)in->extended_data[1];
	y=(float *)in->extended_data[2];

	for(itr=0;itr<in->nb_samples;itr++)
	{
		/*Float pointers to the samples*/

		lf = root8 * (2*(w[itr])+x[itr]+y[itr]);
		lb = root8 * (2*(w[itr])-x[itr]+y[itr]);
		rb = root8 * (2*(w[itr])-x[itr]-y[itr]);
		rf = root8 * (2*(w[itr])+x[itr]-y[itr]);

		out->extended_data[0][itr]= lf;
		out->extended_data[1][itr]= lb;
		out->extended_data[2][itr]= rb;
		out->extended_data[3][itr]= rf;
	}

	if (out != in)
	av_frame_free(&in);
	return ff_filter_frame(outlink, out);
}

static const AVFilterPad inputs[] = {
	{
	.name    = "default",
	.type    = AVMEDIA_TYPE_AUDIO,
	.filter_frame = filter_frame,
	// .config_props = config_input,
	},
	{ NULL }
};

static const AVFilterPad outputs[] = {
	{
	.name = "default",
	.type = AVMEDIA_TYPE_AUDIO,
	},
	{ NULL }
};

AVFilter ff_af_ambisonic = {
	.name      = "ambisonic",
	.description    = NULL_IF_CONFIG_SMALL("An ambisonic filter"),
	.query_formats  = query_formats,
	.priv_size    = sizeof(AmbisonicContext),
	.priv_class  = &ambisonic_class,
	// .uninit   = uninit,
	.inputs  = inputs,
	.outputs    = outputs,
};