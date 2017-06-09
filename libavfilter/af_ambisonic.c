/*
 * Copyright (c) 2017 Sanchit Sinha
 * Copyright (c) 2017 Paul B Mahol
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
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include <math.h>
#include <stdio.h>

enum FilterType {
    shelf
};

typedef struct Cache {
    double i1, i2;
    double o1, o2;
} Cache;


typedef struct AmbisonicContext {
    const AVClass *class;
    enum FilterType filter_type;
    double gain;
    double frequency;
    double width;
    uint64_t channels;
    double a0, a1, a2;
    double b0, b1, b2;
    Cache *cache;

    void (*filter)(struct AmbisonicContext *s, const void *ibuf, void *obuf, int len,
                   double *i1, double *i2, double *o1, double *o2,
                   double b0, double b1, double b2, double a1, double a2, double min, double max, int channelno);

} AmbisonicContext;

static const AVOption ambisonic_options[] = {
    {NULL}
};

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layout = NULL;
    int ret;

    if ((ret = ff_add_format        (&formats, AV_SAMPLE_FMT_FLTP   )) < 0 ||
        (ret = ff_set_common_formats              (ctx    , formats )) < 0 ||
        (ret = ff_add_channel_layout (&layout , AV_CH_LAYOUT_MONO)) < 0 ||
        (ret = ff_set_common_channel_layouts     (ctx    , layout   )) < 0)
        return ret;

    formats = ff_all_samplerates();
    return ff_set_common_samplerates(ctx, formats);
}

static void shelf_flt(      AmbisonicContext *s,
                            const void *input, void *output, int len,
                            double *in1, double *in2,
                            double *out1, double *out2,
                            double b0, double b1, double b2,
                            double a1, double a2, double min, double max, int channelno)
{

    const float *ibuf = input;
    float *obuf = output;
    double i1 = *in1;
    double i2 = *in2;
    double o1 = *out1;
    double o2 = *out2;
    int i;
    a1 = -a1;
    a2 = -a2;

    switch(channelno){
        case 1: s->gain= 1.75;
                break;
        case 2: s->gain= -1.26;
                break;
        case 3: s->gain= -1.26;
                break;
        default: s->gain= 1;
    }

    for (i = 0; i+1 < len; i++) {
        o2 = i2 * b2 + i1 * b1 + ibuf[i] * b0 + o2 * a2 + o1 * a1;
        i2 = ibuf[i];
        if (o2 < min) {
            obuf[i] = min;
        } else if (o2 > max) {
            obuf[i] = max;
        } else {
            obuf[i] = o2;
        }
        i++;
        o1 = i1 * b2 + i2 * b1 + ibuf[i] * b0 + o1 * a2 + o2 * a1;
        i1 = ibuf[i];
        if (o1 < min) {
            obuf[i] = min;
        } else if (o1 > max) {
            obuf[i] = max;
        } else {
            obuf[i] = o1;
        }
    }
    if (i < len) {
        double o0 = ibuf[i] * b0 + i1 * b1 + i2 * b2 + o1 * a1 + o2 * a2;
        i2 = i1;
        i1 = ibuf[i];
        o2 = o1;
        o1 = o0;
        if (o0 < min) {
            obuf[i] = min;
        } else if (o0 > max) {
            obuf[i] = max;
        } else {
            obuf[i] = o0;
        }
    }
    *in1  = i1;
    *in2  = i2;
    *out1 = o1;
    *out2 = o2;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx    = outlink->src;
    AmbisonicContext *s     = ctx->priv;
    AVFilterLink *inlink    = ctx->inputs[0];
    double alpha,A,w0;
    s->frequency=700;
    s->filter_type=shelf;
    s->width=10;
    A = exp(s->gain / 40 * log(10.));
    w0 = 2 * M_PI * s->frequency / inlink->sample_rate;
    alpha=sin(w0) / (2 * s->frequency / s->width);

    switch (s->filter_type) {
    case shelf:
        s->a0 =          (A + 1) + (A - 1) * cos(w0) + 2 * sqrt(A) * alpha;
        s->a1 =    -2 * ((A - 1) + (A + 1) * cos(w0));
        s->a2 =          (A + 1) + (A - 1) * cos(w0) - 2 * sqrt(A) * alpha;
        s->b0 =     A * ((A + 1) - (A - 1) * cos(w0) + 2 * sqrt(A) * alpha);
        s->b1 = 2 * A * ((A - 1) - (A + 1) * cos(w0));
        s->b2 =     A * ((A + 1) - (A - 1) * cos(w0) - 2 * sqrt(A) * alpha);
        break;
    default:
        av_assert0(0);
    }
    s->a1 /= s->a0;
    s->a2 /= s->a0;
    s->b0 /= s->a0;
    s->b1 /= s->a0;
    s->b2 /= s->a0;

    s->cache = av_realloc_f(s->cache, sizeof(Cache), inlink->channels);
    if (!s->cache)
        return AVERROR(ENOMEM);

    memset(s->cache, 0, sizeof(Cache) * inlink->channels);


    return 0;
}


static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AmbisonicContext *s=ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out_buf;
    int itr;
    float *w,*x,*y,*c1,*c2,*c3,*c4;
    float root8 = sqrt(8);
    float lf=0,lb=0,rb=0,rf=0;

    if (av_frame_is_writable(in)) {
        out_buf = in;
    } else {
        out_buf = ff_get_audio_buffer(inlink, in->nb_samples);
        if (!out_buf){
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out_buf, in);
    }

    s->filter = shelf_flt;

    for w channel gain= 1.75
    s->filter(s, in->extended_data[0],
                  out_buf->extended_data[0], in->nb_samples,
                  &s->cache[0].i1, &s->cache[0].i2,
                  &s->cache[0].o1, &s->cache[0].o2,
                  s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,1);
    for x & y channel gain= -1.26
    s->filter(s, in->extended_data[1],
                  out_buf->extended_data[1], in->nb_samples,
                  &s->cache[0].i1, &s->cache[0].i2,
                  &s->cache[0].o1, &s->cache[0].o2,
                  s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,2);

    s->filter(s, in->extended_data[2],
                  out_buf->extended_data[2], in->nb_samples,
                  &s->cache[0].i1, &s->cache[0].i2,
                  &s->cache[0].o1, &s->cache[0].o2,
                  s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,3);

    w=(float *)in->extended_data[0];
    x=(float *)in->extended_data[1];
    y=(float *)in->extended_data[2];
    c1=(float *)out_buf->extended_data[0];
    c2=(float *)out_buf->extended_data[1];
    c3=(float *)out_buf->extended_data[2];
    c4=(float *)out_buf->extended_data[3];

    for(itr=0;itr<in->nb_samples;itr++){
        lf = root8 * (2*(w[itr])+x[itr]+y[itr]);
        lb = root8 * (2*(w[itr])-x[itr]+y[itr]);
        rb = root8 * (2*(w[itr])-x[itr]-y[itr]);
        rf = root8 * (2*(w[itr])+x[itr]-y[itr]);
        c1[itr] = lf;
        c2[itr] = rf;
        c3[itr] = lb;
        c4[itr] = rb;
    }

    if (out_buf != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out_buf);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
    { NULL }
};
static const AVFilterPad outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
    { NULL }
};

AVFILTER_DEFINE_CLASS(ambisonic);

AVFilter ff_af_ambisonic = {
    .name           = "ambisonic",
    .description    = NULL_IF_CONFIG_SMALL("An ambisonic filter"),
    .query_formats  = query_formats,
    .priv_size      = sizeof(AmbisonicContext),
    .priv_class     = &ambisonic_class,
    .inputs         = inputs,
    .outputs        = outputs,
};
