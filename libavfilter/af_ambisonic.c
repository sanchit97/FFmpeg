/*
 * Copyright (c) 2017 Sanchit Sinha
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
    shelf,
    phaseshift
};

typedef struct Cache {
    double i1, i2;
    double o1, o2;
} Cache;

enum Layouts2D {
    MONO,
    STEREO,
    TRIANGLE,
    SQUARE,
    PENTAGON,
    HEXAGON,
    HEPTAGON,
    OCTAGON
};

enum Layouts3D {
    CUBE=8,
    DODECAHEDRON=20,

};

static const struct
{
    float matrix[22][9];
} ambisonic_matrix2d[]={
        [MONO]={
            .matrix={
                {0.22156, 0, 0, 0},
            },
        },
        [TRIANGLE]={
            .matrix={
                {0.17836, 0.32555, 0.18795},
                {0.17836, 0      ,-0.37591},
                {0.17836,-0.32555, 0.18795},
            },
        },
        [SQUARE]={
            .matrix={
                {0.39388, 0.18690, 0.18690, 0},
                {0.39388,-0.18690, 0.18690, 0},
                {0.39388,-0.18690,-0.18690, 0},
                {0.39388, 0.18690,-0.18690, 0},
            },
        },
        [PENTAGON]={
            .matrix={
                {0.20195, 0      , 0.33420, 0},
                {0.11356, 0.2901 , 0.04186, 0},
                {0.19654,-0.07993,-0.34782, 0},
                {0.19654, 0.07993,-0.34782, 0},
                {0.19654,-0.2901 , 0.04186, 0},
            },
        },
        [HEXAGON]={
            .matrix={
                {0.26259, 0      ,  0.31326, 0},
                {0.26259, 0.27129,  0.15663, 0},
                {0.26259, 0.27129, -0.15663, 0},
                {0.26259, 0      , -0.31326, 0},
                {0.26259,-0.27129, -0.15663, 0},
                {0.26259,-0.27129,  0.15663, 0},
            },
        },
        [HEPTAGON]={
            .matrix={
                {0.22501,-0.0    ,  0.26846, 0},
                {0.22507, 0.20989,  0.16741, 0},
                {0.22507, 0.26180, -0.05969, 0},
                {0.22511, 0.11651, -0.24195, 0},
                {0.22511,-0.11651, -0.24195, 0},
                {0.22507,-0.26180, -0.05969, 0},
                {0.22507,-0.20989,  0.16741, 0},
            },
        },
        [OCTAGON]={
            .matrix={
                {0.19694,  0.08991,  0.21706, 0},
                {0.19694,  0.21706,  0.08991, 0},
                {0.19694,  0.21706, -0.08991, 0},
                {0.19694,  0.08991, -0.21706, 0},
                {0.19694, -0.08991, -0.21706, 0},
                {0.19694, -0.21706, -0.08991, 0},
                {0.19694, -0.21706,  0.08991, 0},
                {0.19694, -0.08991,  0.21706, 0},
            },
        },
    };

typedef struct AmbisonicContext {
    const AVClass *class;
    enum FilterType filter_type;
    int dimension;
    int nb_sp;
    int nb_channels;
    int order;
    int enable_shelf;
    double gain;
    double frequency;
    double width;
    uint64_t channels;
    double a0, a1, a2;
    double b0, b1, b2;
    Cache *cache;
    float decode_matrix[22][9];


    void (*filter)(struct AmbisonicContext *s, const void *ibuf, void *obuf, int len,
                   double *i1, double *i2, double *o1, double *o2,
                   double b0, double b1, double b2, double a1, double a2, double min, double max, int channelno);

} AmbisonicContext;

#define OFFSET(x) offsetof(AmbisonicContext,x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ambisonic_options[] = {
    {"dimension","Set 2D or 3D layout", OFFSET(dimension), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 3, FLAGS},
    {"d","Set 2D or 3D layout", OFFSET(dimension), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 3, FLAGS},
    {"speakers","Set number of speakers(regular)", OFFSET(nb_sp), AV_OPT_TYPE_INT, {.i64=4}, 1, 10, FLAGS},
    {"s","Set number of speakers(regular)", OFFSET(nb_sp), AV_OPT_TYPE_INT, {.i64=4}, 1, 10, FLAGS},
    {"enable_shelf","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {"e_s","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {NULL}
};


static int query_formats(AVFilterContext *ctx)
{
    AmbisonicContext *s     = ctx->priv;
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts = NULL;
    int temp;
    int ret;
    int i,j;

    memset(s->decode_matrix,0,22*9);

    //will be changed
    s->order=1;//first order ambisonics
    switch(s->dimension)
    {
        case 2:
        s->nb_channels=2*s->order+1;
        break;
        case 3:
        s->nb_channels=(s->order+1)*(s->order+1);
        break;
        default:
        s->nb_channels=2*s->order+1;
    }

    switch(s->dimension)
    {
        case 2:
        switch(s->nb_sp)
        {
            case 4:  temp=AV_CH_LAYOUT_4POINT0;
            break;
            case 5:  temp=AV_CH_LAYOUT_5POINT0;
            break;
            case 6:  temp=AV_CH_LAYOUT_6POINT0;
            break;
            case 7:  temp=AV_CH_LAYOUT_7POINT0;
            break;
            case 8:  temp=AV_CH_LAYOUT_OCTAGONAL;
            break;
            // case 16: temp=AV_CH_LAYOUT_HEXADECAGONAL;
            // break;
            default: temp=AV_CH_LAYOUT_4POINT0;
        }
        break;

        case 3:
        break;
    }

    ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    if (ret)
        return ret;
    ret = ff_set_common_formats(ctx, formats);
    if (ret)
        return ret;


    ret = ff_add_channel_layout(&layouts, temp);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(layouts, &ctx->outputs[0]->in_channel_layouts);
    if (ret)
        return ret;

    layouts = NULL;

    ret = ff_add_channel_layout(&layouts, AV_CH_LAYOUT_4POINT0);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(layouts, &ctx->inputs[0]->out_channel_layouts);
    if (ret)
        return ret;


    formats = ff_all_samplerates();
    if(!formats)
        return AVERROR(ENOMEM);
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

static float multiply(float decode_matrix[22][9],int row, float *vars[22], int sample_no, int nb_channels)
{
    int j;
    float sum=0;

    for(j=0;j<nb_channels;j++)
    {
        sum+=(decode_matrix[row][j]*vars[j][sample_no]);
    }
    return sum;
}

static void configure_matrix(AmbisonicContext *s)
{
    int channels=s->nb_channels;
    int speakers=s->nb_sp;
    int dimension=s->dimension;

    // switch(s->dimension)
    // {
    //     case 2:
    //     switch(s->nb_sp)
    //     {
    //         case 1:
    //             s->decode_matrix[0][0]=0.22156;
    //             s->decode_matrix[1][0]=0.22156;
    //             s->decode_matrix[2][0]=0.22156;
    //             s->decode_matrix[3][0]=0.22156;
    //         break;
    //         case 3:
    //             s->decode_matrix[0][0]=0.17836;s->decode_matrix[0][1]=0.32555;s->decode_matrix[0][2]=0.18795;
    //             s->decode_matrix[1][0]=0.17836;s->decode_matrix[1][1]=0.0;s->decode_matrix[1][2]=-0.37591;
    //             s->decode_matrix[2][0]=0.17836;s->decode_matrix[2][1]=-0.32555;s->decode_matrix[2][2]=0.18795;
    //         break;
    //         case 4:
    //             s->decode_matrix[0][0]=0.39388;s->decode_matrix[0][1]=0.18690;s->decode_matrix[0][2]=0.18690;
    //             s->decode_matrix[1][0]=0.39388;s->decode_matrix[1][1]=(-1)*0.18690;s->decode_matrix[1][2]=0.18690;
    //             s->decode_matrix[2][0]=0.39388;s->decode_matrix[2][1]=(-1)*0.18690;s->decode_matrix[2][2]=(-1)*0.18690;
    //             s->decode_matrix[3][0]=0.39388;s->decode_matrix[3][1]=0.18690;s->decode_matrix[3][2]=(-1)*0.18690;
    //         break;
    //         case 5:
    //             s->decode_matrix[0][0]=0.20195;s->decode_matrix[0][1]=0;s->decode_matrix[0][2]=        0.33420;
    //             s->decode_matrix[1][0]=0.11356;s->decode_matrix[1][1]=0.2901;s->decode_matrix[1][2]=   0.04186;
    //             s->decode_matrix[2][0]=0.19654;s->decode_matrix[2][1]=-0.07993;s->decode_matrix[2][2]=-0.34782;
    //             s->decode_matrix[3][0]=0.19654;s->decode_matrix[3][1]=0.07993;s->decode_matrix[3][2]= -0.34782;
    //             s->decode_matrix[4][0]=0.19654;s->decode_matrix[4][1]=-0.2901;s->decode_matrix[4][2]=  0.04186;
    //         break;
    //         case 6:
    //             s->decode_matrix[0][0]=0.26259;s->decode_matrix[0][1]=0       ;s->decode_matrix[0][2]= 0.31326;
    //             s->decode_matrix[1][0]=0.26259;s->decode_matrix[1][1]=0.27129 ;s->decode_matrix[1][2]= 0.15663;
    //             s->decode_matrix[2][0]=0.26259;s->decode_matrix[2][1]=0.27129 ;s->decode_matrix[2][2]=-0.15663;
    //             s->decode_matrix[3][0]=0.26259;s->decode_matrix[3][1]=0       ;s->decode_matrix[3][2]=-0.31326;
    //             s->decode_matrix[4][0]=0.26259;s->decode_matrix[4][1]=-0.27129;s->decode_matrix[4][2]=-0.15663;
    //             s->decode_matrix[5][0]=0.26259;s->decode_matrix[5][1]=-0.27129;s->decode_matrix[5][2]= 0.15663;
    //         break;
    //         case 7:
    //             s->decode_matrix[0][0]=0.22501;s->decode_matrix[0][1]= -0.0    ;s->decode_matrix[0][2]= 0.26846;
    //             s->decode_matrix[1][0]=0.22507;s->decode_matrix[1][1]= 0.20989 ;s->decode_matrix[1][2]= 0.16741;
    //             s->decode_matrix[2][0]=0.22507;s->decode_matrix[2][1]= 0.26180 ;s->decode_matrix[2][2]=-0.05969;
    //             s->decode_matrix[3][0]=0.22511;s->decode_matrix[3][1]= 0.11651 ;s->decode_matrix[3][2]=-0.24195;
    //             s->decode_matrix[4][0]=0.22511;s->decode_matrix[4][1]= -0.11651;s->decode_matrix[4][2]=-0.24195;
    //             s->decode_matrix[5][0]=0.22507;s->decode_matrix[5][1]= -0.26180;s->decode_matrix[5][2]=-0.05969;
    //             s->decode_matrix[6][0]=0.22507;s->decode_matrix[6][1]= -0.20989;s->decode_matrix[6][2]= 0.16741;
    //         break;
    //         case 8:
    //             s->decode_matrix[0][0]=0.19694;s->decode_matrix[0][1]= 0.08991;s->decode_matrix[0][2]= 0.21706;
    //             s->decode_matrix[1][0]=0.19694;s->decode_matrix[1][1]= 0.21706;s->decode_matrix[1][2]= 0.08991;
    //             s->decode_matrix[2][0]=0.19694;s->decode_matrix[2][1]= 0.21706;s->decode_matrix[2][2]=-0.08991;
    //             s->decode_matrix[3][0]=0.19694;s->decode_matrix[3][1]= 0.08991;s->decode_matrix[3][2]=-0.21706;
    //             s->decode_matrix[4][0]=0.19694;s->decode_matrix[4][1]=-0.08991;s->decode_matrix[4][2]=-0.21706;
    //             s->decode_matrix[5][0]=0.19694;s->decode_matrix[5][1]=-0.21706;s->decode_matrix[5][2]=-0.08991;
    //             s->decode_matrix[6][0]=0.19694;s->decode_matrix[6][1]=-0.21706;s->decode_matrix[6][2]= 0.08991;
    //             s->decode_matrix[7][0]=0.19694;s->decode_matrix[7][1]=-0.08991;s->decode_matrix[7][2]= 0.21706;
    //         break;
    //     }
    //     break;
    //     case 3:
    //     switch(s->nb_sp)
    //     {
    //         // case
    //     }
    //     break;
    // }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AmbisonicContext *s=ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out_buf;
    int itr;
    float *vars[9];
    float calc[22]={0};
    float *c[22];
    int i;

    out_buf = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out_buf){
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out_buf, in);

    s->filter = shelf_flt;

    if(s->enable_shelf)
    {
        // shelf filter
        // for w channel gain= 1.75
        s->filter(s, in->extended_data[0],
                      out_buf->extended_data[0], in->nb_samples,
                      &s->cache[0].i1, &s->cache[0].i2,
                      &s->cache[0].o1, &s->cache[0].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,1);
        //for x & y channel gain= -1.26
        s->filter(s, in->extended_data[1],
                      out_buf->extended_data[1], in->nb_samples,
                      &s->cache[1].i1, &s->cache[1].i2,
                      &s->cache[1].o1, &s->cache[1].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,2);

        s->filter(s, in->extended_data[2],
                      out_buf->extended_data[2], in->nb_samples,
                      &s->cache[2].i1, &s->cache[2].i2,
                      &s->cache[2].o1, &s->cache[2].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,3);
    }


    // configure_matrix(s);

    for(i=0;i<s->nb_channels;i++)
    {
        vars[i]=(float*)in->extended_data[i];
    }

    for(i=0;i<s->nb_sp;i++)
    {
        c[i]=(float *)out_buf->extended_data[i];
    }

    for(itr=0;itr<in->nb_samples;itr++)
    {
        for(i=0;i<s->nb_sp;i++)
        {
            if(s->dimension==2){
                calc[i]=multiply(ambisonic_matrix2d[s->nb_sp].matrix,i,vars,itr,s->nb_channels);
            } else {
                // calc[i]=multiply(ambisonic_matrix3d[s->nb_sp].matrix,i,vars,itr,s->nb_channels);
            }
        }

        for(i=0;i<s->nb_sp;i++)
        {
            c[i][itr]=calc[i];
        }
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
