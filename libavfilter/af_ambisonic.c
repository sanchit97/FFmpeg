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
    shelf
};

enum InputFormat {
	N3D    =1,
	SN3D   =2,
	FURMUL =3
};

enum Layouts {
    MONO        ,
    STEREO      ,
    TRIANGLE    ,
    SQUARE      ,
    PENTAGON    ,
    HEXAGON     ,
    HEPTAGON    ,
    OCTAGON    	,
	TETRAHEDRON ,
	OCTAHEDRON  ,
    CUBE    	,
    DODECAHEDRON,
    ICOSAHEDRON
};

typedef struct Cache {
    double i1, i2;
    double o1, o2;
} Cache;

static const struct {
	int speakers;
    float matrix[22][15];
} ambisonic_matrix[]= {
    [MONO]={
    	.speakers=1,
        .matrix={
            {0.22156, 0, 0, 0},
        },
    },
    [TRIANGLE]={
    	.speakers=3,
        .matrix={
            {0.17836, 0.32555, 0.18795},
            {0.17836, 0      ,-0.37591},
            {0.17836,-0.32555, 0.18795},
        },
    },
    [SQUARE]={
    	.speakers=4,
        .matrix={
            {0.39388, 0.18690, 0.18690, 0},
            {0.39388,-0.18690, 0.18690, 0},
            {0.39388,-0.18690,-0.18690, 0},
            {0.39388, 0.18690,-0.18690, 0},
        },
    },
    [PENTAGON]={
    	.speakers=5,
        .matrix={
            {0.20195, 0      , 0.33420, 0},
            {0.11356, 0.2901 , 0.04186, 0},
            {0.19654,-0.07993,-0.34782, 0},
            {0.19654, 0.07993,-0.34782, 0},
            {0.19654,-0.2901 , 0.04186, 0},
        },
    },
    [HEXAGON]={
    	.speakers=6,
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
    	.speakers=7,
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
    	.speakers=8,
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
    [OCTAHEDRON]={
    	.speakers=6,
        .matrix={
            {0.45832,  0.41566,  0.00000,  0.13183},
   			{0.95964,  0.41566,  0.00000, -0.36696},
   			{0.45832, -0.41566,  0.00000,  0.13183},
   			{0.95964, -0.41566,  0.00000, -0.36696},
   			{0.35449,  0.00000,  0.00000,  0.23513},
   			{0.35449,  0.00000,  0.00000,  0.23513},
        },
    },
    [CUBE]={
    	.speakers=8,
        .matrix={
            {0.14269, 0.13909,  0.28611, 0.13909},
            {0.14269, 0.13909, -0.28611, 0.13909},
            {0.14269,-0.13909,  0.28611, 0.13909},
            {0.14269,-0.13909, -0.28611, 0.13909},
            {0.14269, 0.13909,  0.28611,-0.13909},
            {0.14269, 0.13909, -0.28611,-0.13909},
            {0.14269,-0.13909,  0.28611,-0.13909},
            {0.14269,-0.13909, -0.28611,-0.13909},
        },
    },
    [ICOSAHEDRON]={
    	.speakers=12,
        .matrix={
            {0.18245, -1.6727e-34,  2.0067e-17,  2.2478e-01},
   			{0.18245,  5.8352e-35,  4.8797e-18,  2.2478e-01},
   			{0.21854, -2.5706e-18,  1.7303e-01,  1.5296e-01},
   			{0.28727,  3.0478e-01,  1.7303e-01,  1.6203e-02},
   			{0.28727, -3.0478e-01,  1.7303e-01,  1.6203e-02},
   			{0.39847,  1.8836e-01,  1.7303e-01, -2.0508e-01},
   			{0.39847, -1.8836e-01,  1.7303e-01, -2.0508e-01},
   			{0.21854,  2.5706e-18, -1.7303e-01,  1.5296e-01},
   			{0.28727,  3.0478e-01, -1.7303e-01,  1.6203e-02},
   			{0.28727, -3.0478e-01, -1.7303e-01,  1.6203e-02},
   			{0.39847,  1.8836e-01, -1.7303e-01, -2.0508e-01},
   			{0.39847, -1.8836e-01, -1.7303e-01, -2.0508e-01},
        },
    },
    [DODECAHEDRON]={
    	.speakers=20,
        .matrix={
            {1.7725e-01,  1.0721e-16, -3.6730e-17,  1.4416e-01},
            {1.7725e-01,  8.4733e-02,  4.8084e-19,  1.1662e-01},
            {1.7725e-01,  1.3710e-01, -3.8973e-18,  4.4547e-02},
            {1.7725e-01,  1.3710e-01, -1.9760e-18, -4.4547e-02},
            {1.7725e-01,  8.4733e-02,  3.0706e-20, -1.1662e-01},
            {1.7725e-01, -1.0415e-16, -1.5089e-19, -1.4416e-01},
            {1.7725e-01, -8.4733e-02, -1.1427e-17,  1.1662e-01},
            {1.7725e-01, -1.3710e-01, -9.4207e-18,  4.4547e-02},
            {1.7725e-01, -1.3710e-01, -5.9922e-18, -4.4547e-02},
            {1.7725e-01, -8.4733e-02, -2.4514e-18, -1.1662e-01},
            {1.7725e-01,  3.5190e-17,  1.9356e-01,  1.1452e-01},
            {1.7725e-01,  1.0891e-01,  1.9356e-01,  3.5389e-02},
            {1.7725e-01,  6.7313e-02,  1.9356e-01, -9.2648e-02},
            {1.7725e-01, -1.0891e-01,  1.9356e-01,  3.5389e-02},
            {1.7725e-01, -6.7313e-02,  1.9356e-01, -9.2648e-02},
            {1.7725e-01,  6.7313e-02, -1.9356e-01,  9.2648e-02},
            {1.7725e-01,  1.0891e-01, -1.9356e-01, -3.5389e-02},
            {1.7725e-01, -4.0103e-17, -1.9356e-01, -1.1452e-01},
            {1.7725e-01, -1.0891e-01, -1.9356e-01, -3.5389e-02},
            {1.7725e-01, -6.7313e-02, -1.9356e-01,  9.2648e-02},
        },
    },
};

static const struct {
    float matrix[4][1];
} scaler_matrix[]= {
    [FURMUL]={
        .matrix={
            {sqrt(2)},
            {sqrt(3)},
            {sqrt(3)},
            {sqrt(3)},
        },
    },
    [SN3D]={
        .matrix={
            {sqrt(2*floor(sqrt(0))+1)},
            {sqrt(2*floor(sqrt(1))+1)},
            {sqrt(2*floor(sqrt(2))+1)},
            {sqrt(2*floor(sqrt(3))+1)},
        },
    },
    [N3D]={
        .matrix={
            {1},
            {1},
            {1},
            {1},
        },
    },
};

typedef struct AmbisonicContext {
    const AVClass *class;
    enum FilterType filter_type;
    int scaler;
    int dimension;
    enum Layouts lyt;
    int nb_channels;
    int order;
    int enable_shelf;
    int enable_nf;
    double gain;
    double frequency;
    double width;
    uint64_t channels;
    double a0, a1, a2;
    double b0, b1, b2;
    Cache *cache;
    float decode_matrix[22][9];
    char* sp_layout;
    char* s_o;


    void (*filter)(struct AmbisonicContext *s, const void *ibuf, void *obuf, int len,
                   double *i1, double *i2, double *o1, double *o2,
                   double b0, double b1, double b2, double a1, double a2, double min, double max, int channelno);

} AmbisonicContext;

#define OFFSET(x) offsetof(AmbisonicContext,x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ambisonic_options[] = {
    {"enable_shelf","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {"e_s","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {"enable_nearfield","Set if Near Field Compensation is required",OFFSET(enable_nf), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, FLAGS},
    {"e_nf","Set if Near Field Compensation is required",OFFSET(enable_nf), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, FLAGS},
    {"output_layout","Enter Layout of output",OFFSET(lyt), AV_OPT_TYPE_INT, {.i64=SQUARE}, MONO, DODECAHEDRON, FLAGS,"lyt"},
    {"o_l","Enter Layout of output",OFFSET(lyt), AV_OPT_TYPE_INT, {.i64=SQUARE}, MONO, DODECAHEDRON, FLAGS,"lyt"},
    {"scaling_option","Set the input format (N3D, SN3D, Furse Malham)",OFFSET(s_o), AV_OPT_TYPE_STRING, {.str="n3d"}, 0, 0, FLAGS},
    {"s_o","Set the input format (N3D, SN3D, Furse Malham)",OFFSET(s_o), AV_OPT_TYPE_STRING, {.str="n3d"}, 0, 0, FLAGS},
    {NULL}
};

static int intval_scaling(char* so)
{
	if(strcmp(so,"n3d")==0 || strcmp(so,"N3D")==0) {
		return 1;
	} else if(strcmp(so,"sn3d")==0 || strcmp(so,"SN3D")==0) {
		return 2;
	} else if(strcmp(so,"fm")==0 || strcmp(so,"FM")==0){
		return 3;
	} else {
		return 1;
	}
}

static int query_formats(AVFilterContext *ctx)
{
    AmbisonicContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts = NULL;
    uint64_t temp;
    int ret;

    s->scaler=intval_scaling(s->s_o);

    printf("LAYOUT:::%d",s->lyt);
    temp=av_get_channel_layout("6.0");
    // if(strcmp(s->sp_layout,"octahedron")==0) {
    // 	temp=av_get_channel_layout("6.0");
    // 	s->lyt=OCTAHEDRON;
    // } else if(strcmp(s->sp_layout,"cube")==0) {
    // 	temp=av_get_channel_layout("octagonal");
    // 	s->lyt=CUBE;
    // } else if(strcmp(s->sp_layout,"dodecahedron")==0) {
    // 	temp=av_get_channel_layout("");
    // 	s->lyt=DODECAHEDRON;
    // } else if(strcmp(s->sp_layout,"icosahedron")==0) {
    // 	temp=av_get_channel_layout("");
    // 	s->lyt=ICOSAHEDRON;
    // } else {
    // 	temp=av_get_channel_layout(s->sp_layout);
    // 	switch(av_get_channel_layout_nb_channels(temp)) {
    // 		case 1:  s->lyt = MONO;      break;
    // 		case 2:  s->lyt = STEREO;    break;
    // 		case 3:  s->lyt = TRIANGLE;  break;
    // 		case 4:  s->lyt = SQUARE;    break;
    // 		case 5:  s->lyt = PENTAGON;  break;
    // 		case 6:  s->lyt = HEXAGON;   break;
    // 		case 7:  s->lyt = HEPTAGON;  break;
    // 		case 8:  s->lyt = OCTAGON;   break;
    // 		default: s->lyt = SQUARE;
    // 	}
    // }

    memset(s->decode_matrix,0,22*9);

    s->order=1;//first order ambisonics

    // if(strcmp(s->sp_layout,"cube")        ==0 ||
    //    strcmp(s->sp_layout,"icosahedron") ==0 ||
    //    strcmp(s->sp_layout,"dodecahedron")==0 ||
    //    strcmp(s->sp_layout,"tetrahedron") ==0  ) {
    // 	s->dimension=3;
    // } else {
    // 	s->dimension=2;
    // }

    // printf("input:%d",av_get_channel_layout_nb_channels(av_get_channel_layout("mono")));
    s->dimension=2;
    switch(s->dimension) {
        case 2:  s->nb_channels=2*s->order+1;                break;
        case 3:  s->nb_channels=(s->order+1)*(s->order+1);   break;
        default: s->nb_channels=2*s->order+1;
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
        case 1:  s->gain=  1.75;  break;
        case 2:  s->gain= -1.26;  break;
        case 3:  s->gain= -1.26;  break;
        default: s->gain=  1;
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

static float multiply(const float decode_matrix[22][15],int row, float *vars[22], int sample_no, int nb_channels)
{
    float sum=0;
    int j;
    for(j=0;j<nb_channels;j++) {
        sum+=(decode_matrix[row][j]*vars[j][sample_no]);
    }
    return sum;
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

    if(s->enable_shelf) {
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

    for(i=0;i<s->nb_channels;i++) {
        vars[i]=(float*)in->extended_data[i];
    }

    for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
        c[i]=(float *)out_buf->extended_data[i];
    }

    for(itr=0;itr<in->nb_samples;itr++) {
        for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
            if(s->dimension==2){
                calc[i]=multiply(ambisonic_matrix[s->lyt].matrix,i,vars,itr,3);
            } else {
            	switch(s->order)
            	{
            		case 1: calc[i]=multiply(ambisonic_matrix[s->lyt].matrix,i,vars,itr,4);
            				break;
            		case 2: calc[i]=multiply(ambisonic_matrix[s->lyt].matrix,i,vars,itr,9);
            				break;
            		case 3: calc[i]=multiply(ambisonic_matrix[s->lyt].matrix,i,vars,itr,16);
            				break;
            	}
            }
        }

        for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
            c[i][itr]=calc[i];
        }

        for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
        	c[i][itr]*=scaler_matrix[s->scaler].matrix[0][i];
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
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
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
