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
    nearfield
};

enum InputFormat {
  N3D    =1,
  SN3D   =2,
  FURMUL =3
};

enum Rotate {
    TILT  ,
    TUMBLE,
    YAW
};

enum Layouts {
    MONO        ,
    STEREO      ,
    TRIANGLE    ,
    SQUARE      ,
    PENTAGON    ,
    HEXAGON     ,
    HEPTAGON    ,
    OCTAGON     ,
    TETRAHEDRON ,
    OCTAHEDRON  ,
    CUBE        ,
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
            {0.17836, 0.32555, 0.18795, 0},
            {0.17836, 0      ,-0.37591, 0},
            {0.17836,-0.32555, 0.18795, 0},
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
    double e_nf, e_ni;
    double gain;
    double frequency;
    double width;
    uint64_t channels;
    double a0, a1, a2;
    double b0, b1, b2;
    double tilt;
    double tumble;
    double yaw;
    Cache *cache;
    float decode_matrix[22][9];
    float angle;
    char* sp_layout;
    char* s_o;
    enum Rotate dir;


    void (*filter1)(struct AmbisonicContext *s, const void *ibuf, void *obuf, int len,
                   double *i1, double *i2, double *o1, double *o2,
                   double b0, double b1, double b2, double a1, double a2, double min, double max, int channelno);
    void (*filter2)(struct AmbisonicContext *s , float **in, float d1, float d2, float g);

} AmbisonicContext;

#define OFFSET(x) offsetof(AmbisonicContext,x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ambisonic_options[] = {
    {"enable_shelf","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {"e_s","Set if shelf filtering is required",OFFSET(enable_shelf), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS},
    {"e_nf","Set if Near Field Compensation is required/Input distance",OFFSET(e_nf), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, 100.0, FLAGS},
    {"e_ni","Set if Near Field Compensation is required/Output distance",OFFSET(e_ni), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, 100.0, FLAGS},
    {"output_layout","Enter Layout of output",OFFSET(lyt), AV_OPT_TYPE_INT, {.i64=SQUARE}, MONO, DODECAHEDRON, FLAGS,"lyt"},
    {"o_l","Enter Layout of output",OFFSET(lyt), AV_OPT_TYPE_INT, {.i64=SQUARE}, MONO, DODECAHEDRON, FLAGS,"lyt"},
    {"scaling_option","Set the input format (N3D, SN3D, Furse Malham)",OFFSET(s_o), AV_OPT_TYPE_STRING, {.str="n3d"}, 0, 0, FLAGS},
    {"s_o","Set the input format (N3D, SN3D, Furse Malham)",OFFSET(s_o), AV_OPT_TYPE_STRING, {.str="n3d"}, 0, 0, FLAGS},
    {"tilt","Set angle for tilt(x-axis)",OFFSET(tilt),AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, 0.0, 180.0, FLAGS},
    {"tumble","Set angle for tumble(y-axis)",OFFSET(tumble),AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, 0.0, 180.0, FLAGS},
    {"yaw","Set angle for yaw(z-axis)",OFFSET(yaw),AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, 0.0, 180.0, FLAGS},
    {"mono","Mono Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=MONO}, 0, 0, FLAGS,"lyt"},
    {"stereo","Stereo Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=STEREO}, 0, 0, FLAGS,"lyt"},
    {"triangle","Triangle Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=TRIANGLE}, 0, 0, FLAGS,"lyt"},
    {"quad","Square Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=SQUARE}, 0, 0, FLAGS,"lyt"},
    {"pentagon","Pentagonal Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=PENTAGON}, 0, 0, FLAGS,"lyt"},
    {"hexagon","Hexagonal Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=HEXAGON}, 0, 0, FLAGS,"lyt"},
    {"hepatagon","Hepatagonal Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=HEPTAGON}, 0, 0, FLAGS,"lyt"},
    {"octagon","Octagonal Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=OCTAGON}, 0, 0, FLAGS,"lyt"},
    {"octahedron","Octahedron Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=OCTAHEDRON}, 0, 0, FLAGS,"lyt"},
    {"cube","Cube Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=CUBE}, 0, 0, FLAGS,"lyt"},
    {"icosahedron","Icosahedron Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=ICOSAHEDRON}, 0, 0, FLAGS,"lyt"},
    {"dodecahedron","Dodecahedron Speaker Layout",0, AV_OPT_TYPE_CONST, {.i64=DODECAHEDRON}, 0, 0, FLAGS,"lyt"},
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

    switch(s->lyt) {
        case MONO:         temp=AV_CH_LAYOUT_MONO;      s->dimension=2;  break;
        case STEREO:       temp=AV_CH_LAYOUT_STEREO;    s->dimension=2;  break;
        case TRIANGLE:     temp=AV_CH_LAYOUT_SURROUND;  s->dimension=2;  break;
        case SQUARE:       temp=AV_CH_LAYOUT_4POINT0;   s->dimension=2;  break;
        case PENTAGON:     temp=AV_CH_LAYOUT_5POINT0;   s->dimension=2;  break;
        case HEXAGON:      temp=AV_CH_LAYOUT_6POINT0;   s->dimension=2;  break;
        case HEPTAGON:     temp=AV_CH_LAYOUT_7POINT0;   s->dimension=2;  break;
        case OCTAGON:      temp=AV_CH_LAYOUT_OCTAGONAL; s->dimension=2;  break;
        case OCTAHEDRON:   temp=AV_CH_LAYOUT_6POINT0;   s->dimension=2;  break;
        case CUBE:         temp=AV_CH_LAYOUT_OCTAGONAL; s->dimension=3;  break;
        case ICOSAHEDRON:  temp=AV_CH_LAYOUT_4POINT0;   s->dimension=3;  break;
        case DODECAHEDRON: temp=AV_CH_LAYOUT_4POINT0;   s->dimension=3;  break;
        default:           temp=AV_CH_LAYOUT_4POINT0;   s->dimension=2;
    }

    memset(s->decode_matrix,0,22*9);

    s->order=1;//first order ambisonics

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

static void nearfield_flt(AmbisonicContext *s, float **in,float d1, float d2,float g)
{
    float b,g1,c,d, x,y,z=0;
    float *input_arr[9];

    if(d1) d1=340.0f / (d1 * 48e3);
    if(d2) d2=340.0f / (d2 * 48e3);

    //set param c
    b  = (d1 * 0.5);
    g1 = b+1;
    g *= g1;
    c  = (2 * b) / g1;

    //set param d
    b  = (d2 * 0.5);
    g1 = b+1;
    g /= g1;
    d  = (2 * b) / g1;


    for(int i=0;i<s->nb_channels;i++)
    {
        input_arr[i]=(float*)in[i];
    }

    //forward filter
    if(d1) {
        for(int i=0;i<s->nb_channels;i++) {
            for(int itr=0;itr<s->nb_channels;itr++) {
                x    =  g * input_arr[i][itr];
                y    =  x - (d*z) + 1e-30f;
                x    =  y + (c*z);
                z   +=  y;
                itr +=  d;

                input_arr[i][itr] = x;
            }
        }
    } else { //inverse filter
        for(int i=0;i<s->nb_channels;i++) {
            for(int itr=0;itr<s->nb_channels;itr++) {
                x   =  g * input_arr[i][itr];
                x   -= (d*z) + 1e-30f;
                z   += x;
                itr += d;

                input_arr[i][itr] = x;
            }
        }
    }
}

static void rotate(AmbisonicContext *s,float **in, float rotate_matrix[9][9],float angle,int samples)
{
    for(int j=0;j<samples;j++) {
        float sum=0;
        for(int k=0;k<s->nb_channels;k++) {
            for(int i=0;i<s->nb_channels;i++) {
                sum+=(in[i][j]*rotate_matrix[k][i]);
            }
            in[k][j]=sum;
        }
    }
}


static void rotate_flt(AmbisonicContext *s, float **in,int dir,float angle,int samples)
{
    double a = (M_PI/180.0f)*angle;

    float rotate_matrix_tilt[][9]=  {{1,  0     , 0      , 0 , 0     ,  0                  , 0                       , 0      , 0                            },
                                     {0,  cos(a),-sin(a) , 0 , 0     ,  0                  , 0                       , 0      , 0                            },
                                     {0,  sin(a), cos(a) , 0 , 0     ,  0                  , 0                       , 0      , 0                            },
                                     {0,  0     , 0      , 1 , 0     ,  0                  , 0                       , 0      , 0                            },
                                     {0,  0     , 0      , 0 , cos(a),  0                  , 0                       , -sin(a), 0                            },
                                     {0,  0     , 0      , 0 , 0     ,  cos(2*a)           , -sqrt(3/4)*sin(2*a)     , 0      ,-(0.5)*sin(2*a)               },
                                     {0,  0     , 0      , 0 , 0     ,  sqrt(0.75)*sin(2*a), (0.25)*(1+3*cos(2*a))   , 0      ,-sqrt(3.0/16.0)*(1-cos(2*a))  },
                                     {0,  0     , 0      , 0 , sin(a),  0                  , 0                       , cos(a) , 0                            },
                                     {0,  0     , 0      , 0 , 0     ,  (0.5)*sin(2*a)     , -sqrt(3/16)*(1-cos(2*a)), 0      , (0.25)*(3+cos(2*a))          }};

    float rotate_matrix_tumble[][9]={{1,  0,  0       ,0      ,0      ,0       , 0                           ,0                    ,0                           },
                                     {0,  1,  0       ,0      ,0      ,0       , 0                           ,0                    ,0                           },
                                     {0,  0,  cos(a)  ,sin(a) ,0      ,0       , 0                           ,0                    ,0                           },
                                     {0,  0,  -sin(a) ,cos(a) ,0      ,0       , 0                           ,0                    ,0                           },
                                     {0,  0,  0       ,0      ,cos(a) ,-sin(a) , 0                           ,0                    ,0                           },
                                     {0,  0,  0       ,0      ,sin(a) , cos(a) , 0                           ,0                    ,0                           },
                                     {0,  0,  0       ,0      ,0      ,0       , (0.25)*(1+3*cos(2*a))       ,sqrt(0.75)*sin(2*a)  ,sqrt(3.0/16.0)*(1-cos(2*a)) },
                                     {0,  0,  0       ,0      ,0      ,0       ,-sqrt(0.75)*sin(2*a)         ,cos(2*a)             ,(0.5)*sin(2*a)              },
                                     {0,  0,  0       ,0      ,0      ,0       , sqrt(3.0/16.0)*(1-cos(2*a)) ,-(0.5)*sin(2*a)      ,(0.25)*(3+cos(2*a))         }};

    float rotate_matrix_yaw[][9]=   {{1,  0     , 0     , 0      ,  0          , 0         , 0       , 0      , 0        },
                                     {0,  cos(a), 0     , sin(a) ,  0          , 0         , 0       , 0      , 0        },
                                     {0,  0     , 1     , 0      ,  0          , 0         , 0       , 0      , 0        },
                                     {0, -sin(a), 0     , cos(a) ,  0          , 0         , 0       , 0      , 0        },
                                     {0,  0     , 0     , 0      ,  cos(2*a)   , 0         , 0       , 0      , sin(2*a) },
                                     {0,  0     , 0     , 0      ,  0          , cos(a)    , 0       ,sin(a)  , 0        },
                                     {0,  0     , 0     , 0      ,  0          , 0         , 1       , 0      , 0        },
                                     {0,  0     , 0     , 0      ,  0          , -sin(a)   , 0       , cos(a) , 0        },
                                     {0,  0     , 0     , 0      ,  -sin(2*a)  , 0         , 0       , 0      , cos(2*a) }};
    switch(s->dir)
    {
        case TILT:   rotate(s,in,rotate_matrix_tilt,angle,samples);   break;
        case TUMBLE: rotate(s,in,rotate_matrix_tumble,angle,samples); break;
        case YAW:    rotate(s,in,rotate_matrix_yaw,angle,samples);    break;
    }
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

    if(s->tilt)   {s->dir=TILT;   s->angle=s->tilt;  }
    if(s->tumble) {s->dir=TUMBLE; s->angle=s->tumble;}
    if(s->yaw)    {s->dir=YAW;    s->angle=s->yaw;  }

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
    if (!out_buf) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out_buf, in);

    s->filter1 = shelf_flt;

    if(s->lyt==MONO) s->enable_shelf=0;
    if(s->enable_shelf) {
        // shelf filter
        // for w channel gain= 1.75
        s->filter1(s, in->extended_data[0],
                      out_buf->extended_data[0], in->nb_samples,
                      &s->cache[0].i1, &s->cache[0].i2,
                      &s->cache[0].o1, &s->cache[0].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,1);
        // for x & y channel gain= -1.26
        s->filter1(s, in->extended_data[1],
                      out_buf->extended_data[1], in->nb_samples,
                      &s->cache[1].i1, &s->cache[1].i2,
                      &s->cache[1].o1, &s->cache[1].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,2);
        s->filter1(s, in->extended_data[2],
                      out_buf->extended_data[2], in->nb_samples,
                      &s->cache[2].i1, &s->cache[2].i2,
                      &s->cache[2].o1, &s->cache[2].o2,
                      s->b0, s->b1, s->b2, s->a1, s->a2, -1.,1.,3);
    }

    s->filter2 = nearfield_flt;
    if(s->e_nf && s->e_ni) {
        s->filter2(s,(float **)in->extended_data,s->e_nf,s->e_ni,1.0);
        rotate_flt(s,(float **)in->extended_data,s->dir,s->angle,in->nb_samples);
    }

    for(i=0;i<s->nb_channels;i++) {
        vars[i]=(float*)in->extended_data[i];
    }

    for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
        c[i]=(float *)out_buf->extended_data[i];
    }

    for(itr=0;itr<in->nb_samples;itr++) {
        for(i=0;i<ambisonic_matrix[s->lyt].speakers;i++) {
            if(s->dimension==2) {
                calc[i]=multiply(ambisonic_matrix[s->lyt].matrix,i,vars,itr,3);
            } else {
                switch(s->order) {
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