/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>

#include "sample_comm.h"

#define PIC_SIZE   PIC_3840X2160
#define SAMPLE_STREAM_PATH "./source_file"
#define UHD_STREAN_WIDTH 3840
#define UHD_STREAM_HEIGHT 2160
#define FHD_STREAN_WIDTH 1920
#define FHD_STREAN_HEIGHT 1080
#define REF_NUM 2
#define DISPLAY_NUM 2
#define SAMPLE_VDEC_COMM_VB_CNT 4
#define SAMPLE_VDEC_VPSS_LOW_DELAY_LINE_CNT 16

static ot_payload_type g_cur_type = OT_PT_H265;

static vdec_display_cfg g_vdec_display_cfg = {
    .pic_size = PIC_3840X2160,
    .intf_sync = OT_VO_OUT_3840x2160_30,
    .intf_type = OT_VO_INTF_HDMI,
};

static ot_size g_disp_size;
static td_s32 g_sample_exit = 0;

#ifndef __LITEOS__
static td_void sample_vdec_handle_sig(td_s32 signo)
{
    if ((signo == SIGINT) || (signo == SIGTERM)) {
        g_sample_exit = 1;
    }
}
#endif

static td_s32 sample_getchar()
{
    int c;
    if (g_sample_exit == 1) {
        return 'e';
    }

    c = getchar();

    if (g_sample_exit == 1) {
        return 'e';
    }
    return c;
}

static td_void sample_vdec_usage(const char *s_prg_nm)
{
    printf("\n/************************************/\n");
    printf("usage : %s <index>\n", s_prg_nm);
    printf("index:\n");
    printf("\t0:  VDEC(H265 PLAYBACK)-VPSS-VO\n");
    printf("\t1:  VDEC(H264 PLAYBACK)-VPSS-VO\n");
    printf("\t2:  VDEC(JPEG PLAYBACK)-VPSS-VO\n");
    printf("\t3:  VDEC(H265 LOWDELAY PREVIEW)-VPSS-VO\n");
    printf("/************************************/\n\n");
}

static td_u32 sample_vdec_get_chn_width()
{
    switch (g_cur_type) {
        case OT_PT_H264:
        case OT_PT_H265:
            return UHD_STREAN_WIDTH;
        case OT_PT_JPEG:
        case OT_PT_MJPEG:
            return UHD_STREAN_WIDTH;
        default:
            sample_print("invalid type %d!\n", g_cur_type);
            return UHD_STREAN_WIDTH;
    }
}

static td_u32 sample_vdec_get_chn_height()
{
    switch (g_cur_type) {
        case OT_PT_H264:
        case OT_PT_H265:
            return UHD_STREAM_HEIGHT;
        case OT_PT_JPEG:
        case OT_PT_MJPEG:
            return UHD_STREAM_HEIGHT;
        default:
            sample_print("invalid type %d!\n", g_cur_type);
            return UHD_STREAM_HEIGHT;
    }
}

static td_s32 sample_init_module_vb(sample_vdec_attr *sample_vdec, td_u32 vdec_chn_num, ot_payload_type type,
    td_u32 len)
{
    td_u32 i;
    td_s32 ret;
    for (i = 0; (i < vdec_chn_num) && (i < len); i++) {
        sample_vdec[i].type                           = type;
        sample_vdec[i].width                         = sample_vdec_get_chn_width(type);
        sample_vdec[i].height                        = sample_vdec_get_chn_height(type);
        sample_vdec[i].mode                           = sample_comm_vdec_get_lowdelay_en() ? OT_VDEC_SEND_MODE_COMPAT :
                                                        OT_VDEC_SEND_MODE_FRAME;
        sample_vdec[i].sample_vdec_video.dec_mode      = OT_VIDEO_DEC_MODE_IP;
        sample_vdec[i].sample_vdec_video.bit_width     = OT_DATA_BIT_WIDTH_8;
        if (type == OT_PT_JPEG) {
            sample_vdec[i].sample_vdec_video.ref_frame_num = 0;
        } else {
            sample_vdec[i].sample_vdec_video.ref_frame_num = REF_NUM;
        }
        sample_vdec[i].display_frame_num               = DISPLAY_NUM;
        sample_vdec[i].frame_buf_cnt = (type == OT_PT_JPEG) ? (sample_vdec[i].display_frame_num + 1) :
            (sample_vdec[i].sample_vdec_video.ref_frame_num + sample_vdec[i].display_frame_num + 1);
        if (type == OT_PT_JPEG) {
            sample_vdec[i].sample_vdec_picture.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
            sample_vdec[i].sample_vdec_picture.alpha      = 255; /* 255:pic alpha value */
        }
    }
    ret = sample_comm_vdec_init_vb_pool(vdec_chn_num, &sample_vdec[0], len);
    if (ret != TD_SUCCESS) {
        sample_print("init mod common vb fail for %#x!\n", ret);
        return ret;
    }
    return ret;
}

static td_s32 sample_init_sys_and_vb(sample_vdec_attr *sample_vdec, td_u32 vdec_chn_num, ot_payload_type type,
    td_u32 len)
{
    ot_vb_cfg vb_cfg;
    ot_pic_buf_attr buf_attr = {0};
    td_s32 ret;

    ret = sample_comm_sys_get_pic_size(g_vdec_display_cfg.pic_size, &g_disp_size);
    if (ret != TD_SUCCESS) {
        sample_print("sys get pic size fail for %#x!\n", ret);
        return ret;
    }
    buf_attr.align = OT_DEFAULT_ALIGN;
    buf_attr.bit_width = OT_DATA_BIT_WIDTH_8;
    buf_attr.compress_mode = OT_COMPRESS_MODE_SEG;
    buf_attr.height = g_disp_size.height;
    buf_attr.width = g_disp_size.width;
    buf_attr.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;

    (td_void)memset_s(&vb_cfg, sizeof(ot_vb_cfg), 0, sizeof(ot_vb_cfg));
    vb_cfg.max_pool_cnt             = 1;
    vb_cfg.common_pool[0].blk_cnt  = SAMPLE_VDEC_COMM_VB_CNT * vdec_chn_num;
    vb_cfg.common_pool[0].blk_size = ot_common_get_pic_buf_size(&buf_attr);
    ret = sample_comm_sys_init(&vb_cfg);
    if (ret != TD_SUCCESS) {
        sample_print("init sys fail for %#x!\n", ret);
        sample_comm_sys_exit();
        return ret;
    }
    ret = sample_init_module_vb(&sample_vdec[0], vdec_chn_num, type, len);
    if (ret != TD_SUCCESS) {
        sample_print("init mod vb fail for %#x!\n", ret);
        sample_comm_vdec_exit_vb_pool();
        sample_comm_sys_exit();
        return ret;
    }
    return ret;
}

static td_s32 sample_vdec_bind_vpss(td_u32 vpss_grp_num)
{
    td_u32 i;
    td_s32 ret = TD_SUCCESS;
    for (i = 0; i < vpss_grp_num; i++) {
        ret = sample_comm_vdec_bind_vpss(i, i);
        if (ret != TD_SUCCESS) {
            sample_print("vdec bind vpss fail for %#x!\n", ret);
            return ret;
        }
    }
    return ret;
}

static td_void sample_stop_vpss(ot_vpss_grp vpss_grp, td_bool *vpss_chn_enable, td_u32 chn_array_size)
{
    td_s32 i;
    for (i = vpss_grp; i >= 0; i--) {
        vpss_grp = i;
        sample_common_vpss_stop(vpss_grp, &vpss_chn_enable[0], chn_array_size);
    }
}

static td_s32 sample_vdec_unbind_vpss(td_u32 vpss_grp_num)
{
    td_u32 i;
    td_s32 ret = TD_SUCCESS;
    for (i = 0; i < vpss_grp_num; i++) {
        ret = sample_comm_vdec_un_bind_vpss(i, i);
        if (ret != TD_SUCCESS) {
            sample_print("vdec unbind vpss fail for %#x!\n", ret);
        }
    }
    return ret;
}

static td_void sample_config_vpss_grp_attr(ot_vpss_grp_attr *vpss_grp_attr)
{
    vpss_grp_attr->max_width = sample_vdec_get_chn_width();
    vpss_grp_attr->max_height = sample_vdec_get_chn_height();
    vpss_grp_attr->frame_rate.src_frame_rate = -1;
    vpss_grp_attr->frame_rate.dst_frame_rate = -1;
    vpss_grp_attr->pixel_format  = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vpss_grp_attr->nr_en   = TD_FALSE;
    vpss_grp_attr->ie_en   = TD_FALSE;
    vpss_grp_attr->dci_en   = TD_FALSE;
    vpss_grp_attr->dei_mode = OT_VPSS_DEI_MODE_OFF;
    vpss_grp_attr->buf_share_en   = TD_FALSE;
}

static td_s32 sample_config_vpss_ldy_attr(td_u32 vpss_grp_num)
{
    td_u32 i;
    td_s32 ret;
    ot_low_delay_info vpss_ldy_info;
    if (!sample_comm_vdec_get_lowdelay_en()) {
        return TD_SUCCESS;
    }
    for (i = 0; i < vpss_grp_num; i++) {
        ret = ss_mpi_vpss_get_low_delay_attr(i, 0, &vpss_ldy_info);
        if (ret != TD_SUCCESS) {
            sample_print("vpss get low delay attr fail for %#x!\n", ret);
            return ret;
        }
        vpss_ldy_info.enable = TD_TRUE;
        vpss_ldy_info.line_cnt = SAMPLE_VDEC_VPSS_LOW_DELAY_LINE_CNT;
        ret = ss_mpi_vpss_set_low_delay_attr(i, 0, &vpss_ldy_info);
        if (ret != TD_SUCCESS) {
            sample_print("vpss set low delay attr fail for %#x!\n", ret);
            return ret;
        }
    }
    return TD_SUCCESS;
}

static td_s32 sample_start_vpss(ot_vpss_grp *vpss_grp, td_u32 vpss_grp_num, td_bool *vpss_chn_enable, td_u32 arr_len)
{
    td_u32 i;
    td_s32 ret;
    ot_vpss_chn_attr vpss_chn_attr[OT_VPSS_MAX_CHN_NUM];
    ot_vpss_grp_attr vpss_grp_attr = {0};
    sample_config_vpss_grp_attr(&vpss_grp_attr);
    (td_void)memset_s(vpss_chn_enable, arr_len * sizeof(td_bool), 0, arr_len * sizeof(td_bool));

    vpss_chn_enable[0] = TD_TRUE;
    vpss_chn_attr[0].width         = g_disp_size.width; /* 4:crop */
    vpss_chn_attr[0].height        = g_disp_size.height; /* 4:crop */
    vpss_chn_attr[0].compress_mode = OT_COMPRESS_MODE_SEG;
    vpss_chn_attr[0].chn_mode                  = OT_VPSS_CHN_MODE_USER;
    vpss_chn_attr[0].pixel_format              = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vpss_chn_attr[0].frame_rate.src_frame_rate = -1;
    vpss_chn_attr[0].frame_rate.dst_frame_rate = -1;
    vpss_chn_attr[0].depth                     = 0;
    vpss_chn_attr[0].mirror_en                 = TD_FALSE;
    vpss_chn_attr[0].flip_en                   = TD_FALSE;
    vpss_chn_attr[0].border_en                 = TD_FALSE;
    vpss_chn_attr[0].aspect_ratio.mode         = OT_ASPECT_RATIO_NONE;

    for (i = 0; i < vpss_grp_num; i++) {
        *vpss_grp = i;
        ret = sample_common_vpss_start(*vpss_grp, &vpss_chn_enable[0],
            &vpss_grp_attr, vpss_chn_attr, OT_VPSS_MAX_CHN_NUM);
        if (ret != TD_SUCCESS) {
            sample_print("start VPSS fail for %#x!\n", ret);
            sample_stop_vpss(*vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
            return ret;
        }
    }

    ret = sample_config_vpss_ldy_attr(vpss_grp_num);
    if (ret != TD_SUCCESS) {
        sample_stop_vpss(*vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
        return ret;
    }

    ret = sample_vdec_bind_vpss(vpss_grp_num);
    if (ret != TD_SUCCESS) {
        sample_vdec_unbind_vpss(vpss_grp_num);
        sample_stop_vpss(*vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
    }
    return ret;
}

static td_s32 sample_vpss_unbind_vo(td_u32 vpss_grp_num, sample_vo_cfg vo_config)
{
    td_u32 i;
    ot_vo_layer vo_layer = vo_config.vo_dev;
    td_s32 ret = TD_SUCCESS;
    for (i = 0; i < vpss_grp_num; i++) {
        ret = sample_comm_vpss_un_bind_vo(i, 0, vo_layer, i);
        if (ret != TD_SUCCESS) {
            sample_print("vpss unbind vo fail for %#x!\n", ret);
        }
    }
    return ret;
}

static td_s32 sample_vpss_bind_vo(sample_vo_cfg vo_config, td_u32 vpss_grp_num)
{
    td_u32 i;
    ot_vo_layer vo_layer;
    td_s32 ret = TD_SUCCESS;
    vo_layer = vo_config.vo_dev;
    for (i = 0; i < vpss_grp_num; i++) {
        ret = sample_comm_vpss_bind_vo(i, 0, vo_layer, i);
        if (ret != TD_SUCCESS) {
            sample_print("vpss bind vo fail for %#x!\n", ret);
            return ret;
        }
    }
    return ret;
}

static td_s32 sample_start_vo(sample_vo_cfg *vo_config, td_u32 vpss_grp_num)
{
    td_s32 ret;
    vo_config->vo_dev            = SAMPLE_VO_DEV_UHD;
    vo_config->vo_intf_type      = g_vdec_display_cfg.intf_type;
    vo_config->intf_sync         = g_vdec_display_cfg.intf_sync;
    vo_config->pic_size          = g_vdec_display_cfg.pic_size;
    vo_config->bg_color          = COLOR_RGB_BLUE;
    vo_config->dis_buf_len       = 3; /* 3:buf length */
    vo_config->dst_dynamic_range = OT_DYNAMIC_RANGE_SDR8;
    vo_config->vo_mode           = VO_MODE_1MUX;
    vo_config->pix_format        = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vo_config->disp_rect.x       = 0;
    vo_config->disp_rect.y       = 0;
    vo_config->disp_rect.width   = g_disp_size.width;
    vo_config->disp_rect.height  = g_disp_size.height;
    vo_config->image_size.width  = g_disp_size.width;
    vo_config->image_size.height = g_disp_size.height;
    vo_config->vo_part_mode      = OT_VO_PARTITION_MODE_SINGLE;
    vo_config->compress_mode     = OT_COMPRESS_MODE_NONE;

    ret = sample_comm_vo_start_vo(vo_config);
    if (ret != TD_SUCCESS) {
        sample_print("start VO fail for %#x!\n", ret);
        sample_comm_vo_stop_vo(vo_config);
        return ret;
    }

    ret = sample_vpss_bind_vo(*vo_config, vpss_grp_num);
    if (ret != TD_SUCCESS) {
        sample_vpss_unbind_vo(vpss_grp_num, *vo_config);
        sample_comm_vo_stop_vo(vo_config);
    }

    return ret;
}

static td_void sample_vdec_cmd_ctrl(td_u32 chn_num, vdec_thread_param *vdec_send, pthread_t *vdec_thread,
    td_u32 send_arr_len, td_u32 thread_arr_len)
{
    td_u32 i;
    ot_vdec_chn_status status;
    td_s32 c;

    for (i = 0; (i < chn_num) && (i < send_arr_len) && (i < thread_arr_len); i++) {
        if (vdec_send[i].circle_send == TD_TRUE) {
            goto circle_send;
        }
    }

    sample_comm_vdec_cmd_not_circle_send(chn_num, vdec_send, vdec_thread, send_arr_len, thread_arr_len);
    return;

circle_send:
    while (1) {
        printf("\n_sample_test:press 'e' to exit; 'q' to query!;\n");
        c = sample_getchar();
        if (c == 'e') {
            break;
        } else if (c == 'q') {
            for (i = 0; (i < chn_num) && (i < send_arr_len) && (i < thread_arr_len); i++) {
                ss_mpi_vdec_query_status(vdec_send[i].chn_id, &status);
                sample_comm_vdec_print_chn_status(vdec_send[i].chn_id, status);
            }
        }
    }
    return;
}
static td_void sample_send_stream_to_vdec(sample_vdec_attr *sample_vdec, td_u32 arr_len, td_u32 vdec_chn_num,
    const char *stream_name)
{
    td_u32 i;
    vdec_thread_param vdec_send[OT_VDEC_MAX_CHN_NUM];
    pthread_t   vdec_thread[OT_VDEC_MAX_CHN_NUM] = {0}; /* 2:thread */
    if (arr_len > OT_VDEC_MAX_CHN_NUM) {
        sample_print("array size(%u) of vdec_send need < %u!\n", arr_len, OT_VDEC_MAX_CHN_NUM);
        return;
    }
    for (i = 0; (i < vdec_chn_num) && (i < arr_len); i++) {
        if (snprintf_s(vdec_send[i].c_file_name, sizeof(vdec_send[i].c_file_name), sizeof(vdec_send[i].c_file_name) - 1,
            stream_name) < 0) {
            return;
        }
        if (snprintf_s(vdec_send[i].c_file_path, sizeof(vdec_send[i].c_file_path), sizeof(vdec_send[i].c_file_path) - 1,
            "%s", SAMPLE_STREAM_PATH) < 0) {
            return;
        }
        vdec_send[i].type          = sample_vdec[i].type;
        vdec_send[i].stream_mode   = sample_vdec[i].mode;
        vdec_send[i].chn_id        = i;
        vdec_send[i].interval_time = 1000; /* 1000: interval time */
        vdec_send[i].pts_init      = 0;
        vdec_send[i].pts_increase  = 0;
        vdec_send[i].e_thread_ctrl = THREAD_CTRL_START;
        vdec_send[i].circle_send   = TD_TRUE;
        vdec_send[i].milli_sec     = 0;
        vdec_send[i].min_buf_size  = (sample_vdec[i].width * sample_vdec[i].height * 3) >> 1; /* 3:yuv */
        vdec_send[i].fps           = 30; /* 30:frame rate */
    }
    sample_comm_vdec_start_send_stream(vdec_chn_num, &vdec_send[0], &vdec_thread[0],
        OT_VDEC_MAX_CHN_NUM, OT_VDEC_MAX_CHN_NUM);

    sample_vdec_cmd_ctrl(vdec_chn_num, &vdec_send[0], &vdec_thread[0],
        OT_VDEC_MAX_CHN_NUM, OT_VDEC_MAX_CHN_NUM);

    sample_comm_vdec_stop_send_stream(vdec_chn_num, &vdec_send[0], &vdec_thread[0],
        OT_VDEC_MAX_CHN_NUM, OT_VDEC_MAX_CHN_NUM);
}

static td_s32 sample_start_vdec(sample_vdec_attr *sample_vdec, td_u32 vdec_chn_num, td_u32 len)
{
    td_s32 ret;

    ret = sample_comm_vdec_start(vdec_chn_num, &sample_vdec[0], len);
    if (ret != TD_SUCCESS) {
        sample_print("start VDEC fail for %#x!\n", ret);
        sample_comm_vdec_stop(vdec_chn_num);
    }

    return ret;
}

static td_s32 sample_h265_vdec_vpss_vo(td_void)
{
    td_s32 ret;
    td_u32 vdec_chn_num, vpss_grp_num;
    sample_vdec_attr sample_vdec[OT_VDEC_MAX_CHN_NUM];
    td_bool vpss_chn_enable[OT_VPSS_MAX_CHN_NUM];
    sample_vo_cfg vo_config;
    ot_vpss_grp vpss_grp;

    vdec_chn_num = 1;
    vpss_grp_num = vdec_chn_num;
    g_cur_type = OT_PT_H265;
    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO), init module VB(for VDEC)
    *************************************************/
    ret = sample_init_sys_and_vb(&sample_vdec[0], vdec_chn_num, g_cur_type, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    /************************************************
    step2:  init VDEC
    *************************************************/
    ret = sample_start_vdec(&sample_vdec[0], vdec_chn_num, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_sys;
    }
    /************************************************
    step3:  start VPSS
    *************************************************/
    ret = sample_start_vpss(&vpss_grp, vpss_grp_num, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_vdec;
    }
    /************************************************
    step4:  start VO
    *************************************************/
    ret = sample_start_vo(&vo_config, vpss_grp_num);
    if (ret != TD_SUCCESS) {
        goto stop_vpss;
    }

    /************************************************
    step5:  send stream to VDEC
    *************************************************/
    sample_send_stream_to_vdec(&sample_vdec[0], OT_VDEC_MAX_CHN_NUM, vdec_chn_num, "3840x2160_8bit.h265");

    ret = sample_vpss_unbind_vo(vpss_grp_num, vo_config);
    sample_comm_vo_stop_vo(&vo_config);
stop_vpss:
    ret = sample_vdec_unbind_vpss(vpss_grp_num);
    sample_stop_vpss(vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
stop_vdec:
    sample_comm_vdec_stop(vdec_chn_num);
    sample_comm_vdec_exit_vb_pool();
stop_sys:
    sample_comm_sys_exit();

    return ret;
}

static td_s32 sample_h264_vdec_vpss_vo(td_void)
{
    td_s32 ret;
    td_u32 vdec_chn_num, vpss_grp_num;
    ot_vpss_grp vpss_grp;
    sample_vdec_attr sample_vdec[OT_VDEC_MAX_CHN_NUM];
    sample_vo_cfg vo_config;
    td_bool vpss_chn_enable[OT_VPSS_MAX_CHN_NUM];

    vdec_chn_num = 1;
    vpss_grp_num = vdec_chn_num;
    g_cur_type = OT_PT_H264;
    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO), init module VB(for VDEC)
    *************************************************/
    ret = sample_init_sys_and_vb(&sample_vdec[0], vdec_chn_num, g_cur_type, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    /************************************************
    step2:  init VDEC
    *************************************************/
    ret = sample_start_vdec(&sample_vdec[0], vdec_chn_num, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_sys;
    }
    /************************************************
    step3:  start VPSS
    *************************************************/
    ret = sample_start_vpss(&vpss_grp, vpss_grp_num, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_vdec;
    }
    /************************************************
    step4:  start VO
    *************************************************/
    ret = sample_start_vo(&vo_config, vpss_grp_num);
    if (ret != TD_SUCCESS) {
        goto stop_vpss;
    }

    /************************************************
    step5:  send stream to VDEC
    *************************************************/
    sample_send_stream_to_vdec(&sample_vdec[0], OT_VDEC_MAX_CHN_NUM, vdec_chn_num, "3840x2160_8bit.h264");

    ret = sample_vpss_unbind_vo(vpss_grp_num, vo_config);
    sample_comm_vo_stop_vo(&vo_config);
stop_vpss:
    ret = sample_vdec_unbind_vpss(vpss_grp_num);
    sample_stop_vpss(vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
stop_vdec:
    sample_comm_vdec_stop(vdec_chn_num);
    sample_comm_vdec_exit_vb_pool();
stop_sys:
    sample_comm_sys_exit();

    return ret;
}

static td_s32 sample_jpeg_vdec_vpss_vo(td_void)
{
    td_s32 ret;
    td_u32 vdec_chn_num, vpss_grp_num;
    ot_vpss_grp vpss_grp;
    sample_vdec_attr sample_vdec[OT_VDEC_MAX_CHN_NUM];
    sample_vo_cfg vo_config;
    td_bool vpss_chn_enable[OT_VPSS_MAX_CHN_NUM];

    vdec_chn_num = 1;
    vpss_grp_num = vdec_chn_num;
    g_cur_type = OT_PT_JPEG;
    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO), init module VB(for VDEC)
    *************************************************/
    ret = sample_init_sys_and_vb(&sample_vdec[0], vdec_chn_num, g_cur_type, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    /************************************************
    step2:  init VDEC
    *************************************************/
    ret = sample_start_vdec(&sample_vdec[0], vdec_chn_num, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_sys;
    }
    /************************************************
    step3:  start VPSS
    *************************************************/
    ret = sample_start_vpss(&vpss_grp, vpss_grp_num, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_vdec;
    }
    /************************************************
    step4:  start VO
    *************************************************/
    ret = sample_start_vo(&vo_config, vpss_grp_num);
    if (ret != TD_SUCCESS) {
        goto stop_vpss;
    }

    /************************************************
    step5:  send stream to VDEC
    *************************************************/
    sample_send_stream_to_vdec(&sample_vdec[0], OT_VDEC_MAX_CHN_NUM, vdec_chn_num, "3840x2160.jpg");

    ret = sample_vpss_unbind_vo(vpss_grp_num, vo_config);
    sample_comm_vo_stop_vo(&vo_config);
stop_vpss:
    ret = sample_vdec_unbind_vpss(vpss_grp_num);
    sample_stop_vpss(vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
stop_vdec:
    sample_comm_vdec_stop(vdec_chn_num);
    sample_comm_vdec_exit_vb_pool();
stop_sys:
    sample_comm_sys_exit();

    return ret;
}

static td_s32 sample_h265_lowdelay_vdec_vpss_vo(td_void)
{
    td_s32 ret;
    td_u32 vdec_chn_num, vpss_grp_num;
    sample_vdec_attr sample_vdec[OT_VDEC_MAX_CHN_NUM];
    td_bool vpss_chn_enable[OT_VPSS_MAX_CHN_NUM];
    sample_vo_cfg vo_config;
    ot_vpss_grp vpss_grp;

    vdec_chn_num = 1;
    vpss_grp_num = vdec_chn_num;
    g_cur_type = OT_PT_H265;
    sample_comm_vdec_set_lowdelay_en(TD_TRUE);
    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO), init module VB(for VDEC)
    *************************************************/
    ret = sample_init_sys_and_vb(&sample_vdec[0], vdec_chn_num, g_cur_type, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    /************************************************
    step2:  init VDEC
    *************************************************/
    ret = sample_start_vdec(&sample_vdec[0], vdec_chn_num, OT_VDEC_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_sys;
    }
    /************************************************
    step3:  start VPSS
    *************************************************/
    ret = sample_start_vpss(&vpss_grp, vpss_grp_num, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
    if (ret != TD_SUCCESS) {
        goto stop_vdec;
    }
    /************************************************
    step4:  start VO
    *************************************************/
    ret = sample_start_vo(&vo_config, vpss_grp_num);
    if (ret != TD_SUCCESS) {
        goto stop_vpss;
    }

    /************************************************
    step5:  send stream to VDEC
    *************************************************/
    sample_send_stream_to_vdec(&sample_vdec[0], OT_VDEC_MAX_CHN_NUM, vdec_chn_num, "3840x2160_8bit.h265");

    ret = sample_vpss_unbind_vo(vpss_grp_num, vo_config);
    sample_comm_vo_stop_vo(&vo_config);
stop_vpss:
    ret = sample_vdec_unbind_vpss(vpss_grp_num);
    sample_stop_vpss(vpss_grp, &vpss_chn_enable[0], OT_VPSS_MAX_CHN_NUM);
stop_vdec:
    sample_comm_vdec_stop(vdec_chn_num);
    sample_comm_vdec_exit_vb_pool();
stop_sys:
    sample_comm_sys_exit();

    return ret;
}

static td_s32 sample_check_parameter(int argc, const char *argv0, const char *argv1)
{
    if ((argc != 2) || (strlen(argv1) != 1)) { /* 2:arg num */
        printf("\n invalid input!  for examples:\n");
        sample_vdec_usage(argv0);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_s32 sample_choose_case(const char argv1, const char *argv0)
{
    td_s32 ret;
    switch (argv1) {
        case '0': {
            ret = sample_h265_vdec_vpss_vo();
            break;
        }
        case '1': {
            ret = sample_h264_vdec_vpss_vo();
            break;
        }
        case '2': {
            ret = sample_jpeg_vdec_vpss_vo();
            break;
        }
        case '3': {
            ret = sample_h265_lowdelay_vdec_vpss_vo();
            break;
        }
        default: {
            sample_print("the index is invalid!\n");
            sample_vdec_usage(argv0);
            ret = TD_FAILURE;
            break;
        }
    }
    return ret;
}

/******************************************************************************
* function    : main()
* description : video vdec sample
******************************************************************************/
#ifdef __LITEOS__
    int app_main(int argc, char *argv[])
#else
    int main(int argc, char *argv[])
#endif
{
    td_s32 ret;

    ret = sample_check_parameter(argc, argv[0], argv[1]);
    if (ret != TD_SUCCESS) {
        return ret;
    }

#ifndef __LITEOS__
    sample_sys_signal(sample_vdec_handle_sig);
#endif

    /******************************************
     choose the case
    ******************************************/
    ret = sample_choose_case(*argv[1], argv[0]);
    if (ret == TD_SUCCESS && g_sample_exit == 0) {
        sample_print("program exit normally!\n");
    } else {
        sample_print("program exit abnormally!\n");
    }

    return ret;
}
