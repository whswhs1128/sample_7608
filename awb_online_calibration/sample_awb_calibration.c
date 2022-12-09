/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <math.h>

#include "ss_mpi_isp.h"
#include "ot_common_isp.h"
#include "ss_mpi_ae.h"

#include "sample_comm.h"

#define VB_BLK_CNT 8

typedef struct {
    sample_vi_cfg vi_config;
    sample_vo_cfg vo_config;
    ot_vo_dev     vo_dev;
    ot_vo_chn     vo_chn;
    ot_vi_pipe    vi_pipe;
    ot_vi_chn     vi_chn;
}awb_cali_prev;

td_s32 sample_vio_start_vi_vo(sample_vi_cfg *vi_config, sample_vo_cfg *vo_config)
{
    td_s32  ret;

    ret = sample_comm_vi_start_vi(vi_config);
    if (TD_SUCCESS != ret) {
        sample_print("start vi failed!\n");
        return ret;
    }

    ret = sample_comm_vo_start_vo(vo_config);
    if (TD_SUCCESS != ret) {
        sample_print("SAMPLE_VIO start VO failed with %#x!\n", ret);
        goto EXIT;
    }

    return ret;

EXIT:
    sample_comm_vi_stop_vi(vi_config);

    return ret;
}

static td_s32 sample_vio_stop_vi_vo(sample_vi_cfg *vi_config, sample_vo_cfg *vo_config)
{
    sample_comm_vo_stop_vo(vo_config);

    sample_comm_vi_stop_vi(vi_config);

    return TD_SUCCESS;
}

static td_void sample_vi_get_default_vb_config(ot_size *size, ot_vb_cfg *vb_cfg)
{
    ot_vb_calc_cfg calc_cfg;
    ot_pic_buf_attr buf_attr;

    (td_void)memset_s(vb_cfg, sizeof(ot_vb_cfg), 0, sizeof(ot_vb_cfg));
    vb_cfg->max_pool_cnt = 128; /* 128 pool limit */

    buf_attr.width         = size->width;
    buf_attr.height        = size->height;
    buf_attr.align         = OT_DEFAULT_ALIGN;
    buf_attr.bit_width     = OT_DATA_BIT_WIDTH_8;
    buf_attr.pixel_format  = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_422;
    buf_attr.compress_mode = OT_COMPRESS_MODE_NONE;
    ot_common_get_pic_buf_cfg(&buf_attr, &calc_cfg);

    vb_cfg->common_pool[0].blk_size = calc_cfg.vb_size;
    vb_cfg->common_pool[0].blk_cnt  = VB_BLK_CNT;
}

static td_s32 sample_vio_sys_init(ot_vi_vpss_mode_type mode_type, ot_vi_video_mode video_mode)
{
    td_s32 ret;
    ot_size size;
    ot_vb_cfg vb_cfg;
    td_u32 supplement_config;

    size.width  = 1920; /* 1920 is width */
    size.height = 1080; /* 1080 is height */
    sample_vi_get_default_vb_config(&size, &vb_cfg);

    supplement_config = OT_VB_SUPPLEMENT_BNR_MOT_MASK;
    ret = sample_comm_sys_init_with_vb_supplement(&vb_cfg, supplement_config);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    ret = sample_comm_vi_set_vi_vpss_mode(mode_type, video_mode);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

td_s32 sample_awb_cali_start_prev(awb_cali_prev *awb_cali_prev)
{
    td_s32             ret;

    sample_sns_type  sns_type;
    ot_size             in_size;
    ot_vi_vpss_mode_type mast_pipe_mode = OT_VI_ONLINE_VPSS_OFFLINE;
    ot_vi_video_mode video_mode = OT_VI_VIDEO_MODE_NORM;

    awb_cali_prev->vo_dev               = SAMPLE_VO_DEV_DHD0;
    awb_cali_prev->vo_chn               = 0;
    awb_cali_prev->vi_pipe              = 0;
    awb_cali_prev->vi_chn               = 0;

    /************************************************
    step1:  get all sensors information
    *************************************************/
    ret = sample_vio_sys_init(mast_pipe_mode, video_mode);
    if (ret != TD_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &awb_cali_prev->vi_config);

    /************************************************
    step4:  init VI and VO
    *************************************************/

    sample_comm_vo_get_def_config(&awb_cali_prev->vo_config);

    ret = sample_comm_vi_start_vi(&awb_cali_prev->vi_config);
    if (ret != TD_SUCCESS) {
        goto EXIT;
    }

    ret = sample_comm_vo_start_vo(&awb_cali_prev->vo_config);
    if (ret != TD_SUCCESS) {
        sample_print("sample_vio_start_vi_vo failed witfh %d\n", ret);
        goto EXIT;
    }

    /************************************************
    step4:  init VI and VO
    *************************************************/
    sample_comm_vo_get_def_config(&awb_cali_prev->vo_config);
    ret = sample_vio_start_vi_vo(&awb_cali_prev->vi_config, &awb_cali_prev->vo_config);
    if (TD_SUCCESS != ret) {
        sample_print("sample_vio_start_vi_vo failed witfh %d\n", ret);
        goto EXIT;
    }

    /************************************************
    step5:  bind VI and VO
    *************************************************/
    ret = sample_comm_vi_bind_vo(awb_cali_prev->vi_pipe, awb_cali_prev->vi_chn,
        awb_cali_prev->vo_dev, awb_cali_prev->vo_chn);
    if (ret != TD_SUCCESS) {
        sample_print("sample_comm_vi_bind_vo failed with %#x!\n", ret);
        goto EXIT1;
    }

    return ret;

EXIT1:
    ret = sample_vio_stop_vi_vo(&awb_cali_prev->vi_config, &awb_cali_prev->vo_config);
EXIT:
    sample_comm_sys_exit();
sys_init_failed:

    return ret;
}

td_s32 sample_awb_cali_stop_prev(awb_cali_prev *awb_cali_prev)
{
    td_s32 ret;

    if (awb_cali_prev == TD_NULL) {
        sample_print("err: awb_cali_prev is NULL \n");
        return TD_FAILURE;
    }

    ret = sample_comm_vi_un_bind_vo(awb_cali_prev->vi_pipe, awb_cali_prev->vi_chn,
        awb_cali_prev->vo_dev, awb_cali_prev->vo_chn);
    ret = sample_vio_stop_vi_vo(&awb_cali_prev->vi_config, &awb_cali_prev->vo_config);
    sample_comm_sys_exit();

    return ret;
}

void sample_awb_correction_usage(char *s_prg_nm)
{
    printf("usage : %s <mode> <intf1> <intf2> <intf3>\n", s_prg_nm);
    printf("mode:\n");
    printf("\t 0) calculate sample gain.\n");
    printf("\t 1) adjust sample gain according to golden sample.\n");

    printf("intf1:\n");
    printf("\t the value of rgain of golden sample.\n");

    printf("intf2:\n");
    printf("\t the value of bgain of golden sample.\n");

    printf("intf3:\n");
    printf("\t the value of alpha ranging from 0 to 1024, 0 means no blending.\n");

    return;
}

td_s32 awb_calib_check_param(td_s32 argc, td_char *argv[], td_u32 *mode,
    ot_isp_awb_calibration_gain *awb_golden_gain, td_s16 *alpha)
{
    if ((argc < 2) || (strlen(argv[1]) != 1)) { /* 2 get sample */
        sample_awb_correction_usage(argv[0]);
        return TD_FAILURE;
    }
    if ((argc < 5) && (*argv[1] == '1')) {      /* 5 cal sample gain */
        sample_awb_correction_usage(argv[0]);
        return TD_FAILURE;
    }

    switch (*argv[1]) {
        case '0':
            *mode = 0;
            break;

        case '1':
            *mode = 1;
            awb_golden_gain->avg_r_gain = atoi(argv[2]); /* para 2 is golden rgain */
            awb_golden_gain->avg_b_gain = atoi(argv[3]); /* para 3 is golden bgain */
            *alpha = atoi(argv[4]);        /* para 4 is blend coef, 0 means no blending, use curr gain */
            break;

        default:
            sample_print("the mode is invalid!\n");
            sample_awb_correction_usage(argv[0]);
            return TD_FAILURE;
    }

    return TD_SUCCESS;
}

td_bool awb_calib_check_ae_status(ot_vi_pipe vi_pipe, td_u8 ac_freq)
{
    td_s16 total_count = 0;
    td_s16 stable_count = 0;
    ot_isp_exp_info exp_info;
    ot_isp_exposure_attr exp_attr;
    const td_u32 stable_upper_limit = 20;
    const td_u32 stable_low_limit = 5;

    ss_mpi_isp_get_exposure_attr(vi_pipe, &exp_attr);

    printf("set antiflicker enable and the value of frequency to 50_hz\n");
    exp_attr.auto_attr.antiflicker.enable = TD_TRUE;
    exp_attr.auto_attr.antiflicker.frequency = ac_freq;
    ss_mpi_isp_set_exposure_attr(vi_pipe, &exp_attr);

    do {
        ss_mpi_isp_query_exposure_info(vi_pipe, &exp_info);
        usleep(100000000 / div_0_to_1(exp_info.fps)); /* 100000000 is 1s * 100, fps normalize 100 */

        /* judge whether AE is stable */
        if (exp_info.hist_error > exp_attr.auto_attr.tolerance) {
            stable_count = 0;
        } else {
            stable_count++;
        }
        total_count++;
    }while ((stable_count < stable_low_limit) && (total_count < stable_upper_limit));

    if (stable_count >= stable_low_limit) {
        return TD_TRUE;
    } else {
        return TD_FALSE;
    }
}

td_s32 awb_calib_get_corr_gain(ot_vi_pipe vi_pipe, ot_isp_awb_calibration_gain *awb_calib_gain,
    ot_isp_awb_calibration_gain *awb_golden_gain, td_u32 mode, td_s16 alpha)
{
    const td_u32 blend_shift = 10; /* 2^10 = 1024 */

    ss_mpi_isp_get_lightbox_gain(vi_pipe, awb_calib_gain);

    /* adjust the value of rgain and bgain of sample according to golden sample */
    if (mode == 1) {
        awb_calib_gain->avg_r_gain = (td_u16)((td_s16)(awb_calib_gain->avg_r_gain) +
            ((((td_s16)awb_golden_gain->avg_r_gain - (td_s16)(awb_calib_gain->avg_r_gain)) * alpha) >> blend_shift));
        awb_calib_gain->avg_b_gain = (td_u16)((td_s16)(awb_calib_gain->avg_b_gain) +
            ((((td_s16)awb_golden_gain->avg_b_gain - (td_s16)(awb_calib_gain->avg_b_gain)) * alpha) >> blend_shift));
    }

    if (mode == 0) {
        printf("calculate sample gain:\n");
    } else if (mode == 1) {
        printf("adjust sample gain:\n");
    }
    printf("avg_rgain =%8d, avg_bgain = %8d\n", awb_calib_gain->avg_r_gain, awb_calib_gain->avg_b_gain);

    return TD_SUCCESS;
}

#ifdef __LITEOS__
td_s32 app_main(td_s32 argc, td_char *argv[])
#else
td_s32 main(td_s32 argc, td_char *argv[])
#endif
{
    const ot_vi_pipe vi_pipe = 0;
    ot_isp_awb_calibration_gain awb_golden_gain = {0};
    td_s16 alpha = 0;
    td_u32 mode = 0;
    td_s32 ret;

    ret = awb_calib_check_param(argc, argv, &mode, &awb_golden_gain, &alpha);
    if (ret != TD_SUCCESS) {
        sample_print("the mode is invalid!\n");
        sample_awb_correction_usage(argv[0]);
        return TD_FAILURE;
    }

#ifdef __LITEOS__
    awb_cali_prev awb_cali_prev;
    s32ret = sample_awb_cali_start_prev(&awb_cali_prev);
    if (s32ret == TD_SUCCESS) {
        sample_print("ISP is now running normally\n");
    } else {
        sample_print("ISP is not running normally!please check it\n");
        return TD_FAILURE;
    }
    printf("input anything to continue....\n");
    getchar();
#endif

    if (awb_calib_check_ae_status(vi_pipe, 50) == TD_TRUE) { /* AC frequency is 50 */
        ot_isp_awb_calibration_gain awb_calib_gain;
        ret = awb_calib_get_corr_gain(vi_pipe, &awb_calib_gain, &awb_golden_gain, mode, alpha);
        goto EXIT;
    } else {
        printf("AE IS NOT STABLE,PLEASE WAIT");
        ret = TD_FAILURE;
        goto EXIT;
    }

EXIT:
#ifdef __LITEOS__
    printf("input anything to continue....\n");
    getchar();
    return sample_awb_cali_stop_prev(&awb_cali_prev);
#endif

    return ret;
}
