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
#include <sys/prctl.h>

#include "sample_comm.h"
#include "ot_common.h"
#include "ot_mipi_rx.h"
#include "ss_mpi_vi.h"
#include "ss_mpi_isp.h"
#include "securec.h"

#define MIPI_DEV_NAME "/dev/ot_mipi_rx"

#define FPN_FILE_NAME_LENGTH 150
#define FPN_CALIB_TIMES 8
#define WIDTH_1920 1920
#define HEIGHT_1080 1080
#define WIDTH_3840 3840
#define HEIGHT_2160 2160
#define WIDTH_2688 2688
#define HEIGHT_1520 1520
#define SLEEP_TIME 1000
#define MIPI_NUM 3
#define OB_HEIGHT_END 24
#define OB_HEIGHT_START 0
#define IMX347_OB_HEIGHT_END 20
#define IMX485_OB_HEIGHT_END 20

typedef struct {
    sample_vi_user_frame_info *user_frame_info;
    ot_vi_pipe vi_pipe;
    td_u32 frame_cnt;
} sample_vi_send_frame_info;

static td_bool g_send_pipe_pthread = TD_FALSE;
static td_bool g_start_isp[OT_VI_MAX_PIPE_NUM] = {TD_FALSE};

static ext_data_type_t g_mipi_ext_data_type_os08a20_12bit_8m_nowdr_attr = {
    .devno = 0,
    .num = MIPI_NUM,
    .ext_data_bit_width = {12, 12, 12},
    .ext_data_type = {0x37, 0x2c, 0x2c}
};

static ext_data_type_t g_mipi_ext_data_type_default_attr = {
    .devno = 0,
    .num = MIPI_NUM,
    .ext_data_bit_width = {12, 12, 12},
    .ext_data_type = {0x2c, 0x2c, 0x2c}
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_dev2_attr = {
    .devno = 2, /* dev2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 5, 6, 7, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_wdr2to1_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate = MIPI_DATA_RATE_X1,
    .img_rect = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_VC,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_wdr2to1_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate = MIPI_DATA_RATE_X1,
    .img_rect = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_VC,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_nowdr_dev2_attr = {
    .devno = 2, /* dev2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 5, 6, 7, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_2lane_chn0_sensor_os05a10_12bit_4m_nowdr_single_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, -1, -1, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_2lane_chn0_sensor_os05a10_12bit_4m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 2, -1, -1, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_2lane_chn1_sensor_os05a10_12bit_4m_nowdr_attr = {
    .devno = 1,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {1, 3, -1, -1, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_2lane_chn2_sensor_os05a10_12bit_4m_nowdr_attr = {
    .devno = 2, /* dev2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 6, -1, -1, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_2lane_chn3_sensor_os05a10_12bit_4m_nowdr_attr = {
    .devno = 3, /* dev3 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {5, 7, -1, -1, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os04a10_12bit_4m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr  = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_os04a10_12bit_4m_nowdr_dev2_attr = {
    .devno = 2, /* dev2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2688, HEIGHT_1520},
    .mipi_attr  = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 5, 6, 7, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_imx347_slave_12bit_4m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, WIDTH_2592, HEIGHT_1520},
    .mipi_attr  = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_imx347_slave_12bit_4m_nowdr_dev2_attr = {
    .devno = 2, /* dev2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 2, WIDTH_2592, HEIGHT_1520},
    .mipi_attr  = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 5, 6, 7, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn0_sensor_imx485_12bit_8m_nowdr_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate = MIPI_DATA_RATE_X1,
    .img_rect = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_4lane_chn2_sensor_imx485_12bit_8m_nowdr_attr = {
    .devno = 2, /* dev 2 */
    .input_mode = INPUT_MODE_MIPI,
    .data_rate = MIPI_DATA_RATE_X1,
    .img_rect = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_12BIT,
        OT_MIPI_WDR_MODE_NONE,
        {4, 5, 6, 7, -1, -1, -1, -1}
    }
};

static combo_dev_attr_t g_mipi_8lane_chn0_sensor_imx485_10bit_8m_wdr3to1_attr = {
    .devno = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate = MIPI_DATA_RATE_X2,
    .img_rect = {0, 0, WIDTH_3840, HEIGHT_2160},
    .mipi_attr = {
        DATA_TYPE_RAW_10BIT,
        OT_MIPI_WDR_MODE_NONE,
        {0, 1, 2, 3, 4, 5, 6, 7}
    }
};

static td_void sample_comm_vi_get_mipi_attr(sample_sns_type sns_type, combo_dev_attr_t *combo_attr)
{
    td_u32 ob_height = OB_HEIGHT_START;
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
            ob_height = OB_HEIGHT_END;
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            break;

        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
            ob_height = OB_HEIGHT_END;
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_wdr2to1_attr, sizeof(combo_dev_attr_t));
            break;

        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os04a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
            break;

        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            break;

        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_wdr2to1_attr, sizeof(combo_dev_attr_t));
            break;

        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_2lane_chn0_sensor_os05a10_12bit_4m_nowdr_single_attr, sizeof(combo_dev_attr_t));
            break;

        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            ob_height = IMX347_OB_HEIGHT_END;
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_imx347_slave_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
            break;

        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
            ob_height = IMX485_OB_HEIGHT_END;
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_imx485_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            break;

        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_8lane_chn0_sensor_imx485_10bit_8m_wdr3to1_attr, sizeof(combo_dev_attr_t));
            break;

        default:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
    }
    combo_attr->img_rect.height = combo_attr->img_rect.height + ob_height;
}

static td_void sample_comm_vi_get_mipi_ext_data_attr(sample_sns_type sns_type, ext_data_type_t *ext_data_attr)
{
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            (td_void)memcpy_s(ext_data_attr, sizeof(ext_data_type_t),
                &g_mipi_ext_data_type_os08a20_12bit_8m_nowdr_attr, sizeof(ext_data_type_t));
            break;

        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
            (td_void)memcpy_s(ext_data_attr, sizeof(ext_data_type_t),
                &g_mipi_ext_data_type_default_attr, sizeof(ext_data_type_t));
            break;

        default:
            (td_void)memcpy_s(ext_data_attr, sizeof(ext_data_type_t),
                &g_mipi_ext_data_type_default_attr, sizeof(ext_data_type_t));
    }
}

static td_void sample_comm_vi_get_os05a10_mipi_attr(ot_vi_dev vi_dev, combo_dev_attr_t *combo_attr)
{
    if (vi_dev == 0) {
        (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
            &g_mipi_2lane_chn0_sensor_os05a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
    } else if (vi_dev == 1) {
        (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
            &g_mipi_2lane_chn1_sensor_os05a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
    } else if (vi_dev == 2) { /* dev2 */
        (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
            &g_mipi_2lane_chn2_sensor_os05a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
    } else if (vi_dev == 3) { /* dev3 */
        (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
            &g_mipi_2lane_chn3_sensor_os05a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
    }
}

static td_void sample_comm_vi_get_mipi_attr_by_dev_id(sample_sns_type sns_type, ot_vi_dev vi_dev,
                                                      combo_dev_attr_t *combo_attr)
{
    td_u32 ob_height = OB_HEIGHT_START;
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
            ob_height = OB_HEIGHT_END;
            if (vi_dev == 0) {
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            } else if (vi_dev == 2) { /* dev2 */
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_dev2_attr, sizeof(combo_dev_attr_t));
            }
            break;

        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
            if (vi_dev == 0) {
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            } else if (vi_dev == 2) { /* dev2 */
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os08b10_12bit_8m_nowdr_dev2_attr, sizeof(combo_dev_attr_t));
            }
            break;

        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
            sample_comm_vi_get_os05a10_mipi_attr(vi_dev, combo_attr);
            break;

        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
            if (vi_dev == 0) {
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os04a10_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
            } else if (vi_dev == 2) { /* dev2 */
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_os04a10_12bit_4m_nowdr_dev2_attr, sizeof(combo_dev_attr_t));
            }
            break;

        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            ob_height = IMX347_OB_HEIGHT_END;
            if (vi_dev == 0) {
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_imx347_slave_12bit_4m_nowdr_attr, sizeof(combo_dev_attr_t));
            } else if (vi_dev == 2) { /* dev2 */
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_imx347_slave_12bit_4m_nowdr_dev2_attr, sizeof(combo_dev_attr_t));
            }
            break;

        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
            ob_height = IMX485_OB_HEIGHT_END;
            if (vi_dev == 0) {
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn0_sensor_imx485_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            } else if (vi_dev == 2) { /* dev2 */
                (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                    &g_mipi_4lane_chn2_sensor_imx485_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
            }
            break;

        default:
            (td_void)memcpy_s(combo_attr, sizeof(combo_dev_attr_t),
                &g_mipi_4lane_chn0_sensor_os08a20_12bit_8m_nowdr_attr, sizeof(combo_dev_attr_t));
    }
    combo_attr->img_rect.height = combo_attr->img_rect.height + ob_height;
}

static ot_vi_dev_attr g_mipi_raw_dev_attr = {
    .intf_mode = OT_VI_INTF_MODE_MIPI,

    /* Invalid argument */
    .work_mode = OT_VI_WORK_MODE_MULTIPLEX_1,

    /* mask component */
    .component_mask = {0xfff00000, 0x00000000},

    .scan_mode = OT_VI_SCAN_PROGRESSIVE,

    /* Invalid argument */
    .ad_chn_id = {-1, -1, -1, -1},

    /* data seq */
    .data_seq = OT_VI_DATA_SEQ_YVYU,

    /* sync param */
    .sync_cfg = {
        .vsync           = OT_VI_VSYNC_FIELD,
        .vsync_neg       = OT_VI_VSYNC_NEG_HIGH,
        .hsync           = OT_VI_HSYNC_VALID_SIG,
        .hsync_neg       = OT_VI_HSYNC_NEG_HIGH,
        .vsync_valid     = OT_VI_VSYNC_VALID_SIG,
        .vsync_valid_neg = OT_VI_VSYNC_VALID_NEG_HIGH,
        .timing_blank    = {
            /* hsync_hfb      hsync_act     hsync_hhb */
            0,                0,            0,
            /* vsync0_vhb     vsync0_act    vsync0_hhb */
            0,                0,            0,
            /* vsync1_vhb     vsync1_act    vsync1_hhb */
            0,                0,            0
        }
    },

    /* data type */
    .data_type = OT_VI_DATA_TYPE_RAW,

    /* data reverse */
    .data_reverse = TD_FALSE,

    /* input size */
    .in_size = {WIDTH_3840, HEIGHT_2160},

    /* data rate */
    .data_rate = OT_DATA_RATE_X1,
};

static td_void sample_comm_vi_get_dev_attr_by_intf_mode(ot_vi_intf_mode intf_mode, ot_vi_dev_attr *dev_attr)
{
    switch (intf_mode) {
        case OT_VI_INTF_MODE_MIPI:
            (td_void)memcpy_s(dev_attr, sizeof(ot_vi_dev_attr), &g_mipi_raw_dev_attr, sizeof(ot_vi_dev_attr));
            break;

        default:
            (td_void)memcpy_s(dev_attr, sizeof(ot_vi_dev_attr), &g_mipi_raw_dev_attr, sizeof(ot_vi_dev_attr));
            break;
    }
}

td_void sample_comm_vi_get_size_by_sns_type(sample_sns_type sns_type, ot_size *size)
{
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
            size->width  = WIDTH_3840;
            size->height = HEIGHT_2160;
            break;
        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
            size->width  = WIDTH_2688;
            size->height = HEIGHT_1520;
            break;
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            size->width  = WIDTH_2592;
            size->height = HEIGHT_1520;
            break;

        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
            size->width = WIDTH_2688;
            size->height = HEIGHT_1520;
            break;

        default:
            size->width  = WIDTH_1920;
            size->height = HEIGHT_1080;
            break;
    }
}

td_u32 sample_comm_vi_get_obheight_by_sns_type(sample_sns_type sns_type)
{
    td_u32 ob_height = OB_HEIGHT_START;
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
            ob_height = OB_HEIGHT_END;
            break;
        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
            ob_height = OB_HEIGHT_END;
            break;
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            ob_height = IMX347_OB_HEIGHT_END;
            break;
        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
            ob_height = IMX485_OB_HEIGHT_END;
            break;
        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
            ob_height = OB_HEIGHT_START;
            break;
        default:
            break;
    }

    return ob_height;
}

static td_u32 sample_comm_vi_get_pipe_num_by_sns_type(sample_sns_type sns_type)
{
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
            return 1;

        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
            return 2; /* 2 pipe */

        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
            return 3; /* 3 pipe */

        default:
            return 1;
    }
}

static ot_wdr_mode sample_comm_vi_get_wdr_mode_by_sns_type(sample_sns_type sns_type)
{
    switch (sns_type) {
        case OV_OS08A20_MIPI_8M_30FPS_12BIT:
        case OV_OS04A10_MIPI_4M_30FPS_12BIT:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT:
        case OV_OS05A10_SLAVE_MIPI_4M_30FPS_12BIT:
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            return OT_WDR_MODE_NONE;

        case OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1:
        case OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1:
            return OT_WDR_MODE_2To1_LINE;

        case SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1:
            return OT_WDR_MODE_3To1_LINE;

        default:
            return OT_WDR_MODE_NONE;
    }
}

td_void sample_comm_vi_get_default_sns_info(sample_sns_type sns_type, sample_sns_info *sns_info)
{
    sns_info->sns_type    = sns_type;
    sns_info->sns_clk_src = 0;
    sns_info->sns_rst_src = 0;
    sns_info->bus_id      = 2; /* i2c2  */
}

td_void sample_comm_vi_get_default_mipi_info(sample_sns_type sns_type, sample_mipi_info *mipi_info)
{
    mipi_info->mipi_dev    = 0;
    mipi_info->divide_mode = LANE_DIVIDE_MODE_0;
    sample_comm_vi_get_mipi_attr(sns_type, &mipi_info->combo_dev_attr);
    sample_comm_vi_get_mipi_ext_data_attr(sns_type, &mipi_info->ext_data_type_attr);
}

/* used for two sensor: mipi lane 4 + 4 */
td_void sample_comm_vi_get_mipi_info_by_dev_id(sample_sns_type sns_type, ot_vi_dev vi_dev, sample_mipi_info *mipi_info)
{
    mipi_info->mipi_dev    = vi_dev;
    mipi_info->divide_mode = LANE_DIVIDE_MODE_1;
    sample_comm_vi_get_mipi_attr_by_dev_id(sns_type, vi_dev, &mipi_info->combo_dev_attr);
    sample_comm_vi_get_mipi_ext_data_attr(sns_type, &mipi_info->ext_data_type_attr);
    mipi_info->ext_data_type_attr.devno = vi_dev;
}

td_void sample_comm_vi_get_default_dev_info(sample_sns_type sns_type, sample_vi_dev_info *dev_info)
{
    ot_size size;
    td_u32 ob_height;

    dev_info->vi_dev = 0;
    sample_comm_vi_get_dev_attr_by_intf_mode(OT_VI_INTF_MODE_MIPI, &dev_info->dev_attr);
    if (sns_type == SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1) {
        dev_info->dev_attr.data_rate = OT_DATA_RATE_X2;
    }
    sample_comm_vi_get_size_by_sns_type(sns_type, &size);
    ob_height = sample_comm_vi_get_obheight_by_sns_type(sns_type);
    dev_info->dev_attr.in_size.width  = size.width;
    dev_info->dev_attr.in_size.height = size.height + ob_height;
    dev_info->bas_attr.enable = TD_FALSE;
}

static td_void sample_comm_vi_get_default_bind_info(sample_sns_type sns_type, ot_vi_bind_pipe *bind_pipe)
{
    td_u32 i;

    bind_pipe->pipe_num = sample_comm_vi_get_pipe_num_by_sns_type(sns_type);
    for (i = 0; i < bind_pipe->pipe_num; i++) {
        bind_pipe->pipe_id[i] = i;
    }
}

static td_void sample_comm_vi_get_default_grp_info(sample_sns_type sns_type, sample_vi_grp_info *grp_info)
{
    td_u32 i;
    td_u32 pipe_num;
    ot_size size;

    sample_comm_vi_get_size_by_sns_type(sns_type, &size);
    grp_info->grp_num = 1;
    grp_info->fusion_grp[0] = 0;
    grp_info->fusion_grp_attr[0].wdr_mode = sample_comm_vi_get_wdr_mode_by_sns_type(sns_type);
    grp_info->fusion_grp_attr[0].cache_line = size.height;
    pipe_num = sample_comm_vi_get_pipe_num_by_sns_type(sns_type);
    for (i = 0; i < pipe_num; i++) {
        grp_info->fusion_grp_attr[0].pipe_id[i] = i;
    }
}

td_void sample_comm_vi_get_default_pipe_info(sample_sns_type sns_type, ot_vi_bind_pipe *bind_pipe,
                                             sample_vi_pipe_info pipe_info[])
{
    td_u32 i;
    ot_size size;

    sample_comm_vi_get_size_by_sns_type(sns_type, &size);

    for (i = 0; i < bind_pipe->pipe_num; i++) {
        /* pipe attr */
        pipe_info[i].pipe_attr.pipe_bypass_mode               = OT_VI_PIPE_BYPASS_NONE;
        pipe_info[i].pipe_attr.isp_bypass                     = TD_FALSE;
        pipe_info[i].pipe_attr.size.width                     = size.width;
        pipe_info[i].pipe_attr.size.height                    = size.height;
        pipe_info[i].pipe_attr.pixel_format                   = OT_PIXEL_FORMAT_RGB_BAYER_12BPP;
        pipe_info[i].pipe_attr.compress_mode                  = OT_COMPRESS_MODE_LINE;
        pipe_info[i].pipe_attr.bit_width                      = OT_DATA_BIT_WIDTH_8;
        pipe_info[i].pipe_attr.bit_align_mode                 = OT_VI_BIT_ALIGN_MODE_HIGH;
        pipe_info[i].pipe_attr.frame_rate_ctrl.src_frame_rate = -1;
        pipe_info[i].pipe_attr.frame_rate_ctrl.dst_frame_rate = -1;

        if (sns_type == SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1) {
            pipe_info[i].pipe_attr.pixel_format  = OT_PIXEL_FORMAT_RGB_BAYER_10BPP;
            pipe_info[i].pipe_attr.compress_mode = OT_COMPRESS_MODE_NONE;
        }

        pipe_info[i].pipe_need_start = TD_TRUE;
        pipe_info[i].isp_need_run = TD_TRUE;

        /* pub attr */
        sample_comm_isp_get_pub_attr_by_sns(sns_type, &pipe_info[i].isp_info.isp_pub_attr);

        /* chn info */
        pipe_info[i].chn_num = 1;
        pipe_info[i].chn_info[0].vi_chn                                  = 0;
        pipe_info[i].chn_info[0].chn_attr.size.width                     = size.width;
        pipe_info[i].chn_info[0].chn_attr.size.height                    = size.height;
        pipe_info[i].chn_info[0].chn_attr.pixel_format                   = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        pipe_info[i].chn_info[0].chn_attr.dynamic_range                  = OT_DYNAMIC_RANGE_SDR8;
        pipe_info[i].chn_info[0].chn_attr.video_format                   = OT_VIDEO_FORMAT_LINEAR;
        pipe_info[i].chn_info[0].chn_attr.compress_mode                  = OT_COMPRESS_MODE_NONE;
        pipe_info[i].chn_info[0].chn_attr.mirror_en                      = TD_FALSE;
        pipe_info[i].chn_info[0].chn_attr.flip_en                        = TD_FALSE;
        pipe_info[i].chn_info[0].chn_attr.depth                          = 0;
        pipe_info[i].chn_info[0].chn_attr.frame_rate_ctrl.src_frame_rate = -1;
        pipe_info[i].chn_info[0].chn_attr.frame_rate_ctrl.dst_frame_rate = -1;
    }
}

td_void sample_comm_vi_init_pipe_info(sample_sns_type sns_type, const ot_size *size, ot_vi_bind_pipe *bind_pipe,
    sample_vi_pipe_info pipe_info[])
{
    td_u32 i;

    for (i = 0; i < bind_pipe->pipe_num; i++) {
        /* pipe attr */
        pipe_info[i].pipe_attr.pipe_bypass_mode               = OT_VI_PIPE_BYPASS_NONE;
        pipe_info[i].pipe_attr.isp_bypass                     = TD_FALSE;
        pipe_info[i].pipe_attr.size.width                     = size->width;
        pipe_info[i].pipe_attr.size.height                    = size->height;
        pipe_info[i].pipe_attr.pixel_format                   = OT_PIXEL_FORMAT_RGB_BAYER_12BPP;
        pipe_info[i].pipe_attr.compress_mode                  = OT_COMPRESS_MODE_LINE;
        pipe_info[i].pipe_attr.bit_width                      = OT_DATA_BIT_WIDTH_8;
        pipe_info[i].pipe_attr.bit_align_mode                 = OT_VI_BIT_ALIGN_MODE_HIGH;
        pipe_info[i].pipe_attr.frame_rate_ctrl.src_frame_rate = -1;
        pipe_info[i].pipe_attr.frame_rate_ctrl.dst_frame_rate = -1;

        if (sns_type == SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1) {
            pipe_info[i].pipe_attr.pixel_format  = OT_PIXEL_FORMAT_RGB_BAYER_10BPP;
            pipe_info[i].pipe_attr.compress_mode = OT_COMPRESS_MODE_NONE;
        }

        pipe_info[i].pipe_need_start = TD_TRUE;
        pipe_info[i].isp_need_run = TD_TRUE;

        /* pub attr */
        sample_comm_isp_get_pub_attr_by_sns(sns_type, &pipe_info[i].isp_info.isp_pub_attr);

        /* chn info */
        pipe_info[i].chn_num = 1;
        pipe_info[i].chn_info[0].vi_chn                                  = 0;
        pipe_info[i].chn_info[0].chn_attr.size.width                     = size->width;
        pipe_info[i].chn_info[0].chn_attr.size.height                    = size->height;
        pipe_info[i].chn_info[0].chn_attr.pixel_format                   = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        pipe_info[i].chn_info[0].chn_attr.dynamic_range                  = OT_DYNAMIC_RANGE_SDR8;
        pipe_info[i].chn_info[0].chn_attr.video_format                   = OT_VIDEO_FORMAT_LINEAR;
        pipe_info[i].chn_info[0].chn_attr.compress_mode                  = OT_COMPRESS_MODE_NONE;
        pipe_info[i].chn_info[0].chn_attr.mirror_en                      = TD_FALSE;
        pipe_info[i].chn_info[0].chn_attr.flip_en                        = TD_FALSE;
        pipe_info[i].chn_info[0].chn_attr.depth                          = 0;
        pipe_info[i].chn_info[0].chn_attr.frame_rate_ctrl.src_frame_rate = -1;
        pipe_info[i].chn_info[0].chn_attr.frame_rate_ctrl.dst_frame_rate = -1;
    }
}

td_void sample_comm_vi_get_default_vi_cfg(sample_sns_type sns_type, sample_vi_cfg *vi_cfg)
{
    (td_void)memset_s(vi_cfg, sizeof(sample_vi_cfg), 0, sizeof(sample_vi_cfg));

    /* sensor info */
    sample_comm_vi_get_default_sns_info(sns_type, &vi_cfg->sns_info);
    /* mipi info */
    sample_comm_vi_get_default_mipi_info(sns_type, &vi_cfg->mipi_info);
    /* dev info */
    sample_comm_vi_get_default_dev_info(sns_type, &vi_cfg->dev_info);
    /* bind info */
    sample_comm_vi_get_default_bind_info(sns_type, &vi_cfg->bind_pipe);
    /* grp info */
    sample_comm_vi_get_default_grp_info(sns_type, &vi_cfg->grp_info);
    /* pipe info */
    sample_comm_vi_get_default_pipe_info(sns_type, &vi_cfg->bind_pipe, vi_cfg->pipe_info);
}

td_void sample_comm_vi_init_vi_cfg(sample_sns_type sns_type, ot_size *size, sample_vi_cfg *vi_cfg)
{
    (td_void)memset_s(vi_cfg, sizeof(sample_vi_cfg), 0, sizeof(sample_vi_cfg));

    /* sensor info */
    sample_comm_vi_get_default_sns_info(sns_type, &vi_cfg->sns_info);
    /* mipi info */
    sample_comm_vi_get_default_mipi_info(sns_type, &vi_cfg->mipi_info);
    /* dev info */
    sample_comm_vi_get_default_dev_info(sns_type, &vi_cfg->dev_info);
    /* bind info */
    sample_comm_vi_get_default_bind_info(sns_type, &vi_cfg->bind_pipe);
    /* grp info */
    sample_comm_vi_get_default_grp_info(sns_type, &vi_cfg->grp_info);
    /* pipe info */
    sample_comm_vi_init_pipe_info(sns_type, size, &vi_cfg->bind_pipe, vi_cfg->pipe_info);
}

td_s32 sample_comm_vi_set_vi_vpss_mode(ot_vi_vpss_mode_type mode_type, ot_vi_video_mode video_mode)
{
    td_u32 i;
    td_s32 ret;
    ot_vi_vpss_mode_type other_pipe_mode_type;
    ot_vi_vpss_mode vi_vpss_mode;

    if (mode_type == OT_VI_OFFLINE_VPSS_ONLINE) {
        other_pipe_mode_type = OT_VI_OFFLINE_VPSS_ONLINE;
    } else {
        other_pipe_mode_type = OT_VI_OFFLINE_VPSS_OFFLINE;
    }

    vi_vpss_mode.mode[0] = mode_type;
    for (i = 1; i < OT_VI_MAX_PIPE_NUM; i++) {
        vi_vpss_mode.mode[i] = other_pipe_mode_type;
    }

    ret = ss_mpi_sys_set_vi_vpss_mode(&vi_vpss_mode);
    if (ret != TD_SUCCESS) {
        sample_print("set vi vpss mode failed!\n");
        return TD_FAILURE;
    }

    ret = ss_mpi_sys_set_vi_video_mode(video_mode);
    if (ret != TD_SUCCESS) {
        sample_print("set vi video mode failed!\n");
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_set_mipi_hs_mode(lane_divide_mode_t hs_mode)
{
    td_s32 fd;
    td_s32 ret;

    fd = open(MIPI_DEV_NAME, O_RDWR);
    if (fd < 0) {
        sample_print("open %s failed!\n", MIPI_DEV_NAME);
        return TD_FAILURE;
    }

    ret = ioctl(fd, OT_MIPI_SET_HS_MODE, &hs_mode);

    close(fd);

    return ret;
}

static td_s32 sample_comm_vi_mipi_ctrl_cmd(td_u32 devno, td_u32 cmd)
{
    td_s32 ret;
    td_s32 fd;

    fd = open(MIPI_DEV_NAME, O_RDWR);
    if (fd < 0) {
        sample_print("open %s failed!\n", MIPI_DEV_NAME);
        return TD_FAILURE;
    }

    ret = ioctl(fd, cmd, &devno);

    close(fd);

    return ret;
}

static td_s32 sample_comm_vi_set_mipi_combo_attr(const combo_dev_attr_t *combo_dev_attr)
{
    td_s32 fd;
    td_s32 ret;

    fd = open(MIPI_DEV_NAME, O_RDWR);
    if (fd < 0) {
        sample_print("open %s failed!\n", MIPI_DEV_NAME);
        return TD_FAILURE;
    }

    ret = ioctl(fd, OT_MIPI_SET_DEV_ATTR, combo_dev_attr);

    close(fd);

    return ret;
}

static td_s32 sample_comm_vi_set_mipi_ext_data_type_attr(const ext_data_type_t *ext_data_type_attr)
{
    td_s32 fd;
    td_s32 ret;

    fd = open(MIPI_DEV_NAME, O_RDWR);
    if (fd < 0) {
        sample_print("open %s failed!\n", MIPI_DEV_NAME);
        return TD_FAILURE;
    }

    ret = ioctl(fd, OT_MIPI_SET_EXT_DATA_TYPE, ext_data_type_attr);

    close(fd);

    return ret;
}

static td_s32 sample_comm_vi_start_mipi_rx(const sample_sns_info *sns_info, const sample_mipi_info *mipi_info)
{
    td_s32 ret;

    ret = sample_comm_vi_set_mipi_hs_mode(mipi_info->divide_mode);
    if (ret != TD_SUCCESS) {
        sample_print("mipi rx set hs_mode failed!\n");
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(mipi_info->mipi_dev, OT_MIPI_ENABLE_MIPI_CLOCK);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d enable mipi rx clock failed!\n", mipi_info->mipi_dev);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(mipi_info->mipi_dev, OT_MIPI_RESET_MIPI);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d reset mipi rx failed!\n", mipi_info->mipi_dev);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(sns_info->sns_clk_src, OT_MIPI_ENABLE_SENSOR_CLOCK);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d enable sensor clock failed!\n", sns_info->sns_clk_src);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(sns_info->sns_rst_src, OT_MIPI_RESET_SENSOR);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d reset sensor failed!\n", sns_info->sns_rst_src);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_set_mipi_combo_attr(&mipi_info->combo_dev_attr);
    if (ret != TD_SUCCESS) {
        sample_print("mipi rx set combo attr failed!\n");
        return TD_FAILURE;
    }

    ret = sample_comm_vi_set_mipi_ext_data_type_attr(&mipi_info->ext_data_type_attr);
    if (ret != TD_SUCCESS) {
        sample_print("mipi rx set ext data attr failed!\n");
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(mipi_info->mipi_dev, OT_MIPI_UNRESET_MIPI);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d unreset mipi rx failed!\n", mipi_info->mipi_dev);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(sns_info->sns_rst_src, OT_MIPI_UNRESET_SENSOR);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d unreset sensor failed!\n", sns_info->sns_rst_src);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_void sample_comm_vi_stop_mipi_rx(const sample_sns_info *sns_info, const sample_mipi_info *mipi_info)
{
    td_s32 ret;

    ret = sample_comm_vi_mipi_ctrl_cmd(mipi_info->mipi_dev, OT_MIPI_RESET_MIPI);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d reset mipi rx failed!\n", mipi_info->mipi_dev);
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(mipi_info->mipi_dev, OT_MIPI_DISABLE_MIPI_CLOCK);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d disable mipi rx clock failed!\n", mipi_info->mipi_dev);
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(sns_info->sns_rst_src, OT_MIPI_RESET_SENSOR);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d reset sensor failed!\n", sns_info->sns_rst_src);
    }

    ret = sample_comm_vi_mipi_ctrl_cmd(sns_info->sns_clk_src, OT_MIPI_DISABLE_SENSOR_CLOCK);
    if (ret != TD_SUCCESS) {
        sample_print("devno %d disable sensor clock failed!\n", sns_info->sns_clk_src);
    }
}

static td_s32 sample_comm_vi_start_dev(ot_vi_dev vi_dev, const ot_vi_dev_attr *dev_attr, const ot_vi_bas_attr *bas_attr)
{
    td_s32 ret;

    ret = ss_mpi_vi_set_dev_attr(vi_dev, dev_attr);
    if (ret != TD_SUCCESS) {
        sample_print("vi set dev attr failed with 0x%x!\n", ret);
        return TD_FAILURE;
    }

    if ((bas_attr->enable == TD_TRUE) && (vi_dev == 0)) {
        ret = ss_mpi_vi_set_bas_attr(vi_dev, bas_attr);
        if (ret != TD_SUCCESS) {
            sample_print("vi set bas attr failed with 0x%x!\n", ret);
            return TD_FAILURE;
        }
    }

    ret = ss_mpi_vi_enable_dev(vi_dev);
    if (ret != TD_SUCCESS) {
        sample_print("vi enable dev failed with 0x%x!\n", ret);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_void sample_comm_vi_stop_dev(ot_vi_dev vi_dev)
{
    td_s32 ret;

    ret = ss_mpi_vi_disable_dev(vi_dev);
    if (ret != TD_SUCCESS) {
        sample_print("vi disable dev failed with 0x%x!\n", ret);
    }
}

static td_s32 sample_comm_vi_dev_bind_pipe(ot_vi_dev vi_dev, const ot_vi_bind_pipe *bind_pipe)
{
    td_u32 i;
    td_s32 j;
    td_s32 ret;

    for (i = 0; i < bind_pipe->pipe_num; i++) {
        ret = ss_mpi_vi_bind(vi_dev, bind_pipe->pipe_id[i]);
        if (ret != TD_SUCCESS) {
            sample_print("vi dev(%d) bind pipe(%d) failed!\n", vi_dev, bind_pipe->pipe_id[i]);
            goto exit;
        }
    }

    return TD_SUCCESS;

exit:
    for (j = i - 1; j >= 0; j--) {
        ret = ss_mpi_vi_unbind(vi_dev, bind_pipe->pipe_id[j]);
        if (ret != TD_SUCCESS) {
            sample_print("vi dev(%d) unbind pipe(%d) failed!\n", vi_dev, bind_pipe->pipe_id[j]);
        }
    }
    return TD_FAILURE;
}

static td_void sample_comm_vi_dev_unbind_pipe(ot_vi_dev vi_dev, const ot_vi_bind_pipe *bind_pipe)
{
    td_u32 i;
    td_s32 ret;

    for (i = 0; i < bind_pipe->pipe_num; i++) {
        ret = ss_mpi_vi_unbind(vi_dev, bind_pipe->pipe_id[i]);
        if (ret != TD_SUCCESS) {
            sample_print("vi dev(%d) unbind pipe(%d) failed!\n", vi_dev, bind_pipe->pipe_id[i]);
        }
    }
}

static td_s32 sample_comm_vi_set_grp_info(const sample_vi_grp_info *grp_info)
{
    td_s32 ret;
    td_u32 i;
    for (i = 0; i < grp_info->grp_num; i++) {
        ret = ss_mpi_vi_set_wdr_fusion_grp_attr(grp_info->fusion_grp[i], &grp_info->fusion_grp_attr[i]);
        if (ret != TD_SUCCESS) {
            sample_print("vi set wdr fusion grp attr failed!\n");
            return TD_FAILURE;
        }
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_start_chn(ot_vi_pipe vi_pipe, const sample_vi_chn_info chn_info[], td_u32 chn_num)
{
    td_u32 i;
    td_s32 ret;

    for (i = 0; i < chn_num; i++) {
        ot_vi_chn vi_chn = chn_info[i].vi_chn;
        const ot_vi_chn_attr *chn_attr = &chn_info[i].chn_attr;

        ret = ss_mpi_vi_set_chn_attr(vi_pipe, vi_chn, chn_attr);
        if (ret != TD_SUCCESS) {
            sample_print("vi set chn(%d) attr failed with 0x%x!\n", vi_chn, ret);
            return TD_FAILURE;
        }

        ret = ss_mpi_vi_enable_chn(vi_pipe, vi_chn);
        if (ret != TD_SUCCESS) {
            sample_print("vi enable chn(%d) failed with 0x%x!\n", vi_chn, ret);
            return TD_FAILURE;
        }
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_switch_mode_start_chn(ot_vi_pipe vi_pipe,
                                                   const sample_vi_chn_info chn_info[],
                                                   td_u32 chn_num)
{
    td_u32 i;
    td_s32 ret;

    for (i = 0; i < chn_num; i++) {
        ot_vi_chn vi_chn = chn_info[i].vi_chn;
        const ot_vi_chn_attr *chn_attr = &chn_info[i].chn_attr;

        ret = ss_mpi_vi_set_chn_attr(vi_pipe, vi_chn, chn_attr);
        if (ret != TD_SUCCESS) {
            sample_print("vi set chn(%d) attr failed with 0x%x!\n", vi_chn, ret);
            return TD_FAILURE;
        }
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_stop_chn(ot_vi_pipe vi_pipe, const sample_vi_chn_info chn_info[], td_u32 chn_num)
{
    td_u32 i;
    td_s32 ret;

    for (i = 0; i < chn_num; i++) {
        ot_vi_chn vi_chn = chn_info[i].vi_chn;

        ret = ss_mpi_vi_disable_chn(vi_pipe, vi_chn);
        if (ret != TD_SUCCESS) {
            sample_print("vi disable chn(%d) failed with 0x%x!\n", vi_chn, ret);
            return TD_FAILURE;
        }
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_start_one_pipe(ot_vi_pipe vi_pipe, const sample_vi_pipe_info *pipe_info)
{
    td_s32 ret;

    ret = ss_mpi_vi_create_pipe(vi_pipe, &pipe_info->pipe_attr);
    if (ret != TD_SUCCESS) {
        sample_print("vi create pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
        return TD_FAILURE;
    }

    if (pipe_info->pipe_need_start == TD_TRUE) {
        ret = ss_mpi_vi_start_pipe(vi_pipe);
        if (ret != TD_SUCCESS) {
            sample_print("vi start pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
            goto start_pipe_failed;
        }
    }

    ret = sample_comm_vi_start_chn(vi_pipe, pipe_info->chn_info, pipe_info->chn_num);
    if (ret != TD_SUCCESS) {
        sample_print("vi pipe(%d) start chn failed!\n", vi_pipe);
        goto start_chn_failed;
    }

    return TD_SUCCESS;

start_chn_failed:
    ss_mpi_vi_stop_pipe(vi_pipe);
start_pipe_failed:
    ss_mpi_vi_destroy_pipe(vi_pipe);
    return TD_FAILURE;
}

static td_void sample_comm_vi_stop_one_pipe(ot_vi_pipe vi_pipe, const sample_vi_pipe_info *pipe_info)
{
    td_s32 ret;

    ret = sample_comm_vi_stop_chn(vi_pipe, pipe_info->chn_info, pipe_info->chn_num);
    if (ret != TD_SUCCESS) {
        sample_print("vi pipe(%d) stop chn failed!\n", vi_pipe);
    }

    ret = ss_mpi_vi_stop_pipe(vi_pipe);
    if (ret != TD_SUCCESS) {
        sample_print("vi stop pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
    }

    ret = ss_mpi_vi_destroy_pipe(vi_pipe);
    if (ret != TD_SUCCESS) {
        sample_print("vi destroy pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
    }
}

static td_s32 sample_comm_vi_start_pipe(const ot_vi_bind_pipe *bind_pipe, const sample_vi_pipe_info pipe_info[])
{
    td_s32 i;
    td_s32 ret;

    for (i = 0; i < (td_s32)bind_pipe->pipe_num; i++) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        ret = sample_comm_vi_start_one_pipe(vi_pipe, &pipe_info[i]);
        if (ret != TD_SUCCESS) {
            goto exit;
        }
    }

    return TD_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        sample_comm_vi_stop_one_pipe(vi_pipe, &pipe_info[i]);
    }
    return TD_FAILURE;
}

static td_void sample_comm_vi_stop_pipe(const ot_vi_bind_pipe *bind_pipe, const sample_vi_pipe_info pipe_info[])
{
    td_s32 i;
    for (i = bind_pipe->pipe_num - 1; i >= 0; i--) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        sample_comm_vi_stop_one_pipe(vi_pipe, &pipe_info[i]);
    }
}

static td_s32 sample_comm_vi_register_sensor_lib(ot_vi_pipe vi_pipe, td_u8 pipe_index, const sample_vi_cfg *vi_cfg)
{
    td_s32 ret;
    td_u32 bus_id;
    sample_sns_type sns_type = vi_cfg->sns_info.sns_type;

    ret = sample_comm_isp_sensor_regiter_callback(vi_pipe, sns_type);
    if (ret != TD_SUCCESS) {
        printf("register sensor to ISP %d failed\n", vi_pipe);
        return TD_FAILURE;
    }

    if (pipe_index > 0) {
        bus_id = -1;
    } else {
        bus_id = vi_cfg->sns_info.bus_id;
    }

    ret = sample_comm_isp_bind_sns(vi_pipe, sns_type, bus_id);
    if (ret != TD_SUCCESS) {
        printf("register sensor bus id %d failed\n", bus_id);
        goto exit0;
    }

    ret = sample_comm_isp_ae_lib_callback(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("isp_mst_comm_ae_lib_callback failed\n");
        goto exit0;
    }

    ret = sample_comm_isp_awb_lib_callback(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("isp_mst_comm_awb_lib_callback failed\n");
        goto exit1;
    }

    return TD_SUCCESS;

exit1:
    sample_comm_isp_ae_lib_uncallback(vi_pipe);
exit0:
    sample_comm_isp_sensor_unregiter_callback(vi_pipe);
    return ret;
}

static td_void sample_comm_vi_deregister_sensor_lib(ot_vi_pipe vi_pipe)
{
    sample_comm_isp_awb_lib_uncallback(vi_pipe);
    sample_comm_isp_ae_lib_uncallback(vi_pipe);
    sample_comm_isp_sensor_unregiter_callback(vi_pipe);
}

static td_s32 sample_comm_vi_start_one_pipe_isp(ot_vi_pipe vi_pipe, td_u8 pipe_index, const sample_vi_cfg *vi_cfg)
{
    td_s32 ret;

    ret = sample_comm_vi_register_sensor_lib(vi_pipe, pipe_index, vi_cfg);
    if (ret != TD_SUCCESS) {
        printf("register sensor to ISP %d failed\n", vi_pipe);
        return TD_FAILURE;
    }

    ret = ss_mpi_isp_mem_init(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("OT_MPI_ISP_MemInit failed with 0x%x!\n", ret);
        goto exit0;
    }

    ret = ss_mpi_isp_set_pub_attr(vi_pipe, &vi_cfg->pipe_info[pipe_index].isp_info.isp_pub_attr);
    if (ret != TD_SUCCESS) {
        printf("OT_MPI_ISP_SetPubAttr failed with 0x%x!\n", ret);
        goto exit1;
    }

    ret = ss_mpi_isp_init(vi_pipe);
    if (ret != TD_SUCCESS) {
        printf("OT_MPI_ISP_Init failed with 0x%x!\n", ret);
        return -1;
    }

    ret = sample_comm_isp_sensor_founction_cfg(vi_pipe, vi_cfg->sns_info.sns_type);
    if (ret != TD_SUCCESS) {
        printf("sensor founction cfg failed with 0x%x!\n", ret);
        return -1;
    }

    if ((vi_pipe < OT_VI_MAX_PHYS_PIPE_NUM) &&
        (vi_cfg->pipe_info[pipe_index].isp_need_run == TD_TRUE)) {
        ret = sample_comm_isp_run(vi_pipe);
        if (ret != TD_SUCCESS) {
            printf("ISP Run failed with 0x%x!\n", ret);
            goto exit1;
        }
    }

    g_start_isp[vi_pipe] = TD_TRUE;

    return TD_SUCCESS;

exit1:
    ss_mpi_isp_exit(vi_pipe);
exit0:
    sample_comm_vi_deregister_sensor_lib(vi_pipe);
    return ret;
}

static td_void sample_comm_vi_stop_one_pipe_isp(ot_vi_pipe vi_pipe)
{
    ss_mpi_isp_exit(vi_pipe);
    sample_comm_isp_stop(vi_pipe);
    sample_comm_vi_deregister_sensor_lib(vi_pipe);

    g_start_isp[vi_pipe] = TD_FALSE;
}

static td_s32 sample_comm_vi_start_isp(const sample_vi_cfg *vi_cfg)
{
    td_s8 i, j;
    td_s32 ret;
    ot_vi_pipe vi_pipe;
    for (i = 0; i < (td_u8)vi_cfg->bind_pipe.pipe_num; i++) {
        vi_pipe = vi_cfg->bind_pipe.pipe_id[i];
        if ((vi_cfg->grp_info.fusion_grp_attr[0].wdr_mode != OT_WDR_MODE_NONE) &&
            (vi_cfg->grp_info.fusion_grp_attr[0].wdr_mode != OT_WDR_MODE_BUILT_IN) &&
            (i > 0)) {
            continue;
        }
        ret = sample_comm_vi_start_one_pipe_isp(vi_pipe, i, vi_cfg);
        if (ret != TD_SUCCESS) {
            for (j = i - 1; (j >= 0) && (i != 0); j--) {
                vi_pipe = vi_cfg->bind_pipe.pipe_id[j];
                sample_comm_vi_stop_one_pipe_isp(vi_pipe);
            }

            return ret;
        }
    }

    return TD_SUCCESS;
}

static td_void sample_comm_vi_stop_isp(const sample_vi_cfg *vi_cfg)
{
    td_u32     i;
    td_bool    start_pipe;
    ot_vi_pipe vi_pipe;

    for (i = 0; i < vi_cfg->bind_pipe.pipe_num; i++) {
        if ((vi_cfg->pipe_info[i].isp_info.isp_pub_attr.wdr_mode == OT_WDR_MODE_NONE) ||
            (vi_cfg->pipe_info[i].isp_info.isp_pub_attr.wdr_mode == OT_WDR_MODE_BUILT_IN)) {
            start_pipe = TD_TRUE;
        } else {
            start_pipe = (i > 0) ? TD_FALSE : TD_TRUE;
        }

        if (start_pipe != TD_TRUE) {
            continue;
        }

        vi_pipe = vi_cfg->bind_pipe.pipe_id[i];
        sample_comm_vi_stop_one_pipe_isp(vi_pipe);
    }
}

td_s32 sample_comm_vi_start_vi(const sample_vi_cfg *vi_cfg)
{
    td_s32 ret;
    ot_vi_dev vi_dev;

    ret = sample_comm_vi_start_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
    if (ret != TD_SUCCESS) {
        sample_print("start mipi rx failed!\n");
        goto start_mipi_rx_failed;
    }

    vi_dev   = vi_cfg->dev_info.vi_dev;
    ret = sample_comm_vi_start_dev(vi_dev, &vi_cfg->dev_info.dev_attr, &vi_cfg->dev_info.bas_attr);
    if (ret != TD_SUCCESS) {
        sample_print("start dev failed!\n");
        goto start_dev_failed;
    }

    ret = sample_comm_vi_dev_bind_pipe(vi_dev, &vi_cfg->bind_pipe);
    if (ret != TD_SUCCESS) {
        sample_print("dev bind pipe failed!\n");
        goto dev_bind_pipe_failed;
    }

    ret = sample_comm_vi_set_grp_info(&vi_cfg->grp_info);
    if (ret != TD_SUCCESS) {
        sample_print("set grp info failed!\n");
        goto set_grp_info_failed;
    }

    ret = sample_comm_vi_start_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
    if (ret != TD_SUCCESS) {
        sample_print("start pipe failed!\n");
        goto start_pipe_failed;
    }

    ret = sample_comm_vi_start_isp(vi_cfg);
    if (ret != TD_SUCCESS) {
        sample_print("start isp failed!\n");
        goto start_isp_failed;
    }

    return TD_SUCCESS;

start_isp_failed:
    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
start_pipe_failed: /* fall through */
set_grp_info_failed:
    sample_comm_vi_dev_unbind_pipe(vi_dev, &vi_cfg->bind_pipe);
dev_bind_pipe_failed:
    sample_comm_vi_stop_dev(vi_dev);
start_dev_failed:
    sample_comm_vi_stop_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
start_mipi_rx_failed:
    return TD_FAILURE;
}

td_void sample_comm_vi_stop_vi(const sample_vi_cfg *vi_cfg)
{
    ot_vi_dev vi_dev = vi_cfg->dev_info.vi_dev;

    sample_comm_vi_stop_isp(vi_cfg);
    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
    sample_comm_vi_dev_unbind_pipe(vi_dev, &vi_cfg->bind_pipe);
    sample_comm_vi_stop_dev(vi_dev);
    sample_comm_vi_stop_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
}

td_void sample_comm_vi_mode_switch_stop_vi(const sample_vi_cfg *vi_cfg)
{
    ot_vi_dev vi_dev = vi_cfg->dev_info.vi_dev;

    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
    sample_comm_vi_dev_unbind_pipe(vi_dev, &vi_cfg->bind_pipe);
    sample_comm_vi_stop_dev(vi_dev);
    sample_comm_vi_stop_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
}

static td_s32 sample_comm_vi_mode_switch_start_one_pipe(ot_vi_pipe vi_pipe, const sample_vi_pipe_info *pipe_info)
{
    td_s32 ret;

    ret = ss_mpi_vi_create_pipe(vi_pipe, &pipe_info->pipe_attr);
    if (ret != TD_SUCCESS) {
        sample_print("vi create pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_switch_mode_start_chn(vi_pipe, pipe_info->chn_info, pipe_info->chn_num);
    if (ret != TD_SUCCESS) {
        sample_print("vi pipe(%d) start chn failed!\n", vi_pipe);
        goto start_chn_failed;
    }

    return TD_SUCCESS;

start_chn_failed:
    ss_mpi_vi_stop_pipe(vi_pipe);
    return TD_FAILURE;
}

static td_s32 sample_comm_vi_mode_switch_start_pipe(const ot_vi_bind_pipe *bind_pipe,
                                                    const sample_vi_pipe_info pipe_info[])
{
    td_s32 i;
    td_s32 ret;

    for (i = 0; i < (td_s32)bind_pipe->pipe_num; i++) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        ret = sample_comm_vi_mode_switch_start_one_pipe(vi_pipe, &pipe_info[i]);
        if (ret != TD_SUCCESS) {
            goto exit;
        }
    }

    return TD_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        sample_comm_vi_stop_one_pipe(vi_pipe, &pipe_info[i]);
    }
    return TD_FAILURE;
}

td_s32 sample_comm_vi_mode_switch_start_vi(const sample_vi_cfg *vi_cfg, td_bool chg_resolution, const ot_size *size)
{
    td_s32 ret;
    ot_vi_dev vi_dev;

    ret = sample_comm_vi_start_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
    if (ret != TD_SUCCESS) {
        sample_print("start mipi rx failed!\n");
        goto start_mipi_rx_failed;
    }

    vi_dev   = vi_cfg->dev_info.vi_dev;
    ret = sample_comm_vi_start_dev(vi_dev, &vi_cfg->dev_info.dev_attr, &vi_cfg->dev_info.bas_attr);
    if (ret != TD_SUCCESS) {
        sample_print("start dev failed!\n");
        goto start_dev_failed;
    }

    ret = sample_comm_vi_dev_bind_pipe(vi_dev, &vi_cfg->bind_pipe);
    if (ret != TD_SUCCESS) {
        sample_print("dev bind pipe failed!\n");
        goto dev_bind_pipe_failed;
    }

    ret = sample_comm_vi_set_grp_info(&vi_cfg->grp_info);
    if (ret != TD_SUCCESS) {
        sample_print("set grp info failed!\n");
        goto set_grp_info_failed;
    }

    ret = sample_comm_vi_mode_switch_start_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
    if (ret != TD_SUCCESS) {
        sample_print("set grp info failed!\n");
        goto set_grp_info_failed;
    }

    if (chg_resolution == TD_TRUE) {
        ret = sample_comm_vi_switch_isp_resolution(vi_cfg, size);
    } else {
        ret = sample_comm_vi_switch_isp_mode(vi_cfg);
    }
    if (ret != TD_SUCCESS) {
        sample_print("sample_comm_vi_start_isp failed!\n");
        goto start_isp_failed;
    }

    return TD_SUCCESS;

start_isp_failed:
    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
set_grp_info_failed:
    sample_comm_vi_dev_unbind_pipe(vi_dev, &vi_cfg->bind_pipe);
dev_bind_pipe_failed:
    sample_comm_vi_stop_dev(vi_dev);
start_dev_failed:
    sample_comm_vi_stop_mipi_rx(&vi_cfg->sns_info, &vi_cfg->mipi_info);
start_mipi_rx_failed:
    return TD_FAILURE;
}

static td_s32 sample_comm_vi_mode_switch_start_one_pipe_chn(ot_vi_pipe vi_pipe, const sample_vi_pipe_info *pipe_info)
{
    td_s32 ret;

    ret = ss_mpi_vi_start_pipe(vi_pipe);
    if (ret != TD_SUCCESS) {
        sample_print("vi start pipe(%d) failed with 0x%x!\n", vi_pipe, ret);
        goto start_pipe_failed;
    }

    ret = sample_comm_vi_start_chn(vi_pipe, pipe_info->chn_info, pipe_info->chn_num);
    if (ret != TD_SUCCESS) {
        sample_print("vi pipe(%d) start chn failed!\n", vi_pipe);
        goto start_chn_failed;
    }

    return TD_SUCCESS;

start_chn_failed:
    ss_mpi_vi_stop_pipe(vi_pipe);
start_pipe_failed:
    ss_mpi_vi_destroy_pipe(vi_pipe);
    return TD_FAILURE;
}

static td_s32 sample_comm_vi_mode_switch_start_pipe_chn(const ot_vi_bind_pipe *bind_pipe,
                                                        const sample_vi_pipe_info pipe_info[])
{
    td_s32 i;
    td_s32 ret;

    for (i = 0; i < (td_s32)bind_pipe->pipe_num; i++) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        ret = sample_comm_vi_mode_switch_start_one_pipe_chn(vi_pipe, &pipe_info[i]);
        if (ret != TD_SUCCESS) {
            goto exit;
        }
    }

    return TD_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        ot_vi_pipe vi_pipe = bind_pipe->pipe_id[i];
        sample_comm_vi_stop_one_pipe(vi_pipe, &pipe_info[i]);
    }
    return TD_FAILURE;
}

static td_void sample_comoon_vi_query_isp_inner_state_info(ot_vi_pipe vi_pipe, td_bool switch_wdr)
{
    ot_isp_inner_state_info inner_state_info;
    td_bool switch_finish;
    td_s32 i;
    const td_u32 dev_num = 1;

    while (1) {
        switch_finish = TD_TRUE;
        for (i = 0; i < dev_num; i++) {
            ss_mpi_isp_query_inner_state_info(vi_pipe, &inner_state_info);
            if (switch_wdr == TD_TRUE) {
                switch_finish &= inner_state_info.wdr_switch_finish;
            } else {
                switch_finish &= inner_state_info.res_switch_finish;
            }
        }
        if (switch_finish == TD_TRUE) {
            sample_print("switch finish !\n");
            break;
        }
        ot_usleep(SLEEP_TIME);
    }
}

static td_bool sample_common_vi_check_need_pipe(ot_vi_pipe vi_pipe, ot_wdr_mode wdr_mode, td_u32 index)
{
    td_bool need_pipe = TD_FALSE;

    if (vi_pipe < 0 || vi_pipe >= OT_VI_MAX_PHYS_PIPE_NUM) {
        return need_pipe;
    }

    if (wdr_mode == OT_WDR_MODE_NONE) {
        need_pipe = TD_TRUE;
    } else {
        need_pipe = (index > 0) ? TD_FALSE : TD_TRUE;
    }

    return need_pipe;
}

td_s32 sample_comm_vi_switch_isp_mode(const sample_vi_cfg *vi_cfg)
{
    td_s32  i, j, ret;
    const td_s32  dev_num = 1;
    td_bool need_pipe;
    td_bool switch_wdr[OT_VI_MAX_PHYS_PIPE_NUM] = {TD_FALSE};
    ot_vi_pipe vi_pipe;
    ot_isp_pub_attr pub_attr, pre_pub_attr;

    for (i = 0; i < dev_num; i++) {
        for (j = 0; j < vi_cfg->bind_pipe.pipe_num; j++) {
            vi_pipe = vi_cfg->bind_pipe.pipe_id[j];
            need_pipe = sample_common_vi_check_need_pipe(vi_pipe,
                vi_cfg->pipe_info[j].isp_info.isp_pub_attr.wdr_mode, j);
            if (TD_TRUE != need_pipe) {
                continue;
            }

            sample_comm_isp_get_pub_attr_by_sns(vi_cfg->sns_info.sns_type, &pub_attr);
            pub_attr.wdr_mode = vi_cfg->pipe_info[j].isp_info.isp_pub_attr.wdr_mode;

            ret = ss_mpi_isp_get_pub_attr(vi_pipe, &pre_pub_attr);
            if (ret != TD_SUCCESS) {
                sample_print("ss_mpi_isp_get_pub_attr failed!\n");
                sample_comm_vi_stop_isp(vi_cfg);
            }
            ret = ss_mpi_isp_set_pub_attr(vi_pipe, &pub_attr);
            if (ret != TD_SUCCESS) {
                sample_print("ss_mpi_isp_set_pub_attr failed!\n");
                sample_comm_vi_stop_isp(vi_cfg);
            }

            if (pre_pub_attr.wdr_mode != pub_attr.wdr_mode) {
                switch_wdr[vi_pipe] = TD_TRUE;
            }
        }
    }

    vi_pipe = vi_cfg->bind_pipe.pipe_id[0];
    sample_comoon_vi_query_isp_inner_state_info(vi_pipe, switch_wdr[vi_pipe]);

    for (i = 0; i < dev_num; i++) {
        ret = sample_comm_vi_mode_switch_start_pipe_chn(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
        if (ret != TD_SUCCESS) {
            sample_print("set grp info failed!\n");
            goto start_pipe_failed;
        }

        return TD_SUCCESS;

        start_pipe_failed: /* fall through */
        sample_comm_vi_dev_unbind_pipe(vi_cfg->dev_info.vi_dev, &vi_cfg->bind_pipe);
    }

    return TD_SUCCESS;
}

td_s32 sample_comm_vi_switch_isp_resolution(const sample_vi_cfg *vi_cfg, const ot_size *size)
{
    td_s32  i, j, ret;
    const td_s32  dev_num = 1;
    td_bool need_pipe;
    td_bool switch_wdr[OT_VI_MAX_PHYS_PIPE_NUM] = {TD_FALSE};
    ot_vi_pipe vi_pipe;
    ot_isp_pub_attr pub_attr, pre_pub_attr;

    for (i = 0; i < dev_num; i++) {
        for (j = 0; j < vi_cfg->bind_pipe.pipe_num; j++) {
            vi_pipe = vi_cfg->bind_pipe.pipe_id[j];
            need_pipe = sample_common_vi_check_need_pipe(vi_pipe,
                vi_cfg->pipe_info[j].isp_info.isp_pub_attr.wdr_mode, j);
            if (TD_TRUE != need_pipe) {
                continue;
            }

            sample_comm_isp_get_pub_attr_by_sns(vi_cfg->sns_info.sns_type, &pub_attr);
            pub_attr.wdr_mode = vi_cfg->pipe_info[j].isp_info.isp_pub_attr.wdr_mode;
            pub_attr.wnd_rect.width = size->width;
            pub_attr.wnd_rect.height = size->height;

            ret = ss_mpi_isp_get_pub_attr(vi_pipe, &pre_pub_attr);
            if (ret != TD_SUCCESS) {
                sample_print("ss_mpi_isp_get_pub_attr failed!\n");
                sample_comm_vi_stop_isp(vi_cfg);
            }
            ret = ss_mpi_isp_set_pub_attr(vi_pipe, &pub_attr);
            if (ret != TD_SUCCESS) {
                sample_print("ss_mpi_isp_set_pub_attr failed!\n");
                sample_comm_vi_stop_isp(vi_cfg);
            }

            if (pre_pub_attr.wdr_mode != pub_attr.wdr_mode) {
                switch_wdr[vi_pipe] = TD_TRUE;
            }
        }
    }

    vi_pipe = vi_cfg->bind_pipe.pipe_id[0];
    sample_comoon_vi_query_isp_inner_state_info(vi_pipe, switch_wdr[vi_pipe]);

    for (i = 0; i < dev_num; i++) {
        ret = sample_comm_vi_mode_switch_start_pipe_chn(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
        if (ret != TD_SUCCESS) {
            sample_print("set grp info failed!\n");
            goto start_pipe_failed;
        }

        return TD_SUCCESS;

        start_pipe_failed: /* fall through */
        sample_comm_vi_dev_unbind_pipe(vi_cfg->dev_info.vi_dev, &vi_cfg->bind_pipe);
    }

    return TD_SUCCESS;
}

/* use this func to exit vi when start 4 route of vi */
td_void sample_comm_vi_stop_four_vi(const sample_vi_cfg *vi_cfg, td_s32 route_num)
{
    td_s32 i;
    ot_vi_dev vi_dev;

    for (i = 0; i < route_num; i++) {
        vi_dev = vi_cfg[i].dev_info.vi_dev;
        sample_comm_vi_stop_isp(&vi_cfg[i]);
        sample_comm_vi_stop_pipe(&vi_cfg[i].bind_pipe, vi_cfg[i].pipe_info);
        sample_comm_vi_dev_unbind_pipe(vi_dev, &vi_cfg[i].bind_pipe);
        sample_comm_vi_stop_dev(vi_dev);
    }

    for (i = 0; i < route_num; i++) {
        sample_comm_vi_stop_mipi_rx(&vi_cfg[i].sns_info, &vi_cfg[i].mipi_info);
    }
}

static td_void sample_comm_vi_get_vb_calc_cfg(sample_vi_get_frame_vb_cfg *get_frame_vb_cfg, ot_vb_calc_cfg *calc_cfg)
{
    ot_pic_buf_attr buf_attr;

    buf_attr.width         = get_frame_vb_cfg->size.width;
    buf_attr.height        = get_frame_vb_cfg->size.height;
    buf_attr.align         = OT_DEFAULT_ALIGN;
    buf_attr.bit_width     =
        (get_frame_vb_cfg->dynamic_range == OT_DYNAMIC_RANGE_SDR8) ? OT_DATA_BIT_WIDTH_8 : OT_DATA_BIT_WIDTH_10;
    buf_attr.pixel_format  = get_frame_vb_cfg->pixel_format;
    buf_attr.compress_mode = get_frame_vb_cfg->compress_mode;

    ot_common_get_pic_buf_cfg(&buf_attr, calc_cfg);
}

static td_s32 sample_comm_vi_malloc_frame_blk(ot_vb_pool pool_id,
                                              sample_vi_get_frame_vb_cfg *get_frame_vb_cfg, ot_vb_calc_cfg *calc_cfg,
                                              sample_vi_user_frame_info *user_frame_info)
{
    ot_vb_blk vb_blk;
    td_phys_addr_t phys_addr;
    td_void *virt_addr = TD_NULL;
    ot_video_frame_info *frame_info = TD_NULL;

    vb_blk = ss_mpi_vb_get_blk(pool_id, calc_cfg->vb_size, TD_NULL);
    if (vb_blk == OT_VB_INVALID_HANDLE) {
        sample_print("ss_mpi_vb_get_blk err, size:%d\n", calc_cfg->vb_size);
        return TD_FAILURE;
    }

    phys_addr = ss_mpi_vb_handle_to_phys_addr(vb_blk);
    virt_addr = (td_u8 *)ss_mpi_sys_mmap(phys_addr, calc_cfg->vb_size);
    if (virt_addr == TD_NULL) {
        sample_print("ss_mpi_sys_mmap err!\n");
        ss_mpi_vb_release_blk(vb_blk);
        return TD_FAILURE;
    }

    user_frame_info->vb_blk   = vb_blk;
    user_frame_info->blk_size = calc_cfg->vb_size;

    frame_info = &user_frame_info->frame_info;

    frame_info->pool_id                   = pool_id;
    frame_info->mod_id                    = OT_ID_VI;
    frame_info->video_frame.phys_addr[0]  = phys_addr;
    frame_info->video_frame.phys_addr[1]  = frame_info->video_frame.phys_addr[0] + calc_cfg->main_y_size;
    frame_info->video_frame.virt_addr[0]  = virt_addr;
    frame_info->video_frame.virt_addr[1]  = frame_info->video_frame.virt_addr[0] + calc_cfg->main_y_size;
    frame_info->video_frame.stride[0]     = calc_cfg->main_stride;
    frame_info->video_frame.stride[1]     = calc_cfg->main_stride;
    frame_info->video_frame.width         = get_frame_vb_cfg->size.width;
    frame_info->video_frame.height        = get_frame_vb_cfg->size.height;
    frame_info->video_frame.pixel_format  = get_frame_vb_cfg->pixel_format;
    frame_info->video_frame.video_format  = get_frame_vb_cfg->video_format;
    frame_info->video_frame.compress_mode = get_frame_vb_cfg->compress_mode;
    frame_info->video_frame.dynamic_range = get_frame_vb_cfg->dynamic_range;
    frame_info->video_frame.field         = OT_VIDEO_FIELD_FRAME;
    frame_info->video_frame.color_gamut   = OT_COLOR_GAMUT_BT601;

    return TD_SUCCESS;
}

td_void sample_comm_vi_free_frame_blk(sample_vi_user_frame_info *user_frame_info)
{
    td_s32 ret;
    ot_vb_blk vb_blk = user_frame_info->vb_blk;
    td_u32 blk_size = user_frame_info->blk_size;
    td_void *virt_addr = user_frame_info->frame_info.video_frame.virt_addr[0];

    ret = ss_mpi_sys_munmap(virt_addr, blk_size);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_sys_munmap failure!\n");
    }

    ret = ss_mpi_vb_release_blk(vb_blk);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_vb_release_blk block 0x%x failure\n", vb_blk);
    }

    user_frame_info->vb_blk = OT_VB_INVALID_HANDLE;
}

td_s32 sample_comm_vi_get_frame_blk(sample_vi_get_frame_vb_cfg *get_frame_vb_cfg,
                                    sample_vi_user_frame_info user_frame_info[], td_s32 frame_cnt)
{
    td_s32 ret;
    td_s32 i;
    ot_vb_pool pool_id;
    ot_vb_calc_cfg calc_cfg = {0};
    ot_vb_pool_cfg vb_pool_cfg = {0};

    sample_comm_vi_get_vb_calc_cfg(get_frame_vb_cfg, &calc_cfg);

    vb_pool_cfg.blk_size   = calc_cfg.vb_size;
    vb_pool_cfg.blk_cnt    = frame_cnt;
    vb_pool_cfg.remap_mode = OT_VB_REMAP_MODE_NONE;
    pool_id = ss_mpi_vb_create_pool(&vb_pool_cfg);
    if (pool_id == OT_VB_INVALID_POOL_ID) {
        sample_print("ss_mpi_vb_create_pool failed!\n");
        return TD_FAILURE;
    }

    for (i = 0; i < frame_cnt; i++) {
        ret = sample_comm_vi_malloc_frame_blk(pool_id, get_frame_vb_cfg, &calc_cfg, &user_frame_info[i]);
        if (ret != TD_SUCCESS) {
            goto exit;
        }
    }

    return TD_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        sample_comm_vi_free_frame_blk(&user_frame_info[i]);
    }
    ss_mpi_vb_destroy_pool(pool_id);
    return TD_FAILURE;
}

td_void sample_comm_vi_release_frame_blk(sample_vi_user_frame_info user_frame_info[], td_s32 frame_cnt)
{
    td_s32 i;
    ot_vb_pool pool_id;

    for (i = 0; i < frame_cnt; i++) {
        sample_comm_vi_free_frame_blk(&user_frame_info[i]);
    }

    pool_id = user_frame_info[0].frame_info.pool_id;
    ss_mpi_vb_destroy_pool(pool_id);
}

static td_s32 sample_comm_vi_get_fpn_frame_info(ot_vi_pipe vi_pipe,
                                                ot_pixel_format pixel_format, ot_compress_mode compress_mode,
                                                sample_vi_user_frame_info *user_frame_info, td_s32 blk_cnt)
{
    td_s32 ret;
    ot_vi_pipe_attr pipe_attr;
    sample_vi_get_frame_vb_cfg vb_cfg;

    ret = ss_mpi_vi_get_pipe_attr(vi_pipe, &pipe_attr);
    if (ret != TD_SUCCESS) {
        sample_print("vi get pipe attr failed!\n");
        return ret;
    }

    vb_cfg.size.width    = pipe_attr.size.width;
    vb_cfg.size.height   = pipe_attr.size.height;
    vb_cfg.pixel_format  = pixel_format;
    vb_cfg.video_format  = OT_VIDEO_FORMAT_LINEAR;
    vb_cfg.compress_mode = compress_mode;
    vb_cfg.dynamic_range = OT_DYNAMIC_RANGE_SDR8;

    ret = sample_comm_vi_get_frame_blk(&vb_cfg, user_frame_info, blk_cnt);
    if (ret != TD_SUCCESS) {
        sample_print("get fpn frame vb failed!\n");
        return ret;
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_get_fpn_calibrate_frame_info(ot_vi_pipe vi_pipe, ot_pixel_format pixel_format,
                                                          ot_compress_mode compress_mode,
                                                          sample_vi_user_frame_info *user_frame_info, td_s32 blk_cnt)
{
    td_s32 ret;
    sample_vi_user_frame_info *last_frame_info = &user_frame_info[blk_cnt - 1];

    ret = sample_comm_vi_get_fpn_frame_info(vi_pipe, OT_PIXEL_FORMAT_RGB_BAYER_16BPP,
                                            compress_mode, user_frame_info, blk_cnt - 1);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    ret = sample_comm_vi_get_fpn_frame_info(vi_pipe, pixel_format, compress_mode, last_frame_info, 1);
    if (ret != TD_SUCCESS) {
        sample_comm_vi_release_frame_blk(user_frame_info, blk_cnt);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_void sample_comm_vi_get_fpn_file_name(ot_video_frame *video_frame, td_char *file_name, td_u32 length)
{
    (td_void)snprintf_s(file_name, length, length - 1, "./FPN_frame_%dx%d_%dbit.raw",
                        video_frame->width, video_frame->height, ot_vi_get_raw_bit_width(video_frame->pixel_format));
}

static td_s32 sample_comm_vi_get_fpn_file_name_iso(ot_video_frame *video_frame, const td_char *dir_name,
                                                   td_char *file_name, td_u32 length, td_u32 iso)
{
    td_s32 err;
    err = snprintf_s(file_name, length, length - 1, "./%s/FPN_frame_%dx%d_%dbit_iso%d.raw",
                     dir_name, video_frame->width, video_frame->height,
                     ot_vi_get_raw_bit_width(video_frame->pixel_format), iso);
    if (err < 0) {
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

td_void sample_comm_vi_save_fpn_file(ot_isp_fpn_frame_info *fpn_frame_info, FILE *pfd)
{
    td_u8 *virt_addr;
    td_u32 fpn_height;
    td_s32 i;

    fpn_height = fpn_frame_info->fpn_frame.video_frame.height;
    virt_addr = (td_u8 *)fpn_frame_info->fpn_frame.video_frame.virt_addr[0];

    /* save Y
        * ---------------------------------------------------------------- */
    (td_void)fprintf(stderr,
                     "FPN: saving......Raw data......stide: %d, width: %d, "
                     "height: %d, iso: %d.\n",
                     fpn_frame_info->fpn_frame.video_frame.stride[0],
                     fpn_frame_info->fpn_frame.video_frame.width, fpn_height,
                     fpn_frame_info->iso);
    (td_void)fprintf(stderr, "phys addr: 0x%lx\n", (td_ulong)fpn_frame_info->fpn_frame.video_frame.phys_addr[0]);
    (td_void)fprintf(stderr, "please wait a moment to save FPN raw data.\n");
    (td_void)fflush(stderr);

    (td_void)fwrite(virt_addr, fpn_frame_info->frm_size, 1, pfd);

    /* save offset */
    for (i = 0; i < OT_VI_MAX_SPLIT_NODE_NUM; i++) {
        (td_void)fwrite(&fpn_frame_info->offset[i], 4, 1, pfd); /* 4: 4byte */
    }

    /* save compress mode */
    (td_void)fwrite(&fpn_frame_info->fpn_frame.video_frame.compress_mode, 4, 1, pfd); /* 4: 4byte */

    /* save fpn frame size */
    (td_void)fwrite(&fpn_frame_info->frm_size, 4, 1, pfd); /* 4: 4byte */

    /* save iso */
    (td_void)fwrite(&fpn_frame_info->iso, 4, 1, pfd); /* 4: 4byte */
    (td_void)fflush(pfd);
}

static td_void *sample_common_vi_send_pipe_frame_proc(td_void *param)
{
    td_s32 ret;
    td_u32 frame_cnt, send_cnt;
    const ot_video_frame_info *frame_info[OT_VI_MAX_WDR_FRAME_NUM] = {TD_NULL};
    sample_vi_send_frame_info *vi_send_frame_info = (sample_vi_send_frame_info *)param;
    const td_u32 milli_sec = -1; /* milli_sec: -1 */
    const td_u32 frame_num = 1;

    frame_cnt = vi_send_frame_info->frame_cnt;
    ret = ss_mpi_vi_set_pipe_frame_source(vi_send_frame_info->vi_pipe, OT_VI_PIPE_FRAME_SOURCE_USER);
    if (ret != TD_SUCCESS) {
        printf("vi set pipe frame source failed!\n");
        goto exit;
    }

    send_cnt = 0;
    while (g_send_pipe_pthread) {
        if (send_cnt < frame_cnt) {
            vi_send_frame_info->user_frame_info[send_cnt].frame_info.video_frame.pts = 0;
            frame_info[0] = &vi_send_frame_info->user_frame_info[send_cnt].frame_info;

            ret = ss_mpi_vi_send_pipe_raw(vi_send_frame_info->vi_pipe, frame_info, frame_num, milli_sec);
            if (ret != TD_SUCCESS) {
                printf("vi send pipe frame failed with %#x!\n", ret);
                continue;
            }
            send_cnt += frame_num;
        } else {
            send_cnt = 0;
        }
    }

    ret = ss_mpi_vi_set_pipe_frame_source(vi_send_frame_info->vi_pipe, OT_VI_PIPE_FRAME_SOURCE_FE);
    if (ret != TD_SUCCESS) {
        printf("vi set pipe frame source failed!\n");
    }

exit:
    return TD_NULL;
}

static td_s32 sample_comm_vi_fpn_multi_calibrate(ot_vi_pipe vi_pipe, sample_vi_user_frame_info *user_frame_info,
    ot_isp_fpn_calibrate_attr *calibrate_attr, td_s32 calib_cnt)
{
    td_s32 i, ret;

    for (i = 0; i < calib_cnt; i++) {
        /* point each fpn dark frame vb to calibrate_attr */
        (td_void)memcpy_s(&calibrate_attr->fpn_cali_frame.fpn_frame, sizeof(ot_video_frame_info),
                          &user_frame_info[i].frame_info, sizeof(ot_video_frame_info));

        ret = ss_mpi_isp_fpn_calibrate(vi_pipe, calibrate_attr);
        if (ret != TD_SUCCESS) {
            sample_print("vi fpn calibrate failed!\n");
            return TD_FAILURE;
        }
        (td_void)memcpy_s(&user_frame_info[i].frame_info, sizeof(ot_video_frame_info),
                          &calibrate_attr->fpn_cali_frame.fpn_frame, sizeof(ot_video_frame_info));
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_fpn_calibrate_process(ot_vi_pipe vi_pipe, sample_vi_user_frame_info *user_frame_info,
    ot_isp_fpn_calibrate_attr *calibrate_attr, td_s32 calib_cnt)
{
    td_s32 ret;
    sample_vi_user_frame_info *final_user_frame_info = &user_frame_info[FPN_CALIB_TIMES];
    pthread_t thread_id = 0;
    sample_vi_send_frame_info vi_send_frame_info;

    /* first calibrate process, save 8 dark frames to user_frame_info */
    calibrate_attr->fpn_mode = OT_ISP_FPN_OUT_MODE_HIGH;
    ret = sample_comm_vi_fpn_multi_calibrate(vi_pipe, user_frame_info, calibrate_attr, calib_cnt);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    printf("first calibrate done, times: %d.\n", calib_cnt);

    vi_send_frame_info.vi_pipe = vi_pipe;
    vi_send_frame_info.frame_cnt = calib_cnt;
    vi_send_frame_info.user_frame_info = user_frame_info;
    g_send_pipe_pthread = TD_TRUE;
    ret = pthread_create(&thread_id, TD_NULL, sample_common_vi_send_pipe_frame_proc, (td_void *)&vi_send_frame_info);
    if (ret != TD_SUCCESS) {
        printf("vi create send frame thread failed!\n");
        g_send_pipe_pthread = TD_FALSE;
        return TD_FAILURE;
    }

    /* second calibrate process */
    calibrate_attr->frame_num = calib_cnt;
    calibrate_attr->fpn_mode = OT_ISP_FPN_OUT_MODE_NORM;
    ret = sample_comm_vi_fpn_multi_calibrate(vi_pipe, final_user_frame_info, calibrate_attr, 1);
    if (ret != TD_SUCCESS) {
        goto exit;
    }
    printf("second calibrate done, times: 1.\n");

exit:
    g_send_pipe_pthread = TD_FALSE;
    pthread_join(thread_id, TD_NULL);
    return ret;
}

td_s32 sample_comm_vi_fpn_calibrate(ot_vi_pipe vi_pipe, sample_vi_fpn_calibration_cfg *calibration_cfg)
{
    td_s32 ret, i;
    const ot_vi_chn vi_chn = 0;
    FILE *pfd = TD_NULL;
    sample_vi_user_frame_info user_frame_info[FPN_CALIB_TIMES + 1];
    ot_isp_fpn_calibrate_attr calibrate_attr;

    td_char fpn_file_name[FPN_FILE_NAME_LENGTH];

    printf("please turn off camera aperture to start calibrate!\nhit any key ,start calibrate!\n");
    getchar();

    ret = ss_mpi_vi_disable_chn(vi_pipe, vi_chn);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    calibrate_attr.threshold = calibration_cfg->threshold;
    calibrate_attr.frame_num = calibration_cfg->frame_num;
    calibrate_attr.fpn_type  = calibration_cfg->fpn_type;
    ret = sample_comm_vi_get_fpn_calibrate_frame_info(vi_pipe, OT_PIXEL_FORMAT_RGB_BAYER_16BPP,
        calibration_cfg->compress_mode, user_frame_info, FPN_CALIB_TIMES + 1);
    if (ret != TD_SUCCESS) {
        ss_mpi_vi_enable_chn(vi_pipe, vi_chn);
        return TD_FAILURE;
    }

    ret = sample_comm_vi_fpn_calibrate_process(vi_pipe, user_frame_info, &calibrate_attr, FPN_CALIB_TIMES);
    if (ret != TD_SUCCESS) {
        sample_print("vi fpn calibrate failed!\n");
        goto exit;
    }

    printf("\nafter calibrate ");
    for (i = 0; i < OT_VI_MAX_SPLIT_NODE_NUM; i++) {
        printf("offset[%d] = 0x%x, ", i, calibrate_attr.fpn_cali_frame.offset[i]);
    }
    printf("frame_size = %d, iso = %d\n", calibrate_attr.fpn_cali_frame.frm_size, calibrate_attr.fpn_cali_frame.iso);

    sample_comm_vi_get_fpn_file_name(&calibrate_attr.fpn_cali_frame.fpn_frame.video_frame,
                                     fpn_file_name, FPN_FILE_NAME_LENGTH);
    printf("save dark frame file: %s!\n", fpn_file_name);
    pfd = fopen(fpn_file_name, "wb");
    if (pfd == TD_NULL) {
        printf("open file %s err!\n", fpn_file_name);
        goto exit;
    }

    sample_comm_vi_save_fpn_file(&calibrate_attr.fpn_cali_frame, pfd);

    (td_void)fclose(pfd);

exit:
    sample_comm_vi_release_frame_blk(user_frame_info, FPN_CALIB_TIMES + 1);
    ret = ss_mpi_vi_enable_chn(vi_pipe, vi_chn);
    return ret;
}

static td_void sample_comm_vi_read_fpn_file(ot_isp_fpn_frame_info *fpn_frame_info, FILE *pfd)
{
    ot_video_frame_info *frame_info;
    td_s32 i;

    frame_info = &fpn_frame_info->fpn_frame;
    (td_void)fread((td_u8 *)frame_info->video_frame.virt_addr[0], fpn_frame_info->frm_size, 1, pfd);

    for (i = 0; i < OT_VI_MAX_SPLIT_NODE_NUM; i++) {
        (td_void)fread((td_u8 *)&fpn_frame_info->offset[i], 4, 1, pfd); /* 4: 4byte */
    }

    (td_void)fread((td_u8 *)&frame_info->video_frame.compress_mode, 4, 1, pfd); /* 4: 4byte */
    (td_void)fread((td_u8 *)&fpn_frame_info->frm_size, 4, 1, pfd); /* 4: 4byte */
    (td_void)fread((td_u8 *)&fpn_frame_info->iso, 4, 1, pfd); /* 4: 4byte */
}

td_s32 sample_comm_vi_enable_fpn_correction(ot_vi_pipe vi_pipe, sample_vi_fpn_correction_cfg *correction_cfg)
{
    td_s32 ret;
    td_u32 i;
    FILE *pfd = TD_NULL;
    ot_isp_fpn_attr correction_attr;
    sample_vi_user_frame_info *user_frame_info = &correction_cfg->user_frame_info;
    td_char fpn_file_name[FPN_FILE_NAME_LENGTH];

    ret = sample_comm_vi_get_fpn_frame_info(vi_pipe, correction_cfg->pixel_format,
                                            correction_cfg->compress_mode, user_frame_info, 1);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }
    (td_void)memcpy_s(&correction_attr.fpn_frm_info.fpn_frame, sizeof(ot_video_frame_info),
                      &user_frame_info->frame_info, sizeof(ot_video_frame_info));

    sample_comm_vi_get_fpn_file_name(&correction_attr.fpn_frm_info.fpn_frame.video_frame,
                                     fpn_file_name, FPN_FILE_NAME_LENGTH);
    pfd = fopen(fpn_file_name, "rb");
    if (pfd == TD_NULL) {
        printf("open file %s err!\n", fpn_file_name);
        goto exit;
    }

    correction_attr.fpn_frm_info.frm_size = user_frame_info->blk_size;
    sample_comm_vi_read_fpn_file(&correction_attr.fpn_frm_info, pfd);

    (td_void)fclose(pfd);

    for (i = 0; i < OT_VI_MAX_SPLIT_NODE_NUM; i++) {
        printf("offset[%d] = 0x%x; ", i, correction_attr.fpn_frm_info.offset[i]);
    }
    printf("\n");
    printf("frame_size = %d.\n", correction_attr.fpn_frm_info.frm_size);
    printf("iso = %d.\n", correction_attr.fpn_frm_info.iso);

    correction_attr.enable = TD_TRUE;
    correction_attr.op_type = correction_cfg->op_mode;
    correction_attr.fpn_type = correction_cfg->fpn_type;
    correction_attr.manual_attr.strength = correction_cfg->strength;
    ret = ss_mpi_isp_set_fpn_attr(vi_pipe, &correction_attr);
    if (ret != TD_SUCCESS) {
        sample_print("set fpn attr failed!\n");
        goto exit;
    }

    return TD_SUCCESS;

exit:
    sample_comm_vi_release_frame_blk(user_frame_info, 1);
    return ret;
}

td_s32 sample_comm_vi_enable_fpn_correction_for_scene(ot_vi_pipe vi_pipe, sample_vi_fpn_correction_cfg *correction_cfg,
    td_u32 iso, sample_scene_fpn_offset_cfg *scene_fpn_offset_cfg, const td_char *dir_name)
{
    td_s32 ret = TD_SUCCESS;
    td_u32 i;
    FILE *pfd = TD_NULL;
    ot_isp_fpn_attr correction_attr;
    sample_vi_user_frame_info *user_frame_info = &correction_cfg->user_frame_info;
    td_char fpn_file_name[FPN_FILE_NAME_LENGTH];
    check_return(sample_comm_vi_get_fpn_frame_info(vi_pipe, correction_cfg->pixel_format, correction_cfg->compress_mode,
                                                   user_frame_info, 1),
                 "sample_comm_vi_get_fpn_frame_info");
    (td_void)memcpy_s(&correction_attr.fpn_frm_info.fpn_frame, sizeof(ot_video_frame_info),
                      &user_frame_info->frame_info, sizeof(ot_video_frame_info));

    check_return(sample_comm_vi_get_fpn_file_name_iso(&correction_attr.fpn_frm_info.fpn_frame.video_frame, dir_name,
                                                      fpn_file_name, FPN_FILE_NAME_LENGTH, iso),
                 "sample_comm_vi_get_fpn_file_name_iso");
    pfd = fopen(fpn_file_name, "rb");
    if (pfd == TD_NULL) {
        printf("open file %s err!\n", fpn_file_name);
        goto exit;
    }
    printf("open file %s success!\n", fpn_file_name);
    correction_attr.fpn_frm_info.frm_size = user_frame_info->blk_size;
    sample_comm_vi_read_fpn_file(&correction_attr.fpn_frm_info, pfd);
    ret = fclose(pfd);
    if (ret != TD_SUCCESS) {
        goto exit;
    }
    correction_attr.fpn_frm_info.iso = iso;
    for (i = 0; i < OT_VI_MAX_SPLIT_NODE_NUM; i++) {
        correction_attr.fpn_frm_info.offset[i] = scene_fpn_offset_cfg->offset;
        printf("offset[%d] = %#x; ", i, scene_fpn_offset_cfg->offset);
    }
    printf("\n frame_size = %d. iso = %d.\n", correction_attr.fpn_frm_info.frm_size, correction_attr.fpn_frm_info.iso);
    correction_attr.enable = TD_TRUE;
    correction_attr.op_type = correction_cfg->op_mode;
    correction_attr.fpn_type = correction_cfg->fpn_type;
    correction_attr.manual_attr.strength = correction_cfg->strength;
    correction_attr.fpn_frm_info.fpn_frame.video_frame.compress_mode = correction_cfg->compress_mode;
    ret = ss_mpi_isp_set_fpn_attr(vi_pipe, &correction_attr);
    if (ret != TD_SUCCESS) {
        sample_print("set fpn attr failed!\n");
        goto exit;
    }
    return TD_SUCCESS;

exit:
    sample_comm_vi_release_frame_blk(user_frame_info, 1);
    return ret;
}


td_s32 sample_comm_vi_disable_fpn_correction(ot_vi_pipe vi_pipe, sample_vi_fpn_correction_cfg *correction_cfg)
{
    td_s32 ret;
    ot_isp_fpn_attr correction_attr;

    ret = ss_mpi_isp_get_fpn_attr(vi_pipe, &correction_attr);
    if (ret != TD_SUCCESS) {
        sample_print("get fpn attr failed!\n");
        return TD_FAILURE;
    }

    correction_attr.enable = TD_FALSE;
    ret = ss_mpi_isp_set_fpn_attr(vi_pipe, &correction_attr);
    if (ret != TD_SUCCESS) {
        sample_print("set fpn attr failed!\n");
        return TD_FAILURE;
    }

    sample_comm_vi_release_frame_blk(&correction_cfg->user_frame_info, 1);

    return TD_SUCCESS;
}

td_s32 sample_comm_vi_start_virt_pipe(const sample_vi_cfg *vi_cfg)
{
    td_s32 ret;

    ret = sample_comm_vi_start_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
    if (ret != TD_SUCCESS) {
        sample_print("start pipe failed!\n");
        goto start_pipe_failed;
    }

    ret = sample_comm_vi_start_isp(vi_cfg);
    if (ret != TD_SUCCESS) {
        sample_print("sample_comm_vi_start_isp failed!\n");
        goto start_isp_failed;
    }

    return TD_SUCCESS;

start_isp_failed:
    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
start_pipe_failed:
    return TD_FAILURE;
}

td_void sample_comm_vi_stop_virt_pipe(const sample_vi_cfg *vi_cfg)
{
    sample_comm_vi_stop_isp(vi_cfg);
    sample_comm_vi_stop_pipe(&vi_cfg->bind_pipe, vi_cfg->pipe_info);
}


static td_s32 sample_comm_vi_convert_chroma_planar_to_sp42x(FILE *file, td_u8 *chroma_data,
    td_u32 luma_stride, td_u32 chroma_width, td_u32 chroma_height)
{
    td_u32 chroma_stride = luma_stride >> 1;
    td_u8 *dst = TD_NULL;
    td_u32 row;
    td_u32 list;
    td_u8 *temp = TD_NULL;

    temp = (td_u8*)malloc(chroma_stride);
    if (temp == TD_NULL) {
        sample_print("vi malloc failed!\n");
        return TD_FAILURE;
    }
    if (memset_s(temp, chroma_stride, 0, chroma_stride) != EOK) {
        sample_print("vi memset_s failed!\n");
        free(temp);
        temp = TD_NULL;
        return TD_FAILURE;
    }

    /* U */
    dst = chroma_data + 1;
    for (row = 0; row < chroma_height; ++row) {
        (td_void)fread(temp, chroma_width, 1, file); /* sp420 U-component data starts 1/2 way from the beginning */
        for (list = 0; list < chroma_stride; ++list) {
            *dst = *(temp + list);
            dst += 2; /* traverse 2 steps away to the next U-component data */
        }
        dst = chroma_data + 1;
        dst += (row + 1) * luma_stride;
    }

    /* V */
    dst = chroma_data;
    for (row = 0; row < chroma_height; ++row) {
        (td_void)fread(temp, chroma_width, 1, file); /* sp420 V-component data starts 1/2 way from the beginning */
        for (list = 0; list < chroma_stride; ++list) {
            *dst = *(temp + list);
            dst += 2; /* traverse 2 steps away to the next V-component data */
        }
        dst = chroma_data;
        dst += (row + 1) * luma_stride;
    }

    free(temp);

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_read_file_to_sp42_x(FILE *file, ot_video_frame *frame)
{
    td_u8 *luma = (td_u8*)(td_uintptr_t)frame->virt_addr[0];
    td_u8 *chroma = (td_u8*)(td_uintptr_t)frame->virt_addr[1];
    td_u32 luma_width = frame->width;
    td_u32 chroma_width = luma_width >> 1;
    td_u32 luma_height = frame->height;
    td_u32 chroma_height = luma_height;
    td_u32 luma_stride = frame->stride[0];

    td_u8 *dst = TD_NULL;
    td_u32 row;

    if (frame->video_format == OT_VIDEO_FORMAT_LINEAR) {
        /* Y */
        dst = luma;
        for (row = 0; row < luma_height; ++row) {
            (td_void)fread(dst, luma_width, 1, file);
            dst += luma_stride;
        }

        if (OT_PIXEL_FORMAT_YUV_400 == frame->pixel_format) {
            return TD_SUCCESS;
        } else if (OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 == frame->pixel_format) {
            chroma_height = chroma_height >> 1;
        }
        if (sample_comm_vi_convert_chroma_planar_to_sp42x(
            file, chroma, luma_stride, chroma_width, chroma_height) != TD_SUCCESS) {
            return TD_FAILURE;
        }
    } else {
        (td_void)fread(luma, luma_stride * luma_height * 3 / 2, 1,  file); /* Tile 64x16 size = stridexheight*3/2 */
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_get_user_pic_frame_info(ot_size *dst_size, sample_vi_user_frame_info *user_frame_info)
{
    td_s32 ret;
    sample_vi_get_frame_vb_cfg vb_cfg;

    vb_cfg.size.width    = dst_size->width;
    vb_cfg.size.height   = dst_size->height;
    vb_cfg.pixel_format  = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    vb_cfg.video_format  = OT_VIDEO_FORMAT_LINEAR;
    vb_cfg.compress_mode = OT_COMPRESS_MODE_NONE;
    vb_cfg.dynamic_range = OT_DYNAMIC_RANGE_SDR8;

    ret = sample_comm_vi_get_frame_blk(&vb_cfg, user_frame_info, 1);
    if (ret != TD_SUCCESS) {
        sample_print("get user pic frame vb failed!\n");
        return ret;
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_add_scale_task(ot_video_frame_info *src_frame, ot_video_frame_info *dst_frame)
{
    td_s32 ret;
    ot_vgs_handle handle;
    ot_vgs_task_attr vgs_task_attr;

    ret = ss_mpi_vgs_begin_job(&handle);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_vgs_begin_job failed, ret:0x%x", ret);
        return TD_FAILURE;
    }

    if (memcpy_s(&vgs_task_attr.img_in, sizeof(ot_video_frame_info),
        src_frame, sizeof(ot_video_frame_info)) != EOK) {
        sample_print("memcpy_s img_in failed\n");
        return TD_FAILURE;
    }

    if (memcpy_s(&vgs_task_attr.img_out, sizeof(ot_video_frame_info),
        dst_frame, sizeof(ot_video_frame_info)) != EOK) {
        sample_print("memcpy_s img_out failed\n");
        return TD_FAILURE;
    }

    if (ss_mpi_vgs_add_scale_task(handle, &vgs_task_attr, OT_VGS_SCALE_COEF_NORM) != TD_SUCCESS) {
        sample_print("ss_mpi_vgs_add_scale_task failed\n");
        return TD_FAILURE;
    }

    ret = ss_mpi_vgs_end_job(handle);
    if (ret != TD_SUCCESS) {
        ss_mpi_vgs_cancel_job(handle);
        sample_print("ss_mpi_vgs_end_job failed, ret:0x%x", ret);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_s32 sample_comm_vi_read_user_frame_file(ot_vi_pipe vi_pipe, sample_vi_user_frame_info *user_frame_info)
{
    td_s32 ret;
    FILE *pfd = TD_NULL;
    const td_char *frame_file = "./UsePic_3840x2160_sp420.yuv";
    ot_size frame_size = {WIDTH_3840, HEIGHT_2160};
    sample_vi_user_frame_info pic_frame_info;

    ret = sample_comm_vi_get_user_pic_frame_info(&frame_size, &pic_frame_info);
    if (ret != TD_SUCCESS) {
        return ret;
    }

    pfd = fopen(frame_file, "rb");
    if (pfd == TD_NULL) {
        sample_print("open file \"%s\" failed!\n", frame_file);
        ret = TD_FAILURE;
        goto exit0;
    }

    ret = sample_comm_vi_read_file_to_sp42_x(pfd, &pic_frame_info.frame_info.video_frame);
    if (ret != TD_SUCCESS) {
        goto exit1;
    }

    (td_void)fflush(pfd);

    ret = sample_comm_vi_add_scale_task(&pic_frame_info.frame_info, &user_frame_info->frame_info);
    if (ret != TD_SUCCESS) {
        sample_print("add vgs scale task failed.\n");
    }

exit1:
    (td_void)fclose(pfd);
exit0:
    sample_comm_vi_release_frame_blk(&pic_frame_info, 1);
    return ret;
}

static td_s32 sample_comm_vi_add_coverex_task(ot_video_frame_info *dst_frame)
{
    td_s32 ret;
    ot_vgs_handle handle;
    ot_vgs_task_attr vgs_task_attr;
    ot_cover cover;

    ret = ss_mpi_vgs_begin_job(&handle);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_vgs_begin_job failed, ret:0x%x", ret);
        return TD_FAILURE;
    }

    if (memcpy_s(&vgs_task_attr.img_in, sizeof(ot_video_frame_info),
        dst_frame, sizeof(ot_video_frame_info)) != EOK) {
        sample_print("memcpy_s img_in failed\n");
        return TD_FAILURE;
    }

    if (memcpy_s(&vgs_task_attr.img_out, sizeof(ot_video_frame_info),
        dst_frame, sizeof(ot_video_frame_info)) != EOK) {
        sample_print("memcpy_s img_out failed\n");
        return TD_FAILURE;
    }

    cover.type = OT_COVER_RECT;
    cover.rect.x = 0;
    cover.rect.y = 0;
    cover.rect.width = dst_frame->video_frame.width;
    cover.rect.height = dst_frame->video_frame.height;
    cover.color = 0xFF0000;
    if (ss_mpi_vgs_add_cover_task(handle, &vgs_task_attr, &cover, 1) != TD_SUCCESS) {
        sample_print("ss_mpi_vgs_add_scale_task failed\n");
        return TD_FAILURE;
    }

    ret = ss_mpi_vgs_end_job(handle);
    if (ret != TD_SUCCESS) {
        ss_mpi_vgs_cancel_job(handle);
        sample_print("ss_mpi_vgs_end_job failed, ret:0x%x", ret);
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

td_s32 sample_common_vi_load_user_pic(ot_vi_pipe vi_pipe, sample_vi_user_pic_type user_pic_type,
    sample_vi_user_frame_info *user_frame_info)
{
    td_s32 ret;
    ot_vi_pipe_attr pipe_attr;

    ret = ss_mpi_vi_get_pipe_attr(vi_pipe, &pipe_attr);
    if (ret != TD_SUCCESS) {
        sample_print("vi get pipe attr failed!\n");
        return ret;
    }

    ret = sample_comm_vi_get_user_pic_frame_info(&pipe_attr.size, user_frame_info);
    if (ret != TD_SUCCESS) {
        return ret;
    }

    if (user_pic_type == VI_USER_PIC_FRAME) {
        ret = sample_comm_vi_read_user_frame_file(vi_pipe, user_frame_info);
    } else {
        ret = sample_comm_vi_add_coverex_task(&user_frame_info->frame_info);
    }

    if (ret != TD_SUCCESS) {
        sample_comm_vi_release_frame_blk(user_frame_info, 1);
        return ret;
    }

    return TD_SUCCESS;
}

td_void sample_common_vi_unload_user_pic(sample_vi_user_frame_info *user_frame_info)
{
    sample_comm_vi_release_frame_blk(user_frame_info, 1);
}

#define WDR_MAX_PTS_DIFF 25000
static td_bool sample_vi_is_frame_pts_suitable(td_u64 pts1, td_u64 pts2)
{
    td_s64 pts_diff;

    pts_diff = pts1 - pts2;
    pts_diff = ((pts_diff >= 0) ? pts_diff : (-pts_diff));

    if (pts_diff <= WDR_MAX_PTS_DIFF) {
        return TD_TRUE;
    } else {
        return TD_FALSE;
    }
}

static td_s32 sample_vi_match_wdr_pts(ot_video_frame_info frame_info[], ot_vi_pipe vi_pipe[], td_s32 pipe_num)
{
    td_s32 cnt = 0;
    td_s32 try_time = 5; /* try 5 times */
    const td_s32 millsec = 1000; /* millsec: 1000ms */
    td_s32 ret;
    td_u64 pts_max, pts_min;
    td_s32 min_id, i;

    while (cnt++ < try_time) {
        pts_max = frame_info[0].video_frame.pts;
        pts_min = frame_info[0].video_frame.pts;
        min_id = 0;
        for (i = 1; i < pipe_num; i++) {
            pts_max = pts_max > frame_info[i].video_frame.pts ? pts_max : frame_info[i].video_frame.pts;
            pts_min = pts_min < frame_info[i].video_frame.pts ? pts_min : frame_info[i].video_frame.pts;
            if (pts_min == frame_info[i].video_frame.pts) {
                min_id = i;
            }
        }
        if (sample_vi_is_frame_pts_suitable(pts_max, pts_min) == TD_TRUE) {
            return TD_SUCCESS;
        }

        (td_void)ss_mpi_vi_release_pipe_frame(vi_pipe[min_id], &frame_info[min_id]);
        ret = ss_mpi_vi_get_pipe_frame(vi_pipe[min_id], &frame_info[min_id], millsec);
        if (ret != TD_SUCCESS) {
            printf("repeated get pipe[%d] frame failed\n", vi_pipe[min_id]);
            return TD_FAILURE;
        }
    }

    return TD_FAILURE;
}

static td_s32 sample_vi_send_pipe_wdr_frame(ot_vi_pipe send_pipe, ot_video_frame_info frame_info[],
                                            td_s32 pipe_num, td_s32 millsec)
{
    const ot_video_frame_info *wdr_frame_info[OT_VI_MAX_WDR_FRAME_NUM] = {TD_NULL};
    td_s32 i, ret;

    for (i = 0; i < pipe_num; i++) {
        wdr_frame_info[i] = &frame_info[i];
    }
    ret = ss_mpi_isp_run_once(send_pipe);
    if (ret != TD_SUCCESS) {
        printf("pipe[%d] isp runonce failed\n", send_pipe);
        return ret;
    }
    ret = ss_mpi_vi_send_pipe_raw(send_pipe, wdr_frame_info, pipe_num, millsec);
    if (ret != TD_SUCCESS) {
        printf("pipe[%d] send frame failed\n", send_pipe);
        return ret;
    }
    ret = ss_mpi_isp_get_vd_time_out(send_pipe, OT_ISP_VD_BE_END, millsec);
    if (ret != TD_SUCCESS) {
        printf("pipe[%d] isp wait_be_end failed\n", send_pipe);
        return ret;
    }

    return ret;
}

static td_void *sample_vi_wdr_dump_and_send_proc(td_void *param)
{
    ot_vi_frame_dump_attr dump_attr = {
        .depth = 2,
        .enable = TD_TRUE
    };
    const td_s32 millsec = 1000;
    ot_video_frame_info dump_frame_info[OT_VI_MAX_WDR_FRAME_NUM];
    ot_vi_bind_pipe *bind_pipe = (ot_vi_bind_pipe*)param;
    ot_vi_pipe *vi_pipe = bind_pipe->pipe_id;
    td_s32 pipe_num = bind_pipe->pipe_num;
    ot_vi_pipe send_pipe = vi_pipe[0];
    td_s32 i, ret;

    for (i = 0; i < pipe_num; i++) {
        ret = ss_mpi_vi_set_pipe_frame_dump_attr(vi_pipe[i], &dump_attr);
        if (ret != TD_SUCCESS) {
            printf("set pipe[%d] dump attr failed\n", vi_pipe[i]);
            return TD_NULL;
        }
    }

    ret = ss_mpi_vi_set_pipe_frame_source(send_pipe, OT_VI_PIPE_FRAME_SOURCE_USER);
    if (ret != TD_SUCCESS) {
        printf("set pipe[%d] user fraem source failed\n", vi_pipe[0]);
        return TD_NULL;
    }

    while (g_send_pipe_pthread) {
        for (i = 0; i < pipe_num; i++) {
            ret = ss_mpi_vi_get_pipe_frame(vi_pipe[i], &dump_frame_info[i], millsec);
            if (ret != TD_SUCCESS) {
                printf("get pipe[%d] frame failed\n", vi_pipe[i]);
                goto release;
            }
        }

        if (sample_vi_match_wdr_pts(dump_frame_info, vi_pipe, pipe_num) != TD_SUCCESS) {
            printf("pipe frame not suitable, lost frame\n");
            goto release;
        }

        ret = sample_vi_send_pipe_wdr_frame(send_pipe, dump_frame_info, pipe_num, millsec);
        if (ret != TD_SUCCESS) {
            printf("pipe[%d] send frame failed\n", send_pipe);
        }

release:
        for (i = i - 1; i >= 0; i--) {
            (td_void)ss_mpi_vi_release_pipe_frame(vi_pipe[i], &dump_frame_info[i]);
        }
    }

    return TD_NULL;
}

td_s32 sample_comm_vi_send_wdr_frame(ot_vi_bind_pipe *bind_pipe)
{
    td_s32 ret;
    pthread_t thread_id = 0;
    g_send_pipe_pthread = TD_TRUE;
    ret = pthread_create(&thread_id, TD_NULL, sample_vi_wdr_dump_and_send_proc, (td_void *)bind_pipe);
    if (ret != TD_SUCCESS) {
        printf("vi create send frame thread failed!\n");
        g_send_pipe_pthread = TD_FALSE;
        return TD_FAILURE;
    }

    printf("wdr send frame thread running, print any key to exit!\n");
    getchar();

    g_send_pipe_pthread = TD_FALSE;
    pthread_join(thread_id, TD_NULL);
    return ret;
}