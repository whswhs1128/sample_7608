/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __LINUX_OT_USB_CAMERA_H
#define __LINUX_OT_USB_CAMERA_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>
#include "ot_type.h"

#define OT_UVC_VC_DESCRIPTOR_UNDEFINED 0x00
#define OT_UVC_VC_HEADER 0x01
#define OT_UVC_VC_INPUT_TERMINAL 0x02
#define OT_UVC_VC_OUTPUT_TERMINAL 0x03
#define OT_UVC_VC_SELECTOR_UNIT 0x04
#define OT_UVC_VC_PROCESSING_UNIT 0x05
#define OT_UVC_VC_EXTENSION_UNIT 0x06

#define OT_UVC_RC_UNDEFINED 0x00
#define OT_UVC_SET_CUR 0x01
#define OT_UVC_GET_CUR 0x81
#define OT_UVC_GET_MIN 0x82
#define OT_UVC_GET_MAX 0x83
#define OT_UVC_GET_RES 0x84
#define OT_UVC_GET_LEN 0x85
#define OT_UVC_GET_INFO 0x86
#define OT_UVC_GET_DEF 0x87

#define OT_UVC_VC_CONTROL_UNDEFINED 0x00
#define OT_UVC_VC_VIDEO_POWER_MODE_CONTROL 0x01
#define OT_UVC_VC_REQUEST_ERROR_CODE_CONTROL 0x02

#define OT_UVC_CT_CONTROL_UNDEFINED 0x00
#define OT_UVC_CT_SCANNING_MODE_CONTROL 0x01
#define OT_UVC_CT_AE_MODE_CONTROL 0x02
#define OT_UVC_CT_AE_PRIORITY_CONTROL 0x03
#define OT_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL 0x04
#define OT_UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL 0x05
#define OT_UVC_CT_FOCUS_ABSOLUTE_CONTROL 0x06
#define OT_UVC_CT_FOCUS_RELATIVE_CONTROL 0x07
#define OT_UVC_CT_FOCUS_AUTO_CONTROL 0x08
#define OT_UVC_CT_IRIS_ABSOLUTE_CONTROL 0x09
#define OT_UVC_CT_IRIS_RELATIVE_CONTROL 0x0a
#define OT_UVC_CT_ZOOM_ABSOLUTE_CONTROL 0x0b
#define OT_UVC_CT_ZOOM_RELATIVE_CONTROL 0x0c
#define OT_UVC_CT_PANTILT_ABSOLUTE_CONTROL 0x0d
#define OT_UVC_CT_PANTILT_RELATIVE_CONTROL 0x0e
#define OT_UVC_CT_ROLL_ABSOLUTE_CONTROL 0x0f
#define OT_UVC_CT_ROLL_RELATIVE_CONTROL 0x10
#define OT_UVC_CT_PRIVACY_CONTROL 0x11

#define OT_UVC_PU_CONTROL_UNDEFINED 0x00
#define OT_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL 0x01
#define OT_UVC_PU_BRIGHTNESS_CONTROL 0x02
#define OT_UVC_PU_CONTRAST_CONTROL 0x03
#define OT_UVC_PU_GAIN_CONTROL 0x04
#define OT_UVC_PU_POWER_LINE_FREQUENCY_CONTROL 0x05
#define OT_UVC_PU_HUE_CONTROL 0x06
#define OT_UVC_PU_SATURATION_CONTROL 0x07
#define OT_UVC_PU_SHARPNESS_CONTROL 0x08
#define OT_UVC_PU_GAMMA_CONTROL 0x09
#define OT_UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL 0x0a
#define OT_UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0b
#define OT_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL 0x0c
#define OT_UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL 0x0d
#define OT_UVC_PU_DIGITAL_MULTIPLIER_CONTROL 0x0e
#define OT_UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL 0x0f
#define OT_UVC_PU_HUE_AUTO_CONTROL 0x10
#define OT_UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL 0x11
#define OT_UVC_PU_ANALOG_LOCK_STATUS_CONTROL 0x12

#define OT_UVC_VS_CONTROL_UNDEFINED 0x00
#define OT_UVC_VS_PROBE_CONTROL 0x01
#define OT_UVC_VS_COMMIT_CONTROL 0x02
#define OT_UVC_VS_STILL_PROBE_CONTROL 0x03
#define OT_UVC_VS_STILL_COMMIT_CONTROL 0x04
#define OT_UVC_VS_STILL_IMAGE_TRIGGER_CONTROL 0x05
#define OT_UVC_VS_STREAM_ERROR_CODE_CONTROL 0x06
#define OT_UVC_VS_GENERATE_KEY_FRAME_CONTROL 0x07
#define OT_UVC_VS_UPDATE_FRAME_SEGMENT_CONTROL 0x08
#define OT_UVC_VS_SYNC_DELAY_CONTROL 0x09

#define VIDEO_EVE_SECRET_BEGIN 0x08000000
#define OT_UVC_EVE_BEGIN (VIDEO_EVE_SECRET_BEGIN + 0)
#define OT_UVC_EVE_CON (VIDEO_EVE_SECRET_BEGIN + 0)
#define OT_UVC_EVE_DISCON (VIDEO_EVE_SECRET_BEGIN + 1)
#define OT_UVC_EVE_STRON (VIDEO_EVE_SECRET_BEGIN + 2)
#define OT_UVC_EVE_STROFF (VIDEO_EVE_SECRET_BEGIN + 3)
#define OT_UVC_EVE_SETTING (VIDEO_EVE_SECRET_BEGIN + 4)
#define OT_UVC_EVE_DATA (VIDEO_EVE_SECRET_BEGIN + 5)
#define OT_UVC_EVE_END (VIDEO_EVE_SECRET_BEGIN + 5)

#define video_four_character_code(a, b, c, d) \
    ((td_u32)(a) | ((td_u32)(b) << 8) | ((td_u32)(c) << 16) | ((td_u32)(d) << 24))

#define OT_UVC_IOC_SEND_RESPONSE _IOW('U', 1, struct uvc_request_data)

#define VIDEO_IOCTL_SET_FORMAT _IOWR('V', 5, struct video_fmt)
#define VIDEO_IOCTL_REQUEST_BUF _IOWR('V', 8, struct video_buf_request)
#define VIDEO_IOCTL_QUERY_BUF _IOWR('V', 9, struct video_cache)
#define VIDEO_IOCTL_QUERY_CAP _IOR('V', 0, struct video_ability)
#define VIDEO_IOCTL_STREAM_OFF _IOW('V', 19, td_s32)
#define VIDEO_IOCTL_QUEUE_BUF _IOWR('V', 15, struct video_cache)
#define VIDEO_IOCTL_SET_CMD _IOWR('V', 28, struct video_control)
#define VIDEO_IOCTL_SET_DV_TIMINGS _IOWR('V', 87, struct video_dv_timings)
#define VIDEO_IOCTL_GET_DV_TIMINGS _IOWR('V', 88, struct video_dv_timings)
#define VIDEO_IOCTL_DQUEUE_EVENT _IOR('V', 89, struct video_event)
#define VIDEO_IOCTL_DESC_EVENT _IOW('V', 90, struct video_event_descriptor)
#define VIDEO_IOCTL_UNDESC_EVENT _IOW('V', 91, struct video_event_descriptor)
#define VIDEO_IOCTL_STREAM_ON _IOW('V', 18, td_s32)
#define VIDEO_IOCTL_DQUEUE_BUF _IOWR('V', 17, struct video_cache)

#define VIDEO_IMG_FORMAT_YUYV video_four_character_code('Y', 'U', 'Y', 'V')       /* 16  YUV 4:2:2     */
#define VIDEO_IMG_FORMAT_YUV420 video_four_character_code('Y', 'U', '1', '2')     /* 16  YUV 4:2:0     */
#define VIDEO_IMG_FORMAT_NV12 video_four_character_code('N', 'V', '1', '2')       /* 16  YUV 4:2:0     */
#define VIDEO_IMG_FORMAT_NV21 video_four_character_code('N', 'V', '2', '1')       /* 16  YUV 4:2:0     */
#define VIDEO_IMG_FORMAT_MJPEG video_four_character_code('M', 'J', 'P', 'G')      /* Motion-JPEG   */
#define VIDEO_IMG_FORMAT_JPEG video_four_character_code('J', 'P', 'E', 'G')       /* JFIF JPEG     */
#define VIDEO_IMG_FORMAT_DV video_four_character_code('d', 'v', 's', 'd')         /* 1394          */
#define VIDEO_IMG_FORMAT_MPEG video_four_character_code('M', 'P', 'E', 'G')       /* MPEG-1/2/4 Multiplexed */
#define VIDEO_IMG_FORMAT_H265 video_four_character_code('H', '2', '6', '5')       /* H265 with start codes */
#define VIDEO_IMG_FORMAT_H264 video_four_character_code('H', '2', '6', '4')       /* H264 with start codes */
#define VIDEO_IMG_FORMAT_H264_NO_SC video_four_character_code('A', 'V', 'C', '1') /* H264 without start codes */
#define VIDEO_IMG_FORMAT_H264_MVC video_four_character_code('M', '2', '6', '4')   /* H264 MVC */
#define VIDEO_IMG_FORMAT_H263 video_four_character_code('H', '2', '6', '3')       /* H263          */
#define VIDEO_IMG_FORMAT_MPEG1 video_four_character_code('M', 'P', 'G', '1')      /* MPEG-1 ES     */
#define VIDEO_IMG_FORMAT_MPEG2 video_four_character_code('M', 'P', 'G', '2')      /* MPEG-2 ES     */
#define VIDEO_IMG_FORMAT_MPEG4 video_four_character_code('M', 'P', 'G', '4')      /* MPEG-4 ES     */
#define VIDEO_IMG_FORMAT_XVID video_four_character_code('X', 'V', 'I', 'D')       /* Xvid           */
#define VIDEO_IMG_FORMAT_VC1_ANNEX_G \
    video_four_character_code('V', 'C', '1', 'G') /* SMPTE 421M Annex G compliant stream */
#define VIDEO_IMG_FORMAT_VC1_ANNEX_L \
    video_four_character_code('V', 'C', '1', 'L')                          /* SMPTE 421M Annex L compliant stream */
#define VIDEO_IMG_FORMAT_VP8 video_four_character_code('V', 'P', '8', '0') /* VP8 */

#define OT_UVC_INTF_CONTROL 0
#define OT_UVC_INTF_STREAMING 1

#define MAX_PAYLOAD_IMAGE_SIZE 16588800
#define VIDEO_MAX_FLATS 8
#define array_size(a) ((sizeof((a)) / sizeof((a)[0])))
#define FRAME_INTERVAL_MAX 30
#define FRAME_INTERVAL_CALC_100NS 10000000

struct ot_uvc_streaming_control {
    td_u16 bm_hint;
    td_u8 b_format_index;
    td_u8 b_frame_index;
    td_u32 dw_frame_interval;
    td_u16 w_key_frame_rate;
    td_u16 wp_frame_rate;
    td_u16 w_comp_quality;
    td_u16 w_comp_window_size;
    td_u16 w_delay;
    td_u32 dw_max_video_frame_size;
    td_u32 dw_max_payload_transfer_size;
    td_u32 dw_clock_frequency;
    td_u8 bm_framing_info;
    td_u8 b_prefered_version;
    td_u8 b_min_version;
    td_u8 b_max_version;
} __attribute__((__packed__));

struct uvc_request_data {
    td_s32 length;
    td_u8 data[60];
};

struct ot_uvc_event {
    union {
        struct usb_ctrlrequest req;
        struct uvc_request_data data;
        enum usb_device_speed speed;
    };
};

struct video_event_descriptor {
    td_u32 dw_type;
    td_u32 dw_id;
    td_u32 dw_flags;
    td_u32 dw_reserved[5];
};

struct video_event_vsync {
    td_u8 by_field;
} __attribute__((packed));

struct video_event_ctrl {
    td_u32 dw_chg;
    td_u32 dw_type;
    union {
        td_s32 i_val;
        td_s64 i_val64;
    };
    td_u32 dw_flag;
    td_s32 i_min;
    td_s32 i_max;
    td_s32 i_step;
    td_s32 i_def_val;
};

struct video_event_frame_sync {
    td_u32 dw_iframe_seq;
};

struct video_event {
    td_u32 dw_type;
    union {
        td_u8 a_data[64];
        struct video_event_vsync v_sync;
        struct video_event_frame_sync frame_sync;
        struct video_event_ctrl ctrl;
    } u;
    td_u32 dw_sequence;
    td_u32 dw_pending;
    struct timespec time_stamp;
    td_u32 dw_id;
    td_u32 dw_reserved[8];
};

enum VideoBufKinds {
    VIDEO_CACHE_TYPE_CAP = 1,
    VIDEO_CACHE_TYPE_OPT = 2,
    VIDEO_CACHE_TYPE_ORLY = 3,
    VIDEO_CACHE_TYPE_VBI_CAP = 4,
    VIDEO_CACHE_TYPE_VBI_OPT = 5,
    VIDEO_CACHE_TYPE_SLD_VBI_CAP = 6,
    VIDEO_CACHE_TYPE_SLD_VBI_OPT = 7,

    /* Experimental */
    VIDEO_CACHE_TYPE_OPT_ORLY = 8,
    VIDEO_CACHE_TYPE_CAP_MLAN = 9,
    VIDEO_CACHE_TYPE_OTP_MLAN = 10,

    /* Deprecated, do not use */
    VIDEO_CACHE_TYPE_SECRET = 0x80,
};

enum VideoMem {
    VIDEO_MM_MM = 1,
    VIDEO_MM_USER = 2,
    VIDEO_MM_ORLY = 3,
    VIDEO_MM_DMA = 4,
};

enum VideoDomain {
    VIDEO_FLD_ALL = 0,      /* driver can choose from none,
                            top, bottom, td_s32erlaced
                            depending on whatever it thinks
                            is approximate ... */
    VIDEO_FLD_NOTHING = 1,  /* this device has no fields ... */
    VIDEO_FLD_ROOF = 2,     /* top field only */
    VIDEO_FLD_BASE = 3,     /* bottom field only */
    VIDEO_FLD_MULTI = 4,    /* both fields td_s32erlaced */
    VIDEO_FLD_SEQ_TB = 5,   /* both fields sequential td_s32o one
                            buffer, top-bottom order */
    VIDEO_FLD_SEQ_BT = 6,   /* same as above + bottom-top order */
    VIDEO_FLD_ALTNT = 7,    /* both fields alternating td_s32o
                            separate buffers */
    VIDEO_FLD_MULTI_TB = 8, /* both fields td_s32erlaced, top field
                            first and the top field is
                            transmitted first */
    VIDEO_FLD_MULTI_BT = 9, /* both fields td_s32erlaced, top field
                            first and the bottom field is
                            transmitted first */
};

struct video_ability {
    td_u8 a_driver[16];
    td_u8 a_card[32];
    td_u8 a_bus_info[32];
    td_u32 dw_ver;
    td_u32 dw_caps;
    td_u32 dw_dev_caps;
    td_u32 dw_reserved[3];
};

struct video_time_fmt {
    td_u32 dw_type;
    td_u32 dw_flag;
    td_u8 b_frame;
    td_u8 b_sec;
    td_u8 b_min;
    td_u8 b_hour;
    td_u8 b_useu[4];
};

struct video_cache {
    td_u32 dw_index;
    td_u32 dw_type;
    td_u32 dw_used;
    td_u32 dw_flags;
    td_u32 dw_fld;
    struct timeval tivs;
    struct video_time_fmt fmt;
    td_u32 dw_seq;

    /* memory location */
    td_u32 dw_mem;
    union {
        td_s32 i_fd;
        td_u32 dw_offset;
        td_ulong ul_usr;
        struct video_flat *flats;
    } n;
    td_u32 dw_len;
    td_u32 dw_reserved2;
    td_u32 dw_reserved;
};

struct video_buf_request {
    td_u32 dw_cnt;
    td_u32 dw_type; /* enum VideoBufKinds */
    td_u32 dw_mem;  /* enum VideoMem */
    td_u32 dw_reserved[2];
};

struct video_img_fmt {
    td_u32 dw_width;
    td_u32 dw_height;
    td_u32 dw_fmt;
    td_u32 dw_fld;      /* enum VideoDomain */
    td_u32 dw_eachline; /* for padding, zero if unused */
    td_u32 dw_size;
    td_u32 dw_colorspace; /* enum v4l2_colorspace */
    td_u32 dw_sec;        /* private data, depends on pixelformat */
};

struct video_flat_img_fmt {
    td_u32 dw_size;
    td_u16 w_eachline;
    td_u16 w_reserved[7];
} __attribute__((packed));

struct video_img_fmt_mflat {
    td_u32 dw_width;
    td_u32 dw_height;
    td_u32 dw_fmt;
    td_u32 dw_fld;
    td_u32 dw_colorspace;

    struct video_flat_img_fmt flat_fmt[VIDEO_MAX_FLATS];
    td_u8 b_num;
    td_u8 b_reserved[11];
} __attribute__((packed));

struct video_shape {
    td_s32 i_left;
    td_s32 i_top;
    td_s32 i_width;
    td_s32 i_height;
};

struct video_clip {
    struct video_shape h;
    struct video_clip *next;
};

struct video_win {
    struct video_shape p;
    td_u32 dw_fld; /* enum VideoDomain */
    td_u32 dw_chromakey;
    struct video_clip *clips;
    td_u32 dw_clipicnt;
    void *bmp;
    td_u8 b_global_alpha;
};

struct video_vbi_fmt {
    td_u32 dw_sam_rate; /* in 1 Hz */
    td_u32 dw_offset;
    td_u32 dw_sam_each_line;
    td_u32 dw_sam_fmt; /* VIDEO_IMG_FORMAT_* */
    td_s32 dw_start[2];
    td_u32 dw_cnt[2];
    td_u32 dw_flag;        /* V4L2_VBI_* */
    td_u32 dw_reserved[2]; /* must be zero */
};

struct video_part_vbi_fmt {
    td_u16 w_service;

    /* service_lines[0][...] specifies lines 0-23 (1-23 used) of the first field
       service_lines[1][...] specifies lines 0-23 (1-23 used) of the second field
       (equals frame lines 313-336 for 625 line video
       standards, 263-286 for 525 line standards) */
    td_u16 w_srv_line[2][24];
    td_u32 dw_size;
    td_u32 dw_reserved[2]; /* must be zero */
};

struct video_fmt {
    td_u32 dw_type;
    union {
        struct video_img_fmt pix;        /* VIDEO_CACHE_TYPE_CAP */
        struct video_img_fmt_mflat pix_mp; /* VIDEO_CACHE_TYPE_CAP_MLAN */
        struct video_win win;           /* VIDEO_CACHE_TYPE_ORLY */
        struct video_vbi_fmt vbi;        /* VIDEO_CACHE_TYPE_VBI_CAP */
        struct video_part_vbi_fmt sliced; /* VIDEO_CACHE_TYPE_SLD_VBI_CAP */
        td_u8 b_raw_data[200];          /* user-defined */
    } fmt;
};

struct ot_uvc_prb {
    td_u8 b_set;
    td_u8 b_get;
    td_u8 b_max;
    td_u8 b_min;
};

struct ot_uvc_dev {
    td_s32 i_fd;
    struct ot_uvc_streaming_control probe;
    struct ot_uvc_streaming_control commit;
    td_s32 i_control;
    td_s32 i_unit_id;
    td_s32 i_intf_id;
    td_u32 dw_fcc;
    td_u32 dw_width;
    td_u32 dw_height;
    td_u32 dw_nbufs;
    td_u32 dw_bulk;
    td_s32 i_fps;
    td_u8 b_color;
    td_u32 dw_img_size;
    td_u32 dw_bulk_size;
    struct ot_uvc_prb probe_status;
    td_s32 i_streaming;

    /* USB speed specific */
    td_s32 m_ult;
    td_s32 b_urst;
    td_s32 m_axpkt;
    enum usb_device_speed speed;

    struct uvc_request_data request_error_code;
};

struct ot_uvc_frame_info {
    td_u32 width;
    td_u32 height;
    td_u32 intervals[8];
    td_u32 ss_xfersize;
    td_u32 hs_xfersize;
};

struct ot_uvc_format_info {
    td_u32 fcc;
    const struct ot_uvc_frame_info *frames;
};

static const struct ot_uvc_frame_info ot_uvc_frames_yuyv[] = {
    {  640,  360, {333333,       0 }, 2304, 1984},
    { 1280,  720, {333333,       0 }, 9216, 2688},
    { 1920, 1080, {333333,       0 }, 20736, 3072},
    { 3840, 2160, {333333,       0 }, 32768, 3072},
    {    0,    0, {     0,         }, 0, 0},
};

static const struct ot_uvc_frame_info ot_uvc_frames_mjpeg[] = {
    {  640,  360, {333333,       0 }, 1024, 256},
    { 1280,  720, {333333,       0 }, 1024, 512},
    { 1920, 1080, {333333,       0 }, 1024, 1024},
    { 3840, 2160, {333333,       0 }, 9216, 3072},
    {    0,    0, {     0,         }, 0, 0},
};

static const struct ot_uvc_frame_info ot_uvc_frames_h264[] = {
    {  640,  360, {333333,       0 }, 1024, 256},
    { 1280,  720, {333333,       0 }, 1024, 512},
    { 1920, 1080, {333333,       0 }, 1024, 1024},
    { 3840, 2160, {333333,       0 }, 9216, 3072},
    {    0,    0, {     0,         }, 0, 0},
};

static const struct ot_uvc_frame_info ot_uvc_frames_h265[] = {
    {  640,  360, {333333,       0 }, 1024, 256},
    { 1280,  720, {333333,       0 }, 1024, 512},
    { 1920, 1080, {333333,       0 }, 1024, 1024},
    { 3840, 2160, {333333,       0 }, 9216, 3072},
    {    0,    0, {     0,         }, 0, 0},
};

static const struct ot_uvc_frame_info ot_uvc_frames_nv12[] = {
    { 640,   360, {333333,       0}, 2304, 1984 },
    { 1280,  720, {333333,       0}, 9216, 2688 },
    { 1920, 1080, {333333,       0}, 20736, 3072 },
    { 3840, 2160, {333333,       0}, 32768, 3072 },
    { 0,       0, {     0,        }, 0, 0 },
};

static const struct ot_uvc_frame_info ot_uvc_frames_nv21[] = {
    { 640,   360, {333333,       0}, 2304, 1984 },
    { 1280,  720, {333333,       0}, 9216, 2688 },
    { 1920, 1080, {333333,       0}, 20736, 3072 },
    { 3840, 2160, {333333,       0}, 32768, 3072 },
    { 0,       0, {     0,        }, 0, 0 },
};

static const struct ot_uvc_format_info ot_fmt[] = {
    { VIDEO_IMG_FORMAT_NV21,  ot_uvc_frames_nv21  },
    { VIDEO_IMG_FORMAT_MJPEG, ot_uvc_frames_mjpeg },
    { VIDEO_IMG_FORMAT_H264,  ot_uvc_frames_h264  },
    { VIDEO_IMG_FORMAT_H265,  ot_uvc_frames_h265  },
};

td_s32 open_uvc_device(const td_char *char_dev_path);
td_s32 close_uvc_device(td_void);
td_s32 run_uvc_data(td_void);
td_s32 run_uvc_device(td_void);
td_void nothing();

#endif /* __LINUX_OT_USB_CAMERA_H */
