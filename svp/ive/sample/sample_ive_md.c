/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */
#include "sample_common_ive.h"
#include "ot_ivs_md.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define OT_SAMPLE_IVE_MD_IMAGE_NUM          2
#define OT_SAMPLE_IVE_MD_MILLIC_SEC         20000
#define OT_SAMPLE_IVE_MD_ADD_X_VAL          32768
#define OT_SAMPLE_IVE_MD_ADD_Y_VAL          32768
#define OT_SAMPLE_IVE_MD_THREAD_NAME_LEN    16
#define OT_SAMPLE_IVE_MD_AREA_THR_STEP      8
#define OT_SAMPLE_IVE_MD_VPSS_CHN           2
#define OT_SAMPLE_IVE_MD_NUM_TWO            2
#define OT_SAMPLE_IVE_SAD_THRESHOLD         100

typedef struct {
    ot_svp_src_img img[OT_SAMPLE_IVE_MD_IMAGE_NUM];
    ot_svp_dst_mem_info blob;
    ot_md_attr md_attr;
    ot_sample_svp_rect_info region;
} ot_sample_ivs_md_info;

typedef struct {
    ot_md_chn md_chn;
    ot_vo_layer vo_layer;
    ot_vo_chn vo_chn;
    td_s32 vpss_grp;
} ot_sample_md_vo_vpss_hld;

static td_bool g_stop_signal = TD_FALSE;
static pthread_t g_md_thread;
static ot_sample_ivs_md_info g_md_info;
static ot_sample_svp_switch g_md_switch = { TD_FALSE, TD_TRUE };
static sample_vi_cfg g_vi_config;
static ot_sample_src_dst_size g_src_dst;

static td_void sample_ivs_md_uninit(ot_sample_ivs_md_info *md_info_ptr)
{
    td_s32 i;
    td_s32 ret;

    sample_svp_check_exps_return_void(md_info_ptr == TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++) {
        sample_svp_mmz_free(md_info_ptr->img[i].phys_addr[0], md_info_ptr->img[i].virt_addr[0]);
    }

    sample_svp_mmz_free(md_info_ptr->blob.phys_addr, md_info_ptr->blob.virt_addr);

    ret = ot_ivs_md_exit();
    if (ret != TD_SUCCESS) {
        sample_svp_trace_err("ot_ivs_md_exit fail,Error(%#x)\n", ret);
        return;
    }
}

static td_s32 sample_ivs_md_init(ot_sample_ivs_md_info *md_inf_ptr, td_u32 width, td_u32 height)
{
    td_s32 ret = OT_ERR_IVE_NULL_PTR;
    td_s32 i;
    td_u32 size, sad_mode;
    td_u8 wnd_size;

    sample_svp_check_exps_return(md_inf_ptr == TD_NULL, ret, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++) {
        ret = sample_common_ive_create_image(&md_inf_ptr->img[i], OT_SVP_IMG_TYPE_U8C1, width, height);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error(%#x),Create img[%d] image failed!\n", ret, i);
    }
    size = sizeof(ot_ive_ccblob);
    ret = sample_common_ive_create_mem_info(&md_inf_ptr->blob, size);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),Create blob mem info failed!\n", ret);

    /* Set attr info */
    md_inf_ptr->md_attr.alg_mode = OT_MD_ALG_MODE_BG;
    md_inf_ptr->md_attr.sad_mode = OT_IVE_SAD_MODE_MB_4X4;
    md_inf_ptr->md_attr.sad_out_ctrl = OT_IVE_SAD_OUT_CTRL_THRESHOLD;
    md_inf_ptr->md_attr.sad_threshold = OT_SAMPLE_IVE_SAD_THRESHOLD * (1 << 1);
    md_inf_ptr->md_attr.width = width;
    md_inf_ptr->md_attr.height = height;
    md_inf_ptr->md_attr.add_ctrl.x = OT_SAMPLE_IVE_MD_ADD_X_VAL;
    md_inf_ptr->md_attr.add_ctrl.y = OT_SAMPLE_IVE_MD_ADD_Y_VAL;
    md_inf_ptr->md_attr.ccl_ctrl.mode = OT_IVE_CCL_MODE_4C;
    sad_mode = (td_u32)md_inf_ptr->md_attr.sad_mode;
    wnd_size = (1 << (OT_SAMPLE_IVE_MD_NUM_TWO + sad_mode));
    md_inf_ptr->md_attr.ccl_ctrl.init_area_threshold = wnd_size * wnd_size;
    md_inf_ptr->md_attr.ccl_ctrl.step = wnd_size;

    ret = ot_ivs_md_init();
    sample_svp_check_exps_goto(ret != TD_SUCCESS,  md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),ot_ivs_md_init failed!\n", ret);

md_init_fail:
    if (ret != TD_SUCCESS) {
        sample_ivs_md_uninit(md_inf_ptr);
    }
    return ret;
}

static td_void sample_ivs_set_src_dst_size(ot_sample_src_dst_size *src_dst, td_u32 src_width,
    td_u32 src_height, td_u32 dst_width, td_u32 dst_height)
{
    src_dst->src.width = src_width;
    src_dst->src.height = src_height;
    src_dst->dst.width = dst_width;
    src_dst->dst.height = dst_height;
}

/* first frame just init reference frame, if not, change the frame idx */
static td_s32 sample_ivs_md_dma_data(td_u32 cur_idx, ot_video_frame_info *frm,
    ot_sample_ivs_md_info *md_ptr, td_bool *is_first_frm)
{
    td_s32 ret;
    td_bool is_instant = TD_TRUE;
    if (*is_first_frm != TD_TRUE) {
            ret = sample_common_ive_dma_image(frm, &md_ptr->img[cur_idx], is_instant);
            sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                "sample_ive_dma_image fail,Err(%#x)\n", ret);
        } else {
            ret = sample_common_ive_dma_image(frm, &md_ptr->img[1 - cur_idx], is_instant);
            sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                "sample_ive_dma_image fail,Err(%#x)\n", ret);

            *is_first_frm = TD_FALSE;
        }
    return TD_SUCCESS;
}

int yuv2rgb_nv12(unsigned char* pYuvBuf, unsigned char* pRgbBuf, int height, int width)
{
    if(width < 1 || height < 1 || pYuvBuf == NULL || pRgbBuf == NULL)
    {
        return 0;
    }

    const long len = height * width;
    
    // Y与UV数据地址
    unsigned char *yData = pYuvBuf;
    unsigned char *uvData = yData + len;

	// 	R、G、B数据地址
    unsigned char *rData = pRgbBuf;
    unsigned char *gData = rData + len;
    unsigned char *bData = gData + len;

    int R[4], G[4], B[4];
    int Y[4], U, V;
    int y0_Idx, y1_Idx, uIdx, vIdx;

    for (int i = 0; i < height; i=i+2)
    {
        for (int j = 0; j < width; j=j+2)
        {
            y0_Idx = i * width + j;
            y1_Idx = (i + 1) * width + j;

			// Y[0]、Y[1]、Y[2]、Y[3]分别代表 Y00、Y01、Y10、Y11
            Y[0] = yData[y0_Idx];
            Y[1] = yData[y0_Idx + 1];
            Y[2] = yData[y1_Idx];
            Y[3] = yData[y1_Idx + 1];

            uIdx = (i / 2) * width + j;
            vIdx = uIdx + 1;

            U = uvData[uIdx];
            V = uvData[vIdx];

            R[0] = Y[0] + 1.402 * (V - 128);
            G[0] = Y[0] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[0] = Y[0] + 1.772 * (U - 128);

            R[1] = Y[1] + 1.402 * (V - 128);
            G[1] = Y[1] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[1] = Y[1] + 1.772 * (U - 128);

            R[2] = Y[2] + 1.402 * (V - 128);
            G[2] = Y[2] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[2] = Y[2] + 1.772 * (U - 128);

            R[3] = Y[3] + 1.402 * (V - 128);
            G[3] = Y[3] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[3] = Y[3] + 1.772 * (U - 128);

			// 像素值限定在 0-255
            for (int k = 0; k < 4; ++k)
            {
                if(R[k] >= 0 && R[k] <= 255)
                {
                    R[k] = R[k];
                }
                else
                {
                    R[k] = (R[k] < 0) ? 0 : 255;
                }

                if(G[k] >= 0 && G[k] <= 255)
                {
                    G[k] = G[k];
                }
                else
                {
                    G[k] = (G[k] < 0) ? 0 : 255;
                }

                if(B[k] >= 0 && B[k] <= 255)
                {
                    B[k] = B[k];
                }
                else
                {
                    B[k] = (B[k] < 0) ? 0 : 255;
                }
            }

            *(rData + y0_Idx) = R[0];
            *(gData + y0_Idx) = G[0];
            *(bData + y0_Idx) = B[0];

            *(rData + y0_Idx + 1) = R[1];
            *(gData + y0_Idx + 1) = G[1];
            *(bData + y0_Idx + 1) = B[1];

            *(rData + y1_Idx) = R[2];
            *(gData + y1_Idx) = G[2];
            *(bData + y1_Idx) = B[2];

            *(rData + y1_Idx + 1) = R[3];
            *(gData + y1_Idx + 1) = G[3];
            *(bData + y1_Idx + 1) = B[3];
        }
    }
    return 1;
}


int shmid_tx;
void* ptr_tx;
//int shmid_rx;
//void* ptr_rx;
unsigned int size;
int p[61];

ot_sample_svp_rect_info region_tmp;



ot_sample_svp_rect_info function_draw_region() {

//	if(strlen(ptr_rx)!= 0 ) {
	if(1) {
//		memcpy(p, ptr_rx, 244);
	//	memset(ptr_rx, 0, 244);
//		memset(ptr_tx, 0, 12441600);

		//for (int i = 0; i < 6; i++)
		//{
		//	printf("p[%d] is %d,",i, p[i]);	
		//}
		//printf("\n");

		region_tmp.num = p[0];
		if(region_tmp.num != 0) {
			printf("num is %d\n",region_tmp.num);
			for(int i = 0; i < region_tmp.num; i++)
			{
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].x =  p[6*i +3];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].y =  p[6*i +4];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].x =   p[6*i +3];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].y =   p[6*i +6];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].x =   p[6*i +5];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].y =   p[6*i +4];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].x = p[6*i +5];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].y = p[6*i +6];
			}
		}
	}
	return region_tmp; 
}

unsigned char *user_addr;

int sockfd;
struct sockaddr_in serverAddr;
char *buf;
#define HOST_IP "192.168.1.66"
const int PORT = 1777;

void *udp_recv_thread() {

	void* ptr_recv;
	ptr_recv = malloc(256);	
	printf("ptr size if %d\n",sizeof(ptr_recv));
        while(1) {
                socklen_t len = sizeof(serverAddr);
                memset(ptr_recv,0,256);
                recvfrom(sockfd,ptr_recv,sizeof(ptr_recv),0,(struct sockaddr*)&serverAddr,&len);
		memcpy(p, ptr_recv, 256);
		printf("ptr_recv = %s\n", ptr_recv);
		printf("p[0] = %d\n", p[0]);		
		region_tmp.num = 0;
#if 0
		if(region_tmp.num != 0) {
			printf("num is %d\n",region_tmp.num);
			for(int i = 0; i < region_tmp.num; i++)
			{
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].x =  p[6*i +3];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].y =  p[6*i +4];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].x =   p[6*i +3];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].y =   p[6*i +6];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].x =   p[6*i +5];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].y =   p[6*i +4];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].x = p[6*i +5];
				region_tmp.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].y = p[6*i +6];
			}
		}
#endif
	}

}


void start_udp_server() {
    socklen_t addr_size;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    printf("start connect udp server...\n");
    memset(&serverAddr, '\0', sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = inet_addr(HOST_IP);

    bind(sockfd,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
    pthread_t recv_thread = 0;


    pthread_create(&recv_thread, NULL, udp_recv_thread, NULL);
    pthread_detach(recv_thread);


}


static td_void *sample_ivs_md_proc(td_void *args)
{
    td_s32 ret;
    ot_sample_ivs_md_info *md_ptr = (ot_sample_ivs_md_info *)(args);
    ot_video_frame_info frm[OT_SAMPLE_IVE_MD_VPSS_CHN]; /* 0:base_frm, 1:ext_frm */
    ot_sample_md_vo_vpss_hld hld = {0};
    td_s32 vpss_chn[] = { OT_VPSS_CHN0, OT_VPSS_CHN1 };
    td_s32 cur_idx = 0;
    td_bool is_first_frm = TD_TRUE;

    sample_svp_check_exps_return(md_ptr == TD_NULL, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    /* Create chn */
    ret = ot_ivs_md_create_chn(hld.md_chn, &(md_ptr->md_attr));
    sample_svp_check_exps_return(ret != TD_SUCCESS, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "ot_ivs_md_create_chn fail\n");
//udp server

    start_udp_server();

//udpserver end
    int count;
    size =12441600;
    ptr_tx = malloc(size);
    user_addr = malloc(size);
    //ptr_rx = malloc(512);
    shmid_tx = shmget(100, size, IPC_CREAT|0664);
    ptr_tx = shmat(shmid_tx, NULL, 0);
   // shmid_rx = shmget(111, 256, IPC_CREAT|0664);
    //ptr_rx = shmat(shmid_rx, NULL, 0);
    
    while (g_stop_signal == TD_FALSE) {
        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        sample_svp_check_exps_continue(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Err(%#x),vpss_get_chn_frame failed, vpss_grp(%d), vpss_chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);

        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        sample_svp_check_failed_goto(ret, ext_free, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Error(%#x),vpss_get_chn_frame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);

        ret = sample_ivs_md_dma_data(cur_idx, &frm[1], md_ptr, &is_first_frm);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "dma data failed, Err(%#x)\n", ret);

        /* change idx */
        if (is_first_frm == TD_TRUE) {
            goto change_idx;
        }

        ret = ot_ivs_md_proc(hld.md_chn, &md_ptr->img[cur_idx], &md_ptr->img[1 - cur_idx], TD_NULL, &md_ptr->blob);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "ivs_md_proc fail,Err(%#x)\n", ret);


	sample_ivs_set_src_dst_size(&g_src_dst, md_ptr->md_attr.width, md_ptr->md_attr.height,
            frm[0].video_frame.width, frm[0].video_frame.height);
        ret = sample_common_ive_blob_to_rect(sample_svp_convert_addr_to_ptr(ot_ive_ccblob, md_ptr->blob.virt_addr),
            &(md_ptr->region), OT_SVP_RECT_NUM, OT_SAMPLE_IVE_MD_AREA_THR_STEP, g_src_dst);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "blob to rect failed!\n");
	count++;
#if 1
	//if(strlen(ptr_tx) == 0) {
	if(1) {
	    user_addr = (unsigned char *)ss_mpi_sys_mmap(frm[0].video_frame.phys_addr[0], size);
	    memcpy(ptr_tx,user_addr,size);
	    ss_mpi_sys_munmap(user_addr, size);
	}
	
	//if(strlen(ptr_rx) != 0) {
//	if(1) {
//	    region_tmp = function_draw_region();
//	}
	
//		memcpy(p, ptr_rx, 244);
//		memset(ptr_rx, 0, 244);
//		memset(ptr_tx, 0, 12441600);
//	
//	for (int i = 0; i < 6; i++)
//	{
//	      printf("p[%d] is %d,",i, p[i]);
//	}
//	printf("\n");

//	region_tmp.num = p[0];
//	if(p[0] != 0) {
//		printf("num is %d\n", p[0]);
//		for(int i = 0; i < region_tmp.num; i++)
//		{
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].x =  p[3];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].y =  p[4];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].x =   p[3];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].y =   p[6];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].x =   p[5];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].y =   p[4];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_THREE].x = p[5];
//		region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_THREE].y = p[6];
//		}
//	}
     //int num = 2;
     ////ot_sample_svp_rect_info region_tmp;
     //region_tmp.num = num;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].x = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].y = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].x = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].y = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].x = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].y = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].x = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ZERO].y = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].x = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_ONE].y = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].x = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_TWO].y = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_THREE].x = 500;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_THREE].y = 100;
     //region_tmp.rect[0].point[OT_SAMPLE_POINT_IDX_THREE].y = 100;

     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_ZERO].x = 1000;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_ZERO].y = 1000;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_ONE].x = 1500;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_ONE].y = 1000;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_TWO].x = 1500;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_TWO].y = 1500;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_THREE].x = 1000;
     //region_tmp.rect[1].point[OT_SAMPLE_POINT_IDX_THREE].y = 1500;

#endif
	
	/* Draw rect */
       // ret = sample_common_svp_vgs_fill_rect(&frm[0], &md_ptr->region, 0x0000FF00);
        ret = sample_common_svp_vgs_fill_rect(&frm[0], &region_tmp, 0x0000FF00);
        sample_svp_check_failed_err_level_goto(ret, base_free, "sample_svp_vgs_fill_rect fail,Err(%#x)\n", ret);

        ret = ss_mpi_vo_send_frame(hld.vo_layer, hld.vo_chn, &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        sample_svp_check_failed_err_level_goto(ret, base_free, "ss_mpi_vo_send_frame fail,Error(%#x)\n", ret);

change_idx:
        /* Change reference and current frame index */
        cur_idx = 1 - cur_idx;
base_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);

ext_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
            "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);
    }

    /* destroy */
    ret = ot_ivs_md_destroy_chn(hld.md_chn);
    sample_svp_check_failed_trace(ret, SAMPLE_SVP_ERR_LEVEL_ERROR,  "ot_ivs_md_destroy_chn fail,Err(%#x)\n", ret);

    return TD_NULL;
}

static td_s32 sample_ive_md_pause(td_void)
{
    printf("---------------press Enter key to exit!---------------\n");
    if (g_stop_signal == TD_TRUE) {
        if (g_md_thread != 0) {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_ivs_md_uninit(&(g_md_info));
        (td_void)memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }

    (void)getchar();

    if (g_stop_signal == TD_TRUE) {
        if (g_md_thread != 0) {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_ivs_md_uninit(&(g_md_info));
        (td_void)memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }
    return TD_FALSE;
}

td_void sample_ive_md(td_void)
{
    ot_size pic_size;
    ot_pic_size pic_type = PIC_1080P;
    td_s32 ret;

    (td_void)memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));
    /*
     * step 1: start vi vpss venc vo
     */
    ret = sample_common_svp_start_vi_vpss_venc_vo(&g_vi_config, &g_md_switch, &pic_type);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),sample_common_svp_start_vi_vpss_venc_vo failed!\n", ret);

    pic_type = PIC_1080P;
    ret = sample_comm_sys_get_pic_size(pic_type, &pic_size);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        "Error(%#x),sample_comm_sys_get_pic_size failed!\n", ret);
    /*
     * step 2: Init Md
     */
    ret = sample_ivs_md_init(&g_md_info, pic_size.width, pic_size.height);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
        " Error(%#x),sample_ivs_md_init failed!\n", ret);
    g_stop_signal = TD_FALSE;
    /*
     * step 3: Create work thread
     */
    ret = prctl(PR_SET_NAME, "ive_md_proc", 0, 0, 0);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "set thread name failed!\n");
    ret = pthread_create(&g_md_thread, 0, sample_ivs_md_proc, (td_void *)&g_md_info);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "pthread_create failed!\n");

//    ret = sample_ive_md_pause();
    while(1);
    sample_svp_check_exps_return_void(ret == TD_TRUE, SAMPLE_SVP_ERR_LEVEL_ERROR, "md exist!\n");
    g_stop_signal = TD_TRUE;
    pthread_join(g_md_thread, TD_NULL);
    g_md_thread = 0;

    sample_ivs_md_uninit(&(g_md_info));
    (td_void)memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

end_md_0:
    g_md_thread = 0;
    g_stop_signal = TD_TRUE;
    sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
    return;
}

/*
 * function : Md sample signal handle
 */
td_void sample_ive_md_handle_sig(td_void)
{
    g_stop_signal = TD_TRUE;
}
