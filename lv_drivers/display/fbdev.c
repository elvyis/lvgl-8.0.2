/**
 * @file fbdev.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "fbdev.h"
#if USE_FBDEV || USE_BSD_FBDEV

#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#if USE_BSD_FBDEV
#include <sys/fcntl.h>
#include <sys/time.h>
#include <sys/consio.h>
#include <sys/fbio.h>
#else  /* USE_BSD_FBDEV */
#include <linux/fb.h>
#endif /* USE_BSD_FBDEV */

/*********************
 *      DEFINES
 *********************/


#ifndef FBDEV_PATH
#define FBDEV_PATH  "/dev/fb0"
#endif

static char *device_Log_GetTime()
{
    static char s[20];
    time_t t;
    struct tm* ltime;
    time(&t);
    ltime = localtime(&t);
    strftime(s, 20, "%Y-%m-%d %H:%M:%S", ltime);
    return s;
}
#define GET_TIME device_Log_GetTime()

#ifdef HMY_LOG_ONLY
#define __DBG_PRINTF(string,args...) printf("");
#else
#define __DBG_PRINTF(string,args...) printf("\033[32m""[%s] <%s> (%d): "string"\033[0m\r\n",GET_TIME,__FUNCTION__,__LINE__,##args)
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *      STRUCTURES
 **********************/

struct bsd_fb_var_info{
    uint32_t xoffset;
    uint32_t yoffset;
    uint32_t xres;
    uint32_t yres;
    int bits_per_pixel;
 };

struct bsd_fb_fix_info{
    long int line_length;
    long int smem_len;
};

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
#if USE_BSD_FBDEV
static struct bsd_fb_var_info vinfo;
static struct bsd_fb_fix_info finfo;
#else
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;
#endif /* USE_BSD_FBDEV */
static char *fbp = 0;
static char *fbpbase = 0;
static long int screensize = 0;
static long int half_screensize = 0;
static int fbfd = 0;
static uint32_t toggle=0;

/**********************
 *      MACROS
 **********************/

#if USE_BSD_FBDEV
#define FBIOBLANK FBIO_BLANK
#endif /* USE_BSD_FBDEV */

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
#if PLATFORM_HISILICON
#include "hi_type.h"
#include "hifb.h"

static struct fb_bitfield s_a32 = {24,8,0};
static struct fb_bitfield s_r32 = {16,8,0};
static struct fb_bitfield s_g32 = {8,8,0};
static struct fb_bitfield s_b32 = {0,8,0};

#define HIFB_WIDTH                  1920
#define HIFB_HEIGHT                 1080


int HIFB_Init(void)
{
    HI_S32 i,x,y,s32Ret;
    HI_U32 u32FixScreenStride = 0;
    HIFB_ALPHA_S stAlpha={0};
    HIFB_POINT_S stPoint = {40, 112};

    HI_BOOL bShow;

    /* 1. open framebuffer device overlay 0 */
    fbfd = open(FBDEV_PATH, O_RDWR, 0);
    if(fbfd < 0)
    {
        perror("hifbfd open failed!\n");
        return HI_FAILURE;
    }

    bShow = HI_FALSE;
    if(ioctl(fbfd, FBIOPUT_SHOW_HIFB, &bShow) < 0)
    {
        perror("FBIOPUT_SHOW_HIFB failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }
	
    /* 2. set the screen original position */
	stPoint.s32XPos = 0;
    stPoint.s32YPos = 0;

    if(ioctl(fbfd, FBIOPUT_SCREEN_ORIGIN_HIFB, &stPoint) < 0)
    {
        perror("set screen original show position failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }

    /* 3. get the variable screen info */
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) < 0)
    {
        perror("Get variable screen info failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }

    /* 4. modify the variable screen info
          the screen size: IMAGE_WIDTH*IMAGE_HEIGHT
          the virtual screen size: VIR_SCREEN_WIDTH*VIR_SCREEN_HEIGHT
          (which equals to VIR_SCREEN_WIDTH*(IMAGE_HEIGHT*2))
          the pixel format: ARGB1555
    */
    usleep(40*1000);

    vinfo.xres_virtual = HIFB_WIDTH;
    vinfo.yres_virtual = HIFB_HEIGHT*2;
    vinfo.xres = HIFB_WIDTH;
    vinfo.yres = HIFB_HEIGHT;
 
	vinfo.transp= s_a32;
    vinfo.red = s_r32;
    vinfo.green = s_g32;
    vinfo.blue = s_b32;
    vinfo.bits_per_pixel = 32;
    vinfo.activate = FB_ACTIVATE_NOW;

    /* 5. set the variable screeninfo */
    if(ioctl(fbfd, FBIOPUT_VSCREENINFO, &vinfo) < 0)
    {
        perror("Put variable screen info failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }

    /* 6. get the fix screen info */
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) < 0)
    {
        perror("Get fix screen info failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }
	
    u32FixScreenStride = finfo.line_length;   /*fix screen stride*/
	screensize =  finfo.smem_len;

    /* 7. map the physical video memory for user use */
    fbp = mmap(HI_NULL, screensize, PROT_READ|PROT_WRITE, MAP_SHARED, fbfd, 0);
    if(MAP_FAILED == fbp)
    {
        perror("mmap framebuffer failed!\n");
		close(fbfd);
        return HI_FAILURE;
    }

    memset(fbp, 0x00, screensize);

    /* time to play*/
    bShow = HI_TRUE;
    if(ioctl(fbfd, FBIOPUT_SHOW_HIFB, &bShow) < 0)
    {
        perror("FBIOPUT_SHOW_HIFB failed!\n");
        munmap(fbp, screensize);
        close(fbfd);
        return HI_FAILURE;
    }

    return 0;
}
#endif

#if PLATFORM_INGENIC_T40
int T40FB_Init(void)
{
	int ret = 0;
	int bpp;
	unsigned int num_buf;
	unsigned int vid_size;
	unsigned int fb_size;

    fbfd = open(FBDEV_PATH, O_RDWR);
    if(fbfd < 0)
    {
        perror("hifbfd open failed!\n");
        return -1;
    }

	/* get framebuffer's var_info */
	if ((ret = ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) < 0) {
		perror("FBIOGET_VSCREENINFO failed");
		goto err_getinfo;
	}

	/* get framebuffer's fix_info */
	if ((ret = ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) < 0) {
		perror("FBIOGET_FSCREENINFO failed");
		goto err_getinfo;
	}
	
	vinfo.width = vinfo.xres;
	vinfo.height = vinfo.yres;
	bpp = vinfo.bits_per_pixel >> 3;
	
	/* format rgb888 use 4 word ; format nv12/nv21 user 2 word */
	fb_size = vinfo.xres * vinfo.yres * bpp;
	num_buf = vinfo.yres_virtual / vinfo.yres;
	vid_size = fb_size*num_buf;
	
	//screensize = vid_size;
    // Figure out the size of the screen in bytes
    screensize =  finfo.smem_len; //finfo.line_length * vinfo.yres;    
	half_screensize = screensize/2;

	LV_LOG_INFO("fb_size:%d num_buf:%d vid_size:%d screensize:%d bpp:%d !!!", fb_size, num_buf, vid_size, screensize, bpp);
	LV_LOG_INFO("vinfo.xres:%d vinfo.yres:%d vinfo.yres_virtual:%d !!!", vinfo.xres, vinfo.yres, vinfo.yres_virtual);

    fbp = mmap(0, screensize, PROT_READ|PROT_WRITE, MAP_SHARED, fbfd, 0);
	if(fbp == 0) {
		perror("Map failed");
		ret = -1;
		goto err_getinfo;
	}
	
	fbpbase = fbp;
	
	return 0;
err_getinfo:
	close(fbfd);
	return ret;
}
#endif


 
#if 1
void fbdev_init(void)
{
#if PLATFORM_HISILICON
	HIFB_Init();
#elif PLATFORM_INGENIC_T40
	T40FB_Init();
#else
    // Open the file for reading and writing
    fbfd = open(FBDEV_PATH, O_RDWR);
    if(fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        return;
    }
    LV_LOG_INFO("The framebuffer device was opened successfully");

    // Make sure that the display is on.
    if (ioctl(fbfd, FBIOBLANK, FB_BLANK_UNBLANK) != 0) {
        perror("ioctl(FBIOBLANK)");
        return;
    }

#if USE_BSD_FBDEV
    struct fbtype fb;
    unsigned line_length;

    //Get fb type
    if (ioctl(fbfd, FBIOGTYPE, &fb) != 0) {
        perror("ioctl(FBIOGTYPE)");
        return;
    }

    //Get screen width
    if (ioctl(fbfd, FBIO_GETLINEWIDTH, &line_length) != 0) {
        perror("ioctl(FBIO_GETLINEWIDTH)");
        return;
    }

    vinfo.xres = (unsigned) fb.fb_width;
    vinfo.yres = (unsigned) fb.fb_height;
    vinfo.bits_per_pixel = fb.fb_depth;
    vinfo.xoffset = 0;
    vinfo.yoffset = 0;
    finfo.line_length = line_length;
    finfo.smem_len = finfo.line_length * vinfo.yres;
#else /* USE_BSD_FBDEV */

    // Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        return;
    }

    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        return;
    }
#endif /* USE_BSD_FBDEV */

    LV_LOG_INFO("%dx%d, %dbpp", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize =  finfo.smem_len; //finfo.line_length * vinfo.yres;   
    #ifdef LVGL_DOUBLE_BUF_EN 
    half_screensize = screensize/2;
    #endif 
    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if((intptr_t)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        return;
    }
    #ifdef LVGL_DOUBLE_BUF_EN
    fbpbase = fbp;
    #endif 
    memset(fbp, 0, screensize);

    LV_LOG_INFO("The framebuffer device was mapped to memory successfully");
#endif
}
#else
void fbdev_init(void)
{
    // Open the file for reading and writing
    fbfd = open(FBDEV_PATH, O_RDWR);
    if(fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        return;
    }
    printf("The framebuffer device was opened successfully.\n");

#if USE_BSD_FBDEV
    struct fbtype fb;
    unsigned line_length;

    //Get fb type
    if (ioctl(fbfd, FBIOGTYPE, &fb) != 0) {
        perror("ioctl(FBIOGTYPE)");
        return;
    }

    //Get screen width
    if (ioctl(fbfd, FBIO_GETLINEWIDTH, &line_length) != 0) {
        perror("ioctl(FBIO_GETLINEWIDTH)");
        return;
    }

    vinfo.xres = (unsigned) fb.fb_width;
    vinfo.yres = (unsigned) fb.fb_height;
    vinfo.bits_per_pixel = fb.fb_depth;
    vinfo.xoffset = 0;
    vinfo.yoffset = 0;
    finfo.line_length = line_length;
    finfo.smem_len = finfo.line_length * vinfo.yres;
#else /* USE_BSD_FBDEV */

    // Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        return;
    }

    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        return;
    }
#endif /* USE_BSD_FBDEV */

    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize =  vinfo.xres*vinfo.yres*2*2;; //finfo.line_length * vinfo.yres;    
    half_screensize = screensize/2;
    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if((intptr_t)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        return;
    }
    //memset(fbp, 0, screensize);
    fbpbase = fbp;
    //memset(fbp, 0, screensize);

    printf("The framebuffer device was mapped to memory successfully.\n");

}
#endif
void fbdev_exit(void)
{
#if PLATFORM_HISILICON
    /* unmap the physical memory */
    munmap(fbp, screensize);

	HI_BOOL bShow;
	bShow = HI_FALSE;
	if (ioctl(fbfd, FBIOPUT_SHOW_HIFB, &bShow) < 0)
	{
		perror("FBIOPUT_SHOW_HIFB failed!\n");
	}
#endif

#if PLATFORM_INGENIC_T40
	/* unmap the physical memory */
	munmap(fbp, screensize);
#endif 
    close(fbfd);
}

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixel to copy to the `area` part of the screen
 */
 void my_fbdev_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
        if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }
    if(toggle == 0)
	{
		toggle = 1;
		vinfo.yoffset = 0;
		fbp =fbpbase;
	}else{
		toggle = 0;
		vinfo.yoffset = vinfo.yres;	
		fbp =  fbpbase + half_screensize;
	}
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;
    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 ) + (y ) * finfo.line_length / 4;
            memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
            color_p += w;
        }
    }
    else if(vinfo.bits_per_pixel == 16) {
        uint16_t * fbp16 = (uint16_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 ) + (y ) * finfo.line_length / 2;
            memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
            color_p += w;
        }
    }else {
    }
	if (ioctl(fbfd, FBIOPAN_DISPLAY, &vinfo) < 0) {
		fprintf(stderr, "active fb swap failed\n");
	}
    lv_disp_flush_ready(drv);
}

void fbdev_2buf_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
	if(fbpbase == NULL ||
			area->x2 < 0 ||
			area->y2 < 0 ||
			area->x1 > (int32_t)vinfo.xres - 1 ||
			area->y1 > (int32_t)vinfo.yres - 1) {
		lv_disp_flush_ready(drv);
		return;
	}
			
	if(toggle == 0)
	{
		toggle = 1;
		vinfo.yoffset = 0;
		fbp = fbpbase;
	}
	else{
		toggle = 0;
		vinfo.yoffset = vinfo.yres; 
		fbp =  fbpbase + half_screensize;
	}

	/*Truncate the area to the screen*/
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
	int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;


	lv_coord_t w = (act_x2 - act_x1 + 1);
	long int location = 0;
	long int byte_location = 0;
	unsigned char bit_location = 0;

	/*32 or 24 bit per pixel*/
	if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
		uint32_t * fbp32 = (uint32_t *)fbp;
		int32_t y;
		for(y = act_y1; y <= act_y2; y++) {
			location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
			memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
			color_p += w;
		}
	}
	/*16 bit per pixel*/
	else if(vinfo.bits_per_pixel == 16) {
		uint16_t * fbp16 = (uint16_t *)fbp;
		int32_t y;
		for(y = act_y1; y <= act_y2; y++) {
			location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
			memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
			color_p += w;
		}
	}
	/*8 bit per pixel*/
	else if(vinfo.bits_per_pixel == 8) {
		uint8_t * fbp8 = (uint8_t *)fbp;
		int32_t y;
		for(y = act_y1; y <= act_y2; y++) {
			location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length;
			memcpy(&fbp8[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1));
			color_p += w;
		}
	}
	/*1 bit per pixel*/
	else if(vinfo.bits_per_pixel == 1) {
		uint8_t * fbp8 = (uint8_t *)fbp;
		int32_t x;
		int32_t y;
		for(y = act_y1; y <= act_y2; y++) {
			for(x = act_x1; x <= act_x2; x++) {
				location = (x + vinfo.xoffset) + (y + vinfo.yoffset) * vinfo.xres;
				byte_location = location / 8; /* find the byte we need to change */
				bit_location = location % 8; /* inside the byte found, find the bit we need to change */
				fbp8[byte_location] &= ~(((uint8_t)(1)) << bit_location);
				fbp8[byte_location] |= ((uint8_t)(color_p->full)) << bit_location;
				color_p++;
			}

			color_p += area->x2 - act_x2;
		}
	} else {
		/*Not supported bit per pixel*/
	}

	//May be some direct update command is required
	//ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));
	
	//vinfo.activate = FB_ACTIVATE_NOW;
	
	if (ioctl(fbfd, FBIOPAN_DISPLAY, &vinfo) < 0) {
		perror("active fb swap failed !\n");
	}

	lv_disp_flush_ready(drv);
}

void set_active_framebuffer(unsigned n)
{
    if(n > 1) return;
    vinfo.yres_virtual = vinfo.yres * 2;
    vinfo.yoffset = n * vinfo.yres;
    if(ioctl(fbfd, FBIOPUT_VSCREENINFO, &vinfo) < 0) {
        fprintf(stderr,"active fb swap failed!\n");
    }
}

void* get_framebuffer(unsigned int index)
{
	if(index>1){
		printf("index:%d invalid!\n", index);
	}
	return (void*) (((unsigned) fbp) + index*screensize/2);
}
void memcpy_venus(void*dst, void*src, int len);
#if 0
static unsigned int render_index=0;
void* get_current_framebuffer()
{
	int index = !(render_index&1);
	if(index>1){
		printf("index:%d invalid!\n", index);
	}
	return (void*) (((unsigned) fbp) + index*screensize/2);
}

void fbdev_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }
	uint32_t enter_time=0,finish_time=0;
	struct timespec tsp={0, 0};
	clock_gettime(CLOCK_MONOTONIC, &tsp);
	enter_time = tsp.tv_sec*1000ULL + tsp.tv_nsec/1000000ULL;

	//void*buf0 = get_framebuffer(0);
	//void*buf1 = get_framebuffer(1);
#if 0
	void*last_buf = get_framebuffer(render_index&1);
	void*cur_buf = get_framebuffer(!(render_index&1));
	memcpy_venus(cur_buf, last_buf, screensize/2);
#else
	//void*cur_buf = color_p;
	void*cur_buf = get_framebuffer(0);
#endif

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;


    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;

    /*32 or 24 bit per pixel*/
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)cur_buf;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
            memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
            color_p += w;
        }
    }
    /*16 bit per pixel*/
    else if(vinfo.bits_per_pixel == 16) {
        uint16_t * fbp16 = (uint16_t *)cur_buf;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
            memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
            color_p += w;
        }
    }
    /*8 bit per pixel*/
    else if(vinfo.bits_per_pixel == 8) {
        uint8_t * fbp8 = (uint8_t *)cur_buf;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length;
            memcpy(&fbp8[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1));
            color_p += w;
        }
    }
    /*1 bit per pixel*/
    else if(vinfo.bits_per_pixel == 1) {
        uint8_t * fbp8 = (uint8_t *)cur_buf;
        int32_t x;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            for(x = act_x1; x <= act_x2; x++) {
                location = (x + vinfo.xoffset) + (y + vinfo.yoffset) * vinfo.xres;
                byte_location = location / 8; /* find the byte we need to change */
                bit_location = location % 8; /* inside the byte found, find the bit we need to change */
                fbp8[byte_location] &= ~(((uint8_t)(1)) << bit_location);
                fbp8[byte_location] |= ((uint8_t)(color_p->full)) << bit_location;
                color_p++;
            }

            color_p += area->x2 - act_x2;
        }
    } else {
        /*Not supported bit per pixel*/
    }

    //May be some direct update command is required
    //ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));
    //set_active_framebuffer(color_p!=fbp);
    //set_active_framebuffer(++render_index&1);
    //set_active_framebuffer(0);

    lv_disp_flush_ready(drv);
	clock_gettime(CLOCK_MONOTONIC, &tsp);
	finish_time = tsp.tv_sec*1000ULL + tsp.tv_nsec/1000000ULL;
	printf("diff time:%u\n", finish_time-enter_time);
}
#else
#ifdef LVGL_DOUBLE_BUF_EN

char gb_lvgl_db_toggle = 0;
void fbdev_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }

	if(gb_lvgl_db_toggle == 0)
	{
		gb_lvgl_db_toggle = 1;

		vinfo.yoffset = 0;
		fbp =fbpbase;
	}
	else
	{
		gb_lvgl_db_toggle = 0;
		vinfo.yoffset = vinfo.yres;	
		fbp =  fbpbase + half_screensize;
		vinfo.yoffset = 0;
		fbp =fbpbase;
	
	}	
    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;


    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;
    __DBG_PRINTF("vinfo.bits_per_pixel=%d",vinfo.bits_per_pixel);
    /*32 or 24 bit per pixel*/
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
            memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
            color_p += w;
        }
    }
    /*16 bit per pixel*/
    else if(vinfo.bits_per_pixel == 16) {
        uint16_t * fbp16 = (uint16_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
            memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
            color_p += w;
        }
    }
    /*8 bit per pixel*/
    else if(vinfo.bits_per_pixel == 8) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length;
            memcpy(&fbp8[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1));
            color_p += w;
        }
    }
    /*1 bit per pixel*/
    else if(vinfo.bits_per_pixel == 1) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t x;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            for(x = act_x1; x <= act_x2; x++) {
                location = (x + vinfo.xoffset) + (y + vinfo.yoffset) * vinfo.xres;
                byte_location = location / 8; /* find the byte we need to change */
                bit_location = location % 8; /* inside the byte found, find the bit we need to change */
                fbp8[byte_location] &= ~(((uint8_t)(1)) << bit_location);
                fbp8[byte_location] |= ((uint8_t)(color_p->full)) << bit_location;
                color_p++;
            }

            color_p += area->x2 - act_x2;
        }
    } else {
        /*Not supported bit per pixel*/
    }

    //May be some direct update command is required
    //ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));
    // 4. 关键：通知硬件切换显示缓冲区（之前缺失，导致黑屏核心原因）
    //if(ioctl(fbfd, FBIOPUT_VSCREENINFO, &vinfo) == -1) {
        //perror("ioctl(FBIOSET_VSCREENINFO) failed (switch buffer)");
    //}

    // 5. 可选：等待垂直同步（VSYNC），避免画面撕裂（硬件支持则启用）
    //if(ioctl(fbfd, FBIO_WAITFORVSYNC, 0) == -1) {
        //LV_LOG_WARN("FBIO_WAITFORVSYNC not supported, may have screen tearing");
    //}
    lv_disp_flush_ready(drv);
}

#else
//static void my_disp_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
//{
    // 1. 将缓冲区数据（color_p指向当前需要刷新的缓冲区）发送到屏幕
    //    例如：通过SPI/I2C发送到屏幕，或通过DMA传输到LCD控制器
    //display_send_data(area->x1, area->y1, area->x2, area->y2, color_p);

    // 2. 传输完成后，通知LVGL缓冲区已释放，可用于下一帧绘制
    //lv_disp_flush_ready(drv);
//}

void fbdev_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;


    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;

    /*32 or 24 bit per pixel*/
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
            memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
            color_p += w;
        }
    }
    /*16 bit per pixel*/
    else if(vinfo.bits_per_pixel == 16) {
        uint16_t * fbp16 = (uint16_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
            memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
            color_p += w;
        }
    }
    /*8 bit per pixel*/
    else if(vinfo.bits_per_pixel == 8) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length;
            memcpy(&fbp8[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1));
            color_p += w;
        }
    }
    /*1 bit per pixel*/
    else if(vinfo.bits_per_pixel == 1) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t x;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            for(x = act_x1; x <= act_x2; x++) {
                location = (x + vinfo.xoffset) + (y + vinfo.yoffset) * vinfo.xres;
                byte_location = location / 8; /* find the byte we need to change */
                bit_location = location % 8; /* inside the byte found, find the bit we need to change */
                fbp8[byte_location] &= ~(((uint8_t)(1)) << bit_location);
                fbp8[byte_location] |= ((uint8_t)(color_p->full)) << bit_location;
                color_p++;
            }

            color_p += area->x2 - act_x2;
        }
    } else {
        /*Not supported bit per pixel*/
    }

    //May be some direct update command is required
    //ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));

    lv_disp_flush_ready(drv);
}
#endif
#endif
#ifdef LCD_DT20QV063

#else
#if 1//close by asame
void rotate_resize_buffer(void*dst, int dst_size,
	void*src, int src_size, int agl);
static void*frame_cache=NULL;
void fbdev_flush_venus(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
//printf("%s:%d\n",__func__,__LINE__);

//return;
#if 1
	int xres = 960;
	int yres = 640;
    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)xres - 1 ||
            area->y1 > (int32_t)yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }
//printf("%s:%d\n",__func__,__LINE__);
	//struct timespec tsp={0, 0};
	//clock_gettime(CLOCK_MONOTONIC, &tsp);
	//uint32_t start = tsp.tv_sec*1000ULL + tsp.tv_nsec/1000000ULL;

	
//printf("%s:%d\n",__func__,__LINE__);
    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)xres - 1 ? (int32_t)xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)yres - 1 ? (int32_t)yres - 1 : area->y2;
	//lv_color32_t cc = color_p[960*320+400];
	///printf("show [%d,%d][%d,%d] xoffset:%u yoffset:%u line_length:%u\n", act_x1, act_y1, act_x2, act_y2,
	//	vinfo.xoffset, vinfo.yoffset, finfo.line_length);
//return;
#if 1
	/*if(toggle == 0)
	{
		toggle = 1;
		vinfo.yoffset = 0;
		fbp = fbpbase;
	}
	else{
		toggle = 0;
		vinfo.yoffset = vinfo.yres; 
		fbp =  fbpbase + half_screensize;
	}*/
	int index = (++toggle&1);
	vinfo.yoffset = index*vinfo.yres; 
	fbp =  fbpbase + index*half_screensize;
	printf("index:%d yoffset:%d half_screensize:%d\n", index, vinfo.yoffset, half_screensize);
	
	int dst_size=960*640*4;
	int src_size=960*640*4;
	rotate_resize_buffer(fbp, dst_size, color_p, src_size, 270);
	//memcpy(((char*)fbp)+src_size, color_p, src_size);
	//set_active_framebuffer(0);

	vinfo.activate = FB_ACTIVATE_NOW;
	if (ioctl(fbfd, FBIOPAN_DISPLAY, &vinfo) < 0) {
		perror("active fb swap failed !\n");
	}
	
	keepon_update_video();
	
	//clock_gettime(CLOCK_MONOTONIC, &tsp);
	//uint32_t end = tsp.tv_sec*1000ULL + tsp.tv_nsec/1000000ULL;
	//printf("diff:%u\n", end - start);
#else
    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;

	if(frame_cache==NULL){
		frame_cache=malloc(960*640*4);
	}

    /*32 or 24 bit per pixel*/
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)frame_cache;
        int32_t y;
		if(w == xres){
			memcpy_venus(&fbp32[act_y1*xres], (uint32_t *)color_p, xres*(act_y2 - act_y1+1) * 4);
			//printf("copy len:%d\n", xres*(act_y2 - act_y1) * 4);
		}else{
	        for(y = act_y1; y <= act_y2; y++) {
	            //location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
	            //memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
	            memcpy(&fbp32[y*960 + act_x1], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
				//printf("copy len:%d\n", (act_x2 - act_x1 + 1) * 4);
	            color_p += w;
	        }
		}
    }
	
	int dst_size=960*640*4;
	int src_size=960*640*4;
	rotate_resize_buffer(fbp, dst_size, frame_cache, src_size, 270);
	//memcpy(((char*)fbp)+src_size, color_p, src_size);
	//set_active_framebuffer(0);
	
	//clock_gettime(CLOCK_MONOTONIC, &tsp);
	//uint32_t end = tsp.tv_sec*1000ULL + tsp.tv_nsec/1000000ULL;
	//printf("diff:%u\n", end - start);
	
    //May be some direct update command is required
    //ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));
#endif
#endif
    lv_disp_flush_ready(drv);
}
#endif
#endif


void fbdev_get_sizes(uint32_t *width, uint32_t *height) {
    if (width)
        *width = vinfo.xres;

    if (height)
        *height = vinfo.yres;
}

void fbdev_set_offset(uint32_t xoffset, uint32_t yoffset) {
    vinfo.xoffset = xoffset;
    vinfo.yoffset = yoffset;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
