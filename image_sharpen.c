/*
* Author - Gitanjali Suresh
* Dated - 26 July 2020
* About - This code generates real-time images obtained from a USB based webcam for creating a Time Lapse Video
* Underlying references - The base code for obtaining real-time images is obtained from Dr. Sam Siewert's webpage in addition to referring to the 
*			  sharpening image transformation.
* Links - For obtaining real-time images V4L2 startup code - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/simple-capture/
*       - For sharpening the image - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/sharpen-psf/sharpen.c
*/

/* Header File declarations */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <time.h>
#include <sys/utsname.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/sysinfo.h>
#include <syslog.h>

/* Macro declarations */
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"
#define IMG_HEIGHT (480)
#define IMG_WIDTH (640)
#define K 4.0

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define MSEC_PER_SEC	(1000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (3+1)

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
sem_t semS1, semS2, semS3;
struct timeval start_time_val;
bool frame_captured = false;

/* Global variable declarations */
// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;
struct utsname hostname;

double *seq_exec_time;	    //Sequencer execution time
double *seq_start_time;		//Sequencer start time
double *seq_stop_time;		//Sequencer stop time 

double *fc_exec_time;	    //Frame Capture execution time
double *fc_start_time;		//Frame Capture start time
double *fc_stop_time;		//Frame Capture stop time 

double *pd_exec_time;	    //PPM Dump execution time
double *pd_start_time;		//PPM Dump start time
double *pd_stop_time;		//PPM Dump stop time 

double *pi_exec_time;	    //Process Image execution time
double *pi_start_time;		//Process Image start time
double *pi_stop_time;		//Process Image stop time 

//struct timespec speed_1fps = {1,0};
struct timespec speed_1fps = {0,100000000};

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
struct buffer           process_image_buffer;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 601;

typedef double FLOAT;
typedef unsigned char UINT8;

char ppm_header[69]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n#Host Info:raspberrypi\n";
char ppm_dumpname[]="test00000000.ppm";

unsigned int framecnt=0;
unsigned char bigbuffer[(1280*960)];
unsigned char image_frame[181][(1280*960)];
int global_size;
struct timespec global_frame_time;

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

struct buffer 
{
        void   *start;
        size_t  length;
};

void sequencer_parameters(void);
void frame_capture_parameters(void);
void ppm_dump_parameters(void);
void process_image_parameters(void);

void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

/* Function to exit with an error */
static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

/* Function to manipulate the ioctl files */
static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

/* Function to write the pixel values to generate an image of form .ppm */
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    strncat(&ppm_header[48], "#Host Info:", 11);
    strncat(&ppm_header[59], hostname.nodename, strlen(hostname.nodename));
    strncat(&ppm_header[59+strlen(hostname.nodename)], "\n", 1);
    /*strncat(&ppm_header[48], "Host Info: ", 11);
    strncat(&ppm_header[59], hostname.sysname, strlen(hostname.sysname));
    strncat(&ppm_header[59+strlen(hostname.sysname)], " Nodename: ", 11);
    strncat(&ppm_header[59+strlen(hostname.sysname)+11], hostname.nodename, strlen(hostname.nodename));
    strncat(&ppm_header[59+strlen(hostname.sysname)+11+strlen(hostname.nodename)], "\n", 1);*/
    written=write(dumpfd, ppm_header, sizeof(ppm_header));

    total=0;
    do
    {
        written=write(dumpfd, p, size);
        total+=written;
        syslog(LOG_CRIT,"Inside do while");
    } while(total < size);
    
    close(dumpfd);
    frame_captured = false;   
}

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


/* Function to carry out various image processing operations */
static void process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    printf("-------------frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

//#if defined(COLOR_CONVERT)
        printf("Dump YUYV converted to RGB size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }
        //sharpen(bigbuffer, ((size*6)/4));
        //dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
        global_size = (size*6)/4;
        global_frame_time = frame_time;
        frame_captured = true;
        syslog(LOG_CRIT, "In process image Frame captured = %d",frame_captured);
        for(i = 0;i < (1280*960);i++)
            image_frame[(framecnt % 60)][i] = bigbuffer[i];
        syslog(LOG_INFO,"Image Frame No. (framecnt % 60) = %d\n",(framecnt % 60));
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("Dump RGB as-is size %d\n", size);
        dump_ppm(p, size, framecnt, &frame_time);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}

/* Function to read the frames of an image and process them */
static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {

        case IO_METHOD_READ:
            //printf("Line 461 - IO_METHOD_READ\n");
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
        //printf("Line 484 - IO_METHOD_MMAP\n");
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                           non-fatal errors too.
                         */
                        return 0;


                    default:
                        printf("mmap failure\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);
            
            process_image_buffer.start = (void *)buffers[buf.index].start;
            process_image_buffer.length = buf.bytesused;
            
            //process_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
        //printf("Line 519 - IO_METHOD_USERPTR\n");
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }
    return 1;
}

/* Main function to perform various computations */
static void mainloop(void)
{
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 4;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno);
                //continue;
            errno_exit("select");
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        if (read_frame() && (!abortS1))
        {
            if(nanosleep(&read_delay, &time_error) != 0)
                perror("nanosleep");
            else;
        }
}
 
/* Function to stop capturing the images */
static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

/* Function to start capturing the images */
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) 
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) 
                {
                        //printf("allocated buffer %d\n", i);
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

/* Function to un-initialize the capturing device */
static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

/* Function to initialize read */
static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

/* Function to initialize memory map */
static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

/* Function to initialize the capturing device */
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}

/* Function to close the capturing device */
static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

/* Function to open the capturing device */
static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

void *Sequencer(void *threadp)
{
    //struct timeval current_time_val;
    struct timespec current_time_val;
    //struct timespec delay_time = {0,33333333}; // delay for 33.33 msec, 30 Hz
    //struct timespec delay_time = {1,0}; // delay for 1s, 1 Hz
    struct timespec delay_time = {0,100000000}; // delay for 100ms, 10 Hz
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
	double start_time_sec;	
	double stop_time_sec;	
    
	if((seq_exec_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Sequencer Execution Time:Malloc Failed\n");
	}
	if((seq_start_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Sequencer Start Time:Malloc Failed\n");
	}
	if((seq_stop_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Sequencer Stop Time:Malloc Failed\n");
	} 

    do
    {
        delay_cnt=0; residual=0.0;
        
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        
        printf("--------Sequence Count = %llu\n",seqCnt);
        
        /* Calculate Start time */
        clock_gettime(CLOCK_REALTIME,&current_time_val);

        /* Store start time in seconds */
        start_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));


        if(((seq_start_time + seqCnt) >= seq_start_time) && ((seq_start_time + seqCnt) <= (seq_start_time + (frame_count))))
        {
            //syslog(LOG_INFO,"[SEQUENCER]: Valid Address");
            //printf("Valid Data Address\n");
            *(seq_start_time + seqCnt) = start_time_sec; 
        }
        
        //gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d\n", seqCnt, (int)(start_time_sec));

        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate
        
        if((seqCnt % 1) == 0) sem_post(&semS1); 
        
        if((seqCnt % 1) == 0) sem_post(&semS3);       
      
        clock_gettime(CLOCK_REALTIME,&current_time_val);
        
        stop_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));
			
        *(seq_stop_time + seqCnt) = stop_time_sec;
        seqCnt++;
        *(seq_exec_time + seqCnt) = stop_time_sec - start_time_sec;
        syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d\n", (int)(stop_time_sec));

    } while(!abortTest && (seqCnt < (frame_count)));

    syslog(LOG_CRIT, "Outside sequencer while");
    sem_post(&semS1); 
    syslog(LOG_CRIT, "Released S1 sem");
    sem_post(&semS3); 
    syslog(LOG_CRIT, "Released S3 sem");
    abortS1=TRUE; 
    syslog(LOG_CRIT, "Aborted S1");
    abortS3=TRUE; 
    syslog(LOG_CRIT, "Aborted S3");
    syslog(LOG_CRIT, "Exiting sequencer...");
    pthread_exit((void *)0);
}

void *Service_1(void *threadp)
{
    //struct timeval current_time_val;
    struct timespec current_time_val;
    double current_time;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    double start_time_sec;	//To store start time in seconds
	double stop_time_sec;	//To store end time in seconds
    
	if((fc_exec_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Frame Capture Execution Time:Malloc Failed\n");
	}
	if((fc_start_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Frame Capture Start Time:Malloc Failed\n");
	}
	if((fc_stop_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Frame Capture Stop Time:Malloc Failed\n");
	}
    
    while(!abortS1)
    {
        sem_wait(&semS1);
        
        clock_gettime(CLOCK_REALTIME,&current_time_val);
        /* Store start time in seconds */
        start_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));
        *(fc_start_time + S1Cnt) = start_time_sec;
        syslog(LOG_CRIT, "Frame Capture cycle %llu @ sec=%d\n", S1Cnt, (int)(start_time_sec));
        	        
        mainloop();        

        clock_gettime(CLOCK_REALTIME,&current_time_val);
        stop_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));			
        *(fc_stop_time + S1Cnt) = stop_time_sec;
        *(fc_exec_time + S1Cnt) = stop_time_sec - start_time_sec;
        S1Cnt++;
        syslog(LOG_CRIT, "Frame Capture release all sub-services @ sec=%d\n", (int)(stop_time_sec));                
    }
    syslog(LOG_CRIT, "Exiting thread 1");
    pthread_exit((void *)0);
}


void *Service_3(void *threadp)
{
    //struct timeval current_time_val;
    struct timespec current_time_val;
    double current_time;
    unsigned long long S3Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    double start_time_sec;	
	double stop_time_sec;	
    
	if((pi_exec_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Process Image Execution Time:Malloc Failed\n");
	}
	if((pi_start_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Process Image Start Time:Malloc Failed\n");
	}
	if((pi_stop_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("Process Image Stop Time:Malloc Failed\n");
	}

    while(!abortS3)
    {
        sem_wait(&semS3);
        
        clock_gettime(CLOCK_REALTIME,&current_time_val);
        /* Store start time in seconds */
        start_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));
        *(pi_start_time + S3Cnt) = start_time_sec;
        syslog(LOG_CRIT, "Process Image cycle %llu @ sec=%d\n", S3Cnt, (int)(start_time_sec));     
                	        
        process_image(process_image_buffer.start,process_image_buffer.length);        

        clock_gettime(CLOCK_REALTIME,&current_time_val);
        stop_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));			
        *(pi_stop_time + S3Cnt) = stop_time_sec;
        S3Cnt++;
        *(pi_exec_time + S3Cnt) = stop_time_sec - start_time_sec;
        syslog(LOG_CRIT, "Process Image release all sub-services @ sec=%d\n", (int)(stop_time_sec));
        sem_post(&semS2);                
    }
    syslog(LOG_CRIT, "------------Exiting thread 3-------------");
    abortS2=TRUE;
    pthread_exit((void *)0);
}

/* Service to dump the ppm images */
void *Service_2(void *threadp)
{
    //struct timeval current_time_val;
    struct timespec current_time_val;
    double current_time;
    unsigned long long S2Cnt=1;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    double start_time_sec;	
	double stop_time_sec;	
    
	if((pd_exec_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("PPM Dump Execution Time:Malloc Failed\n");
	}
	if((pd_start_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("PPM Dump Start Time:Malloc Failed\n");
	}
	if((pd_stop_time = (double *)malloc((frame_count+10)*sizeof(double))) == NULL)
	{
		printf("PPM Dump Stop Time:Malloc Failed\n");
	}

    while(!abortS2)
    {
        sem_wait(&semS2);
        
        clock_gettime(CLOCK_REALTIME,&current_time_val);
        /* Store start time in seconds */
        start_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));
        *(pd_start_time + S2Cnt) = start_time_sec;
        
        dump_ppm(image_frame[((S2Cnt) % 60)], global_size, (S2Cnt), &global_frame_time);
        syslog(LOG_CRIT, "Dumped image of count = %d", ((S2Cnt) % 60)); 
                   
        clock_gettime(CLOCK_REALTIME,&current_time_val);
        stop_time_sec = ((double)current_time_val.tv_sec + (double)((current_time_val.tv_nsec)/(double)1000000000));			
        //printf("---------------------------------------------Difference = %lf\n",(stop_time_sec-start_time_sec));			
        *(pd_stop_time + S2Cnt) = stop_time_sec;
        S2Cnt++;
        syslog(LOG_CRIT, "PPM Dump release all sub-services @ sec=%d\n", (int)(stop_time_sec));  
        *(pd_exec_time + S2Cnt) = stop_time_sec - start_time_sec;    
    }
    syslog(LOG_CRIT, "----------------------Exiting thread 2--------------------");

    pthread_exit((void *)0);
}

/* Function to stabilize the device */
void stabilize_device(void)
{
    struct timespec read_delay;
    struct timespec time_error;
    struct v4l2_buffer buf;
    unsigned int count = 0;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

    while(count != 20)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 4;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno);
                //continue;
            errno_exit("select");
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        {
            switch (errno)
            {
                case EAGAIN:
                    syslog(LOG_ERR, "Stabilizing Device: EAGAIN");

                case EIO:
                    /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                       non-fatal errors too.
                     */
                    syslog(LOG_ERR, "Stabilizing Device: EIO");

                default:
                    printf("mmap failure\n");
                    errno_exit("VIDIOC_DQBUF");
            }
        }

        assert(buf.index < n_buffers);
        
        //process_image(buffers[buf.index].start, buf.bytesused);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        count++;
    }
}

/* Main function */
int main(int argc, char **argv)
{
    struct timeval current_time_val;
    int i, rc, scope;
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;
    
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";
        
    uname(&hostname);
    printf("***************************Starting Time Lapse Project***************************\n");
    printf("Host Platform Details:- \n");
    printf("------------------------\n");
    printf("Version - %s\n",hostname.version);
    printf("Nodename - %s\n",hostname.nodename);
    printf("Machine name - %s\n",hostname.machine);
    printf("System name - %s\n",hostname.sysname);
    
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    CPU_ZERO(&allcpuset);

    for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

    // initialize the sequencer semaphores
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    
    print_scheduler();

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    //printf("rt_max_prio=%d\n", rt_max_prio);
    //printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < NUM_THREADS; i++)
    {
      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }

    open_device();
    init_device();
    start_capturing();
    printf("\nSetting up the device...\n");
    stabilize_device();
    
    usleep(10000);

    // Create Service threads which will block awaiting release for:

    // Servcie_1 = RT_MAX-1	@ 1 Hz
    // For capturing frames
    CPU_ZERO(&threadcpu);
    CPU_SET(2, &threadcpu);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[1], sizeof(cpu_set_t), &threadcpu);
    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");
        
    // Service_3 = RT_MAX-2	@ 1 Hz
    // Process Image
    CPU_ZERO(&threadcpu);
    CPU_SET(2, &threadcpu);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[3], sizeof(cpu_set_t), &threadcpu);
    rt_param[3].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[2]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_3, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");        
        
    // Service_2 = RT_MAX-1	@ 1 Hz
    // Dump PPM
    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[2], sizeof(cpu_set_t), &threadcpu);
    rt_param[2].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");
        

    // Create Sequencer thread, which like a cyclic executive, is highest priority
    syslog(LOG_CRIT,"------------------------Starting Sequencer--------------------");
    threadParams[0].sequencePeriods=frame_count;

    // Sequencer = RT_MAX	@ 1 Hz
    //
    CPU_ZERO(&threadcpu);
    CPU_SET(2, &threadcpu);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[0], sizeof(cpu_set_t), &threadcpu);
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequencer service 0\n");

    for(i = 0;i < NUM_THREADS;i++)
       pthread_join(threads[i], NULL);

    printf("\nTEST COMPLETE\n");
        
    stop_capturing();
    uninit_device();
    close_device();
    
    sequencer_parameters();
    frame_capture_parameters();
    //ppm_dump_parameters();
    process_image_parameters();
    
    fprintf(stderr, "\n");
    //char str[] = "sh ./create_video.sh";
    //system(str);
    return 0;
}

void sequencer_parameters(void)
{
    double seq_wcet = 0;			 //Store worst case execution for Sequencer
    double seq_total_time = 0;		//Store total execution time
    double seq_jitter = 0;			//Store jitter for Sequencer
    double avg_jitter = 0;
    
	printf("--------------------------------SEQUENCER PARAMETER ANALYSIS--------------------------------\n");

	/* Calculate execution time, WCET and average execution time */
	for(int i=0;i < (frame_count) ;i++)
	{
		/* Calculate execution time of each iteration of Frame_Capture Thread (in msecs) */
		*(seq_exec_time + i) = (*(seq_exec_time + i))*MSEC_PER_SEC;

		if(seq_wcet < *(seq_exec_time + i))
		{
			seq_wcet = *(seq_exec_time + i);
		}

		seq_total_time += *(seq_exec_time + i);
	}
	for(int i=1; i < (frame_count) ;i++)
	{
        seq_jitter = (*(seq_start_time + i - 1) + speed_1fps.tv_sec) - (*(seq_start_time + i)) ;
        avg_jitter += seq_jitter;	
	}
    avg_jitter /= (frame_count);
	printf("WCET Sequencer = %lf msec\n",seq_wcet);
	printf("ACET Sequencer = %lf msec\n",seq_total_time/(frame_count));
    printf("Average Jitter = %lf msec\n",avg_jitter);
}

void frame_capture_parameters(void)
{
    double fc_wcet = 0;			    //Store worst case execution for frame capture
    double fc_total_time = 0;		//Store total execution time
    double fc_jitter = 0;			//Store jitter for frame capture
    double avg_jitter = 0;
    
	printf("--------------------------------FRAME CAPTURE PARAMETER ANALYSIS--------------------------------\n");

	/* Calculate execution time, WCET and average execution time */
	for(int i=0;i < (frame_count) ;i++)
	{
		/* Calculate execution time of each iteration of Frame_Capture Thread (in msecs) */
		*(fc_exec_time + i) = (*(fc_exec_time + i))*MSEC_PER_SEC;

		if(fc_wcet < *(fc_exec_time + i))
		{
			fc_wcet = *(fc_exec_time + i);
		}

		fc_total_time += *(fc_exec_time + i);
	}
	for(int i=1; i < (frame_count) ;i++)
	{
        fc_jitter = (*(fc_start_time + i - 1) + speed_1fps.tv_sec) - (*(fc_start_time + i)) ;
        avg_jitter += fc_jitter;	
	}
    avg_jitter /= (frame_count);
	printf("WCET Frame Capture = %lf msec\n",fc_wcet);
	printf("ACET Frame Capture = %lf msec\n",fc_total_time/(frame_count));
    printf("Average Jitter = %lf msec\n",avg_jitter);
}

void ppm_dump_parameters(void)
{
    double pd_wcet = 0;			    //Store worst case execution for ppm dump
    double pd_total_time = 0;		//Store total execution time
    double pd_jitter = 0;			//Store jitter for ppm dump
    double avg_jitter = 0;
    
	printf("--------------------------------PPM DUMP PARAMETER ANALYSIS--------------------------------\n");

	/* Calculate execution time, WCET and average execution time */
	for(int i=0;i < (frame_count) ;i++)
	{
		/* Calculate execution time of each iteration of PPM_Dump Thread (in msecs) */
		*(pd_exec_time + i) = (*(pd_exec_time + i))*MSEC_PER_SEC;

		if(pd_wcet < *(pd_exec_time + i))
		{
			pd_wcet = *(pd_exec_time + i);
            syslog(LOG_CRIT,"pd_wcet = %lf for frame = %d",pd_wcet,i);
		}

		pd_total_time += *(pd_exec_time + i);
	}
	for(int i=2; i < (frame_count) ;i++)
	{
        pd_jitter = (*(pd_start_time + i - 1) + speed_1fps.tv_sec) - (*(pd_start_time + i)) ;
        avg_jitter += pd_jitter;	
	}
    avg_jitter /= (frame_count);
	printf("WCET PPM Dump = %lf msec\n",pd_wcet);
	printf("ACET PPM Dump = %lf msec\n",pd_total_time/(frame_count));
    printf("Average Jitter = %lf msec\n",avg_jitter);                                                         
}

void process_image_parameters(void)
{
    double pi_wcet = 0;			    //Store worst case execution for processing image
    double pi_total_time = 0;		//Store total execution time
    double pi_jitter = 0;			//Store jitter for processing image
    double avg_jitter = 0;
    
	printf("--------------------------------Process Image PARAMETER ANALYSIS--------------------------------\n");

	/* Calculate execution time, WCET and average execution time */
	for(int i=0;i < (frame_count) ;i++)
	{
		/* Calculate execution time of each iteration of Process_Image Thread (in msecs) */
		*(pi_exec_time + i) = (*(pi_exec_time + i))*MSEC_PER_SEC;

		if(pi_wcet < *(pi_exec_time + i))
		{
			pi_wcet = *(pi_exec_time + i);
		}

		pi_total_time += *(pi_exec_time + i);
	}
	for(int i=1; i< (frame_count) ;i++)
	{
        pi_jitter = (*(pi_start_time + i - 1) + speed_1fps.tv_sec) - (*(pi_start_time + i)) ;
        avg_jitter += pi_jitter;	
	}
    avg_jitter /= (frame_count);
	printf("WCET Process Image = %lf msec\n",pi_wcet);
	printf("ACET Process Image = %lf msec\n",pi_total_time/(frame_count));
    printf("Average Jitter = %lf msec\n",avg_jitter);
}

/* References 
* [1]	http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/simple-capture/
* [2]	http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/sharpen-psf/sharpen.c
*/
