
#include "camera.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))


static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do 
    {
        r = ioctl(fh, request, arg);
    } 
    while (-1 == r && EINTR == errno);

    return r;
}

static int read_frame(struct camera * ctx)
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(ctx->fd, VIDIOC_DQBUF, &buf)) 
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

    assert(buf.index < ctx->n_buffers);
    ctx->frame_count++;

    if (ctx->config.frame_cb)
    {
        ctx->config.frame_cb(ctx, ctx->buffers[buf.index].start, buf.bytesused);
    }
    
    if (-1 == xioctl(ctx->fd, VIDIOC_QBUF, &buf))
    {
        errno_exit("VIDIOC_QBUF");
    }
  

    return 1;
}

static void stop_capturing(struct camera * ctx)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(ctx->fd, VIDIOC_STREAMOFF, &type))
    {
            errno_exit("VIDIOC_STREAMOFF");
    }
    
}

static void start_capturing(struct camera * ctx)
{
    unsigned int i;
    enum v4l2_buf_type type;


    for (i = 0; i < ctx->n_buffers; ++i) 
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(ctx->fd, VIDIOC_QBUF, &buf)) 
        {
        	errno_exit("VIDIOC_QBUF");
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(ctx->fd, VIDIOC_STREAMON, &type)) 
    {
        errno_exit("VIDIOC_STREAMON");
    }
}

static void uninit_device(struct camera * ctx)
{
    unsigned int i;

    for (i = 0; i < ctx->n_buffers; ++i)
    {
        if (-1 == munmap(ctx->buffers[i].start, ctx->buffers[i].length))
        {
	        errno_exit("munmap");
	    }
    }


    free(ctx->buffers);
}


static void init_mmap(struct camera * ctx)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(ctx->fd, VIDIOC_REQBUFS, &req)) 
    {
        if (EINVAL == errno) 
        {
                fprintf(stderr, "%s does not support "
                         "memory mapping\n", ctx->dev);
                exit(EXIT_FAILURE);
        } 
        else 
        {
                errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) 
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n",
                 ctx->dev);
        exit(EXIT_FAILURE);
    }

    ctx->buffers = calloc(req.count, sizeof(*(ctx->buffers)));

    if (!ctx->buffers) 
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (ctx->n_buffers = 0; ctx->n_buffers < req.count; ++ctx->n_buffers) 
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = ctx->n_buffers;

        if (-1 == xioctl(ctx->fd, VIDIOC_QUERYBUF, &buf))
                errno_exit("VIDIOC_QUERYBUF");

        ctx->buffers[ctx->n_buffers].length = buf.length;
        ctx->buffers[ctx->n_buffers].start =
                mmap(NULL /* start anywhere */,
                      buf.length,
                      PROT_READ | PROT_WRITE /* required */,
                      MAP_SHARED /* recommended */,
                      ctx->fd, buf.m.offset);

        if (MAP_FAILED == ctx->buffers[ctx->n_buffers].start)
                errno_exit("mmap");
    }
}

static void init_device(struct camera * ctx)
{
    struct v4l2_format fmt;
    struct v4l2_streamparm sparm;
    unsigned int min;

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = ctx->config.width;
    fmt.fmt.pix.height      = ctx->config.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
           
    if (-1 == xioctl(ctx->fd, VIDIOC_S_FMT, &fmt))
    {
        errno_exit("VIDIOC_S_FMT");
    }

    CLEAR(sparm);

    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sparm.parm.capture.timeperframe.numerator = 1;
    sparm.parm.capture.timeperframe.denominator = ctx->config.fps;

    if (-1 == xioctl(ctx->fd, VIDIOC_S_PARM, &sparm)) 
    {
        errno_exit("VIDIOC_S_PARM");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

   	init_mmap(ctx);
}

static void close_device(struct camera * ctx)
{
    if (-1 == close(ctx->fd))
            errno_exit("close");

    ctx->fd = -1;
}

static void open_device(struct camera * ctx)
{
    struct stat st;

    if (-1 == stat(ctx->dev, &st)) {
            fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                     ctx->dev, errno, strerror(errno));
            exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
            fprintf(stderr, "%s is no device\n", ctx->dev);
            exit(EXIT_FAILURE);
    }

    ctx->fd = open(ctx->dev, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == ctx->fd) {
            fprintf(stderr, "Cannot open '%s': %d, %s\n",
                     ctx->dev, errno, strerror(errno));
            exit(EXIT_FAILURE);
    }
}

void cam_init(struct camera * ctx)
{
	if (!ctx->dev)
	{
		ctx->dev = "/dev/video0";
	}
	ctx->fd = -1;

	open_device(ctx);
    init_device(ctx);
}

void cam_uninit(struct camera * ctx)
{
	uninit_device(ctx);
    close_device(ctx);
}

void cam_start_capturing(struct camera * ctx)
{
    
    
	start_capturing(ctx);
}

void cam_stop_capturing(struct camera * ctx)
{
	stop_capturing(ctx);
    
}

void cam_end_loop(struct camera * ctx)
{
    ctx->run = 0;
}

void cam_loop(struct camera * ctx)
{
    unsigned int count;

    ctx->run = 1;
    ctx->frame_count = 0;
    gettimeofday(&(ctx->start), NULL);

    while (ctx->run) 
    {
        for (;;) 
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(ctx->fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(ctx->fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r) 
            {
                if (EINTR == errno)
                        continue;
                errno_exit("select");
            }

            if (0 == r) 
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame(ctx))
            {
                break;
            }
            /* EAGAIN - continue select loop. */
        }
    }

    gettimeofday(&(ctx->end), NULL);
}

double cam_get_measured_fps(struct camera * ctx)
{
    double  elapsed = (ctx->end.tv_sec - ctx->start.tv_sec) * 1000.0;
    elapsed += (ctx->end.tv_usec - ctx->start.tv_usec) / 1000.0;
    elapsed /= 1000.0;

    return ctx->frame_count / elapsed;
}
