#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#define SERVER_PORT         "24000"
#define SERVER_HOSTNAME     "10.42.0.71"


#define WIDTH 			    320
#define HEIGHT 			    240
#define CHANNELS 		    4
#define SIZE 			    (WIDTH * HEIGHT)
#define N_IMG_BYTES         (SIZE * CHANNELS)

#define UP_W                640
#define UP_H                480
#define UP_S                (UP_W * UP_H)

// Some kB extra space / padding for the receive buffer
#define PADDING             4096

#define SOCKET_ERROR        -1
#define BUFFER_SIZE         100
#define QUEUE_SIZE          5

#define B_LEN               76800

static XImage * image;
static Display *display;
static Visual *visual;
static Window window;

// Buffer for the image displayed on in the window
static unsigned char * img_buffer;//[N_IMG_BYTES];
static unsigned char * img_disp_buffer;
// Buffer used when reading the image over socket
static unsigned char * read_buffer;//[WIDTH * HEIGHT + PADDING];

static unsigned char * scaled_up_img;
static unsigned char * scaled_up_img_dbl;

static int get_pixel_offset(int pixel_pos)
{
	return pixel_pos * CHANNELS;
}

static void set_pix(int o, unsigned char r, unsigned char g, unsigned char b)
{
    scaled_up_img[o] = b;
    scaled_up_img[o+1] = g;
    scaled_up_img[o+2] = r;
    scaled_up_img[o+3] = 0;
}

static void set_pixel_value_at_off(int off, unsigned char r, unsigned char g, unsigned char b)
{
    // Assert pixel is inside image buffer
    assert((off + 3) < N_IMG_BYTES);

	img_buffer[off] = b;
    img_buffer[off+1] = g;
    img_buffer[off+2] = r;
    img_buffer[off+3] = 0;

}


static void copy_to_x_buffer(unsigned char * frame, unsigned int size)
{   
    // The received buffer is not a full frame
    if (size < SIZE)
    {
        return;
    }

    int i, p = 0;
    unsigned char v;
    unsigned char * ptr;
    unsigned int x = 0, y = 0, count = 0, error = 0;
    
    ptr = (unsigned char *) frame;

    for (i = 0; i < SIZE; i++)
    {
        v = frame[i];
        set_pixel_value_at_off(p, v, v, v);

        p += 4;
    }
}

static void draw_center_point(int x, int y, int rr, int gg, int bb)
{
    int r, c, o, o1;

    // Skip drawing the point if is so close to the border
    // that we cannot draw a 10x10 dot
    if (x > (WIDTH - 5) || x < 5 || y > (HEIGHT - 5) || y < 5)
    {   
        return;
    }

    // Draw the blue center point
    for (r = y - 5; r < y + 5; r++)
    {
        //o = r * WIDTH * 4;
        for (c = x - 5; c < x + 5; c++)
        {   
            // Calculate pixel offet
            o = r * WIDTH + c;
            assert(o < SIZE);

            o1 = get_pixel_offset(o);
            assert(o1 < N_IMG_BYTES);

            set_pixel_value_at_off(o1, rr, gg, bb);
        }
    }
}

static void draw_center_point2(int x, int y, int rr, int gg, int bb)
{
    int r, c, o, o1;

    // Skip drawing the point if is so close to the border
    // that we cannot draw a 10x10 dot
    if (x > (UP_W - 5) || x < 5 || y > (UP_H - 5) || y < 5)
    {   
        return;
    }

    // Draw the blue center point
    for (r = y - 5; r < y + 5; r++)
    {
        //o = r * WIDTH * 4;
        for (c = x - 5; c < x + 5; c++)
        {   
            // Calculate pixel offet
            o = r * UP_W + c;
            assert(o < UP_S);

            o1 = o * CHANNELS;
            set_pix(o1, rr, gg, bb);
        }
    }
}


static void draw_center_lines()
{
    int r, o, m;

    // Draw the red center line vertical
    m = WIDTH / 2;
    for (r = 0; r < 240; r++)
    {
        o = ((r * WIDTH) + m);
        set_pixel_value_at_off(get_pixel_offset(o), 255, 0, 0);
    }

    // Draw the red center line horizontal
    m = HEIGHT / 2;
    for (r = 0; r < 320; r++)
    {
        o = m * WIDTH + r;
        set_pixel_value_at_off(get_pixel_offset(o), 255, 0, 0);
    }
}

static void draw_center_lines2()
{
    int r, o, m;

    // Draw the red center line vertical
    m = UP_W / 2;
    for (r = 0; r < UP_H; r++)
    {
        o = (r * UP_W + m) * CHANNELS;
        //set_pixel_value_at_off(get_pixel_offset(o), 255, 0, 0);
        set_pix(o, 255, 0, 0);
    }

    // Draw the red center line horizontal
    m = UP_H / 2;
    for (r = 0; r < UP_W; r++)
    {
        o = (m * UP_W + r) * CHANNELS;
        set_pix(o, 255, 0, 0);
        //set_pixel_value_at_off(get_pixel_offset(o), 255, 0, 0);

    }
}


static void upscale_image()
{
    int x, y, o, no;
    unsigned char p;

    for (y = 0; y < HEIGHT; y++)
    {
        for (x = 0; x < WIDTH; x++)
        {
            o = y * WIDTH + x;
            p = read_buffer[o];

            no = 2*y*UP_W + x*2;

            set_pix(no, p, p, p);
            set_pix(no+1, p, p, p);
            set_pix(no+UP_W, p, p, p);
            set_pix(no+1+UP_W, p, p, p);
        }
    }
}

static exitp(const char * msg)
{
    perror(msg);
    exit(errno);
}

int main(int argc, char* argv[])
{
    struct timeval begin, now;
    long counter;

    printf("\n -- EYEBOT Viewer -- \n\n");

    // Allocate buffers
    img_buffer = malloc(SIZE * CHANNELS);
    read_buffer = malloc(SIZE + PADDING);

    scaled_up_img = malloc(UP_S * CHANNELS);
    scaled_up_img_dbl = malloc(UP_S * CHANNELS);

    unsigned char b[4096];

    display = XOpenDisplay(NULL);
    visual = DefaultVisual(display, 0);
    
    window = XCreateSimpleWindow(display, RootWindow(display, 0), 0, 0, 320, 300, 1, 0, 0);
    XMapWindow(display, window);
    XFlush(display);

    if(visual->class!=TrueColor)
    {
        fprintf(stderr, "Cannot handle non true color visual ...\n");
        exit(1);
    }

    //XMapWindow(display, window);
    //XFlush(display);

    image = NULL;

    struct addrinfo hints, *res;
    int socket_fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(socket_fd == -1)
    {
        printf("Could not make a socket\n");
        return 1;
    }

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(SERVER_HOSTNAME, SERVER_PORT, &hints, &res);

    sleep(1);
    memset(b, 0, sizeof(b));
    sprintf(b, "Eyebot Line Follower - LiveEye");
    XSetForeground(display, DefaultGC(display, 0), 0x00ff0000); // red
    XDrawString(display, window, DefaultGC(display, 0), 10, 20, b, strlen(b));
    
    memset(b, 0, sizeof(b));
    sprintf(b, "Copyright (C) 2013 Andre Christensen and Jacob Pedersen");
    XSetForeground(display, DefaultGC(display, 0), 0x000000EF); // red
    XDrawString(display, window, DefaultGC(display, 0), 10, 35, b, strlen(b));

    XFlush(display);

    while (1)
    {
        printf("Connecting to EYEBOT! (%s:%s)\n", SERVER_HOSTNAME, SERVER_PORT);

        // Connect loop
        while (1)
        {
            if (connect(socket_fd, res->ai_addr, res->ai_addrlen) == -1)
            {
                sleep(1);
            } 
            else
            {
                printf("Connected!\n");
                break;
            }
        }

        // Two seconds of timeout. Useful so that recv doesn't wait 
        // forever to receive all its bytes.
        struct timeval tv;
        tv.tv_sec = 2;  
        tv.tv_usec = 0; 
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

        // Measure time at beginning
        counter = 0;
        gettimeofday(&begin, NULL);

        // Frame update loop
        while (1)
        {
            int bytes_read;
            int total = 0;
            unsigned int x = 0, y = 0, error, error_upper, mass;
            int l_x, l_y, u_x, u_y;

            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(socket_fd, &fds);

            struct timeval timeout;
            timeout.tv_sec = 6 * 60;
            timeout.tv_usec = 0;

            if (select(sizeof(fds) * 8, &fds, NULL, NULL, &timeout) < 0)
            {
                exitp("select()");
            }

            if (read(socket_fd, &l_x, sizeof(l_x)) < 0)
            {
                exitp("read() (error x)");
            }

            if (read(socket_fd, &l_y, sizeof(l_y)) < 0)
            {
                exitp("read() (error y)");
            }

            if (read(socket_fd, &u_x, sizeof(u_x)) < 0)
            {
                exitp("read() (error x)");
            }

            if (read(socket_fd, &u_y, sizeof(u_y)) < 0)
            {
                exitp("read() (error y)");
            }

            if (read(socket_fd, &error, sizeof(error)) < 0)
            {
                exitp("read() (error)");
            }

            if (read(socket_fd, &error_upper, sizeof(error_upper)) < 0)
            {
                exitp("read() (error)");
            }


            if (read(socket_fd, &mass, sizeof(mass)) < 0)
            {
                exitp("read() (error mass)");
            }

            // Clear the read buffer
            memset(read_buffer, 0, SIZE);

            //
            // Read the 76800 bytes a frame consist of
            //
            
            bytes_read = recv(socket_fd, read_buffer, SIZE, MSG_WAITALL);
            if (bytes_read < 0)
            {
                perror("recv");
                break;
            }
            else if (bytes_read == SIZE)
            {
                copy_to_x_buffer(read_buffer, bytes_read);
                draw_center_point(l_x, l_y, 0, 0, 255);
                draw_center_point(u_x, u_y, 0, 255, 0);
                draw_center_lines();

                //upscale_image();

                if (image != NULL)
                {
                    XDestroyImage(image);
                }

                //upscale_image();
                //draw_center_point2(l_x, l_y, 0, 0, 255);
                //draw_center_point2(u_x, u_y, 0, 255, 0);
                //draw_center_lines2();

                //scaled_up_img_dbl = malloc(UP_S * CHANNELS);
                //memcpy(scaled_up_img_dbl, scaled_up_img, UP_S * CHANNELS);


                // Allocate a new buffer for next frame. 
                // The old one is freed by XDestroyImage
                img_disp_buffer = (unsigned char *) malloc(SIZE * CHANNELS);
                memcpy(img_disp_buffer, img_buffer, SIZE * CHANNELS);
                
                image = XCreateImage(display, visual, 24, ZPixmap, 0, img_disp_buffer, 320, 240, 32, 0);
                XPutImage(display, window, DefaultGC(display, 0), image, 0, 0, 0, 0, 320, 240);

                //image = XCreateImage(display, visual, 24, ZPixmap, 0, scaled_up_img_dbl, 640, 480, 32, 0);
                //XPutImage(display, window, DefaultGC(display, 0), image, 0, 0, 0, 0, 640, 480);

                // Clear text area
                XSetForeground(display, DefaultGC(display, 0), 0x00ffffff); 
                XFillRectangle(display, window, DefaultGC(display, 0), 0, 240, 320, 60);

                // memset(b, 0, sizeof(b));
                // sprintf(b, "Center: (%d, %d)", x,y);
                XSetForeground(display, DefaultGC(display, 0), 0x00ff0000); // red
                // XDrawString(display, window, DefaultGC(display, 0), 5, 260, b, strlen(b));

                memset(b, 0, sizeof(b));
                sprintf(b, "Error (upper): %d", error_upper);
                XDrawString(display, window, DefaultGC(display, 0), 5, 250, b, strlen(b));

                memset(b, 0, sizeof(b));
                sprintf(b, "Error (lower): %d", error);
                XDrawString(display, window, DefaultGC(display, 0), 5, 280, b, strlen(b));

                memset(b, 0, sizeof(b));
                sprintf(b, "Mass: %d", mass);
                XDrawString(display, window, DefaultGC(display, 0), 150, 250, b, strlen(b));


                // Calculate and display fps
                gettimeofday(&now, NULL);
                double  elapsed = (now.tv_sec - begin.tv_sec) * 1000.0;
                elapsed += (now.tv_usec - begin.tv_usec) / 1000.0;
                elapsed /= 1000.0;
                memset(b, 0, sizeof(b));
                sprintf(b, "FPS: %.2f", ((counter / elapsed) * 3));
                XDrawString(display, window, DefaultGC(display, 0), 150, 280, b, strlen(b));

                counter++;
            }
            else
            {   
                printf("Connection lost - retrying!\n");
                close(socket_fd);
                break;
            }
        }
    }

	printf("Closing the socket\n");
    if(close(socket_fd) == -1)
    {
    	printf("Could not close socket\n");
     	return 0;
    }
}




