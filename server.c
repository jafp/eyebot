#include <sys/types.h>
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

#define SERVER_PORT "24000"
#define SERVER_HOSTNAME "10.42.0.25"


#define WIDTH 			320
#define HEIGHT 			240
#define CHANNELS 		4
#define SIZE 			(WIDTH * HEIGHT)
#define N_IMG_BYTES     (SIZE * CHANNELS)

// Some kB extra space / padding for the receive buffer
#define PADDING         4096

#define SOCKET_ERROR        -1
#define BUFFER_SIZE         100
#define QUEUE_SIZE          5

#define B_LEN  76800

static XImage * image;
static Display *display;
static Visual *visual;
static Window window;

// Buffer for the image displayed on in the window
static unsigned char * img_buffer;//[N_IMG_BYTES];
static unsigned char * img_disp_buffer;
// Buffer used when reading the image over socket
static unsigned char * read_buffer;//[WIDTH * HEIGHT + PADDING];


static int get_pixel_offset(int pixel_pos)
{
	return pixel_pos * CHANNELS;
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

static void draw_center_point(int x, int y)
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

            set_pixel_value_at_off(o1, 0, 0, 255);
        }
    }

    // Draw the red center line
    int middle = WIDTH / 2;
    for (r = 0; r < 240; r++)
    {
    	o = ((r * WIDTH) + middle);
    	set_pixel_value_at_off(get_pixel_offset(o), 255, 0, 0);
    }
}

static exitp(const char * msg)
{
    perror(msg);
    exit(errno);
}

int main(int argc, char* argv[])
{
    // Allocate buffers
    img_buffer = malloc(SIZE * CHANNELS);
    read_buffer = malloc(SIZE + PADDING);

    unsigned char b[4096];

    display = XOpenDisplay(NULL);
    visual = DefaultVisual(display, 0);
    window = XCreateSimpleWindow(display, RootWindow(display, 0), 0, 0, WIDTH, 300, 1, 0, 0);

    if(visual->class!=TrueColor)
    {
        fprintf(stderr, "Cannot handle non true color visual ...\n");
        exit(1);
    }

    XMapWindow(display, window);
    XFlush(display);

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

    if (connect(socket_fd, res->ai_addr, res->ai_addrlen) == -1)
    {
        printf("Failed to connect\n");
        return 1;
    }

    while (1)
    {
        int bytes_read;
        int total = 0;
        unsigned int x = 0, y = 0, error, mass;

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

        if (read(socket_fd, &x, sizeof(x)) < 0)
        {
            exitp("read() (error x)");
        }

        if (read(socket_fd, &y, sizeof(y)) < 0)
        {
            exitp("read() (error y)");
        }

        if (read(socket_fd, &error, sizeof(error)) < 0)
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
        }
        else if (bytes_read == SIZE)
        {
            copy_to_x_buffer(read_buffer, bytes_read);
            draw_center_point(x, y);

            if (image != NULL)
            {
                XDestroyImage(image);
            }

            // Allocate a new buffer for next frame. 
            // The old one is freed by XDestroyImage
            img_disp_buffer = (unsigned char *) malloc(SIZE * CHANNELS);
            memcpy(img_disp_buffer, img_buffer, SIZE * CHANNELS);

            image = XCreateImage(display, visual, 24, ZPixmap, 0, img_disp_buffer, 320, 240, 32, 0);
            XPutImage(display, window, DefaultGC(display, 0), image, 0, 0, 0, 0, 320, 240);

            XSetForeground(display, DefaultGC(display, 0), 0x00ffffff); // red
            XFillRectangle(display, window, DefaultGC(display, 0), 0, 240, 320, 60);

            memset(b, 0, sizeof(b));
            sprintf(b, "Center: (%d, %d)", x,y);
        
            XSetForeground(display, DefaultGC(display, 0), 0x00ff0000); // red
            XDrawString(display, window, DefaultGC(display, 0), 5, 260, b, strlen(b));

            memset(b, 0, sizeof(b));
            sprintf(b, "Error: %d", error);
        
            XSetForeground(display, DefaultGC(display, 0), 0x000000ff); 
            XDrawString(display, window, DefaultGC(display, 0), 5, 280, b, strlen(b));
    
        }
        else
        {
            printf("Eeeeww!\n");
        }
    }


		printf("Closing the socket\n");
        /*
        if(close(hSocket) == SOCKET_ERROR)
        {
        	printf("\nCould not close socket\n");
         	return 0;
        }
        
    }
*/
}




