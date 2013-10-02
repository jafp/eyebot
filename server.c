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



int main(int argc, char* argv[])
{
    // Allocate buffers
    img_buffer = malloc(SIZE * CHANNELS);
    read_buffer = malloc(SIZE + PADDING);


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

    memset(img_buffer, 255, 320*240*4);
    image = XCreateImage(display, visual, 24, ZPixmap, 0, img_buffer, 320, 240, 32, 0);
    XPutImage(display, window, DefaultGC(display, 0), image, 0, 0, 0, 0, 320, 240);


    int hSocket,hServerSocket;  /* handle to socket */
    struct hostent* pHostInfo;   /* holds info about a machine */
    struct sockaddr_in Address; /* Internet socket address stuct */
    int nAddressSize=sizeof(struct sockaddr_in);
    char pBuffer[BUFFER_SIZE];
    int nHostPort = 23000;

    printf("\nStarting server");
    printf("\nMaking socket");

    hServerSocket=socket(AF_INET,SOCK_STREAM,0);
    if(hServerSocket == SOCKET_ERROR)
    {
        printf("\nCould not make a socket\n");
        return 0;
    }

    Address.sin_addr.s_addr=INADDR_ANY;
    Address.sin_port=htons(nHostPort);
    Address.sin_family=AF_INET;

    printf("\nBinding to port %d",nHostPort);
    if(bind(hServerSocket,(struct sockaddr*)&Address,sizeof(Address)) == SOCKET_ERROR)
    {
        printf("\nCould not connect to host\n");
        return 0;
    }

    getsockname( hServerSocket, (struct sockaddr *) &Address,(socklen_t *)&nAddressSize);
    printf("opened socket as fd (%d) on port (%d) for stream i/o\n",hServerSocket, ntohs(Address.sin_port) );
    printf("\nMaking a listen queue of %d elements",QUEUE_SIZE);

    if(listen(hServerSocket,QUEUE_SIZE) == SOCKET_ERROR)
    {
        printf("\nCould not listen\n");
        return 0;
    }

    for(;;)
    {
        printf("\nWaiting for a connection\n");
        /* get the connected socket */
        hSocket=accept(hServerSocket,(struct sockaddr*)&Address,(socklen_t *)&nAddressSize);

        printf("\nGot a connection");
    
        while (1)
        {               
            int bytes_read;
            int total = 0;
            unsigned int x = 0, y = 0;

            read(hSocket, &x, sizeof(x));
            read(hSocket, &y, sizeof(y));

            while ((bytes_read = read(hSocket, read_buffer + total, 4096)) > 0)
            {
                if (bytes_read == -1)
                {
                    return 1;
                }

                //printf("received: %d\n", bytes_read);
                total += bytes_read;
                // one more go..

                if (total >= 76800)
                {
                    copy_to_x_buffer(read_buffer, total);
                    draw_center_point(x, y);

                    //image = XCreateImage(display, visual, 24, ZPixmap, 0, img_buffer, 320, 240, 32, 0);
                    XPutImage(display, window, DefaultGC(display, 0), image, 0, 0, 0, 0, 320, 240);


                    break;
                }
            }

            //while (read(hSocket, buf, 4096) > 0) {}

            // printf("read: %ld\n", total);
//
            if (total == 0)
            {
                break;
            }
        }


		printf("\nClosing the socket");
        if(close(hSocket) == SOCKET_ERROR)
        {
        	printf("\nCould not close socket\n");
         	return 0;
        }
    }

}




