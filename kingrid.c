/*
 * Test Kinect/libfreenect program to display grid-based stats.
 * Created Jan. 11, 2011
 * (C)2011 Mike Bourgeous
 *
 * Some ideas for more work:
 * 3D grid occupation - consider a 3D grid box as "occupied" if 20% or more of
 * the pixels in that 3D grid box's corresponding image-space 2D box are within
 * the range of that 3D grid box.  In this case the 3D grid boxes would
 * actually be pyramid sections in true 3D space, not cubes.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

#include <libfreenect/libfreenect.h>


#define INFO_OUT(...) {\
	printf("%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	printf(__VA_ARGS__);\
}
#define ERROR_OUT(...) {\
	fprintf(stderr, "\e[0;1m%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, "\e[0m");\
}

#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))


#define SM_HIST_SIZE	32

// Get a depth pixel from an 11-bit buffer stored in uint16_t
#define DPT(buf, x, y) (buf[(x) * FREENECT_FRAME_W + (y)])

// Convert pixel number to coordinates
#define PX_TO_X(pix) ((pix) % FREENECT_FRAME_W)
#define PX_TO_Y(pix) ((pix) / FREENECT_FRAME_W)

// Convert pixel number to grid entry
#define PX_TO_GRIDX(pix) (PX_TO_X(pix) * divisions / FREENECT_FRAME_W)
#define PX_TO_GRIDY(pix) (PX_TO_Y(pix) * divisions / FREENECT_FRAME_H)

// Depth gamma look-up table (I wish freenect provided a user data struct for callbacks)
static float depth_lut[2048];
static int out_of_range = 0;
static int divisions = 8; // Grid divisions
static int div_pix = ((FREENECT_FRAME_W + 5) / 6) * ((FREENECT_FRAME_H + 5) / 6);
static int boxwidth = 10; // Display grid box width, less border and padding
static unsigned int frame = 0; // Frame count

static enum {
	GRID,
} disp_mode = GRID;

void repeat_char(int c, int count)
{
	int i;
	for(i = 0; i < count; i++) {
		putchar(c);
	}
}

// Prints horizontal border between grid rows
void grid_hline()
{
	int i;
	for(i = 0; i < divisions; i++) {
		putchar('+');
		repeat_char('-', boxwidth + 2);
	}
	printf("+\n");
}

// Prints a single row in a single grid box
void grid_box_row(const char *text)
{
	printf("| %*s ", boxwidth, text);
	fflush(stdout); // XXX
}

// Prints a formatted single row in a single grid box
void __attribute__((format(printf, 1, 2))) grid_entry(const char *format, ...)
{
	char buf[boxwidth + 1];
	va_list args;

	va_start(args, format);
	vsnprintf(buf, boxwidth + 1, format, args);
	va_end(args);

	grid_box_row(buf);
}

// Prints a horizontal bar chart element in a grid box
void grid_bar(int c, int percent)
{
	putchar(' ');
	repeat_char(c, boxwidth * percent / 100);
	repeat_char(' ', boxwidth * (100 - percent) / 100);
	putchar(' ');
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	uint16_t *buf = (uint16_t *)depthbuf;
	volatile int small_histogram[divisions][divisions][SM_HIST_SIZE];
	volatile int total[divisions][divisions];
	volatile uint16_t min[divisions][divisions];
	volatile uint16_t max[divisions][divisions];
	volatile uint16_t median[divisions][divisions];
	volatile float avg[divisions][divisions];
	volatile int oor_count[divisions][divisions];
	int oor_total = 0; // Out of range count
	volatile int i, j, medcount, histcount;

	// Initialize data structures
	memset(small_histogram, 0, sizeof(small_histogram));
	memset(total, 0, sizeof(total));
	memset(max, 0, sizeof(max));
	memset(oor_count, 0, sizeof(oor_count));

	for(i = 0; i < divisions; i++) {
		for(j = 0; j < divisions; j++) {
			// Set min to 2047 to allow depth_lut use without
			// bounds checking when all pixels in a grid box are
			// out of range
			min[i][j] = 2047;
		}
	}

	// Fill in grid stats
	for(i = 0; i < FREENECT_FRAME_PIX; i++) {
		int gridx = PX_TO_GRIDX(i);
		int gridy = PX_TO_GRIDY(i);
		if(buf[i] == 2047) {
			oor_count[gridy][gridx]++;
			oor_total++;
			continue;
		}

		small_histogram[gridy][gridx][buf[i] * SM_HIST_SIZE / 2048]++;

		if(buf[i] < min[gridy][gridx]) {
			min[gridy][gridx] = buf[i];
		}
		if(buf[i] > max[gridy][gridx]) {
			max[gridy][gridx] = buf[i];
		}
		total[gridy][gridx] += buf[i];
	}

	// Calculate grid averages
	for(i = 0; i < divisions; i++) {
		for(j = 0; j < divisions; j++) {
			if(oor_count[i][j] < div_pix) {
				avg[i][j] = (double)total[i][j] / (double)(div_pix - oor_count[i][j]);
			} else {
				avg[i][j] = 2047;
			}
			for(medcount = 0, histcount = 0; histcount < SM_HIST_SIZE; histcount++) {
				medcount += small_histogram[i][j][histcount];
				if(medcount >= (div_pix - oor_count[i][j]) / 2) {
					break;
				}
			}
			median[i][j] = histcount;
		}
	}

	// Display grid stats
	printf("\e[H\e[2J");
	INFO_OUT("Time: %u frame: %d\n", timestamp, frame);
	for(i = 0; i < divisions; i++) {
		grid_hline();

		for(j = 0; j < divisions; j++) {
			grid_entry("Tot %d", total[i][j]);
		}
		printf("|\n");

		for(j = 0; j < divisions; j++) {
			grid_entry("Avg %f", depth_lut[(int)avg[i][j]]);
		}
		printf("|\n");

		for(j = 0; j < divisions; j++) {
			grid_entry("Min %f", depth_lut[min[i][j]]);
		}
		printf("|\n");

		for(j = 0; j < divisions; j++) {
			grid_entry("Med ~%f", depth_lut[median[i][j] * 2048 / SM_HIST_SIZE]);
			}
		printf("|\n");

		for(j = 0; j < divisions; j++) {
			grid_entry("Max %f", depth_lut[max[i][j]]);
		}
		printf("|\n");
		
		for(j = 0; j < divisions; j++) {
			grid_entry("Out %d%%", oor_count[i][j] * 100 / div_pix);
		}
		printf("|\n");
	}
	grid_hline();

	fflush(stdout);

	// Make LED red if more than 35% of the image is out of range (can't
	// set LED in callback for some reason)
	out_of_range = oor_total > FREENECT_FRAME_PIX * 35 / 100;

	frame++;
}


static int done = 0;

void intr(int signum)
{
	INFO_OUT("Exiting due to signal %d (%s)\n", signum, strsignal(signum));
	done = 1;

	signal(signum, exit);
}

// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21#e98a94ac605b9f21
void init_lut()
{
	int i;

	for(i = 0; i < 2048; i++) {
		depth_lut[i] = 0.1236 * tanf(i / 2842.5 + 1.1863);
	}
}

int main(int argc, char *argv[])
{
	int opt;

	freenect_context *kn;
	freenect_device *kn_dev;

	while((opt = getopt(argc, argv, "g:")) != -1) {
		switch(opt) {
			case 'g':
				// Grid divisions
				disp_mode = GRID;
				divisions = atoi(optarg);
				div_pix = ((FREENECT_FRAME_W + (divisions - 1)) / divisions) * ((FREENECT_FRAME_H + (divisions - 1)) / divisions);
				break;
			default:
				fprintf(stderr, "Usage: %s [-g divisions]\n", argv[0]);
				fprintf(stderr, "Use one of:\n");
				fprintf(stderr, "\tg - Set grid divisions for both dimensions\n");
				return -1;
		}
	}

	if(signal(SIGINT, intr) == SIG_ERR ||
			signal(SIGTERM, intr) == SIG_ERR) {
		ERROR_OUT("Error setting signal handlers\n");
		return -1;
	}

	init_lut();

	if(freenect_init(&kn, NULL) < 0) {
		ERROR_OUT("libfreenect init failed.\n");
		return -1;
	}

	INFO_OUT("Found %d Kinect devices.\n", freenect_num_devices(kn));

	if(freenect_num_devices(kn) == 0) {
		ERROR_OUT("No Kinect devices present.\n");
		return -1;
	}

	if(freenect_open_device(kn, &kn_dev, 0)) {
		ERROR_OUT("Error opening Kinect #0.\n");
		return -1;
	}

	freenect_set_tilt_degs(kn_dev, -5);
	freenect_set_led(kn_dev, LED_GREEN);
	freenect_set_depth_callback(kn_dev, depth);
	freenect_set_depth_format(kn_dev, FREENECT_DEPTH_11BIT);

	freenect_start_depth(kn_dev);

	int last_oor = out_of_range;
	while(!done && freenect_process_events(kn) >= 0) {
		if(last_oor != out_of_range) {
			freenect_set_led(kn_dev, out_of_range ? LED_BLINK_RED_YELLOW : LED_GREEN);
			last_oor = out_of_range;
		}
	}

	freenect_stop_depth(kn_dev);
	freenect_set_led(kn_dev, LED_OFF);
	freenect_close_device(kn_dev);
	freenect_shutdown(kn);

	return 0;
}

