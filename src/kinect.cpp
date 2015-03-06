/**
 * @File kinect.cpp
 *    kinect communications
 * @autor Erik Sandrén
 * @date  2015-02-27
 */
#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

FILE *open_dump(const char *filename)
{
	//open file
	FILE* fp = fopen(filename, "w");
	if (fp == NULL) {
		fprintf(stderr, "Error: Cannot open file [%s]\n", filename);
		exit(1);
	}
	printf("%s\n", filename);
	return fp;
}

void dump_rgb(FILE *fp, void *data, unsigned int width, unsigned int height)
{
	//*3 = channel
	fprintf(fp, "P6 %u %u 255\n", width, height);
	//write to file
	fwrite(data, width * height * 3, 1, fp);
}

int main(int argc, char const *argv[])
{
  int ret;
  char *rgb = 0;
  short *dep = 0;

  uint32_t ts;
  freenect_raw_tilt_state *state = 0;

  FILE *fp;

  //set led to red
  freenect_sync_set_led(LED_RED,0);

  //set Kinect to 0 degrees(pointing forward), -27<tilt<27
  freenect_sync_set_tilt_degs(0, 0);
  //Tilt state function, starts the runloop if it isn't running
  freenect_sync_get_tilt_state(&state, 0);

  //make the Kinect to wait until decried degree
  while(state->tilt_angle != 0)
  {
    freenect_sync_get_tilt_state(&state, 0);
    std::cout << "angle" << static_cast<int>(state->tilt_angle) << std::endl;
  }

  //ts= timestamp, take a pic
  ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
  std::cout << "first " << ts << std::endl;
  freenect_sync_set_tilt_degs(27, 0);

  //open file, .ppm = format and set the name of the file to "bild"
  fp = open_dump("bild.ppm");
  //resolution 640x480
  dump_rgb(fp, rgb, 640, 480);
  //close file
  fclose(fp);

  while(state->tilt_angle != 27)
  {
    freenect_sync_get_tilt_state(&state, 0);
    int ang = state->tilt_angle;
    std::cout << "angle " << ang << std::endl;
  }
  //take a pic
  ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);

  std::cout << "second " << ts << std::endl;
  freenect_sync_set_led(LED_BLINK_GREEN,0);

  return 0;
}