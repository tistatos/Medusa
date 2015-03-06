/**
 * @File kinect_depth.cpp
 *    test with async kinect
 * @autor Erik Sandrén
 * @date 2015-03-06
 */

#include <stdio.h>
#include <iostream>
#include "libfreenect/libfreenect.h"

#include <pthread.h>

freenect_context *f_ctx;
freenect_device *f_dev;
pthread_t freenect_thread;

volatile int die = 0;

int counter = 0;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
  std::cout << "Doing stuff..." << std::endl;
}





void *freenect_threadfunc(void *arg)
{
  printf("start!");

  freenect_set_tilt_degs(f_dev,0);
  freenect_set_led(f_dev,LED_YELLOW);
  freenect_set_depth_callback(f_dev, depth_cb);
  freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT_PACKED));
  freenect_start_depth(f_dev);

  while (!die && freenect_process_events(f_ctx) >= 0) {
      std::cout << "running!" << std::endl;

      counter++;
      if(counter > 150)
        die = 1;
  }

  printf("\nshutting down streams...\n");

  freenect_set_led(f_dev,LED_BLINK_GREEN);
  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);

  printf("-- done!\n");
  return NULL;
}


int main(int argc, char const *argv[])
{

  printf("Trying to connect to kinect\n");
  if(freenect_init(&f_ctx, NULL) <0)
  {
    printf("Failed to initalized freenect\n");
    return 1;
  }

  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));


  int nr_devices = freenect_num_devices(f_ctx);
  printf("Number of devices: %d\n", nr_devices);

  if(nr_devices < 1)
  {
    freenect_shutdown(f_ctx);
    return 1;
  }

  if(freenect_open_device(f_ctx, &f_dev, 0) < 0)
  {
    printf("Error Connecting Device\n");
  }

  int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc,NULL);

  if (res) {
    printf("pthread_create failed\n");
    freenect_shutdown(f_ctx);
    return 1;
  }


  printf("-- STOP!\n");

  return 0;
}