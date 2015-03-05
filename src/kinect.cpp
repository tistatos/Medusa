/**
 * @File kinect.cpp
 *    kinect communications
 * @autor Erik Sandrén
 * @date  2015-02-27
 */
#include <iostream>
#include "libfreenect/libfreenect_sync.h"


int main(int argc, char const *argv[])
{
  int ret;
  char *rgb = 0;
  uint32_t ts;
  freenect_raw_tilt_state *state = 0;

  freenect_sync_set_led(LED_RED,0);

  freenect_sync_set_tilt_degs(0, 0);
  freenect_sync_get_tilt_state(&state, 0);

  while(state->tilt_angle != 0)
  {
    freenect_sync_get_tilt_state(&state, 0);
    std::cout << "angle" << static_cast<int>(state->tilt_angle) << std::endl;
  }

  ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
  std::cout << "first " << ts << std::endl;
  freenect_sync_set_tilt_degs(27, 0);

  while(state->tilt_angle != 27)
  {
    freenect_sync_get_tilt_state(&state, 0);
    int ang = state->tilt_angle;
    std::cout << "angle " << ang << std::endl;
  }

  ret = freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB);
  std::cout << "second " << ts << std::endl;
  freenect_sync_set_led(LED_BLINK_GREEN,0);

  return 0;
}