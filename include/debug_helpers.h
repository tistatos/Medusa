/**
 * @File debug_helpers.h
 *    Helper functions for debugging
 * @autor Erik Sandrén
 * @date 2015-04-10
 */
#ifndef __DEBUG_HELPER_H__
#define __DEBUG_HELPER_H__

#include <iostream>

void inline printDebugMessage(const char* msg)
{
  std::cout << msg << std::endl;
}

void inline printDebugMessage(const int number)
{
  std::cout << number << std::endl;
}

void inline printDebugMessage(const bool state)
{
  std::cout << state << std::endl;
}


#endif
