/**
 * @file debug_helpers.h
 *    Helper functions for debugging
 * @author Erik Sandrén
 * @date 2015-04-10
 */
#ifndef __DEBUG_HELPER_H__
#define __DEBUG_HELPER_H__

#include <iostream>

/**
 * @brief print message in console
 * @param msg message to print
 */
void inline printDebugMessage(const char* msg)
{
  std::cout << msg << std::endl;
}

/**
 * @brief print number in console
 * @param number number to print
 */
void inline printDebugMessage(const int number)
{
  std::cout << number << std::endl;
}

/**
 * @brief print state in console
 *
 * @param state to print
 */
void inline printDebugMessage(const bool state)
{
  std::cout << state << std::endl;
}

#endif
