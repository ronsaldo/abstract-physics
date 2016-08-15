#ifndef _APHY_BULLET_COMMON_HPP
#define _APHY_BULLET_COMMON_HPP

#include <APHY/aphy.h>
#include <stdarg.h>
#include <stdio.h>

#define CHECK_POINTER(pointer) if (!(pointer)) return APHY_NULL_POINTER;

void printError(const char *format, ...);

#endif //_APHY_BULLET_COMMON_HPP
