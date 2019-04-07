#ifndef _APHY_BULLET_COMMON_HPP
#define _APHY_BULLET_COMMON_HPP

#include <APHY/aphy_impl.hpp>
#include <stdarg.h>
#include <stdio.h>

#define CHECK_POINTER(pointer) if (!(pointer)) return APHY_NULL_POINTER;

namespace APhyBullet
{
using namespace aphy;

void printError(const char *format, ...);

}// End of namespace APhyBullet

#endif //_APHY_BULLET_COMMON_HPP
