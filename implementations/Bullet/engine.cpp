#include <stdio.h>
#include "engine.hpp"

aphy_cstring aphy_engine::getName (  )
{
    return "Bullet";
}

aphy_int aphy_engine::getVersion (  )
{
    return 10;
}

APHY_EXPORT aphy_error aphyGetEngines ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines )
{
    if(ret_numengines)
        *ret_numengines = 1;

    if(engines && numengines >= 1)
        engines[0] = new aphy_engine();

    return APHY_OK;
}

// The exported C interface
APHY_EXPORT aphy_cstring aphyGetEngineName ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;
    return engine->getName();
}

APHY_EXPORT aphy_int aphyGetEngineVersion ( aphy_engine* engine )
{
    if(!engine)
        return 0;
    return engine->getVersion();
}

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

void printError(const char *format, ...)
{
    char buffer[2048];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 2048, format, args);
#ifdef _WIN32
    if(!GetConsoleCP())
        OutputDebugStringA(buffer);
    else
        fputs(buffer, stderr);
#else
    fputs(buffer, stderr);
#endif
    va_end(args);
}
