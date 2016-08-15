
#ifndef _APHY_H_
#define _APHY_H_

#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef _WIN32
#   ifdef APHY_BUILD
#       define APHY_EXPORT __declspec(dllexport)
#   else
#       define APHY_EXPORT __declspec(dllimport)
#   endif
#else
#   if __GNUC__ >= 4
#       define APHY_EXPORT __attribute__ ((visibility ("default")))
#   endif
#endif

typedef unsigned char aphy_byte;
typedef signed char aphy_sbyte;
typedef signed short aphy_short;
typedef unsigned short aphy_ushort;
typedef signed int aphy_int;
typedef unsigned int aphy_uint;
typedef void* aphy_pointer;
typedef unsigned int aphy_size;
typedef int aphy_enum;
typedef int aphy_bool;
typedef float aphy_float;
typedef double aphy_double;
typedef unsigned int aphy_bitfield;
typedef const char* aphy_cstring;
typedef const char* aphy_string;
typedef int aphy_string_length;
typedef char* aphy_cstring_buffer;
typedef char* aphy_string_buffer;

typedef struct _aphy_engine aphy_engine;

typedef enum {
	APHY_OK = 0,
	APHY_ERROR = -1,
	APHY_NULL_POINTER = -2,
	APHY_INVALID_OPERATION = -3,
	APHY_INVALID_PARAMETER = -4,
	APHY_OUT_OF_BOUNDS = -5,
	APHY_UNSUPPORTED = -6,
	APHY_UNIMPLEMENTED = -7,
} aphy_error;


/* Global functions. */
typedef aphy_error (*aphyGetEngines_FUN) ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines );

APHY_EXPORT aphy_error aphyGetEngines ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines );

/* Methods for interface aphy_engine. */
typedef aphy_error (*aphyAddEngineReference_FUN) ( aphy_engine* engine );
typedef aphy_error (*aphyReleaseEngine_FUN) ( aphy_engine* engine );
typedef aphy_cstring (*aphyGetEngineName_FUN) ( aphy_engine* engine );
typedef aphy_int (*aphyGetEngineVersion_FUN) ( aphy_engine* engine );

APHY_EXPORT aphy_error aphyAddEngineReference ( aphy_engine* engine );
APHY_EXPORT aphy_error aphyReleaseEngine ( aphy_engine* engine );
APHY_EXPORT aphy_cstring aphyGetEngineName ( aphy_engine* engine );
APHY_EXPORT aphy_int aphyGetEngineVersion ( aphy_engine* engine );

/* Installable client driver interface. */
typedef struct _aphy_icd_dispatch {
	int icd_interface_version;
	aphyGetEngines_FUN aphyGetEngines;
	aphyAddEngineReference_FUN aphyAddEngineReference;
	aphyReleaseEngine_FUN aphyReleaseEngine;
	aphyGetEngineName_FUN aphyGetEngineName;
	aphyGetEngineVersion_FUN aphyGetEngineVersion;
} aphy_icd_dispatch;


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* _APHY_H_ */
