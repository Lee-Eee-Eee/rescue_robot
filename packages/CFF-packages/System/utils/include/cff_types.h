#ifndef _CFF_TYPES_H_
#define _CFF_TYPES_H_

#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} Point;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    Point position;
    Quaternion orientation;
} Pose;

typedef struct
{
    float x;
    float y;
    float theta;
} Pose2D;

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

typedef Vector3 linear;
typedef Vector3 angular;

typedef struct
{
    linear linear;
    angular angular;
} Accel;

#endif // _CFF_TYPES_H_