#ifndef __EASY_MATH__
#define __EASY_MATH__

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352
#define GRAVITY_EARTH  9.80665f /*! Earth's gravity in m/s^2 */

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#ifndef radians
#define radians(deg) ((deg)*DEG_TO_RAD)
#endif

#ifndef degrees
#define degrees(rad) ((rad)*RAD_TO_DEG)
#endif

#ifndef sq
#define sq(x) ((x)*(x))
#endif

// interrupts() / noInterrupts() must be defined by the core

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet((value), (bit)) : bitClear((value), (bit)))

#ifndef bit
#define bit(b) (1UL << (b))
#endif


#ifdef __cplusplus
  template<class T, class L> 
  auto min(const T& a, const L& b) -> decltype((b < a) ? b : a)
  {
    return (b < a) ? b : a;
  }

  template<class T, class L> 
  auto max(const T& a, const L& b) -> decltype((b < a) ? b : a)
  {
    return (a < b) ? b : a;
  }
#else
#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })
#endif
#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })
#endif
#endif

#ifdef __cplusplus

/* C++ prototypes */
inline uint16_t makeWord(uint8_t h, uint8_t l)
{
    return (h << 8) | l;
}

#define word(...) makeWord(__VA_ARGS__)

inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#include <rtconfig.h>

#ifdef PKG_CMSIS_DSP_BASIC_MATH
#include <arm_math.h>
#ifdef PKG_CMSIS_DSP_FAST_MATH
#include <fast_math_functions.h>
#endif
#else
#include <math.h>
#endif

#endif // __cplusplus

#endif /* __EASY_MATH__ */
