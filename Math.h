/**
 * @file math.hpp
 * @brief General math declarations and definitions.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#pragma once

#include <algorithm>
#include <cmath>

// floating point precision set by this typedef
typedef double real_t;

class Color3;

// since the standard library happily does not provide one
#define PI 3.141592653589793238

#define Epsilon 1e-10

template<typename T>
inline T clamp( T val, T min, T max )
{
    return std::min( max, std::max( min, val ) );
}

