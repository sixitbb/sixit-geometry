/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_vec4_h_included
#define sixit_geometry__internal_simd_vec4_h_included

#include <cmath>

#include "sixit/geometry/_internal/simd/internal/simd.h"
#include "sixit/geometry/_internal/simd/internal/math.h"

namespace sixit {
namespace geometry {
namespace gpu {

struct alignas(16) vec4 {
	union {
		struct {
			float x,y,z,w;
		};
		struct {
			float r,g,b,a;
		};
		float vec[4];
#ifdef GEOMETRY_USE_SIMD
		sfloat4 svec;
#endif
	};

	vec4() {}
	vec4(float a);
	vec4(float a, float b, float c);
	vec4(float a, float b, float c, float d);
	vec4(const vec4& v);
#ifdef GEOMETRY_USE_SIMD
	vec4(sfloat4 sv);
#endif

	vec4& operator=(const vec4& v);

	float& operator[](unsigned n) { return vec[n]; }
	float  operator[](unsigned n) const { return vec[n]; }

	const float* const_ptr() const { return vec; }

	template <int i0, int i1, int i2, int i3>
	vec4 swizzle();

	bool operator==(const vec4& v) const;

	vec4& operator+=(const vec4& v);
	vec4& operator-=(const vec4& v);
	vec4& operator*=(const vec4& v);
	vec4& operator/=(const vec4& v);
	vec4& operator*=(float a);
	vec4& operator/=(float a);

	void set(float a, float b, float c);
	void set(float a, float b, float c, float d);

	static float dot(const vec4& a, const vec4& b);
	static float angle(const vec4& a, const vec4& b);
	static vec4 cross(const vec4& a, const vec4& b);
	static vec4 max(const vec4& a, const vec4& b);
	static vec4 min(const vec4& a, const vec4& b);
	static vec4 abs(const vec4& v);

	float magnitude() const;
	float sqr_magnitude() const;

	void normalize();
	vec4 normalized() const;

	void scale(float a);
	static vec4 scale(const vec4& v, float a);
	void scale3(float a);
	static vec4 scale3(const vec4& v, float a);

	static vec4 forward();
	static vec4 back();
	static vec4 up();
	static vec4 down();
	static vec4 left();
	static vec4 right();

	static vec4 one();
	static vec4 zero();
};

inline vec4 operator+(const vec4& a, const vec4& b);
inline vec4 operator-(const vec4& a, const vec4& b);
inline vec4 operator*(const vec4& a, const vec4& b);
inline vec4 operator/(const vec4& a, const vec4& b);
inline vec4 operator*(const vec4& a, float b);
inline vec4 operator/(const vec4& a, float b);

}
}
}

#if defined(__SSE2__)
#include "sixit/geometry/_internal/simd/internal/vec4_impl_sse2.h"
#elif defined(__ARM_NEON)
#include "sixit/geometry/_internal/simd/internal/vec4_impl_neon.h"
#else
#include "sixit/geometry/_internal/simd/internal/vec4_impl_generic.h"
#endif

#include "sixit/geometry/_internal/simd/internal/vec4_impl_common.h"

#endif // sixit_geometry__internal_simd_vec4_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Leonid Maksymchuk

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/