/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4base_h_included
#define sixit_geometry__internal_simd_internal_vec4base_h_included

#include <cmath>

#include "sixit/geometry/_internal/simd/internal/simd.h"
#include "sixit/geometry/_internal/simd/internal/math.h"
#include "sixit/core/lwa.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
struct /*alignas(16)*/ vec4base {
	fp vec_fp[4];

	vec4base() {}
	vec4base(fp a);
	vec4base(fp a, fp b, fp c);
	vec4base(fp a, fp b, fp c, fp d);
	vec4base(const vec4base<fp>& v);

	vec4base& operator=(const vec4base& v);

	const fp* const_ptr() const;
	const fp& operator[](unsigned index) const;	
	fp& operator[](unsigned index);
	
	template <int i0, int i1, int i2, int i3>
	vec4base swizzle();

	bool operator==(const vec4base& v) const;

	vec4base& operator+=(const vec4base& v);
	vec4base& operator-=(const vec4base& v);
	vec4base& operator*=(const vec4base& v);
	vec4base& operator/=(const vec4base& v);
	vec4base& operator*=(fp a);
	vec4base& operator/=(fp a);
	
	void set(fp a, fp b, fp c);
	void set(fp a, fp b, fp c, fp d);

	static fp dot(const vec4base& a, const vec4base& b);
	static fp angle(const vec4base& a, const vec4base& b);
	static auto cross(const vec4base& a, const vec4base& b);
	static vec4base max(const vec4base& a, const vec4base& b);
	static vec4base min(const vec4base& a, const vec4base& b);
	static vec4base abs(const vec4base& v);

	fp magnitude() const;
	fp sqr_magnitude() const;

	void normalize();
	vec4base normalized() const;

	void scale(fp a);
	static vec4base scale(const vec4base& v, fp a);
	void scale3(fp a);
	static vec4base scale3(const vec4base& v, fp a);

};

#ifdef SIXIT_GEOMETRY_USE_SIMD
template <>
struct alignas(16) vec4base<float> {
	sfloat4 vec;

	vec4base() {}
	vec4base(float a);
	vec4base(float a, float b, float c);
	vec4base(float a, float b, float c, float d);
	vec4base(const vec4base<float>& v);
	vec4base(sfloat4 sv);

	vec4base& operator=(const vec4base<float>& v);

	const float* const_ptr() const;
	const float& operator[](unsigned index) const;
	float& operator[](unsigned index);

	template <int i0, int i1, int i2, int i3>
	vec4base swizzle();

	bool operator==(const vec4base<float>& v) const;

	vec4base<float>& operator+=(const vec4base<float>& v);
	vec4base<float>& operator-=(const vec4base<float>& v);
	vec4base<float>& operator*=(const vec4base<float>& v);
	vec4base<float>& operator/=(const vec4base<float>& v);
	vec4base<float>& operator*=(float a);
	vec4base<float>& operator/=(float a);

	void set(float a, float b, float c);
	void set(float a, float b, float c, float d);

	static float dot(const vec4base<float>& a, const vec4base<float>& b);
	static float angle(const vec4base<float>& a, const vec4base<float>& b);
	static auto cross(const vec4base<float>& a, const vec4base<float>& b);
	static vec4base max(const vec4base<float>& a, const vec4base<float>& b);
	static vec4base min(const vec4base<float>& a, const vec4base<float>& b);
	static vec4base abs(const vec4base<float>& v);

	float magnitude() const;
	float sqr_magnitude() const;

	void normalize();
	vec4base normalized() const;

	void scale(float a);
	static vec4base scale(const vec4base<float>& v, float a);
	void scale3(float a);
	static vec4base scale3(const vec4base<float>& v, float a);

};
#endif

template <typename fp>
inline vec4base<fp> operator+(const vec4base<fp>& a, const vec4base<fp>& b);
template <typename fp>
inline vec4base<fp> operator-(const vec4base<fp>& a, const vec4base<fp>& b);
template <typename fp>
inline vec4base<fp> operator*(const vec4base<fp>& a, const vec4base<fp>& b);
template <typename fp>
inline vec4base<fp> operator/(const vec4base<fp>& a, const vec4base<fp>& b);
template <typename fp>
inline vec4base<fp> operator*(const vec4base<fp>& a, fp b);
template <typename fp>
inline vec4base<fp> operator/(const vec4base<fp>& a, fp b);

}

namespace gpu 
{
    // aliases 
    using vec4basef = vec4base<float>;
}

}
}

#ifdef SIXIT_GEOMETRY_USE_SIMD
#if defined(__SSE2__)
#include "sixit/geometry/_internal/simd/internal/vec4base_impl_sse2.h"
#elif defined(__ARM_NEON)
#include "sixit/geometry/_internal/simd/internal/vec4base_impl_neon.h"
#endif
#endif

#include "sixit/geometry/_internal/simd/internal/vec4base_impl_generic.h"
#include "sixit/geometry/_internal/simd/internal/vec4base_impl_common.h"

#endif //sixit_geometry__internal_simd_internal_vec4base_h_included

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
