/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4base_impl_neon_h_included
#define sixit_geometry__internal_simd_internal_vec4base_impl_neon_h_included

#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {
namespace gpu {

namespace internal {

inline float32x4_t __vdivq_f32(float32x4_t a, float32x4_t b) {
#if defined(__aarch64__)
	return vdivq_f32(a, b);
#elif defined(__ARMV7_NEON_FAST_DIV__)
	float32x4_t r = vreqpeq_f32(b);
	r = vmulq_f32(vrecpsq_f32(b, r), r);
	return vmulq_f32(a, r);
#else
	float32x4_t r;
	r[0] = a[0] / b[0];
	r[1] = a[1] / b[1];
	r[2] = a[2] / b[2];
	r[3] = a[3] / b[3];
	return r;
#endif
}

inline float32x4_t __vdivq_n_f32(float32x4_t a, float b) {
#if defined(__aarch64__)
	return vdivq_f32(a, vdupq_n_f32(b));
#elif defined(__ARMV7_NEON_FAST_DIV__)
	float r = vreqpe_f32(b);
	r = vmul_f32(vreqps_f32(b, r), r);
	return vmulq_n_f32(a, r);
#else
	float32x4_t r;
	r[0] = a[0] / b;
	r[1] = a[1] / b;
	r[2] = a[2] / b;
	r[3] = a[3] / b;
	return r;
#endif
}

}

inline const float* vec4base<float>::const_ptr() const {
#   ifdef SIXIT_COMPILER_MSVC
	return &(vec.n128_f32[0]);
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&vec);
#   elif defined SIXIT_COMPILER_GCC
	return &vec[0];
#	else
#	error unsupported compiler
#   endif
}

inline const float& vec4base<float>::operator[](unsigned index) const {
#ifdef SIXIT_COMPILER_MSVC
	return vec.n128_f32[index];
#	elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&vec)[index];
#   elif defined SIXIT_COMPILER_GCC
	return vec[index];
#	else
#	error unsupported compiler
#endif
}

inline float& vec4base<float>::operator[](unsigned index) {
#ifdef SIXIT_COMPILER_MSVC
	return vec.n128_f32[index];
#	elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<float*>(&vec)[index];
#   elif defined SIXIT_COMPILER_GCC
	return vec[index];
#	else
#	error unsupported compiler
#endif
}

inline vec4base<float>::vec4base(float a) {
	vec = vdupq_n_f32(a);
}

inline vec4base<float>::vec4base(float a, float b, float c) {
	vec = (float32x4_t) {a, b, c, 0};
}

inline vec4base<float>::vec4base(float a, float b, float c, float d) {
	vec = (float32x4_t) {a, b, c, d};
}

inline vec4base<float>::vec4base(const vec4base& v) {
	vec = v.vec;
}

inline vec4base<float>::vec4base(sfloat4 sv) {
	vec = sv;
}

inline vec4base<float>& vec4base<float>::operator=(const vec4base& v) {
	vec = v.vec;
	return *this;
}

template <int i0, int i1, int i2, int i3>
inline vec4base<float> vec4base<float>::swizzle() {
	return vec4base<float>(vec[i0], vec[i1], vec[i2], vec[i3]);
}

inline vec4base<float>& vec4base<float>::operator+=(const vec4base& v) {
	vec = vaddq_f32(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator-=(const vec4base& v) {
	vec = vsubq_f32(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator*=(const vec4base& v) {
	vec = vmulq_f32(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator/=(const vec4base& v) {
	vec = internal::__vdivq_f32(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator*=(float a) {
	vec = vmulq_n_f32(vec, a);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator/=(float a) {
	vec = internal::__vdivq_n_f32(vec, a);
	return *this;
}

inline void vec4base<float>::set(float a, float b, float c) {
	vec = (float32x4_t) {a, b, c, 0.0f};
}

inline void vec4base<float>::set(float a, float b, float c, float d) {
	vec = (float32x4_t) {a, b, c, d};
}

inline float vec4base<float>::dot(const vec4base<float>& a, const vec4base<float>& b) {
	float32x4_t vd = vmulq_f32(a.vec, b.vec);
	float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_low_f32(vd));
	x = vadd_f32(x, vget_high_f32(vd));
	return vget_lane_f32(x, 0);
}

inline auto vec4base<float>::cross(const vec4base<float>& a, const vec4base<float>& b) {
	float32x4_t t, v;
	float32x2_t t_low = vget_low_f32(a.vec);
	float32x2_t v_low = vget_low_f32(b.vec);
	t = vcombine_f32(vext_f32(t_low, vget_high_f32(a.vec), 1), t_low);
	v = vcombine_f32(vext_f32(v_low, vget_high_f32(b.vec), 1), v_low);
	v = vmulq_f32(v, a.vec);
	t = vmulq_f32(t, b.vec);
	v = vsubq_f32(v, t);
	v_low = vget_low_f32(v);
	v = vcombine_f32(vext_f32(v_low, vget_high_f32(v), 1), v_low);
	v = (float32x4_t)vandq_s32((int32x4_t)v, (int32x4_t){-1,-1,-1,0});
	return vec4base<float>(v);
}

inline vec4base<float> vec4base<float>::max(const vec4base& a, const vec4base& b) {
	return vec4base<float>(vmaxq_f32(a.vec, b.vec));
}

inline vec4base<float> vec4base<float>::min(const vec4base& a, const vec4base& b) {
	return vec4base<float>(vminq_f32(a.vec, b.vec));
}

inline vec4base<float> vec4base<float>::abs(const vec4base& v) {
	return vec4base<float>((float32x4_t) vandq_s32((int32x4_t) v.vec, vdupq_n_s32(0x7FFFFFFF)));
}

inline void vec4base<float>::scale3(float a) {
	vec = vmulq_f32(vec, (float32x4_t) {a, a, a, 1.0f});
}

inline vec4base<float> vec4base<float>::scale3(const vec4base& v, float a) {
	return vec4base<float>(vmulq_f32(v.vec, (float32x4_t) {a, a, a, 1.0f}));
}


inline bool vec4base<float>::operator==(const vec4base<float>& v) const {
	return	this->operator[](0) == v[0]
		&& this->operator[](1) == v[1]
		&& this->operator[](2) == v[2]
		&& this->operator[](3) == v[3];
}

inline float vec4base<float>::angle(const vec4base<float>& a, const vec4base<float>& b) {
	float c = dot(a, b);
	return sixit::geometry::low_level::mathf::acos(c);
}

inline float vec4base<float>::magnitude() const {
	return sixit::geometry::low_level::mathf::sqrt(sqr_magnitude());
}

inline float vec4base<float>::sqr_magnitude() const {
	return dot(*this, *this);
}

inline void vec4base<float>::normalize() {
	vec = internal::__vdivq_n_f32(vec, magnitude());
}

inline vec4base<float> vec4base<float>::normalized() const {
	return vec4base<float>(internal::__vdivq_n_f32(vec, magnitude()));
}

inline void vec4base<float>::scale(float a) {
	vec = vmulq_n_f32(vec, a);
}

inline vec4base<float> vec4base<float>::scale(const vec4base<float>& v, float a) {
	return vec4base<float>(vmulq_n_f32(v.vec, a));
}


template<>
inline vec4base<float> operator+(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(vaddq_f32(a.vec, b.vec));
}

template<>
inline vec4base<float> operator-(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(vsubq_f32(a.vec, b.vec));
}

template<>
inline vec4base<float> operator*(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(vmulq_f32(a.vec, b.vec));
}

template<>
inline vec4base<float> operator/(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(internal::__vdivq_f32(a.vec, b.vec));
}

template<>
inline vec4base<float> operator*(const vec4base<float>& a, float b) {
	return vec4base<float>(vmulq_n_f32(a.vec, b));
}

template<>
inline vec4base<float> operator/(const vec4base<float>& a, float b) {
	return vec4base<float>(internal::__vdivq_n_f32(a.vec, b));
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4base_impl_neon_h_included

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
