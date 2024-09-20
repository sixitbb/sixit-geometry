/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4base_impl_sse2_h_included
#define sixit_geometry__internal_simd_internal_vec4base_impl_sse2_h_included

#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {
namespace gpu {

inline vec4base<float>::vec4base(float a) {
	vec = _mm_set1_ps(a);
}

inline vec4base<float>::vec4base(float a, float b, float c) {
	vec = _mm_setr_ps(a, b, c, 0.0f);
}

inline vec4base<float>::vec4base(float a, float b, float c, float d) {
	vec = _mm_setr_ps(a, b, c, d);
}

inline vec4base<float>::vec4base(const vec4base<float>& v) {
	vec = v.vec;
}

inline vec4base<float>::vec4base(sfloat4 sv) {
	vec = sv;
}

inline vec4base<float>& vec4base<float>::operator=(const vec4base<float>& v) {
	vec = v.vec;
	return *this;
}

inline const float& vec4base<float>::operator[](unsigned index) const {
#   ifdef SIXIT_COMPILER_MSVC
	return vec.m128_f32[index];
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&vec)[index];
#   elif defined SIXIT_COMPILER_GCC
	return vec[index];
#	else
#	error unsupported compiler
#   endif
}

inline float& vec4base<float>::operator[](unsigned index) {
#   ifdef SIXIT_COMPILER_MSVC
	return vec.m128_f32[index];
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<float*>(&vec)[index];
#   elif defined SIXIT_COMPILER_GCC
	return vec[index];
#	else
#	error unsupported compiler
#   endif
}

inline const float* vec4base<float>::const_ptr() const {
#   ifdef SIXIT_COMPILER_MSVC
	return &(vec.m128_f32[0]);
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&vec);
#   elif defined SIXIT_COMPILER_GCC
	return &vec[0];
#	else
#	error unsupported compiler
#   endif
}

template <int i0, int i1, int i2, int i3>
inline vec4base<float> vec4base<float>::swizzle() {
	return vec4base(_mm_shuffle_ps(vec, vec, _MM_SHUFFLE(i3, i2, i1, i0)));
}

inline vec4base<float>& vec4base<float>::operator+=(const vec4base<float>& v) {
	vec = _mm_add_ps(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator-=(const vec4base<float>& v) {
	vec = _mm_sub_ps(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator*=(const vec4base<float>& v) {
	vec = _mm_mul_ps(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator/=(const vec4base<float>& v) {
	vec = _mm_div_ps(vec, v.vec);
	return *this;
}

inline vec4base<float>& vec4base<float>::operator*=(float a) {
	vec = _mm_mul_ps(vec, _mm_set1_ps(a));
	return *this;
}

inline vec4base<float>& vec4base<float>::operator/=(float a) {
	vec = _mm_div_ps(vec, _mm_set1_ps(a));
	return *this;
}

inline void vec4base<float>::set(float a, float b, float c) {
	vec = _mm_setr_ps(a, b, c, 0.0f);
}

inline void vec4base<float>::set(float a, float b, float c, float d) {
	vec = _mm_setr_ps(a, b, c, d);
}

inline float vec4base<float>::dot(const vec4base<float>& a, const vec4base<float>& b) {
	__m128 m0 = _mm_mul_ps(a.vec, b.vec);
	__m128 m1 = _mm_movehl_ps(m0, m0);
	__m128 m2 = _mm_shuffle_ps(m0, m0, 0x55);
	m0 = _mm_add_ps(m0, m1);
	m0 = _mm_add_ps(m0, m2);
	return _mm_cvtss_f32(m0);
}

inline auto vec4base<float>::cross(const vec4base<float>& a, const vec4base<float>& b) {
	__m128 m0 = _mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 m1 = _mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 0, 2, 1));
	m0 = _mm_mul_ps(m0, b.vec);
	m1 = _mm_mul_ps(m1, a.vec);
	m0 = _mm_sub_ps(m1, m0);
	return vec4base<float>(_mm_shuffle_ps(m0, m0, _MM_SHUFFLE(3, 0, 2, 1)));
}

inline vec4base<float> vec4base<float>::max(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_max_ps(a.vec, b.vec));
}

inline vec4base<float> vec4base<float>::min(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_min_ps(a.vec, b.vec));
}

inline vec4base<float> vec4base<float>::abs(const vec4base<float>& v) {
	return vec4base<float>(_mm_and_ps(v.vec, _mm_castsi128_ps(_mm_setr_epi32(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))));
}

inline void vec4base<float>::scale3(float a) {
	vec = _mm_mul_ps(vec, _mm_setr_ps(a, a, a, 1.0f));
}

inline vec4base<float> vec4base<float>::scale3(const vec4base<float>& v, float a) {
	return vec4base<float>(_mm_mul_ps(v.vec, _mm_setr_ps(a, a, a, 1.0f)));
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
	vec = _mm_div_ps(vec, _mm_set1_ps(magnitude()));
}

inline vec4base<float> vec4base<float>::normalized() const {
	return vec4base<float>(_mm_div_ps(vec, _mm_set1_ps(magnitude())));
}

inline void vec4base<float>::scale(float a) {
	vec = _mm_mul_ps(vec, _mm_set1_ps(a));
}

inline vec4base<float> vec4base<float>::scale(const vec4base<float>& v, float a) {
	return vec4base<float>(_mm_mul_ps(v.vec, _mm_set1_ps(a)));
}

template <>
inline vec4base<float> operator+(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_add_ps(a.vec, b.vec));
}

template <>
inline vec4base<float> operator-(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_sub_ps(a.vec, b.vec));
}

template <>
inline vec4base<float> operator*(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_mul_ps(a.vec, b.vec));
}

template <>
inline vec4base<float> operator/(const vec4base<float>& a, const vec4base<float>& b) {
	return vec4base<float>(_mm_div_ps(a.vec, b.vec));
}

template <>
inline vec4base<float> operator*(const vec4base<float>& a, float b) {
	return vec4base<float>(_mm_mul_ps(a.vec, _mm_set1_ps(b)));
}

template <>
inline vec4base<float> operator/(const vec4base<float>& a, float b) {
	return vec4base<float>(_mm_div_ps(a.vec, _mm_set1_ps(b)));
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4base_impl_sse2_h_included

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
