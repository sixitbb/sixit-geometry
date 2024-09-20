/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4_impl_sse2_h_included
#define sixit_geometry__internal_simd_internal_vec4_impl_sse2_h_included

namespace sixit {
namespace geometry {
namespace gpu {

inline vec4::vec4(float a) {
	svec = _mm_set1_ps(a);
}

inline vec4::vec4(float a, float b, float c) {
	svec = _mm_setr_ps(a, b, c, 0.0f);
}

inline vec4::vec4(float a, float b, float c, float d) {
	svec = _mm_setr_ps(a, b, c, d);
}

inline vec4::vec4(const vec4& v) {
	svec = v.svec;
}

inline vec4::vec4(sfloat4 sv) {
	svec = sv;
}

inline vec4& vec4::operator=(const vec4& v) {
	svec = v.svec;
	return *this;
}

template <int i0, int i1, int i2, int i3>
inline vec4 vec4::swizzle() {
	return vec4(_mm_shuffle_ps(svec, svec, _MM_SHUFFLE(i3, i2, i1, i0)));
}

inline vec4& vec4::operator+=(const vec4& v) {
	svec = _mm_add_ps(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator-=(const vec4& v) {
	svec = _mm_sub_ps(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator*=(const vec4& v) {
	svec = _mm_mul_ps(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator/=(const vec4& v) {
	svec = _mm_div_ps(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator*=(float a_) {
	svec = _mm_mul_ps(svec, _mm_set1_ps(a_));
	return *this;
}

inline vec4& vec4::operator/=(float a_) {
	svec = _mm_div_ps(svec, _mm_set1_ps(a_));
	return *this;
}

inline void vec4::set(float a_, float b_, float c_) {
	svec = _mm_setr_ps(a_, b_, c_, 0.0f);
}

inline void vec4::set(float a_, float b_, float c_, float d_) {
	svec = _mm_setr_ps(a_, b_, c_, d_);
}


inline float vec4::dot(const vec4& a, const vec4& b) {
	__m128 m0 = _mm_mul_ps(a.svec, b.svec);
	__m128 m1 = _mm_movehl_ps(m0, m0);
	__m128 m2 = _mm_shuffle_ps(m0, m0, 0x55);
	m0 = _mm_add_ps(m0, m1);
	m0 = _mm_add_ps(m0, m2);
	return _mm_cvtss_f32(m0);
}

inline vec4 vec4::cross(const vec4& a, const vec4& b) {
	__m128 m0 = _mm_shuffle_ps(a.svec, a.svec, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 m1 = _mm_shuffle_ps(b.svec, b.svec, _MM_SHUFFLE(3, 0, 2, 1));
	m0 = _mm_mul_ps(m0, b.svec);
	m1 = _mm_mul_ps(m1, a.svec);
	m0 = _mm_sub_ps(m1, m0);
	return vec4(_mm_shuffle_ps(m0, m0, _MM_SHUFFLE(3, 0, 2, 1)));
}

inline vec4 vec4::max(const vec4& a, const vec4& b) {
	return vec4(_mm_max_ps(a.svec, b.svec));
}

inline vec4 vec4::min(const vec4& a, const vec4& b) {
	return vec4(_mm_min_ps(a.svec, b.svec));
}

inline vec4 vec4::abs(const vec4& v) {
	return vec4(_mm_and_ps(v.svec, _mm_castsi128_ps(_mm_setr_epi32(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))));
}

inline void vec4::scale3(float a_) {
	svec = _mm_mul_ps(svec, _mm_setr_ps(a_, a_, a_, 1.0f));
}

inline vec4 vec4::scale3(const vec4& v, float a) {
	return vec4(_mm_mul_ps(v.svec, _mm_setr_ps(a, a, a, 1.0f)));
}


inline vec4 operator+(const vec4& a, const vec4& b) {
	return vec4(_mm_add_ps(a.svec, b.svec));
}

inline vec4 operator-(const vec4& a, const vec4& b) {
	return vec4(_mm_sub_ps(a.svec, b.svec));
}

inline vec4 operator*(const vec4& a, const vec4& b) {
	return vec4(_mm_mul_ps(a.svec, b.svec));
}

inline vec4 operator/(const vec4& a, const vec4& b) {
	return vec4(_mm_div_ps(a.svec, b.svec));
}

inline vec4 operator*(const vec4& a, float b) {
	return vec4(_mm_mul_ps(a.svec, _mm_set1_ps(b)));
}

inline vec4 operator/(const vec4& a, float b) {
	return vec4(_mm_div_ps(a.svec, _mm_set1_ps(b)));
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4_impl_sse2_h_included

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