/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4base_impl_generic_h_included
#define sixit_geometry__internal_simd_internal_vec4base_impl_generic_h_included

#include <algorithm>
#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
inline const fp& vec4base<fp>::operator[](unsigned index) const {
	return vec_fp[index];
}

template <typename fp>
inline fp& vec4base<fp>::operator[](unsigned index) {
	return vec_fp[index];
}

template <typename fp>
inline const fp* vec4base<fp>::const_ptr() const{
	return vec_fp;
}

template <typename fp>
inline vec4base<fp>::vec4base(fp a) {
	vec_fp[0] = a;
	vec_fp[1] = a;
	vec_fp[2] = a;
	vec_fp[3] = a;
}

template <typename fp>
inline vec4base<fp>::vec4base(fp a, fp b, fp c) {
	vec_fp[0] = a;
	vec_fp[1] = b;
	vec_fp[2] = c;
	vec_fp[3] = 0.0f;
}

template <typename fp>
inline vec4base<fp>::vec4base(fp a, fp b, fp c, fp d) {
	vec_fp[0] = a;
	vec_fp[1] = b;
	vec_fp[2] = c;
	vec_fp[3] = d;
}

template <typename fp>
inline vec4base<fp>::vec4base(const vec4base& v) {
	vec_fp[0] = v.vec_fp[0];
	vec_fp[1] = v.vec_fp[1];
	vec_fp[2] = v.vec_fp[2];
	vec_fp[3] = v.vec_fp[3];
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator=(const vec4base& v) {
	vec_fp[0] = v.vec_fp[0];
	vec_fp[1] = v.vec_fp[1];
	vec_fp[2] = v.vec_fp[2];
	vec_fp[3] = v.vec_fp[3];
	return *this;
}

template <typename fp>
template <int i0, int i1, int i2, int i3>
inline vec4base<fp> vec4base<fp>::swizzle() {
	return vec4base(vec_fp[i0], vec_fp[i1], vec_fp[i2], vec_fp[i3]);
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator+=(const vec4base<fp>& v) {
	vec_fp[0] = vec_fp[0] + v.vec_fp[0];
	vec_fp[1] = vec_fp[1] + v.vec_fp[1];
	vec_fp[2] = vec_fp[2] + v.vec_fp[2];
	vec_fp[3] = vec_fp[3] + v.vec_fp[3];
	return *this;
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator-=(const vec4base<fp>& v) {
	vec_fp[0] = vec_fp[0] - v.vec_fp[0];
	vec_fp[1] = vec_fp[1] - v.vec_fp[1];
	vec_fp[2] = vec_fp[2] - v.vec_fp[2];
	vec_fp[3] = vec_fp[3] - v.vec_fp[3];
	return *this;
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator*=(const vec4base<fp>& v) {
	vec_fp[0] = vec_fp[0] * v.vec_fp[0];
	vec_fp[1] = vec_fp[1] * v.vec_fp[1];
	vec_fp[2] = vec_fp[2] * v.vec_fp[2];
	vec_fp[3] = vec_fp[3] * v.vec_fp[3];
	return *this;
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator/=(const vec4base<fp>& v) {
	vec_fp[0] = vec_fp[0] / v.vec_fp[0];
	vec_fp[1] = vec_fp[1] / v.vec_fp[1];
	vec_fp[2] = vec_fp[2] / v.vec_fp[2];
	vec_fp[3] = vec_fp[3] / v.vec_fp[3];
	return *this;
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator*=(fp a) {
	vec_fp[0] = vec_fp[0] * a;
	vec_fp[1] = vec_fp[1] * a;
	vec_fp[2] = vec_fp[2] * a;
	vec_fp[3] = vec_fp[3] * a;
	return *this;
}

template <typename fp>
inline vec4base<fp>& vec4base<fp>::operator/=(fp a) {
	vec_fp[0] = vec_fp[0] / a;
	vec_fp[1] = vec_fp[1] / a;
	vec_fp[2] = vec_fp[2] / a;
	vec_fp[3] = vec_fp[3] / a;
	return *this;
}

template <typename fp>
inline void vec4base<fp>::set(fp a, fp b, fp c) {
	vec_fp[0] = a;
	vec_fp[1] = b;
	vec_fp[2] = c;
	vec_fp[3] = 0.0f;
}

template <typename fp>
inline void vec4base<fp>::set(fp a, fp b, fp c, fp d) {
	vec_fp[0] = a;
	vec_fp[1] = b;
	vec_fp[2] = c;
	vec_fp[3] = d;
}

template <typename fp>
inline fp vec4base<fp>::dot(const vec4base& a, const vec4base& b) {
	return	a.vec_fp[0] * b.vec_fp[0] +
			a.vec_fp[1] * b.vec_fp[1] +
			a.vec_fp[2] * b.vec_fp[2];
}

template <typename fp>
inline auto vec4base<fp>::cross(const vec4base<fp>& a, const vec4base<fp>& b) {
	auto tv = a.vec_fp[1] * b.vec_fp[2];
	auto fp_1 = fp(1.f);
	vec4base<decltype(tv)> r;
	r.vec_fp[0] = a.vec_fp[1] * b.vec_fp[2] - a.vec_fp[2] * b.vec_fp[1];
	r.vec_fp[1] = a.vec_fp[2] * b.vec_fp[0] - a.vec_fp[0] * b.vec_fp[2];
	r.vec_fp[2] = a.vec_fp[0] * b.vec_fp[1] - a.vec_fp[1] * b.vec_fp[0];
	r.vec_fp[3] = a.vec_fp[3] * fp_1;
	return r;
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::max(const vec4base<fp>& a, const vec4base<fp>& b) {
	vec4base<fp> r;
	r.vec_fp[0] = std::max(a.vec_fp[0], b.vec_fp[0]);
	r.vec_fp[1] = std::max(a.vec_fp[1], b.vec_fp[1]);
	r.vec_fp[2] = std::max(a.vec_fp[2], b.vec_fp[2]);
	r.vec_fp[3] = std::max(a.vec_fp[3], b.vec_fp[3]);
	return r;
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::min(const vec4base<fp>& a, const vec4base<fp>& b) {
	vec4base<fp> r;
	r.vec_fp[0] = std::min(a.vec_fp[0], b.vec_fp[0]);
	r.vec_fp[1] = std::min(a.vec_fp[1], b.vec_fp[1]);
	r.vec_fp[2] = std::min(a.vec_fp[2], b.vec_fp[2]);
	r.vec_fp[3] = std::min(a.vec_fp[3], b.vec_fp[3]);
	return r;
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::abs(const vec4base<fp>& v) {
	return vec4base<fp>(
			sixit::geometry::low_level::mathf::abs(v.vec_fp[0]),
			sixit::geometry::low_level::mathf::abs(v.vec_fp[1]),
			sixit::geometry::low_level::mathf::abs(v.vec_fp[2]),
			sixit::geometry::low_level::mathf::abs(v.vec_fp[3]));
}

template <typename fp>
inline void vec4base<fp>::scale3(fp a) {
	vec_fp[0] *= a;
	vec_fp[1] *= a;
	vec_fp[2] *= a;
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::scale3(const vec4base<fp>& v, fp a) {
	return vec4base<fp>(
			v.vec_fp[0] * a,
			v.vec_fp[1] * a,
			v.vec_fp[2] * a,
			v.vec_fp[3]);
}

template <typename fp>
inline vec4base<fp> operator+(const vec4base<fp>& a, const vec4base<fp>& b) {
	return vec4base<fp>(a.vec_fp[0] + b.vec_fp[0],
					a.vec_fp[1] + b.vec_fp[1],
					a.vec_fp[2] + b.vec_fp[2],
					a.vec_fp[3] + b.vec_fp[3]);
}

template <typename fp>
inline vec4base<fp> operator-(const vec4base<fp>& a, const vec4base<fp>& b) {
	return vec4base<fp>(a.vec_fp[0] - b.vec_fp[0],
					a.vec_fp[1] - b.vec_fp[1],
					a.vec_fp[2] - b.vec_fp[2],
					a.vec_fp[3] - b.vec_fp[3]);
}

template <typename fp>
inline vec4base<fp> operator*(const vec4base<fp>& a, const vec4base<fp>& b) {
	return vec4base<fp>(a.vec_fp[0] * b.vec_fp[0],
					a.vec_fp[1] * b.vec_fp[1],
					a.vec_fp[2] * b.vec_fp[2],
					a.vec_fp[3] * b.vec_fp[3]);
}

template <typename fp>
inline vec4base<fp> operator/(const vec4base<fp>& a, const vec4base<fp>& b) {
	return vec4base<fp>(a.vec_fp[0] / b.vec_fp[0],
					a.vec_fp[1] / b.vec_fp[1],
					a.vec_fp[2] / b.vec_fp[2],
					a.vec_fp[3] / b.vec_fp[3]);
}

template <typename fp>
inline vec4base<fp> operator*(const vec4base<fp>& a, fp b) {
	return vec4base<fp>(a.vec_fp[0] * b,
					a.vec_fp[1] * b,
					a.vec_fp[2] * b,
					a.vec_fp[3] * b);
}

template <typename fp>
inline vec4base<fp> operator/(const vec4base<fp>& a, fp b) {
	return vec4base<fp>(a.vec_fp[0] / b,
					a.vec_fp[1] / b,
					a.vec_fp[2] / b,
					a.vec_fp[3] / b);
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4base_impl_generic_h_included

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
