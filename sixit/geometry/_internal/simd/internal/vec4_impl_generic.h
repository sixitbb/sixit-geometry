/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4_impl_generic_h_included
#define sixit_geometry__internal_simd_internal_vec4_impl_generic_h_included

#include <algorithm>

namespace sixit {
namespace geometry {
namespace gpu {

inline vec4::vec4(float a) {
	x = a;
	y = a;
	z = a;
	w = a;
}

inline vec4::vec4(float a, float b, float c) {
	x = a;
	y = b;
	z = c;
	w = 0.0f;
}

inline vec4::vec4(float a, float b, float c, float d) {
	x = a;
	y = b;
	z = c;
	w = d;
}

inline vec4::vec4(const vec4& v) {
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
}

inline vec4& vec4::operator=(const vec4& v) {
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
	return *this;
}

template <int i0, int i1, int i2, int i3>
inline vec4 vec4::swizzle() {
	return vec4(vec[i0], vec[i1], vec[i2], vec[i3]);
}

inline vec4& vec4::operator+=(const vec4& v) {
	x += v.x;
	y += v.y;
	z += v.z;
	w += v.w;
	return *this;
}

inline vec4& vec4::operator-=(const vec4& v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	w -= v.w;
	return *this;
}

inline vec4& vec4::operator*=(const vec4& v) {
	x *= v.x;
	y *= v.y;
	z *= v.z;
	w *= v.w;
	return *this;
}

inline vec4& vec4::operator/=(const vec4& v) {
	x /= v.x;
	y /= v.y;
	z /= v.z;
	w /= v.w;
	return *this;
}

inline vec4& vec4::operator*=(float a) {
	x *= a;
	y *= a;
	z *= a;
	w *= a;
	return *this;
}

inline vec4& vec4::operator/=(float a) {
	x /= a;
	y /= a;
	z /= a;
	w /= a;
	return *this;
}

inline void vec4::set(float a, float b, float c) {
	x = a;
	y = b;
	z = c;
	w = 0.0f;
}

inline void vec4::set(float a, float b, float c, float d) {
	x = a;
	y = b;
	z = c;
	w = d;
}


inline float vec4::dot(const vec4& a, const vec4& b) {
	return	a.x * b.x +
			a.y * b.y +
			a.z * b.z;
}

inline vec4 vec4::cross(const vec4& a, const vec4& b) {
	vec4 r;
	r.x = a.y*b.z - a.z*b.y;
	r.y = a.z*b.x - a.x*b.z;
	r.z = a.x*b.y - a.y*b.x;
	r.w = a.w;
	return r;
}

inline vec4 vec4::max(const vec4& a, const vec4& b) {
	vec4 r;
	r.x = std::max(a.x, b.x);
	r.y = std::max(a.y, b.y);
	r.z = std::max(a.z, b.z);
	r.w = std::max(a.w, b.w);
	return r;
}

inline vec4 vec4::min(const vec4& a, const vec4& b) {
	vec4 r;
	r.x = std::min(a.x, b.x);
	r.y = std::min(a.y, b.y);
	r.z = std::min(a.z, b.z);
	r.w = std::min(a.w, b.w);
	return r;
}

inline vec4 vec4::abs(const vec4& v) {
	return vec4(
			std::abs(v.x),
			std::abs(v.y),
			std::abs(v.z),
			std::abs(v.w));
}

inline void vec4::scale3(float a) {
	x *= a;
	y *= a;
	z *= a;
}

inline vec4 vec4::scale3(const vec4& v, float a) {
	return vec4(
			v.x * a,
			v.y * a,
			v.z * a,
			v.w);
}


inline vec4 operator+(const vec4& a, const vec4& b) {
	return vec4(a.x + b.x,
				a.y + b.y,
				a.z + b.z,
				a.w + b.w);
}

inline vec4 operator-(const vec4& a, const vec4& b) {
	return vec4(a.x - b.x,
				a.y - b.y,
				a.z - b.z,
				a.w - b.w);
}

inline vec4 operator*(const vec4& a, const vec4& b) {
	return vec4(a.x * b.x,
				a.y * b.y,
				a.z * b.z,
				a.w * b.w);
}

inline vec4 operator/(const vec4& a, const vec4& b) {
	return vec4(a.x / b.x,
				a.y / b.y,
				a.z / b.z,
				a.w / b.w);
}

inline vec4 operator*(const vec4& a, float b) {
	return vec4(a.x * b,
				a.y * b,
				a.z * b,
				a.w * b);
}

inline vec4 operator/(const vec4& a, float b) {
	return vec4(a.x / b,
				a.y / b,
				a.z / b,
				a.w / b);
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4_impl_generic_h_included

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