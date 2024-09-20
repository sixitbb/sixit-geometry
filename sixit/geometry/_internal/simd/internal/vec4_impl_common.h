/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4_impl_common_h_included
#define sixit_geometry__internal_simd_internal_vec4_impl_common_h_included

namespace sixit {
namespace geometry {
namespace gpu {

inline bool vec4::operator==(const vec4& v) const {
	return	x == v.x && y == v.y && z == v.z && w == v.w;
}

inline float vec4::angle(const vec4& a, const vec4& b) {
	float c = dot(a, b);
	return std::acos(c);
}

inline float vec4::magnitude() const {
	return std::sqrt(sqr_magnitude());
}

inline float vec4::sqr_magnitude() const {
	return dot(*this, *this);
}

inline void vec4::normalize() {
	*this /= magnitude();
}

inline vec4 vec4::normalized() const {
	return *this / magnitude();
}

inline void vec4::scale(float a_) {
	*this *= a_;
}

inline vec4 vec4::scale(const vec4& v, float a) {
	return vec4(v * a);
}

inline vec4 vec4::forward() {
	return vec4(0.0f, 0.0f, -1.0f);
}

inline vec4 vec4::back() {
	return vec4(0.0f, 0.0f, 1.0f);
}

inline vec4 vec4::up() {
	return vec4(0.0f, 1.0f, 0.0f);
}

inline vec4 vec4::down() {
	return vec4(0.0f, -1.0f, 0.0f);
}

inline vec4 vec4::left() {
	return vec4(-1.0f, 0.0f, 0.0f);
}

inline vec4 vec4::right() {
	return vec4(1.0f, 0.0f, 0.0f);
}

inline vec4 vec4::one() {
	return vec4(1.0f);
}

inline vec4 vec4::zero() {
	return vec4(0.0f);
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4_impl_common_h_included

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