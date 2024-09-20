/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_mat4_impl_generic_h_included
#define sixit_geometry__internal_simd_internal_mat4_impl_generic_h_included

#include <cstring>

namespace sixit {
namespace geometry {
namespace gpu {


template <typename fp>
inline mat4<fp>::mat4(fp a) {
	mat[0] = vec4p<fp>(a, 0.0f, 0.0f, 0.0f);
	mat[1] = vec4p<fp>(0.0f, a, 0.0f, 0.0f);
	mat[2] = vec4p<fp>(0.0f, 0.0f, a, 0.0f);
	mat[3] = vec4p<fp>(0.0f, 0.0f, 0.0f, a);
}

template <typename fp>
inline mat4<fp>::mat4(const mat4<fp>& m) {
	std::memcpy(mat, m.mat, sizeof(mat));
}


template <typename fp>
inline mat4<fp>::mat4(fp m00, fp m01, fp m02, fp m03, fp m10, fp m11, fp m12, fp m13,
		fp m20, fp m21, fp m22, fp m23, fp m30, fp m31, fp m32, fp m33) {
	mat[0] = vec4p(m00, m01, m02, m03);
	mat[1] = vec4p(m10, m11, m12, m13);
	mat[2] = vec4p(m20, m21, m22, m23);
	mat[3] = vec4p(m30, m31, m32, m33);
}

template <typename fp>
inline mat4<fp>& mat4<fp>::operator=(const mat4<fp>& m) {
	std::memcpy(mat, m.mat, sizeof(mat));
	return *this;
}

template <typename fp>
inline vec4p<fp> operator*(const mat4<fp>& m, const vec4p<fp>& v) {
	vec4p<fp> r;
	r.x() = v.x() * m[0][0] + v.y() * m[1][0] + v.z() * m[2][0] + v.w() * m[3][0];
	r.y() = v.x() * m[0][1] + v.y() * m[1][1] + v.z() * m[2][1] + v.w() * m[3][1];
	r.z() = v.x() * m[0][2] + v.y() * m[1][2] + v.z() * m[2][2] + v.w() * m[3][2];
	r.w() = v.x() * m[0][3] + v.y() * m[1][3] + v.z() * m[2][3] + v.w() * m[3][3];
	return r;
}

template <typename fp>
inline mat4<fp> operator*(const mat4<fp>& m1, const mat4<fp>& m2) {
	mat4<fp> r;

	for (int i = 0; i < 4; i++) {
		r[i][0] = m1[0][0] * m2[i][0] + m1[1][0] * m2[i][1] + m1[2][0] * m2[i][2] + m1[3][0] * m2[i][3];
		r[i][1] = m1[0][1] * m2[i][0] + m1[1][1] * m2[i][1] + m1[2][1] * m2[i][2] + m1[3][1] * m2[i][3];
		r[i][2] = m1[0][2] * m2[i][0] + m1[1][2] * m2[i][1] + m1[2][2] * m2[i][2] + m1[3][2] * m2[i][3];
		r[i][3] = m1[0][3] * m2[i][0] + m1[1][3] * m2[i][1] + m1[2][3] * m2[i][2] + m1[3][3] * m2[i][3];
	}

	return r;
}

template <typename fp>
inline mat4<fp> operator+(const mat4<fp>& m1, const mat4<fp>& m2) {
	mat4<fp> r;
	r[0] = m1[0] + m2[0];
	r[1] = m1[1] + m2[1];
	r[2] = m1[2] + m2[2];
	r[3] = m1[3] + m2[3];
	return r;
}

template <typename fp>
inline mat4<fp> operator-(const mat4<fp>& m1, const mat4<fp>& m2) {
	mat4<fp> r;
	r[0] = m1[0] - m2[0];
	r[1] = m1[1] - m2[1];
	r[2] = m1[2] - m2[2];
	r[3] = m1[3] - m2[3];
	return r;
}

template <typename fp>
inline mat4<fp> operator*(const mat4<fp>& m, fp a) {
	mat4<fp> r;
	r[0] = m[0] * a;
	r[1] = m[1] * a;
	r[2] = m[2] * a;
	r[3] = m[3] * a;
	return r;
}

template <typename fp>
inline mat4<fp> operator/(const mat4<fp>& m, fp a) {
	mat4<fp> r;
	r[0] = m[0] / a;
	r[1] = m[1] / a;
	r[2] = m[2] / a;
	r[3] = m[3] / a;
	return r;
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_mat4_impl_generic_h_included

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