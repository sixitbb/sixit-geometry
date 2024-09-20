/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_mat4_impl_neon_h_included
#define sixit_geometry__internal_simd_internal_mat4_impl_neon_h_included

namespace sixit {
namespace geometry {
namespace gpu {

template<>
inline mat4<float>::mat4(float a) {
	mat[0].vec = (float32x4_t) {a, 0.0f, 0.0f, 0.0f};
	mat[1].vec = (float32x4_t) {0.0f, a, 0.0f, 0.0f};
	mat[2].vec = (float32x4_t) {0.0f, 0.0f, a, 0.0f};
	mat[3].vec = (float32x4_t) {0.0f, 0.0f, 0.0f, a};
}

template<>
inline mat4<float>::mat4(const mat4<float>& m) {
	mat[0].vec = m.mat[0].vec;
	mat[1].vec = m.mat[1].vec;
	mat[2].vec = m.mat[2].vec;
	mat[3].vec = m.mat[3].vec;
}

template<>
inline mat4<float>::mat4(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
	mat[0].vec = (float32x4_t) {m00, m01, m02, m03};
	mat[1].vec = (float32x4_t) {m10, m11, m12, m13};
	mat[2].vec = (float32x4_t) {m20, m21, m22, m23};
	mat[3].vec = (float32x4_t) {m30, m31, m32, m33};
}

template<>
inline mat4<float>::mat4(sfloat4 row0, sfloat4 row1, sfloat4 row2, sfloat4 row3) {
	mat[0].vec = row0;
	mat[1].vec = row1;
	mat[2].vec = row2;
	mat[3].vec = row3;
}

template<>
inline mat4<float>& mat4<float>::operator=(const mat4<float>& m) {
	mat[0].vec = m.mat[0].vec;
	mat[1].vec = m.mat[1].vec;
	mat[2].vec = m.mat[2].vec;
	mat[3].vec = m.mat[3].vec;
	return *this;
}

template<>
inline vec4p<float> operator*(const mat4<float>& m, const vec4p<float>& v) {
	float32x4_t v0 = vmulq_n_f32(m[0].vec, v[0]);
	v0 = vmlaq_n_f32(v0, m[1].vec, v[1]);
	v0 = vmlaq_n_f32(v0, m[2].vec, v[2]);
	v0 = vmlaq_n_f32(v0, m[3].vec, v[3]);
	return vec4p<float>(v0);
}

template<>
inline mat4<float> operator*(const mat4<float>& m1, const mat4<float>& m2) {
	mat4<float> r;
	r[0].vec = vmulq_n_f32(m1[0].vec, vgetq_lane_f32(m2[0].vec, 0));
	r[1].vec = vmulq_n_f32(m1[0].vec, vgetq_lane_f32(m2[1].vec, 0));
	r[2].vec = vmulq_n_f32(m1[0].vec, vgetq_lane_f32(m2[2].vec, 0));
	r[3].vec = vmulq_n_f32(m1[0].vec, vgetq_lane_f32(m2[3].vec, 0));
	r[0].vec = vmlaq_n_f32(r[0].vec, m1[1].vec, vgetq_lane_f32(m2[0].vec, 1));
	r[1].vec = vmlaq_n_f32(r[1].vec, m1[1].vec, vgetq_lane_f32(m2[1].vec, 1));
	r[2].vec = vmlaq_n_f32(r[2].vec, m1[1].vec, vgetq_lane_f32(m2[2].vec, 1));
	r[3].vec = vmlaq_n_f32(r[3].vec, m1[1].vec, vgetq_lane_f32(m2[3].vec, 1));
	r[0].vec = vmlaq_n_f32(r[0].vec, m1[2].vec, vgetq_lane_f32(m2[0].vec, 2));
	r[1].vec = vmlaq_n_f32(r[1].vec, m1[2].vec, vgetq_lane_f32(m2[1].vec, 2));
	r[2].vec = vmlaq_n_f32(r[2].vec, m1[2].vec, vgetq_lane_f32(m2[2].vec, 2));
	r[3].vec = vmlaq_n_f32(r[3].vec, m1[2].vec, vgetq_lane_f32(m2[3].vec, 2));
	r[0].vec = vmlaq_n_f32(r[0].vec, m1[3].vec, vgetq_lane_f32(m2[0].vec, 3));
	r[1].vec = vmlaq_n_f32(r[1].vec, m1[3].vec, vgetq_lane_f32(m2[1].vec, 3));
	r[2].vec = vmlaq_n_f32(r[2].vec, m1[3].vec, vgetq_lane_f32(m2[2].vec, 3));
	r[3].vec = vmlaq_n_f32(r[3].vec, m1[3].vec, vgetq_lane_f32(m2[3].vec, 3));
	return r;
}

template<>
inline mat4<float> operator+(const mat4<float>& m1, const mat4<float>& m2) {
	mat4<float> r;
	r[0].vec = vaddq_f32(m1[0].vec, m2[0].vec);
	r[1].vec = vaddq_f32(m1[1].vec, m2[1].vec);
	r[2].vec = vaddq_f32(m1[2].vec, m2[2].vec);
	r[3].vec = vaddq_f32(m1[3].vec, m2[3].vec);
	return r;
}

template<>
inline mat4<float> operator-(const mat4<float>& m1, const mat4<float>& m2) {
	mat4<float> r;
	r[0].vec = vsubq_f32(m1[0].vec, m2[0].vec);
	r[1].vec = vsubq_f32(m1[1].vec, m2[1].vec);
	r[2].vec = vsubq_f32(m1[2].vec, m2[2].vec);
	r[3].vec = vsubq_f32(m1[3].vec, m2[3].vec);
	return r;
}

template<>
inline mat4<float> operator*(const mat4<float>& m, float a) {
	mat4<float> r;
	r[0].vec = vmulq_n_f32(m[0].vec, a);
	r[1].vec = vmulq_n_f32(m[1].vec, a);
	r[2].vec = vmulq_n_f32(m[2].vec, a);
	r[3].vec = vmulq_n_f32(m[3].vec, a);
	return r;
}

template<>
inline mat4<float> operator/(const mat4<float>& m, float a) {
	mat4<float> r;
	r[0].vec = internal::__vdivq_n_f32(m[0].vec, a);
	r[1].vec = internal::__vdivq_n_f32(m[1].vec, a);
	r[2].vec = internal::__vdivq_n_f32(m[2].vec, a);
	r[3].vec = internal::__vdivq_n_f32(m[3].vec, a);
	return r;
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_mat4_impl_neon_h_included

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