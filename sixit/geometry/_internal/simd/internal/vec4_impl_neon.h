/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4_impl_neon_h_included
#define sixit_geometry__internal_simd_internal_vec4_impl_neon_h_included

namespace sixit {
namespace geometry {
namespace gpu {

namespace internal {
// Uncomment next line to use inaccurate but fast division on ARMv7
// #define __ARMV7_NEON_FAST_DIV__

inline float32x4_t __vdivq_f32(float32x4_t a, float32x4_t b) {
#if defined(__aarch64__)
	return vdivq_f32(a, b);
#elif defined(__ARMV7_NEON_FAST_DIV__)
	// Division by multiplying by reciprocal value of b with medium precision
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

inline vec4::vec4(float a) {
	svec = vdupq_n_f32(a);
}

inline vec4::vec4(float a, float b, float c) {
	svec = (float32x4_t) {a, b, c, 0};
}

inline vec4::vec4(float a, float b, float c, float d) {
	svec = (float32x4_t) {a, b, c, d};
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
	return vec4(vec[i0], vec[i1], vec[i2], vec[i3]);
}

inline vec4& vec4::operator+=(const vec4& v) {
	svec = vaddq_f32(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator-=(const vec4& v) {
	svec = vsubq_f32(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator*=(const vec4& v) {
	svec = vmulq_f32(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator/=(const vec4& v) {
	svec = internal::__vdivq_f32(svec, v.svec);
	return *this;
}

inline vec4& vec4::operator*=(float a) {
	svec = vmulq_n_f32(svec, a);
	return *this;
}

inline vec4& vec4::operator/=(float a) {
	svec = internal::__vdivq_n_f32(svec, a);
	return *this;
}

inline void vec4::set(float a, float b, float c) {
	svec = (float32x4_t) {a, b, c, 0.0f};
}

inline void vec4::set(float a, float b, float c, float d) {
	svec = (float32x4_t) {a, b, c, d};
}


inline float vec4::dot(const vec4& a, const vec4& b) {
	float32x4_t vd = vmulq_f32(a.svec, b.svec);
	float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_low_f32(vd));
	x = vadd_f32(x, vget_high_f32(vd));
	return vget_lane_f32(x, 0);
}

inline vec4 vec4::cross(const vec4& a, const vec4& b) {
	float32x4_t t, v;
	float32x2_t t_low = vget_low_f32(a.svec);
	float32x2_t v_low = vget_low_f32(b.svec);
	t = vcombine_f32(vext_f32(t_low, vget_high_f32(a.svec), 1), t_low);
	v = vcombine_f32(vext_f32(v_low, vget_high_f32(b.svec), 1), v_low);
	v = vmulq_f32(v, a.svec);
	t = vmulq_f32(t, b.svec);
	v = vsubq_f32(v, t);
	v_low = vget_low_f32(v);
	v = vcombine_f32(vext_f32(v_low, vget_high_f32(v), 1), v_low);
	v = (float32x4_t)vandq_s32((int32x4_t)v, (int32x4_t){-1,-1,-1,0});
	return vec4(v);
}

inline vec4 vec4::max(const vec4& a, const vec4& b) {
	return vec4(vmaxq_f32(a.svec, b.svec));
}

inline vec4 vec4::min(const vec4& a, const vec4& b) {
	return vec4(vminq_f32(a.svec, b.svec));
}

inline vec4 vec4::abs(const vec4& v) {
	return vec4((float32x4_t) vandq_s32((int32x4_t) v.svec, vdupq_n_s32(0x7FFFFFFF)));
}

inline void vec4::scale3(float a) {
	svec = vmulq_f32(svec, (float32x4_t) {a, a, a, 1.0f});
}

inline vec4 vec4::scale3(const vec4& v, float a) {
	return vec4(vmulq_f32(v.svec, (float32x4_t) {a, a, a, 1.0f}));
}


inline vec4 operator+(const vec4& a, const vec4& b) {
	return vec4(vaddq_f32(a.svec, b.svec));
}

inline vec4 operator-(const vec4& a, const vec4& b) {
	return vec4(vsubq_f32(a.svec, b.svec));
}

inline vec4 operator*(const vec4& a, const vec4& b) {
	return vec4(vmulq_f32(a.svec, b.svec));
}

inline vec4 operator/(const vec4& a, const vec4& b) {
	return vec4(internal::__vdivq_f32(a.svec, b.svec));
}

inline vec4 operator*(const vec4& a, float b) {
	return vec4(vmulq_n_f32(a.svec, b));
}

inline vec4 operator/(const vec4& a, float b) {
	return vec4(internal::__vdivq_n_f32(a.svec, b));
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4_impl_neon_h_included

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