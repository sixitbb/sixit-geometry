/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_mat4_impl_common_h_included
#define sixit_geometry__internal_simd_internal_mat4_impl_common_h_included

#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
inline vec4p<fp> mat4<fp>::row(unsigned n) const {
	return vec4p<fp>(mat[0][n], mat[1][n], mat[2][n], mat[3][n]);
}

template <typename fp>
inline mat4<fp> mat4<fp>::translate(const vec4p<fp>& v) {
	mat4<fp> m(1.0f);
	m[3][0] = v.x();
	m[3][1] = v.y();
	m[3][2] = v.z();
	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::rotate(fp a, const vec4p<fp>& axis) {
	mat4<fp> m(1.0f);
	vec4p axis_norm = axis.normalized();

	fp cos_ = sixit::geometry::low_level::mathf::cos(a);
	fp sin_ = sixit::geometry::low_level::mathf::sin(a);
	fp rcos_ = fp(1.f) - cos_;

	m[0][0] = rcos_ * axis_norm.x() * axis_norm.x() + cos_;
	m[0][1] = rcos_ * axis_norm.x() * axis_norm.y() + sin_ * axis_norm.z();
	m[0][2] = rcos_ * axis_norm.x() * axis_norm.z() - sin_ * axis_norm.y();

	m[1][0] = rcos_ * axis_norm.x() * axis_norm.y() - sin_ * axis_norm.z();
	m[1][1] = rcos_ * axis_norm.y() * axis_norm.y() + cos_;
	m[1][2] = rcos_ * axis_norm.y() * axis_norm.z() + sin_ * axis_norm.x();

	m[2][0] = rcos_ * axis_norm.x() * axis_norm.z() + sin_ * axis_norm.y();
	m[2][1] = rcos_ * axis_norm.y() * axis_norm.z() - sin_ * axis_norm.x();
	m[2][2] = rcos_ * axis_norm.z() * axis_norm.z() + cos_;

	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::rotate(const quat4<fp>& q) {
	mat4<fp> m(1.0f);

	m[0][0] = fp(1.0f) - fp(2.0f) * (q.y() * q.y() + q.z() * q.z());
	m[0][1] = fp(2.0f) * (q.x() * q.y() + q.z() * q.w());
	m[0][2] = fp(2.0f) * (q.x() * q.z() - q.y() * q.w());

	m[1][0] = fp(2.0f) * (q.x() * q.y() - q.z() * q.w());
	m[1][1] = fp(1.0f) - fp(2.0f) * (q.x() * q.x() + q.z() * q.z());
	m[1][2] = fp(2.0f) * (q.y() * q.z() + q.x() * q.w());

	m[2][0] = fp(2.0f) * (q.x() * q.z() + q.y() * q.w());
	m[2][1] = fp(2.0f) * (q.y() * q.z() - q.x() * q.w());
	m[2][2] = fp(1.0f) - fp(2.0f) * (q.x() * q.x() + q.y() * q.y());

	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::scale(const vec4p<fp>& v) {
	mat4<fp> m(1.0f);
	m[0][0] = v.x();
	m[1][1] = v.y();
	m[2][2] = v.z();
	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::compose(const vec4p<fp>& t, const quat4<fp>& r, const vec4p<fp>& s) {
	mat4<fp> m = mat4<fp>::rotate(r);
	m[0] *= s.x();
	m[1] *= s.y();
	m[2] *= s.z();
	m[3] = t;
	m[3][3] = 1.0f;
	return m;
}

template <typename fp>
inline void mat4<fp>::decompose(const mat4<fp>& m, vec4p<fp>& t, quat4<fp>& r, vec4p<fp>& s) {
	mat4<fp> tm(m);
	fp d = determinant(tm);
	t = tm[3];

	s.x() = tm[0].magnitude();
	s.y() = tm[1].magnitude();
	s.z() = tm[2].magnitude();

	if (d < 0) {
		s *= -1.0f;
	}

	t.w() = 1.0f;
	s.w() = 0.0f;

	tm[0] /= s.x();
	tm[1] /= s.y();
	tm[2] /= s.z();

	fp di = tm[0][0] + tm[1][1] + tm[2][2];

	if (di > 0.0f) {
		fp f = std::sqrt(1 + di) * 2.0f;
		r.x() = (tm[1][2] - tm[2][1]) / f;
		r.y() = (tm[2][0] - tm[0][2]) / f;
		r.z() = (tm[0][1] - tm[1][0]) / f;
		r.w() = 0.25f * f;
	}
	else if (tm[0][0] > tm[1][1] && tm[0][0] > tm[2][2]) {
		fp f = std::sqrt(1 + tm[0][0] - tm[1][1] - tm[2][2]) * 2.0f;
		r.x() = 0.25f * f;
		r.y() = (tm[0][1] + tm[1][0]) / f;
		r.z() = (tm[2][0] + tm[0][2]) / f;
		r.w() = (tm[1][2] - tm[2][1]) / f;
	}
	else if (tm[1][1] > tm[2][2]) {
		fp f = std::sqrt(1 + tm[1][1] - tm[0][0] - tm[2][2]) * 2.0f;
		r.x() = (tm[0][1] + tm[1][0]) / f;
		r.y() = 0.25f * f;
		r.z() = (tm[1][2] + tm[2][1]) / f;
		r.w() = (tm[2][0] - tm[0][2]) / f;
	}
	else {
		fp f = std::sqrt(1 + tm[2][2] - tm[0][0] - tm[1][1]) * 2.0f;
		r.x() = (tm[2][0] + tm[0][2]) / f;
		r.y() = (tm[1][2] + tm[2][1]) / f;
		r.z() = 0.25f * f;
		r.w() = (tm[0][1] - tm[1][0]) / f;
	}
}

template <typename fp>
inline mat4<fp> mat4<fp>::inverse(const mat4<fp>& m) {
	mat4<fp> r;
	fp d;

	r[0][0] = m[1][1] * m[2][2] * m[3][3] -
			m[1][1] * m[2][3] * m[3][2] -
			m[2][1] * m[1][2] * m[3][3] +
			m[2][1] * m[1][3] * m[3][2] +
			m[3][1] * m[1][2] * m[2][3] -
			m[3][1] * m[1][3] * m[2][2];

	r[1][0] = -m[1][0] * m[2][2] * m[3][3] +
			m[1][0] * m[2][3] * m[3][2] +
			m[2][0] * m[1][2] * m[3][3] -
			m[2][0] * m[1][3] * m[3][2] -
			m[3][0] * m[1][2] * m[2][3] +
			m[3][0] * m[1][3] * m[2][2];

	r[2][0] = m[1][0] * m[2][1] * m[3][3] -
			m[1][0] * m[2][3] * m[3][1] -
			m[2][0] * m[1][1] * m[3][3] +
			m[2][0] * m[1][3] * m[3][1] +
			m[3][0] * m[1][1] * m[2][3] -
			m[3][0] * m[1][3] * m[2][1];

	r[3][0] = -m[1][0] * m[2][1] * m[3][2] +
			m[1][0] * m[2][2] * m[3][1] +
			m[2][0] * m[1][1] * m[3][2] -
			m[2][0] * m[1][2] * m[3][1] -
			m[3][0] * m[1][1] * m[2][2] +
			m[3][0] * m[1][2] * m[2][1];

	r[0][1] = -m[0][1] * m[2][2] * m[3][3] +
			m[0][1] * m[2][3] * m[3][2] +
			m[2][1] * m[0][2] * m[3][3] -
			m[2][1] * m[0][3] * m[3][2] -
			m[3][1] * m[0][2] * m[2][3] +
			m[3][1] * m[0][3] * m[2][2];

	r[1][1] = m[0][0] * m[2][2] * m[3][3] -
			m[0][0] * m[2][3] * m[3][2] -
			m[2][0] * m[0][2] * m[3][3] +
			m[2][0] * m[0][3] * m[3][2] +
			m[3][0] * m[0][2] * m[2][3] -
			m[3][0] * m[0][3] * m[2][2];

	r[2][1] = -m[0][0] * m[2][1] * m[3][3] +
			m[0][0] * m[2][3] * m[3][1] +
			m[2][0] * m[0][1] * m[3][3] -
			m[2][0] * m[0][3] * m[3][1] -
			m[3][0] * m[0][1] * m[2][3] +
			m[3][0] * m[0][3] * m[2][1];

	r[3][1] = m[0][0] * m[2][1] * m[3][2] -
			m[0][0] * m[2][2] * m[3][1] -
			m[2][0] * m[0][1] * m[3][2] +
			m[2][0] * m[0][2] * m[3][1] +
			m[3][0] * m[0][1] * m[2][2] -
			m[3][0] * m[0][2] * m[2][1];

	r[0][2] = m[0][1] * m[1][2] * m[3][3] -
			m[0][1] * m[1][3] * m[3][2] -
			m[1][1] * m[0][2] * m[3][3] +
			m[1][1] * m[0][3] * m[3][2] +
			m[3][1] * m[0][2] * m[1][3] -
			m[3][1] * m[0][3] * m[1][2];

	r[1][2] = -m[0][0] * m[1][2] * m[3][3] +
			m[0][0] * m[1][3] * m[3][2] +
			m[1][0] * m[0][2] * m[3][3] -
			m[1][0] * m[0][3] * m[3][2] -
			m[3][0] * m[0][2] * m[1][3] +
			m[3][0] * m[0][3] * m[1][2];

	r[2][2] = m[0][0] * m[1][1] * m[3][3] -
			m[0][0] * m[1][3] * m[3][1] -
			m[1][0] * m[0][1] * m[3][3] +
			m[1][0] * m[0][3] * m[3][1] +
			m[3][0] * m[0][1] * m[1][3] -
			m[3][0] * m[0][3] * m[1][1];

	r[3][2] = -m[0][0] * m[1][1] * m[3][2] +
			m[0][0] * m[1][2] * m[3][1] +
			m[1][0] * m[0][1] * m[3][2] -
			m[1][0] * m[0][2] * m[3][1] -
			m[3][0] * m[0][1] * m[1][2] +
			m[3][0] * m[0][2] * m[1][1];

	r[0][3] = -m[0][1] * m[1][2] * m[2][3] +
			m[0][1] * m[1][3] * m[2][2] +
			m[1][1] * m[0][2] * m[2][3] -
			m[1][1] * m[0][3] * m[2][2] -
			m[2][1] * m[0][2] * m[1][3] +
			m[2][1] * m[0][3] * m[1][2];

	r[1][3] = m[0][0] * m[1][2] * m[2][3] -
			m[0][0] * m[1][3] * m[2][2] -
			m[1][0] * m[0][2] * m[2][3] +
			m[1][0] * m[0][3] * m[2][2] +
			m[2][0] * m[0][2] * m[1][3] -
			m[2][0] * m[0][3] * m[1][2];

	r[2][3] = -m[0][0] * m[1][1] * m[2][3] +
			m[0][0] * m[1][3] * m[2][1] +
			m[1][0] * m[0][1] * m[2][3] -
			m[1][0] * m[0][3] * m[2][1] -
			m[2][0] * m[0][1] * m[1][3] +
			m[2][0] * m[0][3] * m[1][1];

	r[3][3] = m[0][0] * m[1][1] * m[2][2] -
			m[0][0] * m[1][2] * m[2][1] -
			m[1][0] * m[0][1] * m[2][2] +
			m[1][0] * m[0][2] * m[2][1] +
			m[2][0] * m[0][1] * m[1][2] -
			m[2][0] * m[0][2] * m[1][1];

	d = m[0][0] * r[0][0] + m[0][1] * r[1][0] + m[0][2] * r[2][0] + m[0][3] * r[3][0];

	d = fp(1.f )/ d;
	r = r * d;
	return r;
}

template <typename fp>
inline fp mat4<fp>::determinant(const mat4<fp>& m) {
	return
			m[3][0] * m[2][1] * m[1][2] * m[0][3]-
			m[2][0] * m[3][1] * m[1][2] * m[0][3]-
			m[3][0] * m[1][1] * m[2][2] * m[0][3]+
			m[1][0] * m[3][1] * m[2][2] * m[0][3]+
			m[2][0] * m[1][1] * m[3][2] * m[0][3]-
			m[1][0] * m[2][1] * m[3][2] * m[0][3]-
			m[3][0] * m[2][1] * m[0][2] * m[1][3]+
			m[2][0] * m[3][1] * m[0][2] * m[1][3]+
			m[3][0] * m[0][1] * m[2][2] * m[1][3]-
			m[0][0] * m[3][1] * m[2][2] * m[1][3]-
			m[2][0] * m[0][1] * m[3][2] * m[1][3]+
			m[0][0] * m[2][1] * m[3][2] * m[1][3]+
			m[3][0] * m[1][1] * m[0][2] * m[2][3]-
			m[1][0] * m[3][1] * m[0][2] * m[2][3]-
			m[3][0] * m[0][1] * m[1][2] * m[2][3]+
			m[0][0] * m[3][1] * m[1][2] * m[2][3]+
			m[1][0] * m[0][1] * m[3][2] * m[2][3]-
			m[0][0] * m[1][1] * m[3][2] * m[2][3]-
			m[2][0] * m[1][1] * m[0][2] * m[3][3]+
			m[1][0] * m[2][1] * m[0][2] * m[3][3]+
			m[2][0] * m[0][1] * m[1][2] * m[3][3]-
			m[0][0] * m[2][1] * m[1][2] * m[3][3]-
			m[1][0] * m[0][1] * m[2][2] * m[3][3]+
			m[0][0] * m[1][1] * m[2][2] * m[3][3];
}

template <typename fp>
inline mat4<fp> mat4<fp>::ortho(fp left, fp right, fp top, fp bottom, fp z_near, fp z_far) {
	mat4<fp> m(1.0f);

	m[0][0] =  2.0f / (right - left);
	m[1][1] =  2.0f / (top - bottom);
	m[2][2] = -2.0f / (z_far - z_near);
	m[3][0] = -(right + left) / (right - left);
	m[3][1] = -(top + bottom) / (top - bottom);
	m[3][2] = -(z_far + z_near) / (z_far - z_near);

	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::perspective(fp fov, fp aspect_ratio, fp z_near, fp z_far) {
	fp tan_half_fov = sixit::geometry::low_level::mathf::tan(fov / fp(2.0f));
	mat4<fp> m(0.0f);

	m[0][0] = fp(1.0f) / (aspect_ratio * tan_half_fov);
	m[1][1] = fp(1.0f) / (tan_half_fov);
	m[2][2] = -(z_far + z_near) / (z_far - z_near);
	m[2][3] = fp(-1.0f);
	m[3][2] = -(fp(2.0f) * z_far * z_near) / (z_far - z_near);

	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::frustum(fp left, fp right, fp bottom, fp top, fp z_near, fp z_far) {
	fp z_near2, width, height, depth;
	z_near2 = 2.0f * z_near;
	width = right - left;
	height = top - bottom;
	depth = z_far - z_near;
	mat4 m(0.0f);

	m[0][0] = z_near2 / width;
	m[1][1] = z_near2 / height;
	m[2][0] = (right + left) / width;
	m[2][1] = (top + bottom) / height;
	m[2][2] = (-z_far - z_near) / depth;
	m[2][3] = -1.0f;
	m[3][2] = (-z_near2 * z_far) / depth;

	return m;
}

template <typename fp>
inline mat4<fp> mat4<fp>::look_at(const vec4p<fp> &eye, const vec4p<fp> &center, const vec4p<fp> &up) {
	vec4p<fp> dir = (center - eye).normalized();
	vec4p<fp> up_norm = up.normalized();
	vec4p<fp> cross = vec4p<fp>::cross(dir, up_norm).normalized();
	up_norm = vec4p<fp>::cross(cross, dir);

	mat4 m(1.0f);

	m[0][0] =  cross[0];
	m[1][0] =  cross[1];
	m[2][0] =  cross[2];
	m[0][1] =  up_norm[0];
	m[1][1] =  up_norm[1];
	m[2][1] =  up_norm[2];
	m[0][2] = -dir[0];
	m[1][2] = -dir[1];
	m[2][2] = -dir[2];
	m[3][0] = -vec4p<fp>::dot(cross, eye);
	m[3][1] = -vec4p<fp>::dot(up_norm, eye);
	m[3][2] =  vec4p<fp>::dot(dir, eye);

	return m;
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_mat4_impl_common_h_included

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