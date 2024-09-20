/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_quat4_impl_common_h_included
#define sixit_geometry__internal_simd_internal_quat4_impl_common_h_included

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
inline quat4<fp>::quat4() {}

template <typename fp>
inline quat4<fp> quat4<fp>::inverse(const quat4<fp>& q) {
	quat4<fp> i;
	fp m = dot(q, q);
	i.x() = -q.x() / m;
	i.y() = -q.y() / m;
	i.z() = -q.z() / m;
	i.w() = q.w() / m;
	return i;
}

template <typename fp>
inline quat4<fp> quat4<fp>::normalize(const quat4<fp>& q) {
	fp m = fp(1.0f) / sixit::geometry::low_level::mathf::sqrt(dot(q, q));
	quat4 n(q);
	n.x() = n.x() * m;
	n.y() = n.y() * m;
	n.z() = n.z() * m;
	n.w() = n.w() * m;
	return n;
}

template <typename fp>
inline quat4<fp> quat4<fp>::from_euler(fp roll, fp pitch, fp yaw) {
	quat4<fp> q;

	fp hy = yaw * fp(0.5f);
	fp hp = pitch * fp(0.5f);
	fp hr = roll * fp(0.5f);

	fp cy = sixit::geometry::low_level::mathf::cos(hy);
	fp sy = sixit::geometry::low_level::mathf::sin(hy);
	fp cp = sixit::geometry::low_level::mathf::cos(hp);
	fp sp = sixit::geometry::low_level::mathf::sin(hp);
	fp cr = sixit::geometry::low_level::mathf::cos(hr);
	fp sr = sixit::geometry::low_level::mathf::sin(hr);

	q.x() = sr * cp * cy - cr * sp * sy;
	q.y() = cr * sp * cy + sr * cp * sy;
	q.z() = cr * cp * sy - sr * sp * cy;
	q.w() = cr * cp * cy + sr * sp * sy;

	return q;
}

template <typename fp>
inline quat4<fp> quat4<fp>::from_axis_angle(const gpu::vec4p<fp>& axis, fp angle) {
	quat4<fp> q;

	fp ha = angle * fp(0.5f);
	fp sha = sixit::geometry::low_level::mathf::sin(ha);

	q.x() = axis.x() * sha;
	q.y() = axis.y() * sha;
	q.z() = axis.z() * sha;
	q.w() = sixit::geometry::low_level::mathf::cos(ha);

	return q;
}

template <typename fp>
inline quat4<fp> operator*(const quat4<fp>& a, const quat4<fp>& b) {
	quat4<fp> r;
	r.x() = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
	r.y() = a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x();
	r.z() = a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w();
	r.w() = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
	return r;
}

template <typename fp>
inline const fp& quat4<fp>::x() const
{
	return this->operator[](0);
}

template <typename fp>
inline const fp& quat4<fp>::y() const
{
	return this->operator[](1);
}

template <typename fp>
inline const fp& quat4<fp>::z() const
{
	return this->operator[](2);
}

template <typename fp>
inline const fp& quat4<fp>::w() const
{
	return this->operator[](3);
}

template <typename fp>
inline fp& quat4<fp>::x() {
	return this->operator[](0);
}

template <typename fp>
inline fp& quat4<fp>::y() {
	return this->operator[](1);
}

template <typename fp>
inline fp& quat4<fp>::z() {
	return this->operator[](2);
}

template <typename fp>
inline fp& quat4<fp>::w() {
	return this->operator[](3);
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_quat4_impl_common_h_included

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