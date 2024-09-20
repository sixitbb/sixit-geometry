/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_quat4_impl_sse2_h_included
#define sixit_geometry__internal_simd_internal_quat4_impl_sse2_h_included

namespace sixit {
namespace geometry {
namespace gpu {

inline quat4<float>::quat4() {
	squat = _mm_setr_ps(0.0f, 0.0f, 0.0f, 0.0f);
}

inline quat4<float>::quat4(float x_, float y_, float z_, float w_) {
	squat = _mm_setr_ps(x_, y_, z_, w_);
}

inline quat4<float>::quat4(const quat4<float>& q) {
	squat = q.squat;
}

inline quat4<float>::quat4(sfloat4 sq) {
	squat = sq;
}

inline quat4<float>& quat4<float>::operator=(const quat4<float>& q) {
	squat = q.squat;
	return *this;
}

inline const float& quat4<float>::operator[](unsigned index) const {
#   ifdef SIXIT_COMPILER_MSVC
	return squat.m128_f32[index];
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&squat)[index];
#   elif defined SIXIT_COMPILER_GCC
	return squat[index];
#	else
#	error unsupported compiler
#   endif
}

inline float& quat4<float>::operator[](unsigned index) {
#   ifdef SIXIT_COMPILER_MSVC
	return squat.m128_f32[index];
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<float*>(&squat)[index];
#   elif defined SIXIT_COMPILER_GCC
	return squat[index];
#	else
#	error unsupported compiler
#   endif
}

inline const float* quat4<float>::const_ptr() const {
#   ifdef SIXIT_COMPILER_MSVC
	return &(squat.m128_f32[0]);
#   elif defined SIXIT_COMPILER_ANY_CLANG
	return reinterpret_cast<const float*>(&squat);
#   elif defined SIXIT_COMPILER_GCC
	return &squat[0];
#	else
#	error unsupported compiler
#   endif
}

inline const float& quat4<float>::x() const
{
	return this->operator[](0);
}

inline const float& quat4<float>::y() const
{
	return this->operator[](1);
}

inline const float& quat4<float>::z() const
{
	return this->operator[](2);
}

inline const float& quat4<float>::w() const
{
	return this->operator[](3);
}

inline float& quat4<float>::x() {
	return this->operator[](0);
}

inline float& quat4<float>::y() {
	return this->operator[](1);
}

inline float& quat4<float>::z() {
	return this->operator[](2);
}

inline float& quat4<float>::w() {
	return this->operator[](3);
}

inline float quat4<float>::dot(const quat4<float>& a, const quat4<float>& b) {
	__m128 m0 = _mm_mul_ps(a.squat, b.squat);
	__m128 m1 = _mm_shuffle_ps(m0, m0, _MM_SHUFFLE(2, 3, 0, 1));
	__m128 m2 = _mm_add_ps(m0, m1);
	m1 = _mm_movehl_ps(m1, m2);
	m2 = _mm_add_ss(m2, m1);
	return _mm_cvtss_f32(m2);
}

inline quat4<float> quat4<float>::inverse(const quat4<float>& q) {
	quat4<float> i;
	float m = dot(q, q);
	i.x() = -q.x() / m;
	i.y() = -q.y() / m;
	i.z() = -q.z() / m;
	i.w() = q.w() / m;
	return i;
}

inline quat4<float> quat4<float>::from_euler(float roll, float pitch, float yaw) {
	quat4<float> q;

	float hy = yaw * 0.5f;
	float hp = pitch * 0.5f;
	float hr = roll * 0.5f;

	float cy = sixit::geometry::low_level::mathf::cos(hy);
	float sy = sixit::geometry::low_level::mathf::sin(hy);
	float cp = sixit::geometry::low_level::mathf::cos(hp);
	float sp = sixit::geometry::low_level::mathf::sin(hp);
	float cr = sixit::geometry::low_level::mathf::cos(hr);
	float sr = sixit::geometry::low_level::mathf::sin(hr);

	q.x() = sr * cp * cy - cr * sp * sy;
	q.y() = cr * sp * cy + sr * cp * sy;
	q.z() = cr * cp * sy - sr * sp * cy;
	q.w() = cr * cp * cy + sr * sp * sy;

	return q;
}

inline quat4<float> quat4<float>::from_axis_angle(const gpu::vec4p<float>& axis, float angle) {
	quat4<float> q;

	float ha = angle * 0.5f;
	float sha = sixit::geometry::low_level::mathf::sin(ha);

	q.x() = axis.x() * sha;
	q.y() = axis.y() * sha;
	q.z() = axis.z() * sha;
	q.w() = sixit::geometry::low_level::mathf::cos(ha);

	return q;
}

inline quat4<float> quat4<float>::normalize(const quat4<float>& q) {
	float m = 1.0f / sixit::geometry::low_level::mathf::sqrt(dot(q, q));
	quat4 n(q);
	n.x() = n.x() * m;
	n.y() = n.y() * m;
	n.z() = n.z() * m;
	n.w() = n.w() * m;
	return n;
}

template<>
inline quat4<float> operator+(const quat4<float>& a, const quat4<float>& b) {
	return quat4<float>(_mm_add_ps(a.squat, b.squat));
}

template<>
inline quat4<float> operator-(const quat4<float>& a, const quat4<float>& b) {
	return quat4<float>(_mm_sub_ps(a.squat, b.squat));
}

template<>
inline quat4<float> operator*(const quat4<float>& q, float f) {
	return quat4<float>(_mm_mul_ps(q.squat, _mm_set1_ps(f)));
}

template<>
inline quat4<float> operator/(const quat4<float>& q, float f) {
	return quat4<float>(_mm_div_ps(q.squat, _mm_set1_ps(f)));
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_quat4_impl_sse2_h_included

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