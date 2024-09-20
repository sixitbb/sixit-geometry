/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_quat4_impl_generic_h_included
#define sixit_geometry__internal_simd_internal_quat4_impl_generic_h_included

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
inline quat4<fp>::quat4(fp x_, fp y_, fp z_, fp w_) {
	quat[0] = x_;
	quat[1] = y_;
	quat[2] = z_;
	quat[3] = w_;
}

template <typename fp>
inline quat4<fp>::quat4(const quat4<fp>& q) {
	for(int i = 0; i < 4; ++i)
		quat[i] = q.quat[i];
}

template <typename fp>
inline quat4<fp>& quat4<fp>::operator=(const quat4<fp>& q) {
	for (int i = 0; i < 4; ++i)
		quat[i] = q.quat[i];
	return *this;
}

template <typename fp>
inline fp quat4<fp>::dot(const quat4<fp>& a, const quat4<fp>& b) {
	fp result = fp(0.0f);
	for (int i = 0; i < 4; ++i)
		result = result + a.quat[i] * b.quat[i];
	return result;
}

template <typename fp>
inline quat4<fp> operator+(const quat4<fp>& a, const quat4<fp>& b) {
	quat4<fp> r;
	for (int i = 0; i < 4; ++i)
		r.quat[i] = a.quat[i] + b.quat[i];
	return r;
}

template <typename fp>
inline quat4<fp> operator-(const quat4<fp>& a, const quat4<fp>& b) {
	quat4<fp> r;
	for (int i = 0; i < 4; ++i)
		r.quat[i] = a.quat[i] - b.quat[i];
	return r;
}

template <typename fp>
inline quat4<fp> operator*(const quat4<fp>& q, fp f) {
	quat4<fp> r;
	for (int i = 0; i < 4; ++i)
		r.quat[i] = q.quat[i] * f;
	return r;
}

template <typename fp>
inline quat4<fp> operator/(const quat4<fp>& q, fp f) {
	quat4<fp> r;
	for (int i = 0; i < 4; ++i)
		r.quat[i] = q.quat[i] / f;
	return r;
}

template <typename fp>
inline const fp& quat4<fp>::operator[](unsigned index) const {
	return quat[index];
}

template <typename fp>
inline fp& quat4<fp>::operator[](unsigned index) {
	return quat[index];
}

template <typename fp>
inline const fp* quat4<fp>::const_ptr() const {
	return quat;
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_quat4_impl_generic_h_included

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