/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_quat4_h_included
#define sixit_geometry__internal_simd_quat4_h_included

#include <cmath>

#include "sixit/geometry/_internal/simd/internal/simd.h"
#include "sixit/geometry/_internal/simd/internal/math.h"
#include "sixit/geometry/_internal/simd/vec4p.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
struct quat4 {
	fp quat[4];

	quat4();
	quat4(fp x_, fp y_, fp z_, fp w_);
	quat4(const quat4<fp>& q);


	quat4& operator=(const quat4<fp>& q);
	const fp* const_ptr() const;

	const fp& x() const;
	const fp& y() const;
	const fp& z() const;
	const fp& w() const;
	fp& x();
	fp& y();
	fp& z();
	fp& w();

	static fp dot(const quat4<fp>& a, const quat4<fp>& b);
	static quat4<fp> inverse(const quat4<fp>& q);
	static quat4<fp> normalize(const quat4<fp>& q);
	static quat4<fp> from_euler(fp roll, fp pitch, fp yaw);
	static quat4<fp> from_axis_angle(const gpu::vec4p<fp>& axis, fp angle);
private:

	const fp& operator[](unsigned index) const;
	fp& operator[](unsigned index);

};

#ifdef SIXIT_GEOMETRY_USE_SIMD

#include "sixit/geometry/_internal/simd/internal/simd.h"

template <>
struct alignas(16) quat4<float> {
	sfloat4 squat;

	quat4();
	quat4(float x_, float y_, float z_, float w_);
	quat4(const quat4<float>& q);
	quat4(sfloat4 sq);

	quat4& operator=(const quat4<float>& q);
	const float* const_ptr() const;

	const float& x() const;
	const float& y() const;
	const float& z() const;
	const float& w() const;
	float& x();
	float& y();
	float& z();
	float& w();

	static float dot(const quat4<float>& a, const quat4<float>& b);
	static quat4<float> inverse(const quat4<float>& q);
	static quat4<float> normalize(const quat4<float>& q);
	static quat4<float> from_euler(float roll, float pitch, float yaw);
	static quat4<float> from_axis_angle(const gpu::vec4p<float>& axis, float angle);
private:

	const float& operator[](unsigned index) const;
	float& operator[](unsigned index);

};

#endif

template <typename fp>
inline quat4<fp> operator*(const quat4<fp>& a, const quat4<fp>& b);

template <typename fp>
inline quat4<fp> operator+(const quat4<fp>& a, const quat4<fp>& b);

template <typename fp>
inline quat4<fp> operator-(const quat4<fp>& a, const quat4<fp>& b);

template <typename fp>
inline quat4<fp> operator*(const quat4<fp>& q, fp f);

template <typename fp>
inline quat4<fp> operator/(const quat4<fp>& q, fp f);

}

namespace gpu 
{
    // aliases 
    using quat4f = quat4<float>;
}

}
}

#ifdef SIXIT_GEOMETRY_USE_SIMD
	#if defined(__SSE2__)
	#include "sixit/geometry/_internal/simd/internal/quat4_impl_sse2.h"
	#elif defined(__ARM_NEON)
	#include "sixit/geometry/_internal/simd/internal/quat4_impl_neon.h"
	#endif
#endif


#include "sixit/geometry/_internal/simd/internal/quat4_impl_generic.h"
#include "sixit/geometry/_internal/simd/internal/quat4_impl_common.h"

#endif //sixit_geometry__internal_simd_quat4_h_included

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