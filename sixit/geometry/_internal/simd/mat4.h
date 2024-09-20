/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_mat4_h_included
#define sixit_geometry__internal_simd_mat4_h_included

#include "sixit/geometry/_internal/simd/vec4p.h"
#include "sixit/geometry/_internal/simd/quat4.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
struct /*alignas(16)*/ mat4 {
	vec4p<fp> mat[4];

	mat4() {}
	mat4(fp a);
	mat4(const mat4& m);
	mat4(fp m00, fp m01, fp m02, fp m03, fp m10, fp m11, fp m12, fp m13,
		 fp m20, fp m21, fp m22, fp m23, fp m30, fp m31, fp m32, fp m33);

#ifdef SIXIT_GEOMETRY_USE_SIMD
	mat4(sfloat4 row0, sfloat4 row1, sfloat4 row2, sfloat4 row3);
#endif

	mat4& operator=(const mat4& m);

	vec4p<fp>& operator[](unsigned n) { return mat[n]; }
	const vec4p<fp>& operator[](unsigned n) const { return mat[n]; }

	vec4p<fp> row(unsigned n) const;

	const fp* const_ptr() const { return mat[0].const_ptr(); }

	static mat4 identity() { return mat4(1.0); }
	static mat4 zero() { return mat4(0.0); }

	static mat4 translate(const vec4p<fp>& v);
	static mat4 rotate(fp a, const vec4p<fp>& axis);
	static mat4 rotate(const quat4<fp>& q);
	static mat4 scale(const vec4p<fp>& v);

	static mat4 compose(const vec4p<fp>& t, const quat4<fp>& r, const vec4p<fp>& s);
	static void decompose(const mat4<fp>& m, vec4p<fp>& t, quat4<fp>& r, vec4p<fp>& s);

	static mat4 inverse(const mat4<fp>& m);
	static fp determinant(const mat4<fp>& m);

	static mat4 ortho(fp left, fp right, fp top, fp bottom, fp z_near, fp z_far);
	static mat4 perspective(fp fov, fp aspect_ratio, fp z_near, fp z_far);
	static mat4 frustum(fp left, fp right, fp bottom, fp top, fp z_near, fp z_far);
	static mat4 look_at(const vec4p<fp>& eye, const vec4p<fp>& center, const vec4p<fp>& up);
};

template <typename fp>
inline vec4p<fp> operator*(const mat4<fp>& m, const vec4p<fp>& v);
template <typename fp>
inline mat4<fp> operator*(const mat4<fp>& m1, const mat4<fp>& m2);

template <typename fp>
inline mat4<fp> operator+(const mat4<fp>& m1, const mat4<fp>& m2);
template <typename fp>
inline mat4<fp> operator-(const mat4<fp>& m1, const mat4<fp>& m2);

template <typename fp>
inline mat4<fp> operator*(const mat4<fp>& m, fp v);
template <typename fp>
inline mat4<fp> operator/(const mat4<fp>& m, fp v);

}

namespace gpu
{
    // aliases 
    using mat4f = mat4<float>;
}

}
}

#if defined(__SSE2__)
#include "sixit/geometry/_internal/simd/internal/mat4_impl_sse2.h"
#elif defined(__ARM_NEON)
#include "sixit/geometry/_internal/simd/internal/mat4_impl_neon.h"
#endif

#include "sixit/geometry/_internal/simd/internal/mat4_impl_generic.h"
#include "sixit/geometry/_internal/simd/internal/mat4_impl_common.h"

#endif //sixit_geometry__internal_simd_mat4_h_included

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
