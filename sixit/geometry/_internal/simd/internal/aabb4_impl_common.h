/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_aabb4_impl_common_h_included
#define sixit_geometry__internal_simd_internal_aabb4_impl_common_h_included

#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
inline aabb4<fp>::aabb4() {
	vmin = vec4p<fp>(low_level::mathf::max_float);
	vmax = vec4p<fp>(-low_level::mathf::max_float);
}
template <typename fp>
inline aabb4<fp>::aabb4(const vec4p<fp>& v) {
	vmin = v;
	vmax = v;
}
template <typename fp>
inline aabb4<fp>::aabb4(const vec4p<fp>& a, const vec4p<fp>& b) {
	vmin = vec4p<fp>::min(a, b);
	vmax = vec4p<fp>::max(a, b);
}
template <typename fp>
inline aabb4<fp>::aabb4(const aabb4<fp>& ab) {
	vmin = ab.vmin;
	vmax = ab.vmax;
}
template <typename fp>
inline void aabb4<fp>::reset() {
	vmin = vec4p(low_level::mathf::max_float);
	vmax = vec4p(-low_level::mathf::max_float);
}
template <typename fp>
inline void aabb4<fp>::extend(const vec4p<fp>& v) {
	vmin = vec4p<fp>::min(vmin, v);
	vmax = vec4p<fp>::max(vmax, v);
}
template <typename fp>
inline void aabb4<fp>::extend(const aabb4& ab) {
	vmin = vec4p<fp>::min(vmin, ab.vmin);
	vmax = vec4p<fp>::max(vmax, ab.vmax);
}
template <typename fp>
inline bool aabb4<fp>::is_empty() const {
	vec4p<fp> d = vmax - vmin;
	return !(d.x() > 0.0f &&
			 d.y() > 0.0f &&
			 d.z() > 0.0f);
}
template <typename fp>
inline aabb4<fp> aabb4<fp>::intersect(const aabb4<fp>& a, const aabb4<fp>& b) {
	aabb4 its;
	its.vmin = vec4p<fp>::max(a.vmin, b.vmin);
	its.vmax = vec4p<fp>::min(a.vmax, b.vmax);
	return its;
}
template <typename fp>
inline bool aabb4<fp>::overlaps(const aabb4<fp>& a, const aabb4<fp>& b) {
	return !intersect(a, b).is_empty();
}

template <typename fp>
inline aabb4<fp> operator*(const mat4<fp>& m, const aabb4<fp>& ab) {
	gpu::vec4p vmin = ab.vmin;
	gpu::vec4p vmax = ab.vmax;

	gpu::vec4p xmin = m[0] * vmin.x();
	gpu::vec4p xmax = m[0] * vmax.x();
	gpu::vec4p ymin = m[1] * vmin.y();
	gpu::vec4p ymax = m[1] * vmax.y();
	gpu::vec4p zmin = m[2] * vmin.z();
	gpu::vec4p zmax = m[2] * vmax.z();

	aabb4<fp> r;
	r.vmin = gpu::vec4p<fp>::min(xmin, xmax) + gpu::vec4p<fp>::min(ymin, ymax) + gpu::vec4p<fp>::min(zmin, zmax) + m[3];
	r.vmax = gpu::vec4p<fp>::max(xmin, xmax) + gpu::vec4p<fp>::max(ymin, ymax) + gpu::vec4p<fp>::max(zmin, zmax) + m[3];
	return r;
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_aabb4_impl_common_h_included

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