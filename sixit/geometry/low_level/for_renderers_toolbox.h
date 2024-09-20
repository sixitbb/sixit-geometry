/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_for_renderers_toolbox_h_included
#define sixit_geometry_low_level_for_renderers_toolbox_h_included

#include "sixit/geometry/_internal/simd/vec4p.h"
#include "sixit/geometry/_internal/simd/mat4.h"
#include "sixit/geometry/_internal/simd/aabb4.h"

#include "sixit/geometry/line.h"
#include "sixit/geometry/point.h"

namespace sixit {
namespace geometry {
namespace for_renderers_only {

template <typename fp>
struct /*alignas(16)*/ frustum {
	gpu::vec4p<fp> planes[6];

	void extract(const gpu::mat4<fp>& m);
	void extract(const low_level::matrix4x4<fp>& m);

	fp test(const gpu::aabb4<fp>& ab) const;
	fp test(const gpu::mat4<fp>& m, const gpu::aabb4<fp>& ab) const;

	fp test(const bounds3<fp>& b) const;
	fp test(const low_level::matrix4x4<fp>& m, const bounds3<fp>& b) const;
	fp test(const gpu::mat4<fp>& m, const bounds3<fp>& b) const;

private:
	static bounds3<fp> transform(const gpu::mat4<fp>& m, const bounds3<fp>& b);
};

template <typename fp>
inline void frustum<fp>::extract(const gpu::mat4<fp>& m) {
	gpu::vec4p<fp> row0 = m.row(0);
	gpu::vec4p<fp> row1 = m.row(1);
	gpu::vec4p<fp> row2 = m.row(2);
	gpu::vec4p<fp> row3 = m.row(3);

	planes[0] = (row3 - row0).normalized();	// right plane
	planes[1] = (row3 + row0).normalized();	// left plane
	planes[2] = (row3 + row1).normalized();	// bottom plane
	planes[3] = (row3 - row1).normalized();	// top plane
	planes[4] = (row3 - row2).normalized();	// far plane
	planes[5] = (row3 + row2).normalized();	// near plane
}

template <typename fp>
inline void frustum<fp>::extract(const low_level::matrix4x4<fp>& m) {
	extract((const gpu::mat4<fp>&) m);
}

template <typename fp>
inline fp frustum<fp>::test(const gpu::aabb4<fp>& ab) const {
	gpu::vec4p<fp> center = (ab.vmax + ab.vmin) * fp(0.5f);
	gpu::vec4p<fp> radius = ab.vmax - center;

	fp dist, rad;

	for (int n = 0; n < 6; n++) {
		dist = gpu::vec4p<fp>::dot(planes[n], center) + planes[n].w();
		rad = gpu::vec4p<fp>::dot(gpu::vec4p<fp>::abs(planes[n]), radius);

		if (dist < -rad) {
			return fp(-1.0f);
		}
	}

	return dist + rad;
}

template <typename fp>
inline fp frustum<fp>::test(const gpu::mat4<fp>& m, const gpu::aabb4<fp>& ab) const {
	return test<fp>((m * ab));
}

// Returns distance from Z near clip plane to specified bounds,
// or negative value if bounds are outside of frustum
template <typename fp>
inline fp frustum<fp>::test(const bounds3<fp>& b) const {
	gpu::vec4p<fp> vmin = b.minn.vec();
	gpu::vec4p<fp> vmax = b.maxx.vec();

	gpu::vec4p<fp> center = (vmax + vmin) * fp(0.5f);
	gpu::vec4p<fp> radius = vmax - center;

	fp dist, rad;

	for (int n = 0; n < 6; n++) {
		dist = gpu::vec4p<fp>::dot(planes[n], center) + planes[n].w();
		rad = gpu::vec4p<fp>::dot(gpu::vec4p<fp>::abs(planes[n]), radius);

		if (dist < -rad) {
			return fp(-1.0f);
		}
	}

	return dist + rad;
}

template <typename fp>
inline fp frustum<fp>::test(const low_level::matrix4x4<fp>& m, const bounds3<fp>& b) const {
	return test(transform((const gpu::mat4<fp>&)m, b));
}

template <typename fp>
inline fp frustum<fp>::test(const gpu::mat4<fp>& m, const bounds3<fp>& b) const {
	return test(transform(m, b));
}

template <typename fp>
inline bounds3<fp> frustum<fp>::transform(const gpu::mat4<fp>& m, const bounds3<fp>& b) {
	gpu::vec4p<fp> vmin = b.minn.vec();
	gpu::vec4p<fp> vmax = b.maxx.vec();

	gpu::vec4p<fp> xmin = m[0] * vmin.x();
	gpu::vec4p<fp> xmax = m[0] * vmax.x();
	gpu::vec4p<fp> ymin = m[1] * vmin.y();
	gpu::vec4p<fp> ymax = m[1] * vmax.y();
	gpu::vec4p<fp> zmin = m[2] * vmin.z();
	gpu::vec4p<fp> zmax = m[2] * vmax.z();

	point3<fp> pmin = point3<fp>(gpu::vec4p<fp>::min(xmin, xmax) + gpu::vec4p<fp>::min(ymin, ymax) + gpu::vec4p<fp>::min(zmin, zmax) + m[3]);
	point3<fp> pmax = point3<fp>(gpu::vec4p<fp>::max(xmin, xmax) + gpu::vec4p<fp>::max(ymin, ymax) + gpu::vec4p<fp>::max(zmin, zmax) + m[3]);

	return bounds3<fp>(pmin, pmax);
}

template <typename fp>
inline bool frustum_cull(const frustum<fp>& f, const bounds3<fp>& b) {
	return f.test(b) >= fp(0.0f);
}

template <typename fp>
inline bool frustum_cull(const frustum<fp>& f, const bounds3<fp>& b, const low_level::matrix4x4<fp>& m) {
	return f.test(m, b) >= fp(0.0f);
}

}

namespace for_renderers_only 
{
    // aliases 
    using frustumf = frustum<float>;
}


}
}

#endif //sixit_geometry_low_level_for_renderers_toolbox_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Volodymyr Melnychuk

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
