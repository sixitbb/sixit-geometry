/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_aabb4_h_included
#define sixit_geometry__internal_simd_aabb4_h_included

#include <cmath>
#include "sixit/geometry/_internal/simd/internal/simd.h"
#include "sixit/geometry/_internal/simd/internal/math.h"
#include "sixit/geometry/_internal/simd/vec4p.h"
#include "sixit/geometry/_internal/simd/mat4.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
struct /*alignas(16)*/ aabb4 {
	vec4p<fp> vmin, vmax;

	aabb4();
	aabb4(const vec4p<fp>& v);
	aabb4(const vec4p<fp>& a, const vec4p<fp>& b);
	aabb4(const aabb4<fp>& ab);

	void reset();
	void extend(const vec4p<fp>& v);
	void extend(const aabb4<fp>& ab);

	bool is_empty() const;

	static aabb4<fp> intersect(const aabb4<fp>& a, const aabb4<fp>& b);
	static bool overlaps(const aabb4<fp>& a, const aabb4<fp>& b);
};

template <typename fp>
inline aabb4<fp> operator*(const mat4<fp>& m, const aabb4<fp>& ab);

}

namespace gpu 
{
    // aliases 
    using aabb4f = aabb4<float>;
}

}
}

#include "sixit/geometry/_internal/simd/internal/aabb4_impl_common.h"

#endif //sixit_geometry__internal_simd_aabb4_h_included

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