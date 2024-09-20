/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_internal_vec4base_impl_common_h_included
#define sixit_geometry__internal_simd_internal_vec4base_impl_common_h_included

#include "sixit/geometry/sixit_mathf.h"

namespace sixit {
namespace geometry {

namespace gpu {

template <typename fp>
inline bool vec4base<fp>::operator==(const vec4base<fp>& v) const {
	return	this->operator[](0) == v[0] 
		&& this->operator[](1) == v[1] 
		&& this->operator[](2) == v[2] 
		&& this->operator[](3) == v[3];
}

template <typename fp>
inline fp vec4base<fp>::angle(const vec4base<fp>& a, const vec4base<fp>& b) {
	fp c = dot(a, b);
	return sixit::geometry::low_level::mathf::acos(c);
}

template <typename fp>
inline fp vec4base<fp>::magnitude() const {
	return sixit::geometry::low_level::mathf::sqrt(sqr_magnitude());
}

template <typename fp>
inline fp vec4base<fp>::sqr_magnitude() const {
	return dot(*this, *this);
}

template <typename fp>
inline void vec4base<fp>::normalize() {
	*this /= magnitude();
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::normalized() const {
	return *this / magnitude();
}

template <typename fp>
inline void vec4base<fp>::scale(fp a) {
	*this *= a;
}

template <typename fp>
inline vec4base<fp> vec4base<fp>::scale(const vec4base<fp>& v, fp a) {
	return vec4base(v * a);
}

}
}
}

#endif //sixit_geometry__internal_simd_internal_vec4base_impl_common_h_included

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
