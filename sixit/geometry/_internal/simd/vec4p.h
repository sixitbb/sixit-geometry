/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_vec4p_h_included
#define sixit_geometry__internal_simd_vec4p_h_included

#include "sixit/geometry/_internal/simd/internal/vec4base.h"

namespace sixit {
namespace geometry {
namespace gpu {

template <typename fp>
struct vec4p : public vec4base<fp> {
	vec4p() {}
	vec4p(fp a) : vec4base<fp>(a) {}
	vec4p(fp a, fp b, fp c) : vec4base<fp>(a, b, c) {}
	vec4p(fp a, fp b, fp c, fp d) : vec4base<fp>(a, b, c, d) {}
	vec4p(const vec4p<fp>& v) : vec4base<fp>(v) {}
	vec4p(const vec4base<fp>& v) : vec4base<fp>(v) {}
#ifdef GEOMETRY_USE_SIMD
	vec4p(sfloat4 sv) : vec4base<fp>(sv) {}
#endif

	fp& x() {
		return this->operator[](0);
	}
	fp& y() {
		return this->operator[](1);
	}
	fp& z() {
		return this->operator[](2);
	}
	fp& w() {
		return this->operator[](3);
	}

	const fp& x() const {
		return this->operator[](0);
	}
	const fp& y() const {
		return this->operator[](1);
	}
	const fp& z() const{
		return this->operator[](2);
	}
	const fp& w() const {
		return this->operator[](3);
	}

	static vec4p<fp> forward() {
		return vec4p<fp>(0.0f, 0.0f, -1.0f);
	}
	static vec4p<fp> back() {
		return vec4p<fp>(0.0f, 0.0f, 1.0f);
	}
	static vec4p<fp> up() {
		return vec4p<fp>(0.0f, 1.0f, 0.0f);
	}
	static vec4p<fp> down() {
		return vec4p<fp>(0.0f, -1.0f, 0.0f);
	}
	static vec4p<fp> left() {
		return vec4p<fp>(-1.0f, 0.0f, 0.0f);
	}
	static vec4p<fp> right() {
		return vec4p<fp>(1.0f, 0.0f, 0.0f);
	}

	static vec4p<fp> one() {
		return vec4p<fp>(1.0f, 1.0f, 1.0f);
	}
	static vec4p<fp> zero() {
		return vec4p<fp>(0.0f, 0.0f, 0.0f);
	}
};

}

namespace gpu
{
    // aliases 
    using vec4pf = vec4p<float>;
}

}
}

#endif //sixit_geometry__internal_simd_vec4p_h_included

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
