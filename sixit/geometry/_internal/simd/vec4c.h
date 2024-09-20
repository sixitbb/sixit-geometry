/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Leonid Maksymchuk
*/

#ifndef sixit_geometry__internal_simd_vec4c_h_included
#define sixit_geometry__internal_simd_vec4c_h_included

#include "sixit/geometry/_internal/simd/internal/vec4base.h"

namespace sixit {
namespace geometry {
namespace gpu {

struct vec4c : public vec4base<float> {
	vec4c() {}
	vec4c(float a) : vec4base(a) {}
	vec4c(float a, float b, float c) : vec4base(a, b, c) {}
	vec4c(float a, float b, float c, float d) : vec4base(a, b, c, d) {}
	vec4c(const vec4c& v) : vec4base(v) {}
#ifdef GEOMETRY_USE_SIMD
	vec4c(sfloat4 sv) : vec4base(sv) {}
#endif

	vec4c& operator=(const vec4c& v);

	float r() const {
		return this->operator[](0);
	}
	float g() const {
		return this->operator[](1);
	}
	float b() const {
		return this->operator[](2);
	}
	float a() const {
		return this->operator[](3);
	}

	float& r() {
		return this->operator[](0);
	}
	float& g() {
		return this->operator[](1);
	}
	float& b() {
		return this->operator[](2);
	}
	float& a() {
		return this->operator[](3);
	}

	static vec4c blue() {
		return vec4c(0.0f, 0.0f, 1.0f);
	}
	static vec4c green() {
		return vec4c(0.0f, 1.0f, 0.0f);
	}
	static vec4c red() {
		return vec4c(1.0f, 0.0f, 0.0f);
	}

	static vec4c white() {
		return vec4c(1.0f, 1.0f, 1.0f);
	}
	static vec4c black() {
		return vec4c(0.0f, 0.0f, 0.0f);
	}
};

}
}
}

#endif //sixit_geometry__internal_simd_vec4c_h_included

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
