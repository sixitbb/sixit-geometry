/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_shapes_h_included
#define sixit_geometry_shapes_h_included

#include <optional>

namespace sixit
{
	namespace geometry
	{

		template <typename fp>
		struct shape_line2 
		{
			// nothing to define
		};

		template <typename fp>
		struct shape_arc2
		{
			inline shape_arc2(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius, bool cw) : radius(radius), cw(cw) {}

			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius;
			bool cw;
		};

		template <typename fp>
		struct shape_arc2_w_radius
		{
			inline shape_arc2_w_radius(bool cw) : cw(cw) {}
			bool cw;
		};

		template <typename fp>
		struct shape_qbezier2
		{
			// nothin to define
		};

		template <typename fp>
		struct shape_composite_cbezier2
		{
			inline shape_composite_cbezier2(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_radius) : min_radius(min_radius) {}
			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_radius;
		};

		template <typename fp>
		struct shape_line_arc_line2
		{
			inline shape_line_arc_line2(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius) : radius(radius) {}
			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius;
		};

		template <typename fp>
		struct shape_arc_line_arc2
		{
			inline shape_arc_line_arc2(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius) : radius(radius) {}
			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> radius;
		};
	}

	namespace geometry 
	{
		// aliases 
		using shape_line2f = shape_line2<float>;
		using shape_arc2f = shape_arc2<float>;
		using shape_arc2_w_radiusf = shape_arc2_w_radius<float>;
		using shape_qbezier2f = shape_qbezier2<float>;
		using shape_composite_cbezier2f = shape_composite_cbezier2<float>;
		using shape_line_arc_line2f = shape_line_arc_line2<float>;
		using shape_arc_line_arc2f = shape_arc_line_arc2<float>;
	}
}

#endif //sixit_geometry_shapes_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Mykhailo Borovyk

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