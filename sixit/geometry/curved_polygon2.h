/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_curved_polygon2_h_included
#define sixit_geometry_curved_polygon2_h_included

#include "sixit/geometry/curve.h"

namespace sixit
{
namespace geometry
{
	

	template <typename fp>
	struct curved_polygon2 : curve2<fp>
	{
	public:
		typedef std::pair<point2<fp>, direction2<fp>> path_point;

		inline curved_polygon2() = default;
		inline curved_polygon2(const curved_polygon2<fp>& other) = delete;
		inline curved_polygon2(curved_polygon2<fp>&& other) = default;
		inline curved_polygon2(const path_point& initial_point) : curve2<fp>(initial_point) {}

		inline void close_path();
		
		template<typename TSetPixel>
		inline void rasterize(TSetPixel& SET_PIXEL, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness = 
											sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)) const;

		template<typename TSetPixel>
		inline void rasterize_polygon(TSetPixel& SET_PIXEL, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness = 
											sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f));

		template<typename TSetPixelAA>
		inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness = 
											sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)) const;

		template<typename TSetPixelAA>
		inline void rasterize_polygon_aa(TSetPixelAA& SET_PIXEL_AA, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness = 
											sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f));

		inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
		
		template <typename P2>
		inline bool is_inside(const P2& point) const;

	private:
		inline bool is_clockwise() const;
		inline std::vector<point2<fp>> to_points(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness = 
											sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)) const;
		inline void segment_to_points(const typename curve2_primitive<fp>::pcurve_segment& segment, std::vector<point2<fp>>& points,
										sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, bool skip_first_point = false) const;
		inline void segment_exterior_to_points(const typename curve2_primitive<fp>::pcurve_segment& segment, std::vector<point2<fp>>& points,
										sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, 
										sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness, bool clockwise, bool skip_first_point = false) const;

	};
}

namespace geometry 
{
    // aliases 
    using curved_polygon2f = curved_polygon2<float>;
}

}

#endif //sixit_geometry_curved_polygon2_h_included

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