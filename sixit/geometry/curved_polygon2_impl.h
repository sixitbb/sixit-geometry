/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_curved_polygon2_impl_h_included
#define sixit_geometry_curved_polygon2_impl_h_included

#include "sixit/geometry/curved_polygon2.h"
#include "sixit/core/guidelines.h"

namespace sixit
{
namespace geometry
{
	template <typename fp>
	inline void curved_polygon2<fp>::close_path()
	{
		if (curve2<fp>::end_point() != curve2<fp>::start_point())
			curve2<fp>::append(shape_line2<fp>(), curve2<fp>::start_point());
	}

	template <typename fp>
	inline bool curved_polygon2<fp>::is_clockwise() const
	{
		std::vector<point2<fp>> points;
		auto dist_between_points = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(std::numeric_limits<float>::infinity()); // use less poitns to build a path. no detail needed

		for (const auto& segment : curve2<fp>::segments)
			segment_to_points(segment, points, dist_between_points, true);
		points.pop_back();

		return low_level::polygon2_impl<fp>::normal(points) < 0;
	}

	template <typename fp>
	template<typename TSetPixel>
	inline void curved_polygon2<fp>::rasterize(TSetPixel& SET_PIXEL, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness) const
	{
		auto dist = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(20.f);
		if (!curve2<fp>::segments.empty())
		{
			if (thickness <= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f))
			{
				for (const auto& segment : curve2<fp>::segments)
					curve2<fp>::raster_segment(segment, SET_PIXEL, dist);
			}
			else
			{
				size_t count = curve2<fp>::segments.size() - 1;
				auto it1 = curve2<fp>::segments.begin();
				for (size_t i = 0; i < count; i++)
				{
					curve2<fp>::raster_segment(*it1, SET_PIXEL, dist, thickness);
					//build edge corner if needed:
					auto it2 = it1;
					it1++;
					curve2<fp>::raster_corner(*it2, *it1, SET_PIXEL, thickness);
				}
				curve2<fp>::raster_segment(*it1, SET_PIXEL, dist, thickness);
				curve2<fp>::raster_corner(*it1, *curve2<fp>::segments.begin(), SET_PIXEL, thickness);
			}
		}
	}

	template <typename fp>
	template<typename TSetPixel>
	inline void curved_polygon2<fp>::rasterize_polygon(TSetPixel& SET_PIXEL, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness)
	{
		close_path();
		auto points = to_points(thickness);
		if (points.back() == points.front())
			points.erase(points.begin() + points.size() - 1);
		return curve2<fp>::rasterize_scanline(points, SET_PIXEL);
	}

	template <typename fp>
	template<typename TSetPixelAA>
	void curved_polygon2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness) const
	{
		auto dist = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(20.f);
		if (!curve2<fp>::segments.empty())
		{
			if (thickness <= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f))
			{
				for (const auto& segment : curve2<fp>::segments)
					curve2<fp>::raster_segment_aa(segment, SET_PIXEL_AA, dist);
			}
			else
			{
				size_t count = curve2<fp>::segments.size() - 1;
				auto it1 = curve2<fp>::segments.begin();
				auto set_pixel = [&](int x, int y) { SET_PIXEL_AA(x, y, 1.f); };
				for (size_t i = 0; i < count; i++)
				{
					curve2<fp>::raster_segment_aa(*it1, SET_PIXEL_AA, dist, thickness);

					//build edge corner if needed:
					auto it2 = it1;
					it1++;

					curve2<fp>::raster_corner(*it2, *it1, set_pixel, thickness);
				}
				curve2<fp>::raster_segment_aa(*it1, SET_PIXEL_AA, dist, thickness);
				curve2<fp>::raster_corner(*it1, *curve2<fp>::segments.begin(), set_pixel, thickness);
			}
		}
	}

	template <typename fp>
	template<typename TSetPixelAA>
	inline void curved_polygon2<fp>::rasterize_polygon_aa(TSetPixelAA& SET_PIXEL_AA, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness)
	{		
		close_path();
		auto points = to_points(thickness);
		if (points.back() == points.front())
			points.erase(points.begin() + points.size() - 1);
		auto set_pixel = [&](const int& x, const int& y) {SET_PIXEL_AA(x, y, 1.f); };
		rasterize_aa(SET_PIXEL_AA, thickness);
		return curve2<fp>::rasterize_scanline(points, set_pixel);
	}

	template <typename fp>
	inline std::vector<point2<fp>> curved_polygon2<fp>::to_points(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness) const
	{
		if (curve2<fp>::segments.empty())
			return {};

		auto dist = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(20.0f);
		std::vector<point2<fp>> points;
		if (thickness <= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f))
		{
			for (const auto& segment : curve2<fp>::segments)
				segment_to_points(segment, points, dist, true);
		}
		else
		{
			bool clockwise = is_clockwise();

			size_t count = curve2<fp>::segments.size();
			for (size_t i = 0; i < count; i++)
			{
				auto it1 = std::next(curve2<fp>::segments.begin(), i);
				auto it2 = std::next(curve2<fp>::segments.begin(), (i+1) % count);

				segment_exterior_to_points(*it1, points, dist, thickness, clockwise, false);

				//append extra corner point if needed:
				auto path1 = (*it1)->track((*it1)->length());
				auto path2 = (*it2)->track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero());

				//	find exterior points
				direction2 dir_outside = clockwise ? direction2<fp>::left() : direction2<fp>::right();
				direction2 dir1 = path1.second.rotate(direction2<fp>::down());
				direction2 dir2 = path2.second.rotate(direction2<fp>::down());
				const auto half_thickness = thickness * sixit::units::create_dimensionless_scalar<fp>(0.5f);
				low_level::dimensional_vector2 perp1 = path1.second.rotate(dir_outside).vec() * half_thickness;
				low_level::dimensional_vector2 perp2 = path2.second.rotate(dir_outside).vec() * half_thickness;

				//	find intersection
				auto e1 = path1.first.vec() + perp1;
				auto e2 = path2.first.vec() + perp2;
				
				typename curve2<fp>::path_point p1(e1, dir1);
				typename curve2<fp>::path_point p2(e2, dir2);

				std::optional<point2<fp>> intersection = curve2<fp>::find_path_intersection(p1, p2);
				if (intersection.has_value())
				{
					points.push_back(intersection.value());
				}
			}
		}
		return points;
	}


	template <typename fp>
	void curved_polygon2<fp>::segment_to_points(const typename curve2_primitive<fp>::pcurve_segment& segment,
												std::vector<point2<fp>>& points, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, bool skip_last_point) const
	{
		const auto segment_len = segment->length();
		switch (segment->get_primitive_type())
		{
		case curve2_primitive<fp>::primitive_type::line_segment:

			points.push_back(segment->track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero()).first);
			if (!skip_last_point)
				points.push_back(segment->track(segment_len).first);
			break;

		case curve2_primitive<fp>::primitive_type::arc:
		case curve2_primitive<fp>::primitive_type::qbezier:
		case curve2_primitive<fp>::primitive_type::cbezier:
		{
			fp point_count = low_level::mathf::max(fp(4.f), sixit::units::extract_dim_less_scalar(low_level::mathf::floor(segment_len / dist_between_points)));

			const auto delta = segment_len / sixit::units::create_dimensionless_scalar<fp>(point_count);
			if (skip_last_point)
				point_count = point_count - fp(1.f);
			
			for (fp i = fp(0.f); i <= point_count; i = i + fp(1.f))
				points.push_back(segment->track(delta * sixit::units::create_dimensionless_scalar<fp>(i)).first);
		}
		break;
		default: break;
		}
	}

	template <typename fp>
	void curved_polygon2<fp>::segment_exterior_to_points(const typename curve2_primitive<fp>::pcurve_segment& segment, std::vector<point2<fp>>& points, 
														 sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, 
														 sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness, bool clockwise, bool skip_last_point) const
	{
		//convert base point to exterior point
		auto extrude_point = [&](const std::pair<point2<fp>, rotation2<fp>>& pair) -> point2<fp>
		{
			const low_level::dimensional_vector2 perpendicular = pair.second.rotate(direction2<fp>::right()).vec() * thickness * sixit::units::create_dimensionless_scalar<fp>(0.5f);
			return clockwise
				? pair.first.vec() - perpendicular	// left point
				: pair.first.vec() + perpendicular; // right point
		};
		
		const auto segment_len = segment->length();
		switch (segment->get_primitive_type())
		{
		case curve2_primitive<fp>::primitive_type::line_segment:
		{
			points.push_back(extrude_point(segment->track(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f))));
			const auto extruded = extrude_point(segment->track(segment_len));
			if (!skip_last_point || points.empty() || extruded.distance(points.back()) > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(2.f))
				points.push_back(extruded);
			break;
		}

		case curve2_primitive<fp>::primitive_type::arc:
		case curve2_primitive<fp>::primitive_type::qbezier:
		case curve2_primitive<fp>::primitive_type::cbezier:
		{
			fp point_count = low_level::mathf::max(fp(4.f), sixit::units::extract_dim_less_scalar(low_level::mathf::floor(segment_len / dist_between_points)));
			fp delta = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(segment_len) / point_count;
			if (skip_last_point)
				point_count = point_count - fp(1.f);
			
			for (fp i = fp(0.f) ; i <= point_count; i = i + fp(1.f))
				points.push_back(extrude_point(segment->track(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(delta * i))));
		}
		break;
		default: break;
		}
	}

	template <typename fp>
	inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> curved_polygon2<fp>::square() const
	{
		auto points = to_points();
		if (points.back() == points.front())
			points.erase(points.begin() + points.size() - 1);

		return low_level::polygon2_impl<fp>::square(points);
	}

	template <typename fp>
	template <typename P2>
	inline bool curved_polygon2<fp>::is_inside(const P2& point) const
	{
		auto vec = point.vec();
		const auto min = curve2<fp>::bound_box.minn.vec();
		const auto max = curve2<fp>::bound_box.maxx.vec();
		
		if (vec.x < min.x || vec.x > max.x || vec.y < min.y || vec.y > max.y)
			return false;

		line_segment2 ls(point2(min.x - 10, vec.y), point);
		std::vector<point2<fp>> intersections = intersection(ls);		

		return intersections.size() % 2 == 1;
	}
}
}

#endif //sixit_geometry_curved_polygon2_impl_h_included

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