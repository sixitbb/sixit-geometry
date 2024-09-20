/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_curve_h_included
#define sixit_geometry_curve_h_included

#include <memory>
#include <vector>
#include <list>
#include <tuple>
#include <functional>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/curve_primitive.h"
#include "sixit/geometry/qbezier.h"
#include "sixit/geometry/cbezier.h"
#include "sixit/geometry/arc.h"
#include "sixit/geometry/shapes.h"
#include "sixit/geometry/line_segment.h"

namespace sixit
{
	namespace geometry
	{
		template<typename fp>
		struct curve2: curve2_primitive<fp>
		{
			typedef std::pair<point2<fp>, direction2<fp>> path_point;

            friend struct curved_polygon2<fp>;
			friend class for_importer_only_helper;

			class for_importer_only_helper
			{
			public:
				inline static bool append_dont_use_outside_of_importer(curve2<fp>& curve, const line_segment2<fp>& line)
				{
					return curve.append(std::make_unique<line_segment2<fp>>(line));
				}
				inline static bool append_dont_use_outside_of_importer(curve2<fp>& curve, const arc2<fp>& arc)
				{
					return curve.append(std::make_unique<arc2<fp>>(arc));
				}
				inline static bool append_dont_use_outside_of_importer(curve2<fp>& curve, const qbezier2<fp>& qbezier)
				{
					return curve.append(std::make_unique<qbezier2<fp>>(qbezier));
				}
				inline static bool append_dont_use_outside_of_importer(curve2<fp>& curve, const cbezier2<fp>& cbezier)
				{
					return curve.append(std::make_unique<cbezier2<fp>>(cbezier));
				}
			};

		public:
			using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;

			inline curve2() = default;
			inline curve2(const curve2<fp>& other) = delete;
			inline curve2(curve2<fp>&& other) = default;
			inline curve2(const path_point& initial_point): initial_point(initial_point), bound_box(initial_point.first, initial_point.first){}


            [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2_primitive<fp>& other, size_t n = 1) const override
            {
                if (other.get_primitive_type() != get_primitive_type())
                    return false;

                // rationale: using dynamic cast to convert other object to its actual type for comparison
                return _for_test_only_approximate_eq(dynamic_cast<const curve2<fp>&>(other), n);
            }

            [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2<fp>& other, size_t n = 1) const
            {
                if ((initial_point != other.initial_point) || (segments.size() != other.segments.size()))
                    return false;

                auto it1 = segments.begin();
                auto it2 = other.segments.begin();
                while (it1 != segments.end())
                {
                    const typename curve2_primitive<fp>::pcurve_segment& p_segment1 = *it1;
                    const typename curve2_primitive<fp>::pcurve_segment& p_segment2 = *it2;
                    if (!p_segment1 || !p_segment2 || !p_segment1->_for_test_only_approximate_eq(*p_segment2, n))
                        return false;
                    it1++;
                    it2++;
                }
                return true;
            }

			// initialization with primitive
			inline bool add_initial(const line_segment2<fp>&) { return true; }
			inline bool add_initial(const arc2<fp>&) { return true; }
			inline bool add_initial(const qbezier2<fp>&) { return true; }
			inline bool add_initial(const cbezier2<fp>&) { return true; }


			// one-primitive construct methods
			inline bool append(const shape_line2<fp>& line, const point2<fp>& dest_point);
			inline bool append(const shape_arc2<fp>& arc, const point2<fp>& dest_point);
			inline bool append_smooth(const shape_line2<fp>& line, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length);
			inline bool append_smooth(const shape_arc2<fp>& arc, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length);
			inline bool append_smooth(const shape_arc2<fp>& arc, const direction2<fp>& dest_dir);
			inline bool append_smooth(const shape_arc2_w_radius<fp>& arc, const point2<fp>& dest_point);
			inline bool append_smooth(const shape_qbezier2<fp>& qbezier, const path_point& dest_point);

			inline bool prepend(const shape_line2<fp>& line, const point2<fp>& dest_point);
			inline bool prepend(const shape_arc2<fp>& arc, const point2<fp>& dest_point);
			inline bool prepend_smooth(const shape_line2<fp>& line, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length);
			inline bool prepend_smooth(const shape_arc2<fp>& arc, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length);
			inline bool prepend_smooth(const shape_arc2<fp>& arc, const direction2<fp>& dest_dir);
			inline bool prepend_smooth(const shape_arc2_w_radius<fp>& arc, const point2<fp>& dest_point);
			inline bool prepend_smooth(const shape_qbezier2<fp>& qbezier, const path_point& dest_point);

			// multi-primitive construct methods
			inline bool append_smooth(const shape_line_arc_line2<fp>& composite_shape, const path_point& dest_point);
			inline void append_smooth(const shape_arc_line_arc2<fp>& composite_shape, const path_point& dest_point);
			inline void append_smooth(const shape_composite_cbezier2<fp>& composite_shape, const path_point& dest_point);
			inline bool prepend_smooth(const shape_line_arc_line2<fp>& composite_shape, const path_point& dest_point);
			inline void prepend_smooth(const shape_arc_line_arc2<fp>& composite_shape, const path_point& dest_point);
			inline void prepend_smooth(const shape_composite_cbezier2<fp>& composite_shape, const path_point& dest_point);

			// primitive methods
			inline typename curve2_primitive<fp>::primitive_type get_primitive_type() const override { return  curve2_primitive<fp>::primitive_type::curve; }
			inline std::pair<point2<fp>, rotation2<fp>> track(class_value x) const override;
			inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const override;
			inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const override;
			inline bounds2<fp> bounds() const override { return bound_box; }
			inline class_value length() const override { return curve_length; }
			inline class_value min_curvature_radius() const override { return min_curv_radius; }

			// intersections methods
			inline std::vector<point2<fp>> intersection(const line2<fp>& line) const;
			inline std::vector<point2<fp>> intersection(const line_segment2<fp>& line_segment) const;
			inline std::vector<point2<fp>> intersection(const curve2<fp>& other) const;

			inline void intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const arc2<fp>& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const qbezier2<fp>& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const override;
			inline void intersection(std::vector<point2<fp>>& intersections, const curve2<fp>& other) const override;

			inline bool intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const override;
			inline bool intersect(const line2<fp>& other) const override;
			inline bool intersect(const line_segment2<fp>& other) const override;
			inline bool intersect(const arc2<fp>& other) const override;
			inline bool intersect(const qbezier2<fp>& other) const override;
			inline bool intersect(const cbezier2<fp>& other) const override;
			inline bool intersect(const curve2<fp>& other) const override;

			inline void to_quadbeziers();

            //rasterize
			template<typename TSetPixel>
			inline void rasterize(TSetPixel& SET_PIXEL, class_value thickness = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)) const;

			template<typename TSetPixel>
			inline void raster_segment(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixel& SET_PIXEL, class_value dist_between_points) const;

			template<typename TSetPixel>
			inline void raster_segment(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixel& SET_PIXEL, class_value dist_between_points, class_value thickness) const;

			template<typename TSetPixel>
			inline static void raster_corner(const typename curve2_primitive<fp>::pcurve_segment& segment1, const typename curve2_primitive<fp>::pcurve_segment& segment2, 
												TSetPixel& SET_PIXEL, class_value thickness);

            //rasterize anti-aliasing
            template<typename TSetPixelAA>
			inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA, class_value thickness = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)) const;

			template<typename TSetPixelAA>
			inline void raster_segment_aa(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixelAA& SET_PIXEL_AA, class_value dist_between_points) const;

			template<typename TSetPixelAA>
			inline void raster_segment_aa(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixelAA& SET_PIXEL_AA, class_value dist_between_points, class_value thickness) const;

			inline static std::optional<point2<fp>> find_path_intersection(const typename curve2<fp>::path_point& path1, const curve2<fp>::path_point& path2);

			struct raster_edge
			{
				int ymin, ymax;
				fp slope, x;
			};
			
			static inline std::vector<raster_edge> create_edge_table(const std::vector<point2<fp>>& points);

			template<typename TSetPixel>
			static inline void rasterize_scanline(const std::vector<point2<fp>>& points, TSetPixel& SET_PIXEL);

        private:

            // for serialization
            struct curve_segment
            {
                typename curve2_primitive<fp>::primitive_type ptype = curve2_primitive<fp>::primitive_type::base;
                line_segment2<fp> line = line_segment2<fp>(point2<fp>(), point2<fp>());
                arc2<fp> arc = arc2<fp>(point2<fp>(), sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero(), 
											sixit::units::create_dimensionless_scalar<fp>(0.0f), sixit::units::create_dimensionless_scalar<fp>(0.0f));
                qbezier2<fp> qbezier = qbezier2<fp>(point2<fp>(), point2<fp>(), point2<fp>());
                cbezier2<fp> cbezier = cbezier2<fp>(point2<fp>(), point2<fp>(), point2<fp>(), point2<fp>());

                template <typename This, typename ComparserT>
                static void read_write(This& obj, ComparserT& comparser)
                {
                    using with_wire_default_helper_base = typename ComparserT::aliases::template with_wire_default_helper<curve2_primitive<fp>::primitive_type::base>;
                    using with_filter_helper_line_segment = typename ComparserT::aliases::template with_filter_helper<curve2_primitive<fp>::primitive_type::line_segment>;
                    using with_filter_helper_arc = typename ComparserT::aliases::template with_filter_helper<curve2_primitive<fp>::primitive_type::arc>;
                    using with_filter_helper_qbezier = typename ComparserT::aliases::template with_filter_helper<curve2_primitive<fp>::primitive_type::qbezier>;
                    using with_filter_helper_cbezier = typename ComparserT::aliases::template with_filter_helper<curve2_primitive<fp>::primitive_type::cbezier>;

                    sixit::rw::begin_struct<"curve_segment">(comparser, obj);
                    sixit::rw::read_write<&curve_segment::ptype, "type", sixit::rw::ENUM>(comparser, with_wire_default_helper_base());
                    sixit::rw::read_write<&curve_segment::line, "line", sixit::rw::STRUCT>(comparser, with_filter_helper_line_segment(obj.ptype));
                    sixit::rw::read_write<&curve_segment::arc, "arc", sixit::rw::STRUCT>(comparser, with_filter_helper_arc(obj.ptype));
                    sixit::rw::read_write<&curve_segment::qbezier, "qbezier", sixit::rw::STRUCT>(comparser, with_filter_helper_qbezier(obj.ptype));
                    sixit::rw::read_write<&curve_segment::cbezier, "cbezier", sixit::rw::STRUCT>(comparser, with_filter_helper_cbezier(obj.ptype));
                    sixit::rw::end_struct(comparser);
				}
            };

            template <typename ComparserT>
            struct curve_segment_converter : public ComparserT::aliases::template is_element_validating_converter<void>
            {
                using ConvertedType = curve_segment;
                using MemberT = typename curve2_primitive<fp>::pcurve_segment;
                auto create() { return curve_segment();}

                auto element2value(const auto& val)
                {
                    curve_segment cs;
                    cs.ptype = val->get_primitive_type();
                    switch (cs.ptype)
                    {
                    case curve2_primitive<fp>::primitive_type::line_segment: cs.line = dynamic_cast<line_segment2<fp>&>(*val); break;
                    case curve2_primitive<fp>::primitive_type::arc: cs.arc = dynamic_cast<arc2<fp>&>(*val); break;
                    case curve2_primitive<fp>::primitive_type::qbezier: cs.qbezier= dynamic_cast<qbezier2<fp>&>(*val); break;
                    case curve2_primitive<fp>::primitive_type::cbezier: cs.cbezier = dynamic_cast<cbezier2<fp>&>(*val); break;
                    }
                    return typename ComparserT::aliases::template conversion_result<curve_segment>(ComparserT::aliases::qualifier_outcomes::converted, cs);
                }

                auto value2element(auto& val)
                {
                    typename curve2_primitive<fp>::pcurve_segment ps;

                    switch (val.ptype)
                    {
                    case curve2_primitive<fp>::primitive_type::line_segment:
                        ps = std::make_unique<line_segment2<fp>>(val.line);
                        break;
                    case curve2_primitive<fp>::primitive_type::arc:
                        ps = std::make_unique<arc2<fp>>(val.arc);
                        break;
                    case curve2_primitive<fp>::primitive_type::qbezier:
                        ps = std::make_unique<qbezier2<fp>>(val.qbezier);
                        break;
                    case curve2_primitive<fp>::primitive_type::cbezier:
                        ps = std::make_unique<cbezier2<fp>>(val.cbezier);
                        break;
                    case curve2_primitive<fp>::primitive_type::base:
                    default:
                        assert(false);
                    }

                    return std::make_tuple(true, std::move(ps));
                }
            };

        public:
            template <typename This, typename ComparserT>
			static void read_write(This& obj, ComparserT& comparser)
			{
				using namespace sixit::rw;
				using sixit::rw::read_write;

				begin_struct<"curve2">(comparser, obj);
				read_write_runtime_name_slower_than_complie_time_name<typename ComparserT::aliases::STRUCT>(comparser, "initial_point", obj.initial_point.first);
				read_write_runtime_name_slower_than_complie_time_name<typename ComparserT::aliases::STRUCT>(comparser, "initial_direction", obj.initial_point.second);
				read_write<&curve2::segments, "segments", typename ComparserT::aliases::VofSTRUCT>(comparser, curve_segment_converter<ComparserT>());
				end_struct(comparser);
				comparser.postread(obj, &curve2::recalculate);
			}

        private:

			inline void add_initial(typename curve2_primitive<fp>::pcurve_segment&& p);
			inline bool append(typename curve2_primitive<fp>::pcurve_segment&& p);
			inline void append(std::vector<typename curve2_primitive<fp>::pcurve_segment>& primitives);
			inline bool prepend(typename curve2_primitive<fp>::pcurve_segment&& p);
			inline void prepend(std::vector<typename curve2_primitive<fp>::pcurve_segment>& primitives);
			inline void update(const bounds2<fp>& b, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> l,
								 sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r);
			inline void recalculate();

			inline point2<fp> end_point() const;
			inline point2<fp> start_point() const;
			inline direction2<fp> end_tangent() const;
			inline direction2<fp> start_tangent() const;

		private:

			path_point initial_point = path_point({point2<fp>(), direction2<fp>(degrees(fp(0.0f)))});
			std::vector<typename curve2_primitive<fp>::pcurve_segment> segments;

			bounds2<fp> bound_box = bounds2(point2<fp>(), point2<fp>());
			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> curve_length = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
			sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_curv_radius = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity);
		};

		template<typename fp>
		class curv_builder
		{
			friend struct curve2<fp>;
            friend struct curved_polygon2<fp>;

		private:
			inline static const fp arc_threshold = fp(0.1f);

			inline static std::vector<typename curve2_primitive<fp>::pcurve_segment> build(const shape_composite_cbezier2<fp>& shape, const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1);
			inline static std::vector<typename curve2_primitive<fp>::pcurve_segment> build(const shape_line_arc_line2<fp>& shape, const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1);
			inline static std::vector<typename curve2_primitive<fp>::pcurve_segment> build(const shape_arc_line_arc2<fp>& shape, const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1);

			inline static hermit2<fp> build_optimized_hermit(const typename curve2<fp>::path_point& p0, const typename curve2<fp>::path_point& p1, 
																sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_radius);
			inline static hermit2<fp> optimize_hermit(const hermit2<fp>& h, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_r);
			inline static hermit2<fp> optimize_hermit2(const hermit2<fp>& h, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_r);

			inline static std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>> build_arc_line_arc_same_dir(const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1, 
																											bool cw, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r);
			inline static std::optional<std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>>> build_arc_line_arc_opp_dir(const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1, 
																										bool cw_0, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r);

			inline static sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> arc_line_arc_length(const std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>> ala);
		};
	}

	namespace geometry
	{
		// aliases
		using curve2f = curve2<float>;
		using curv_builderf = curv_builder<float>;
	}
}

#endif //sixit_geometry_curve_h_included

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
