/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_qbezier_h_included
#define sixit_geometry_qbezier_h_included

#include <optional>
#include <utility>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/curve_primitive.h"


namespace sixit
{
    namespace geometry
    {
        template <typename fp>
        struct ellitical_arc_parameters
        {
            using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;

            point2<fp> start;
            point2<fp> end;
            class_value rx;
            class_value ry;
            class_value x_axis_rotation;
            bool large_arc_flag;
            bool sweep_flag;

            template <typename This, typename ComparserT>
            static void read_write(This& obj, ComparserT& comparser)
            {
                sixit::rw::begin_struct<"ellitical_arc_parameters">(comparser, obj);
                sixit::rw::read_write<&ellitical_arc_parameters::start, "start", sixit::rw::STRUCT>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::end, "end", sixit::rw::STRUCT>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::rx, "rx", sixit::rw::F32>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::ry, "ry", sixit::rw::F32>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::x_axis_rotation, "x_axis_rotation", sixit::rw::F32>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::large_arc_flag, "large_arc_flag", sixit::rw::BOOL>(comparser);
                sixit::rw::read_write<&ellitical_arc_parameters::sweep_flag, "sweep_flag", sixit::rw::BOOL>(comparser);
                sixit::rw::end_struct(comparser);
            }
        };
        
        template <typename fp>
        struct qbezier2 : low_level::three_points_holder<point2<fp>>, curve2_primitive<fp>
        {
            template <typename fp2> friend struct qbezier2;
            friend struct curve2<fp>;
          public:
            using class_value = typename point2<fp>::class_value;

            inline qbezier2(const sixit::rw::comparsers::constructor_for_rw_tag&) : low_level::three_points_holder<point2<fp>>{ {}, {}, {} } {}
            inline qbezier2(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3) : low_level::three_points_holder<point2<fp>>{ p1, p2, p3 } 
            {
                recalculate();
            }
            inline bool operator==(const qbezier2& o) const;
            template<typename fp2>
            inline bool operator==(const qbezier2<fp2>& o) const;

            [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2_primitive<fp>& other, size_t n = 1) const override
            {
                if (other.get_primitive_type() != get_primitive_type())
                    return false;

                // rationale: using dynamic cast to convert other object to its actual type for comparison
                return _for_test_only_approximate_eq(dynamic_cast<const qbezier2<fp>&>(other), n);
            }

            template<typename fp2>
            [[nodiscard]] inline bool _for_test_only_approximate_eq(const qbezier2<fp2>& other, size_t n = 1) const
            {
                return this->p1()._for_test_only_approximate_eq(other.p1(), n)
                    && this->p2()._for_test_only_approximate_eq(other.p2(), n)
                    && this->p3()._for_test_only_approximate_eq(other.p3(), n);
            }
            inline typename curve2_primitive<fp>::primitive_type get_primitive_type() const { return curve2_primitive<fp>::primitive_type::qbezier; }
            inline class_value length() const { return _length_internal; }
            inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
            inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
            inline bounds2<fp> bounds() const;
            inline class_value min_curvature_radius() const;

            // virtual intersection methods with all other curve primitives
            inline void intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const;
            inline void intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const;
            inline void intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const;
            inline void intersection(std::vector<point2<fp>>& intersections, const arc2<fp>& other) const;
            inline void intersection(std::vector<point2<fp>>& intersections, const qbezier2<fp>& other) const;
            inline void intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const;

            inline bool intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const;
            inline bool intersect(const line2<fp>& other) const;
            inline bool intersect(const line_segment2<fp>& other) const;
            inline bool intersect(const arc2<fp>& other) const;
            inline bool intersect(const qbezier2<fp>& other) const;
            inline bool intersect(const cbezier2<fp>& other) const;

            inline static std::optional<qbezier2<fp>> build2(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p0, 
															    const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p1, 
															    const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& t0, 
															    const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& t1);
            inline static std::vector<qbezier2<fp>> elliptical_arc_to_qbeziers(const ellitical_arc_parameters<fp>& param);
            
            template<typename TSetPixel>
            inline void rasterize(TSetPixel& SET_PIXEL) const;

            template<typename TSetPixel>
            inline void rasterize_qsegment(int x0, int y0, int x1, int y1, int x2, int y2, TSetPixel& SET_PIXEL) const;

            template<typename TSetPixelAA>
            inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const;

            template<typename TSetPixelAA>
            inline void rasterize_qsegment_aa(int x0, int y0, int x1, int y1, int x2, int y2, TSetPixelAA& SET_PIXEL_AA) const;

            template <typename This, typename ComparserT>
            static void read_write(This& obj, ComparserT& comparser)
            {
                // TODO: refactor to AofSTRUCT after TR-2362
                sixit::rw::begin_struct<"qbezier2">(comparser, obj);
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p3", obj.p3());
                sixit::rw::end_struct(comparser);
                comparser.postread(obj, &qbezier2::recalculate);
            }

          private:
            void recalculate()
            {
                _length_internal = _length();
            }

            inline class_value _length() const;
            inline std::pair<std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> >, 
                std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> >> 
                    line_intersection_params(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& lp1, 
                                             const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& lp2) const;

            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> _length_internal = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
        };
    }

    namespace geometry 
    {
        using qbezier2f = qbezier2<float>;
        using ellitical_arc_parametersf = ellitical_arc_parameters<float>;
    }
} // namespace sixit

#endif //sixit_geometry_qbezier_h_included

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
