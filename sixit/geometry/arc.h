/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_arc_h_included
#define sixit_geometry_arc_h_included

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
        struct arc2 : curve2_primitive<fp>
        {
          public:
            using class_value = typename point2<fp>::class_value;

            point2<fp> center;
            class_value radius;
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle0;
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle1;

          public:
            inline arc2(const sixit::rw::comparsers::constructor_for_rw_tag&) : center{}, 
            radius(decltype(radius)::zero()), angle0{decltype(angle0)::zero()}, angle1{decltype(angle1)::zero()} {}
            inline arc2(const point2<fp>& c, class_value r, 
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a0, 
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a1) : center(c), radius(r), angle0(a0), angle1(a1)
            {
                // since arc can have difference between angles  more than pi
                // we need make some agreements about possible diapason of angles
                // if a0 < a1 it means that arc goes in CCW from a0 to a1
                // if a1 < a0 it means that arc goes in CW from a0 to a1
                // a0 always in [-pi, pi] and a1 shoud be normalized relative to a0 by function norm_angle
                // diapason of angles very important for intersection functions
            }
            inline bool operator==(const arc2<fp>& o) const;

            [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2_primitive<fp>& other, size_t n = 1) const override
            {
                if (other.get_primitive_type() != get_primitive_type())
                    return false;

                // rationale: using dynamic cast to convert other object to its actual type for comparison

                return _for_test_only_approximate_eq(dynamic_cast<const arc2<fp>&>(other), n);
            }

            [[nodiscard]] inline bool _for_test_only_approximate_eq(const arc2<fp>& other, size_t n = 1) const
            {
                return center._for_test_only_approximate_eq(other.center, n)
                    && sixit::dmath::test_helpers::approximate_eq(radius, other.radius, n)
                    && sixit::dmath::test_helpers::approximate_eq(angle0, other.angle0, n)
                    && sixit::dmath::test_helpers::approximate_eq(angle1, other.angle1, n);
            }
            inline typename curve2_primitive<fp>::primitive_type get_primitive_type() const { return curve2_primitive<fp>::primitive_type::arc; }
            inline class_value length() const;
            inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
            inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
            inline bounds2<fp> bounds() const;
            inline class_value min_curvature_radius() const { return radius; }



            inline bool is_in_arc(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a) const
            {
                auto fp_two_pi = sixit::units::create_dimensionless_scalar<fp>(2.0f * low_level::mathf::pi);
                if (angle1 > angle0)
                {
                    // make input angle more then angle0
                    auto norm_a = a < angle0 ? a + fp_two_pi : a;
                    return norm_a <= angle1;
                }
                else
                {
                    // make input angle less than angle 0
                    auto norm_a = a > angle0 ? a - fp_two_pi : a;
                    return norm_a >= angle1;
                }
            }

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
            inline bool intersect(const arc2<fp>&other) const;
            inline bool intersect(const qbezier2<fp>& other) const;
            inline bool intersect(const cbezier2<fp>& other) const;

            template<typename TSetPixel>
            inline void rasterize(TSetPixel& SET_PIXEL, class_value dist_between_points = 10.0f) const;

            template<typename TSetPixelAA>
            inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA, class_value dist_between_points = 10.0f) const;

            // service function
            inline bool point_in_both_arcs(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p, const arc2& other) const;

            //static inline sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> 
            static inline auto 
                    angle_from_radius_vector(const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& r_vec)
            {
                return low_level::mathf::atan2(r_vec.y, r_vec.x);
            }
            static inline fp angle_to_positive(fp a)
            {
                fp fp_0(0.f);
                fp fp_two_pi(2.0f * low_level::mathf::pi);
                return a >= fp_0 ? a : a + fp_two_pi;
            }
            static inline low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> radius_vector_from_tangent(const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& t_vec, bool cw)
            {
                return cw ? low_level::dimensional_vector2(-t_vec.y, t_vec.x) : low_level::dimensional_vector2(t_vec.y, -t_vec.x);
            }
            static inline sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> 
            norm_angle(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a0, 
                        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a1, bool cw)
            {
                auto fp_two_pi = sixit::units::create_dimensionless_scalar<fp>(2.0f * low_level::mathf::pi);
                return cw ? ((a1 > a0) ? a1 - fp_two_pi : a1) : ((a1 < a0) ? a1 + fp_two_pi : a1);
            }

            template<typename This, typename ComparserT>
            static void read_write(This& obj, ComparserT& comparser) 
            {
                sixit::rw::begin_struct<"arc2">(comparser, obj);
                sixit::rw::read_write<&arc2::center, "center", sixit::rw::STRUCT>(comparser);
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "radius", obj.radius);
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "angle0", obj.angle0);
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "angle1", obj.angle1);
                sixit::rw::end_struct(comparser);
            }
        };
    }
    namespace geometry 
    {
        using arc2f = arc2<float>;
    }
}

#endif //sixit_geometry_arc_h_included

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
