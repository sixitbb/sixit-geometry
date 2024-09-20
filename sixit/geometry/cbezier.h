/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_cbezier_h_included
#define sixit_geometry_cbezier_h_included

#include <tuple>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/curve_primitive.h"
#include "sixit/geometry/qbezier.h"
#include "sixit/geometry/line.h"

namespace sixit
{
    namespace geometry
    {
        template <typename fp>
        struct cbezier2 : low_level::four_points_holder<point2<fp>>, curve2_primitive<fp>
        {
            template <typename fp2> friend struct cbezier2;
            friend struct curve2<fp>;
        public:
            using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;

            inline cbezier2(const sixit::rw::comparsers::constructor_for_rw_tag&) : low_level::four_points_holder<point2<fp>>{ {}, {}, {}, {} } {}
            inline cbezier2(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3, const point2<fp>& p4) : low_level::four_points_holder<point2<fp>>{ p1, p2, p3, p4 }
            {
                if ((p1 == p2) && (p2 == p3) && (p3 == p4))
                    return;

                recalculate();
            }

            inline bool operator==(const cbezier2<fp>& o) const 
            {
                return  (cbezier2<fp>::p1().vec() == o.p1().vec()) &&
                    (cbezier2<fp>::p2().vec() == o.p2().vec()) &&
                    (cbezier2<fp>::p3().vec() == o.p3().vec()) && 
                    (cbezier2<fp>::p4().vec() == o.p4().vec());
            }
            template<typename fp2>
            inline bool operator==(const cbezier2<fp2>& o) const
            {
                return (this->p1() == o.p1()) &&
                    (this->p2() == o.p2()) &&
                    (this->p3() == o.p3()) &&
                    (this->p4() == o.p4());
            }

            [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2_primitive<fp>& other, size_t n = 1) const override
            {
                if (other.get_primitive_type() != get_primitive_type())
                    return false;

                // rationale: using dynamic cast to convert other object to its actual type for comparison

                return _for_test_only_approximate_eq(dynamic_cast<const cbezier2<fp>&>(other), n);
            }

            template<typename fp2>
            [[nodiscard]] inline bool _for_test_only_approximate_eq(const cbezier2<fp2>& other, size_t n = 1) const
            {
                return cbezier2<fp>::p1()._for_test_only_approximate_eq(other.p1(), n)
                    && cbezier2<fp>::p2()._for_test_only_approximate_eq(other.p2(), n)
                    && cbezier2<fp>::p3()._for_test_only_approximate_eq(other.p3(), n)
                    && cbezier2<fp>::p4()._for_test_only_approximate_eq(other.p4(), n);
            }
            inline typename curve2_primitive<fp>::primitive_type get_primitive_type() const { return curve2_primitive<fp>::primitive_type::cbezier; }
            inline class_value length() const
            {
                return _approximate_length;
            }
            inline bounds2<fp> bounds() const
            {
                return _approximate_bounds;
            }

            inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
            {
                auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
                auto fp_3 = sixit::units::create_dimensionless_scalar<fp>(3.f);
                auto s = fp_1 - t;
                return point2<fp>(cbezier2<fp>::p1().vec() * (s * s * s) + cbezier2<fp>::p2().vec() * (fp_3 * s * s * t) + 
                                    cbezier2<fp>::p3().vec() * (fp_3 * s * t * t) + cbezier2<fp>::p4().vec() * (t * t * t));
            }
            inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
            {
                auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
                auto fp_3 = sixit::units::create_dimensionless_scalar<fp>(3.f);
                auto fp_6 = sixit::units::create_dimensionless_scalar<fp>(6.f);
                auto s = fp_1 - t;
                low_level::dimensional_vector2 p10 = (cbezier2<fp>::p2().vec() - cbezier2<fp>::p1().vec()) / sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
                low_level::dimensional_vector2 p21 = (cbezier2<fp>::p3().vec() - cbezier2<fp>::p2().vec()) / sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
                low_level::dimensional_vector2 p32 = (cbezier2<fp>::p4().vec() - cbezier2<fp>::p3().vec()) / sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
                return direction2<fp>(p10 * (fp_3 * s * s) + p21 * (fp_6 * s * t) + p32 * (fp_3 * t * t));
            }

            inline point2<fp> track_curvature(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
            {
                auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
                auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
                auto fp_6 = sixit::units::create_dimensionless_scalar<fp>(6.f);
                low_level::dimensional_vector2 p210 = cbezier2<fp>::p3().vec() - cbezier2<fp>::p2().vec() * fp_2 + cbezier2<fp>::p1().vec();
                low_level::dimensional_vector2 p321 = cbezier2<fp>::p4().vec() - cbezier2<fp>::p3().vec() * fp_2 + cbezier2<fp>::p2().vec();
                return point2(p210 * (fp_6 * (fp_1 - t)) + p321 * (fp_6 * t));
            }
            inline class_value min_curvature_radius() const
            {
                return _approximate_min_curvature_r;
            }

            inline std::vector<typename curve2_primitive<fp>::pcurve_segment> to_qbezier(int count = 2) const;

            // virtual intersection methods with all other curve primitives
            inline void intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const { return other->intersection(intersections, *this); }
            inline void intersection(std::vector<point2<fp>>&, const line2<fp>&) const {  }
            inline void intersection(std::vector<point2<fp>>&, const line_segment2<fp>&) const {  }
            inline void intersection(std::vector<point2<fp>>&, const arc2<fp>&) const {  }
            inline void intersection(std::vector<point2<fp>>&, const qbezier2<fp>&) const {  }
            inline void intersection(std::vector<point2<fp>>&, const cbezier2&) const {  }

            inline bool intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const { return other->intersect(*this); }
            inline bool intersect(const line2<fp>&) const { return false; }
            inline bool intersect(const line_segment2<fp>&) const { return false; }
            inline bool intersect(const arc2<fp>&) const { return false; }
            inline bool intersect(const qbezier2<fp>&) const { return false; }
            inline bool intersect(const cbezier2&) const { return false; }

            template<typename TSetPixel>
            inline void rasterize(TSetPixel& SET_PIXEL) const;
            
            template<typename TSetPixelAA>
            inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const;

            template <typename This, typename ComparserT>
            static void read_write(This& obj, ComparserT& comparser)
            {
                // TODO: refactor to AofSTRUCT after TR-2362
                sixit::rw::begin_struct<"cbezier2">(comparser, obj);
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p3", obj.p3());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p4", obj.p4());
                sixit::rw::end_struct(comparser);
                comparser.postread(obj, &cbezier2::recalculate);
            }

          private:
            // for constructing empty objects when parsing
            cbezier2() : low_level::four_points_holder<point2<fp>>({}, {}, {}, {})
            {
            }

            // calculation of minimal curvature radius, length, bounds is complicated for cubic bezier
            // so we calculate these parameters in constructor and keep value
            void recalculate()
            {
                auto qbeziers = to_qbezier(4);
                _approximate_min_curvature_r = _min_curvature_radius(qbeziers);
                _approximate_length = _length(qbeziers);
                _approximate_bounds = _bounds(qbeziers);
            }

            inline class_value _min_curvature_radius(const std::vector<typename curve2_primitive<fp>::pcurve_segment>& qbeziers) const
            {
                typename std::vector<typename curve2_primitive<fp>::pcurve_segment>::const_iterator it = qbeziers.begin();
                auto r = (*it++)->min_curvature_radius();

                for (; it != qbeziers.end(); it++)
                    r = low_level::mathf::min(r, (*it)->min_curvature_radius());

                return r;
            }

            inline class_value _length(const std::vector<typename curve2_primitive<fp>::pcurve_segment>& qbeziers) const
            {
                typename std::vector<typename curve2_primitive<fp>::pcurve_segment>::const_iterator it = qbeziers.begin();
                auto l = (*it++)->length();

                for (; it != qbeziers.end(); it++)
                    l = l + (*it)->length();

                return l;
            }

            inline bounds2<fp> _bounds(const std::vector<typename curve2_primitive<fp>::pcurve_segment>& qbeziers) const
            {
                typename std::vector<typename curve2_primitive<fp>::pcurve_segment>::const_iterator it = qbeziers.begin();
                bounds2 out_bound = (*it++)->bounds();

                for (; it != qbeziers.end(); it++)
                {
                    bounds2<fp> bb = (*it)->bounds();
                    out_bound.expand_by(bb.minn);
                    out_bound.expand_by(bb.maxx);
                }

                return out_bound;
            }

        private:
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> _approximate_min_curvature_r = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> _approximate_length = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
            bounds2<fp> _approximate_bounds = bounds2<fp>(point2<fp>());
        };

        // hermit2: more convenient way to operate and modify qubic bezier curve
        template <typename fp>
        struct hermit2
        {
            friend class curv_builder<fp>;
        public:
            inline hermit2(const point2<fp>& p0, const point2<fp>& p1, const point2<fp>& v0, const point2<fp>& v1): p0(p0), p1(p1), v0(v0), v1(v1) {
                t0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
                t1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
            }
            inline hermit2(const point2<fp>& p0, const point2<fp>& p1, sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a0,
                                                                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a1): p0(p0), p1(p1)
            {
                auto fp_05 = sixit::units::create_dimensionless_scalar<fp>(0.5f);
                t0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
                t1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
                auto initial_tangent_length = fp_05 * (p0.vec() - p1.vec()).magnitude();
                v0 = point2(initial_tangent_length * low_level::mathf::cos(a0), initial_tangent_length * low_level::mathf::sin(a0));
                v1 = point2(initial_tangent_length * low_level::mathf::cos(a1), initial_tangent_length * low_level::mathf::sin(a1));
            }
            inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> t) const
            {
                auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
                auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
                auto fp_3 = sixit::units::create_dimensionless_scalar<fp>(3.f);

                auto dt = t1 - t0;
                auto s = (t - t0) / dt;
                auto c0 = (fp_2 * s + fp_1) * (s - fp_1) * (s - fp_1);
                auto c1 = (fp_3 - fp_2 * s) * s * s;
                auto c2 = (fp_1 - s) * (fp_1 - s) * s * dt;
                auto c3 = (s - fp_1) * s * s * dt;
                return point2(p0.vec() * c0 + p1.vec() * c1 + v0.vec() * c2 + v1.vec() * c3);
            }
            inline cbezier2<fp> to_cubic_bezier() const
            {
                auto fp_3 = sixit::units::create_dimensionless_scalar<fp>(3.f);
                auto dt = (t1 - t0) / fp_3;
                return cbezier2<fp>(p0, point2<fp>(p0.vec() + v0.vec() * dt), point2<fp>(p1.vec() - v1.vec() * dt), p1);
            }
            inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_curvature_r() const
            {
                auto cb = to_cubic_bezier();
                return cb.min_curvature_radius();
            }

        private:
            point2<fp> p0;
            point2<fp> p1;
            point2<fp> v0;
            point2<fp> v1;
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
        };
    }

    namespace geometry 
    {
        // aliases

        using cbezier2f = cbezier2<float>;
        using hermit2f = hermit2<float>;
    }
}

#endif //sixit_geometry_cbezier_h_included

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
