/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_line_h_included
#define sixit_geometry_line_h_included

#include <optional>
#include <tuple>
#include <array>
#include <utility>

#include "sixit/geometry/rotation.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/plane_common_impl.h"
#include "sixit/geometry/low_level/line_common_impl.h"
#include "sixit/geometry/points_span.h"
//#include "sixit/core/containers.h"
#include "sixit/geometry/curve_primitive.h"

#include "sixit/core/guidelines.h"

namespace sixit
{
namespace geometry
{
    template<typename T>
    using triple = std::tuple<T,T,T>;

    template<typename T>
    using couple = std::tuple<T,T>;

    enum class XYZ
    {
        X = 0, Y = 1, Z = 2
    };

    template <typename fp>
    struct line3: low_level::two_points_holder<point3<fp>>
    {
        friend sixit::graphics::intersectable_mesh;
        friend struct projection_perspective3<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line3<fp>>;
        friend struct plane3<fp>;

        using projectionT = line2<fp>;

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES(sixit::vlo, T)
        inline T _get_projection_points() const
        {
            T span;
            span.reserve(2);
            span.push_back(line3<fp>::p1());
            span.push_back(line3<fp>::p2());
            return span;
        }

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES_FP(sixit::geometry::points3_span, T, fp)
        inline line3(const T& span): low_level::two_points_holder<point3<fp>>{span[0], span[1]} {}

      public:
        inline line3(const point3<fp>& p1, const point3<fp>& p2): low_level::two_points_holder<point3<fp>>{p1, p2} {}

        inline bool operator==(const line3<fp>& other) const
        {
            return line3<fp>::p1() == other.p1() && line3<fp>::p2() == other.p2();
        }

        inline bounds3<fp> bounds() const
        {
            return low_level::ls3_impl<fp>::bounds(line3<fp>::p1(), line3<fp>::p2());
        }

        template<typename fp2>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const line3<fp2>& other, size_t n = 1) const
        {
            return line3<fp>::p1()._for_test_only_approximate_eq(other.p1()) 
                && line3<fp>::p2()._for_test_only_approximate_eq(other.p2());
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            // TODO: refactor to AofSTRUCT after TR-2362
            sixit::rw::begin_struct<"line3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct line2: low_level::two_points_holder<point2<fp>>
    {
        friend struct arc2<fp>;
        friend struct qbezier2<fp>;
        friend struct projection2<fp>;
        friend struct projection2to1<fp>;
        friend struct projection2to1ex<fp>;
        friend struct projection2to1noscale<fp>;
        friend struct mirror_transform2<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line2<fp>>;
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> _ltr(const point2<fp>& p) const;

        using projectionT = line1<fp>;

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES(sixit::vlo, T)
        inline T _get_projection_points() const
        {
            T span;
            span.reserve(2);
            span.push_back(line2<fp>::p1());
            span.push_back(line2<fp>::p2());
            return span;
        }

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES_FP(sixit::geometry::points2_span, T, fp)
            inline line2(const T& span) : low_level::two_points_holder<point2<fp>>{ span[0], span[1] } {}

      public:
        inline line2(const sixit::rw::comparsers::constructor_for_rw_tag&) : low_level::two_points_holder<point2<fp>>{ {}, {} } {}
        inline line2(const point2<fp>& p1, const point2<fp>& p2): low_level::two_points_holder<point2<fp>>{p1, p2} {};
        inline bool strict_intersects(const line_segment2<fp>& ls2) const;
        inline bool intersects(const line_segment2<fp>& ls2) const;
        inline std::optional<point2<fp>> strict_intersection(const line_segment2<fp>& ls2) const;

        inline bool operator==(const line2& other) const
        {
            return line2<fp>::p1() == other.p1() && line2<fp>::p2() == other.p2();
        }

        inline bounds2<fp> bounds() const
        {
            return low_level::ls2_impl<fp>::bounds(line2<fp>::p1(), line2<fp>::p2());
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            // TODO: refactor to AofSTRUCT after TR-2362
            sixit::rw::begin_struct<"line2">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct line1: low_level::two_points_holder<point1<fp>>
    {
        friend struct projection2to1noscale<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line1<fp>>;

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES(sixit::vlo, T)
        inline T _get_projection_points() const
        {
            T span;
            span.reserve(2);
            span.push_back(line1<fp>::p1());
            span.push_back(line1<fp>::p2());
            return span;
        }

        template <typename T>
        SIXIT_LWA_OPTIONAL_REQUIRES_FP(sixit::geometry::points1_span, T, fp)
            inline line1(const T& span) : low_level::two_points_holder<point1<fp>>{ span[0], span[1] } {}

      public:
        inline line1(const point1<fp>& p1, const point1<fp>& p2): low_level::two_points_holder<point1<fp>>{p1, p2} {};

        inline bounds1<fp> bounds() const
        {
            return low_level::ls1_impl<fp>::bounds(line1<fp>::p1(), line1<fp>::p2());
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            // TODO: refactor to AofSTRUCT after TR-2362
            sixit::rw::begin_struct<"line1">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
            sixit::rw::end_struct(comparser);
        }
};

    template <typename fp>
    struct ray3
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::ray3<fp>>;

      private:
        point3<fp> origin;
        direction3<fp> direction;

      public:
        inline ray3(const point3<fp>& p, const direction3<fp>& v);
        inline ray3(const point3<fp>& p1, const point3<fp>& p2);
        inline line3<fp> line() const;
        inline point3<fp> track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> p) const;
        inline bounds3<fp> bounds() const
        {
            return bounds3<fp>(origin);
        }

    };

    template <typename fp>
    struct ray2
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::ray2<fp>>;
        friend struct line_segment2<fp>;
        friend struct line2<fp>;
        friend struct triangle2<fp>;
        friend struct polygon2<fp>;
        friend struct low_level::tri2_impl<fp>;

      private:
        point2<fp> origin_data;
        direction2<fp> direction_data;

        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> _ltr(const point2<fp>& p) const;

      public:
        inline direction2<fp> direction() const;
        inline point2<fp> origin() const;
        inline ray2(const point2<fp>& p, const direction2<fp>& v);
        inline ray2(const point2<fp>& p1, const point2<fp>& p2);
        inline line2<fp> line() const;
        inline point2<fp> track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> p) const;
        inline bool strict_intersects(const line_segment2<fp>& ls2) const;
        inline bool intersects(const line_segment2<fp>& ls2) const;
        inline std::optional<point2<fp>> strict_intersection(const line_segment2<fp>& ls2) const;
        inline bounds2<fp> bounds() const
        {
            return bounds2<fp>(origin_data);
        }
    };

    template <typename fp, XYZ axis>
    struct axis_oriented_ray2
    {
    private:
        const int32_t coord_index;
        point2<fp> origin_data;

        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> _ltr(const point2<fp>& p) const
        {
            if (p.vec()[int(coord_index)] < origin_data.vec()[int(coord_index)])
                return  sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(-1.f);

            if (p.vec()[int(coord_index)] > origin_data.vec()[int(coord_index)])
                return sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(1.f);

            return sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>::zero();
        }

        inline static std::pair<point2<fp>, point2<fp>> get_points(const line_segment2<fp>& ls2)
        {
            static_assert(axis == XYZ::X || axis == XYZ::Y);
            return std::make_pair(ls2.p1(), ls2.p2());
        }
        
    public:
        inline axis_oriented_ray2(const point2<fp>& p) : 
            coord_index(axis == XYZ::X ? 1 : 0),
            origin_data(p)
        {}

        inline direction2<fp> direction() const
        {
            return coord_index == 1 ? direction2(
                    sixit::units::create_dimensionless_scalar<fp>(1.f), 
                    sixit::units::create_dimensionless_scalar<fp>(0.f)) : 
                direction2(
                    sixit::units::create_dimensionless_scalar<fp>(0.f), 
                    sixit::units::create_dimensionless_scalar<fp>(1.f));
        }

        inline point2<fp> origin() const
        {
            return origin_data;
        }
        
        inline line2<fp> line() const
        {
            return line2<fp>(origin_data, origin_data.vec() + direction().vec());
        }
        
        inline point2<fp> track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> p) const
        {
            return point2(origin_data.vec() + direction().vec() * p);
        }

        inline bounds2<fp> bounds() const
        {
            return bounds2<fp>(origin_data);
        }

        inline bool intersects(const line_segment2<fp>& ls2) const
        {
            std::pair<point2<fp>, point2<fp>> line_segment2_points = get_points(ls2);
            auto d1 = _ltr(line_segment2_points.first);
            auto d2 = _ltr(line_segment2_points.second);

            if (d1 != decltype(d1)::zero() && d2 != decltype(d2)::zero())
            {
                if ((d1 < decltype(d1)::zero()) == (d2 < decltype(d2)::zero()))
                    return false;
                if ((d1 > decltype(d1)::zero()) == (d2 > decltype(d2)::zero()))
                    return false;
            }

            low_level::dimensional_vector2 s = line_segment2_points.second.vec() - line_segment2_points.first.vec();

            auto dev = coord_index == 1 ? s.y : -s.x;

            low_level::dimensional_vector2 p = origin_data.vec();
            low_level::dimensional_vector2 q = line_segment2_points.first.vec();

            low_level::dimensional_vector2 qp = q - p;

            auto t = (qp.x * s.y - qp.y * s.x) / dev;
            return sixit::geometry::low_level::mathf::isfinite(t) && t >= decltype(t)::zero() ? true : false;
        }

        inline bool strict_intersects(const line_segment2<fp>& ls2) const
        {
            if (!intersects(ls2))
                return false;

            std::pair<point2<fp>, point2<fp>> line_segment2_points = get_points(ls2);
            auto d2 = _ltr(line_segment2_points.second);

            if (d2 == decltype(d2)::zero())
                return false;
            return true;
        }

        inline std::optional<point2<fp>> strict_intersection(const line_segment2<fp> &ls2) const
        {
            if (!strict_intersects(ls2))
                return {};

            low_level::dimensional_vector2 r = direction().vec();
            std::pair<point2<fp>, point2<fp>> line_segment2_points = get_points(ls2);
            low_level::dimensional_vector2 s = line_segment2_points.second.vec() - line_segment2_points.first.vec();

            auto dev = coord_index == 1 ? s.y : -s.x;

            low_level::dimensional_vector2 p = origin_data.vec();
            low_level::dimensional_vector2 q = line_segment2_points.first.vec();

            low_level::dimensional_vector2 qp = q - p;

            auto t = (qp.x * s.y - qp.y * s.x) / dev;
            if (sixit::geometry::low_level::mathf::isfinite(t))
                return point2<fp>(p + r * t);
            else
                return {};
        }
    };

    template <typename fp>
    struct bounds3
    {
        friend class voxelizer;
        friend sixit::graphics::intersectable_mesh;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::bounds3<fp>>;
        friend struct triangle3<fp>;
        friend struct low_level::ls3_impl<fp>;
        friend struct low_level::tri3_impl<fp>;
        friend struct for_renderers_only::frustum<fp>;
        template <typename fp1>
        friend bounds3<fp1> low_level::for_renderers_only::bounds3_projected_dont_use_outside_of_renderer(const trs3<fp1>& trs, const bounds3<fp1>& bounds);

      private:
        point3<fp> minn;
        point3<fp> maxx;

        bounds3() = delete;
        inline bounds3(point3<fp> minn, point3<fp> maxx): minn(minn), maxx(maxx) {};

      public:
        inline bounds3(const point3<fp>& o): minn(point3<fp>::max_point()), maxx(point3<fp>::min_point())
        {
            expand_by(o);
        }
        inline bounds3(const bounds3& o) = default;
        bounds3& operator=(const bounds3&) = default;
        inline bool intersects(const bounds3& o) const;
        inline bool is_exactly_on_boundary(const point3<fp>& p) const;
        inline bool is_inside(const point3<fp>& p) const;
        inline void expand_by(const point3<fp>& p);
        inline void expand_by(const bounds3<fp>& other);

        inline bool operator==(const bounds3& other) const
        {
            return minn == other.minn && maxx == other.maxx;
        }

        template<typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser) 
        {
            sixit::rw::begin_struct<"bounds3">(comparser, obj);
            sixit::rw::read_write<&bounds3::minn, "min", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&bounds3::maxx, "max", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct bounds2
    {
        template <typename fp2> friend struct bounds2;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::bounds2<fp>>;
        friend struct triangle2<fp>;
        friend struct low_level::ls2_impl<fp>;
        friend struct low_level::tri2_impl<fp>;
        friend struct low_level::polygon2_impl<fp>;
        friend struct curve2<fp>;
        friend struct curved_polygon2<fp>;
        friend struct arc2<fp>;
        friend struct qbezier2<fp>;
        friend struct cbezier2<fp>;
        friend class grid2<fp>;
        template <class Triangle, typename fp1> friend class triangle2_stack;

      private:
        point2<fp> minn;
        point2<fp> maxx;

        bounds2() = delete;
        inline bounds2(const point2<fp>& minn, const point2<fp>& maxx): minn(minn), maxx(maxx) {};

      public:
        
        inline bounds2(const point2<fp>& o): minn(point2<fp>::max_point()), maxx(point2<fp>::min_point())
        {
            expand_by(o);
        }
        inline bounds2(const bounds2<fp>& o) = default;
        bounds2<fp>& operator=(const bounds2<fp>&) = default;
        inline bool intersects(const bounds2<fp>& o) const;
        inline polygon2<fp> to_polygon() const;
        inline bool is_exactly_on_boundary(const point2<fp>& p) const;
        inline bool is_inside(const point2<fp>& p) const;
        inline void expand_by(const point2<fp>& p);
        inline void expand_by(const bounds2<fp>& other);

        inline bool operator==(const bounds2<fp>& other) const
        {
            return minn == other.minn && maxx == other.maxx;
        }
        template<typename fp2>
        inline bool operator==(const bounds2<fp2>& other) const
        {
            return minn == other.minn && maxx == other.maxx;
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"bounds2">(comparser, obj);
            sixit::rw::read_write<&bounds2::minn, "min", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&bounds2::maxx, "max", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }

        template<typename fp2>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const bounds2<fp2>& other, size_t n = 1) const
        {
            return minn._for_test_only_approximate_eq(other.minn, n) && maxx._for_test_only_approximate_eq(other.maxx, n);
        }
    };

    template <typename fp>
    struct bounds1
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::bounds1<fp>>;
        friend struct low_level::ls1_impl<fp>;

      private:
        point1<fp> minn;
        point1<fp> maxx;

        bounds1() = delete;
        inline bounds1(point1<fp> minn, point1<fp> maxx): minn(minn), maxx(maxx) {};

      public:
        inline bounds1(const point1<fp>& o): minn(point1<fp>::max_point()), maxx(point1<fp>::min_point())
        {
            expand_by(o);
        }
        inline bounds1(const bounds1<fp>& o) = default;
        bounds1& operator=(const bounds1<fp>&) = default;
        inline bool intersects(const bounds1<fp>& o) const;
        inline bool is_exactly_on_boundary(const point1<fp>& p) const;
        inline bool is_inside(const point1<fp>& p) const;
        inline void expand_by(const point1<fp>& p);
        inline void expand_by(const bounds1<fp>& other);
    };

}; // namespace geometry

namespace geometry
{
    using line3f = line3<float>;
    using line2f = line2<float>;
    using line1f = line1<float>;

    using ray3f = ray3<float>;
    using ray2f = ray2<float>;
    template <XYZ axis>
    using axis_oriented_ray2f = axis_oriented_ray2<float, axis>;

    using bounds3f = bounds3<float>;
    using bounds2f = bounds2<float>;
    using bounds1f = bounds1<float>;
}

}; // namespace sixit

#endif //sixit_geometry_line_h_included

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
