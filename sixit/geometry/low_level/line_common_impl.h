/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_line_common_impl_h_included
#define sixit_geometry_low_level_line_common_impl_h_included

#include <utility>

#include "sixit/core/core.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/plane.h"
#include "sixit/geometry/low_level/plane_common_impl.h"
#include "sixit/geometry/projection3to2noscale.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    struct ls3_impl
    {
      public:

        inline static bool is_equal(const point3<fp>& p11, const point3<fp>& p12, const point3<fp>& p21, const point3<fp>& p22)
        {
            return p11.vec() == p21.vec() && p12.vec() == p22.vec();
        }

        inline static line3<fp> line(const point3<fp>& p11, const point3<fp>& p12)
        {
            return line3(p11, p12);
        }

        inline static direction3<fp> direction(const point3<fp>& p11, const point3<fp>& p12)
        {
            return direction3(p11, p12);
        }

        inline static sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length(const point3<fp>& p11, const point3<fp>& p12)
        {
            low_level::dimensional_vector3 d = p12.vec() - p11.vec();
            return d.magnitude();
        }

        inline static point3<fp> interpolate(const point3<fp>& p1, const point3<fp>& p2, fp t)
        {
            low_level::dimensional_vector3 v = p1.vec() * 
                            sixit::units::create_dimensionless_scalar<fp>(1.0f - t) + 
                            p2.vec() * sixit::units::create_dimensionless_scalar<fp>(t);
            return point3(v);
        }

        inline static std::pair<point3<fp>, rotation3<fp>> track(const point3<fp>& p11, const point3<fp>& p12, fp x)
        {
            point3<fp> res = point3<fp>(ls3_impl::direction(p11, p12).vec() * x);
            return std::make_pair(res, rotation3<fp>(direction3<fp>(p11.vec()), direction3<fp>(p12.vec())));
        }

        inline static bounds3<fp> bounds(const point3<fp>& p11, const point3<fp>& p12)
        {
            low_level::dimensional_vector3 min = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::min(p11.vec(), p12.vec());
            low_level::dimensional_vector3 max = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::max(p11.vec(), p12.vec());
            return bounds3(min, max);
        }
    };

    template <typename fp>
    struct ls2_impl
    {
        using class_value = typename point2<fp>::class_value;

        inline static bool is_equal(const point2<fp>& p11, const point2<fp>& p12, const point2<fp>& p21, const point2<fp>& p22)
        {
            return p11.vec() == p21.vec() && p12.vec() == p22.vec();
        }

        inline static line2<fp> line(const point2<fp>& p11, const point2<fp>& p12)
        {
            return line2(p11, p12);
        }

        inline static direction2<fp> direction(const point2<fp>& p11, const point2<fp>& p12)
        {
            return direction2(p11, p12);
        }

        inline static class_value length(const point2<fp>& p11, const point2<fp>& p12)
        {
            auto d = p12.vec() - p11.vec();
            return d.magnitude();
        }

        inline static std::pair<point2<fp>, rotation2<fp>> track(const point2<fp>& p11, const point2<fp>& p12, class_value x)
        {
            point2<fp> res = point2<fp>(ls2_impl::direction(p11, p12).vec() * x);
            auto fp_1 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
            return std::make_pair(res, rotation2<fp>(p11.vec() / fp_1, res.vec() / fp_1));
        }

        inline static bounds2<fp> bounds(const point2<fp>& p11, const point2<fp>& p12)
        {
            low_level::dimensional_vector2 min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(p11.vec(), p12.vec());
            low_level::dimensional_vector2 max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(p11.vec(), p12.vec());
            return bounds2<fp>(min, max);
        }
    };

    template <typename fp>
    struct ls1_impl
    {
        inline static bool is_equal(const point1<fp>& p11, const point1<fp>& p12, const point1<fp>& p21, const point1<fp>& p22)
        {
            return p11.vec() == p21.vec() && p12.vec() == p22.vec();
        }

        inline static line1<fp> line(const point1<fp>& p11, const point1<fp>& p12)
        {
            return line1<fp>(p11, p12);
        }

        inline static direction1<fp> direction(const point1<fp>& p11, const point1<fp>& p12)
        {
            return direction1<fp>(p11, p12);
        }

        inline static fp length(const point1<fp>& p11, const point1<fp>& p12)
        {
            return low_level::mathf::abs(p12.vec() - p11.vec());
        }

        inline static std::pair<point1<fp>, rotation1<fp>> track(const point1<fp>& p11, const point1<fp>& p12, fp x)
        {
            point1<fp> res = point1(ls1_impl::direction(p11, p12).vec() * x);
            return std::make_pair(res, rotation1<fp>(p11.vec(), res.vec()));
        }

        inline static bounds1<fp> bounds(const point1<fp>& p11, const point1<fp>& p12)
        {
            fp min = low_level::mathf::min(p11.vec(), p12.vec());
            fp max = low_level::mathf::max(p11.vec(), p12.vec());
            return bounds1<fp>(point1(min), point1(max));
        }
    };

    template <typename fp>
    struct tri2_impl
    {
      using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;
      private:

        // RATIONALE: this is the ray2::_ltr variation for _ltr_for_is_inside_combinable that 
        // avoids inlining and unnecessary normalization, to be as deterministic as possible
        static auto SIXIT_NOINLINE _ltr0(point2<fp> start, point2<fp> end, point2<fp> p) 
        {
            auto v1 = end.vector - start.vector;
            auto v2 = end.vector - p.vector;
            return v1.x * v2.y - v1.y * v2.x;
        }

        /**
         * Consistently checks whether the point is on the left of deterministically reordered ray(start, end) 
         * with guarantee that: _is_left_of_...(start, end, P) != _is_left_of_...(end, start, P)
         * 
         * see is_inside_combinable() for additional details
         */
        // EXTREMELY-FRAGILE-CODE: BEGIN (see discussion in TR-3457 in 6it.dev Jira)
        static bool _is_left_of_for_the_purposes_of_combinable_is_inside(point2<fp> ray_start, point2<fp> ray_end, const point2<fp>& p)
        {
            // order is arbitrary but deterministic. The rationale is to perform exactly the same calculation 
            // in exactly the same order for opposite to guarantee that `_is_left_of_...(A,B,P) != _is_left_of_...(B,A,P)`
            bool needs_reverse = std::tie(ray_end.vector.x, ray_end.vector.y) 
                                < std::tie(ray_start.vector.x, ray_start.vector.y);

            if (needs_reverse)
                std::swap(ray_start, ray_end);

            auto ltr = _ltr0(ray_start, ray_end, p);

            // if the point is on edge (ltr is strict zero), we always 
            // assume it to be on the right from the "ordered" ray 
            return (ltr < sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) ^ needs_reverse;
        }
        // EXTREMELY-FRAGILE-CODE: END

      public:
        inline static bounds2<fp> bounds(const point2<fp>& p11, const point2<fp>& p12, const point2<fp> p13)
        {
            low_level::dimensional_vector2 max1 = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(p11.vec(), p12.vec());
            low_level::dimensional_vector2 max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(max1, p13.vec());

            low_level::dimensional_vector2 min1 = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(p11.vec(), p12.vec());
            low_level::dimensional_vector2 min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(min1, p13.vec());
            return bounds2<fp>(point2<fp>(min), point2<fp>(max));
        }

        inline static sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square(const point2<fp>& p11, const point2<fp>& p12, const point2<fp> p13)
        {
            low_level::dimensional_vector2 v1 = p12.vec() - p11.vec();
            low_level::dimensional_vector2 v2 = p13.vec() - p11.vec();

            auto rv = v1.x * v2.y - v1.y * v2.x;
            rv = low_level::mathf::abs(rv) * sixit::units::create_dimensionless_scalar<fp>(0.5f);
            return rv;
        }

        inline static int normal(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3)
        {
            low_level::dimensional_vector2 v1 = p2.vec() - p1.vec();
            low_level::dimensional_vector2 v2 = p3.vec() - p2.vec();
            auto val = v1.x * v2.y - v2.x * v1.y;

            auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);

            if (val < fp_0)
                return -1;
            if (val > fp_0)
                return 1;
            return 0;
        }

        inline static point2<fp> barycenter(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3)
        {
            const fp k01 = 1.0f / 3.0f;
            const fp k2 = 1.0f - k01 - k01;
            return point2<fp>((p1.vec() * k01 + p2.vec() * k01 + p3.vec() * k2));
        }

      /*  inline static bool is_inside_basic(const point2& p1, const point2& p2, const point2& p3, const point2& p)
        {
            int n = normal(p1, p2, p3);
            if (normal(p1, p2, p) != n)
                return false;
            if (normal(p2, p3, p) != n)
                return false;
            if (normal(p3, p1, p) != n)
                return false;
            return true;
        }*/

        inline static bool is_inside_basic(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3, const point2<fp>& p)
        {
            ray2<fp> rays[3] =
            {
                ray2(p1, p2),
                ray2(p2, p3),
                ray2(p3, p1)
            };

            auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);

            sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> d[3] = {fp_0, fp_0, fp_0};

            for (int i = 0; i < 3; ++i)
            {
                d[i] = rays[i]._ltr(p);
            }

            if (d[0] > fp_0 && d[1] > fp_0 && d[2] > fp_0)
                return true;
            if (d[0] < fp_0 && d[1] < fp_0 && d[2] < fp_0)
                return true;
            return false;
        }


        /**
         * Checks that point `p` is inside triangle defined by 3 edges: (p1,p2), (p2,p3), (p3,p1), 
         * in consistent clockwise or counter-clockwise order, providing additional guarantees:
         *  - is_inside_combinable(A, B, C, P) == is_inside_combinable(C, B, A, P);
         *  - with any quadrilateral ABCD, there are no points that belongs to both ABC and ADC
         *      - if the point is well within the quadrilateral it should belong to exactly one triangle ABC or ADC.
         */
        inline static bool is_inside_combinable(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3, const point2<fp>& p)
        {
            // EXTREMELY-FRAGILE-CODE: BEGIN (see discussion in TR-3457 in 6it.dev Jira)
            bool b0 = _is_left_of_for_the_purposes_of_combinable_is_inside(p1, p2, p);
            bool b1 = _is_left_of_for_the_purposes_of_combinable_is_inside(p2, p3, p);
            bool b2 = _is_left_of_for_the_purposes_of_combinable_is_inside(p3, p1, p);
            return b0 == b1 && b1 == b2;
            // EXTREMELY-FRAGILE-CODE: END
        }

        inline static fp sign(const point2<fp>& p1, const point2<fp>& p2, const point2<fp> p3)
        {
            auto rv = (p1.vec().x - p3.vec().x) * (p2.vec().y - p3.vec().y) - (p2.vec().x - p3.vec().x) * (p1.vec().y - p3.vec().y);
            return sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::square_meter>(rv);
        }

        inline static bool point_inside(const point2<fp>& p11, const point2<fp>& p12, const point2<fp> p13, const point2<fp>& p)
        {
            fp d1, d2, d3;
            bool has_neg, has_pos;

            d1 = tri2_impl::sign(p, p11, p12);
            d2 = tri2_impl::sign(p, p12, p13);
            d3 = tri2_impl::sign(p, p13, p11);
            has_neg = (d1 < fp(0.0f)) || (d2 < fp(0.0f)) || (d3 < fp(0.0f));
            has_pos = (d1 > fp(0.0f)) || (d2 > fp(0.0f)) || (d3 > fp(0.0f));
            return !(has_neg && has_pos);
        }
        
        inline static fp circumradius(const point2<fp>& p11, const point2<fp>& p12, const point2<fp> p13)
        {
            fp a = (p12.vec() - p11.vec()).magnitude();
            fp b = (p13.vec() - p12.vec()).magnitude();
            fp c = (p11.vec() - p13.vec()).magnitude();
            fp s = (a + b + c) / 2.0f;
            fp inv_a = s - a;
            fp inv_b = s - b;
            fp inv_c = s - c;
            if (inv_a <= fp(0.0f) || inv_b <= fp(0.0f) || inv_c <= fp(0.0f))
                return mathf::max_float;

            fp res = a * b * c / mathf::sqrt(s * inv_a * inv_b * inv_c);
            return sixit::dmath::fp_traits<fp>::isfinite(res) ? res : mathf::max_float;
        }

        inline static point2<fp> circumcenter(const point2<fp>& p11, const point2<fp>& p12, const point2<fp> p13)
        {
            fp a = (p12.vec() - p11.vec()).magnitude();
            fp b = (p13.vec() - p12.vec()).magnitude();
            fp c = (p11.vec() - p13.vec()).magnitude();
            fp a2 = a * a;
            fp b2 = b * b;
            fp c2 = c * c;

            fp dnom_a = ((b + c) * (b + c) - a2) * (a2 - (b - c) * (b - c));
            fp dnom_b = ((a + c) * (a + c) - b2) * (b2 - (a - c) * (a - c));
            fp dnom_c = ((b + a) * (b + a) - c2) * (c2 - (b - a) * (b - a));
            fp nom_a = a2 * (b2 + c2 - a2);
            fp nom_b = b2 * (a2 + c2 - b2);
            fp nom_c = c2 * (b2 + a2 - c2);

            auto center = p13.vec() * (nom_a / dnom_a) + p11.vec() * (nom_b / dnom_b) + p12.vec() * (nom_c / dnom_c);
            return point2(center);
        }


        inline static bool intersects_line_segment(const point2<fp>&, const point2<fp>&, const point2<fp>&, const point2<fp>&, const point2<fp>&)
        {

            return false;
        }
  };

    template <typename fp>
    struct tri3_impl
    {
        inline static plane3<fp> plane(const point3<fp>& p11, const point3<fp>& p12, const point3<fp> p13)
        {
            low_level::dimensional_vector3 d1 = p12.vec() - p11.vec();
            low_level::dimensional_vector3 d2 = p13.vec() - p11.vec();
            low_level::dimensional_vector3 normal = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::cross(d1, d2).normalized();

            return plane3(direction3<fp>(normal), p11);
        }

        inline static sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square(const point3<fp>& p11, const point3<fp>& p12, const point3<fp> p13)
        {
            low_level::dimensional_vector3 v1 = p12.vec() - p11.vec();
            low_level::dimensional_vector3 v2 = p13.vec() - p11.vec();
            low_level::dimensional_vector3 cross = dimensional_vector3<fp, sixit::units::length_unit::dim>::cross(v1, v2);
            return cross.magnitude() * sixit::units::create_dimensionless_scalar<fp>(0.5f);
        }

        inline static direction3<fp> normal(const point3<fp>& p1, const point3<fp>& p2, const point3<fp>& p3)
        {
            low_level::dimensional_vector3 v1 = p2.vec() - p1.vec();
            low_level::dimensional_vector3 v2 = p3.vec() - p2.vec();
            low_level::dimensional_vector3 cross = dimensional_vector3<fp, sixit::units::length_unit::dim>::cross(v1, v2);
            return direction3<fp>(cross);
        }

        inline static point3<fp> barycenter(const point3<fp>& p1, const point3<fp>& p2, const point3<fp>& p3)
        {
            const auto k01 = sixit::units::create_dimensionless_scalar<fp>(1.0f / 3.0f);
            const auto k2 = sixit::units::create_dimensionless_scalar<fp>(1.0f) - k01 - k01;
            return point3<fp>((p1.vec() * k01 + p2.vec() * k01 + p3.vec() * k2));
        }

        inline static fp circumradius(const point3<fp>& p11, const point3<fp>& p12, const point3<fp> p13)
        {
            fp a = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>((p12.vec() - p11.vec()).magnitude());
            fp b = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>((p13.vec() - p12.vec()).magnitude());
            fp c = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>((p11.vec() - p13.vec()).magnitude());
            fp s = (a + b + c) / 2.0f;
            fp inv_a = s - a;
            fp inv_b = s - b;
            fp inv_c = s - c;
            if (inv_a <= 0.0f || inv_b <= 0.0f || inv_c <= 0.0f)
                return mathf::max_float;

            fp res = a * b * c / mathf::sqrt(s * inv_a * inv_b * inv_c);
            return sixit::dmath::fp_traits<float>::isfinite(res) ? res : mathf::max_float;
        }

        inline static ray3<fp> bisector(const point3<fp>& p11, const point3<fp>& p12, const point3<fp> p13, int idx)
        {
            assert(idx >= 0 && idx < 3);
            low_level::dimensional_vector3 d1 = p12.vec() - p11.vec();
            low_level::dimensional_vector3 d2 = p13.vec() - p11.vec();
            low_level::dimensional_vector3<fp, sixit::units::area_unit::dim> tmp_normal = {
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(d1.y() * d2.z() - d1.x() * d2.y()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(d1.z() * d2.x() - d1.x() * d2.z()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(d1.x() * d2.y() - d1.y() * d2.x())
            };
            low_level::dimensional_vector3 normal = tmp_normal.normalized();

            point3<fp> vertex = p11;
            if (idx == 1)
            {
                vertex = p12;
            }
            else if (idx == 2)
            {
                vertex = p13;
            }

            return ray3<fp>(vertex, direction3(normal));
        }

        inline static bounds3<fp> bounds(const point3<fp>& p11, const point3<fp>& p12, const point3<fp> p13)
        {
            low_level::dimensional_vector3 max1 = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::max(p11.vec(), p12.vec());
            low_level::dimensional_vector3 max = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::max(max1, p13.vec());

            low_level::dimensional_vector3 min1 = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::min(p11.vec(), p12.vec());
            low_level::dimensional_vector3 min = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::min(min1, p13.vec());
            return bounds3(point3(min), point3(max));
        }

        inline static std::optional<point3<fp>> intersects_line(const point3<fp>& p11, const point3<fp>& p12, const point3<fp>& p13, const point3<fp>& p21, const point3<fp>& p22)
        {
            plane3 plane = tri3_impl<fp>::plane(p11, p12, p13);
            return plane3_impl<fp>::intersection(plane.normal, plane.point, p21, p22);
        }

        inline static bool intersects_line_segment(
            const point3<fp>& p11, const point3<fp>& p12, const point3<fp>& p13,
            const point3<fp>& segmentStart, const point3<fp>& segmentEnd, point3<fp>* i_point = nullptr)
        {
            auto intersectionPoint = tri3_impl::intersects_line(p11, p12, p13, segmentStart, segmentEnd);
            if (!intersectionPoint.has_value())
            {
                return false;
            }

            // Check if the intersection is within the line segment
            low_level::dimensional_vector3 intersection = intersectionPoint.value().vec();
            low_level::dimensional_vector3 end_minus_start = segmentEnd.vec() - segmentStart.vec();
            auto dot1 = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(intersection - segmentStart.vec(), end_minus_start);
            auto dot2 = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(end_minus_start, end_minus_start);
            auto t = dot1 / dot2;
            bool isIntersectionInRange = t >= sixit::units::create_dimensionless_scalar<fp>(0.0f) && t <= sixit::units::create_dimensionless_scalar<fp>(1.0f);
            if (!isIntersectionInRange)
            {
                return false;
            }

            // Project the 3D points onto a 2D plane and check if the intersection point is inside the triangle
            projection3to2noscale proj3DTo2D(tri3_impl::plane(p11, p12, p13));
            auto projP1  = proj3DTo2D.transform(p11);
            auto projP2  = proj3DTo2D.transform(p12);
            auto projP3  = proj3DTo2D.transform(p13);
            auto projIntersection = proj3DTo2D.transform(intersectionPoint.value());

            if (i_point != nullptr)
            {
                *i_point = intersectionPoint.value();
            }
            
            return tri2_impl<fp>::point_inside(projP1, projP2, projP3, projIntersection);
        }

        inline static bool intersects(const point3<fp>& p11, const point3<fp>& p12, const point3<fp>& p13,
            const point3<fp>& p21, const point3<fp>& p22, const point3<fp>& p23)
        {
            if (tri3_impl::intersects_line_segment(p11, p12, p13, p21, p22))
                return true;
            if (tri3_impl::intersects_line_segment(p11, p12, p13, p22, p23))
                return true;
            if (tri3_impl::intersects_line_segment(p11, p12, p13, p21, p23))
                return true;
            if (tri3_impl::intersects_line_segment(p21, p22, p23, p11, p12))
                return true;
            if (tri3_impl::intersects_line_segment(p21, p22, p23, p12, p13))
                return true;
            if (tri3_impl::intersects_line_segment(p21, p22, p23, p11, p13))
                return true;
            return false;
        }

        inline static std::optional<std::vector<point3<fp>>> intersection(const point3<fp>& p11, const point3<fp>& p12, const point3<fp>& p13,
            const point3<fp>& p21, const point3<fp>& p22, const point3<fp>& p23)
        {
            point3<fp> inter_point;
            std::vector<point3<fp>> i_points;
            
            if (tri3_impl::intersects_line_segment(p11, p12, p13, p21, p22, &inter_point))
            {
                i_points.push_back(inter_point);
            }

            if (tri3_impl::intersects_line_segment(p11, p12, p13, p22, p23, &inter_point))
            {
                i_points.push_back(inter_point);
                if (i_points.size() > 1)
                    return i_points;
            }

            if (tri3_impl::intersects_line_segment(p11, p12, p13, p21, p23, &inter_point))
            {
                i_points.push_back(inter_point);
                if (i_points.size() > 1)
                    return i_points;
            }

            if (tri3_impl::intersects_line_segment(p21, p22, p23, p11, p12, &inter_point))
            {
                i_points.push_back(inter_point);
                if (i_points.size() > 1)
                    return i_points;
            }

            if (tri3_impl::intersects_line_segment(p21, p22, p23, p12, p13, &inter_point))
            {
                i_points.push_back(inter_point);
                if (i_points.size() > 1)
                    return i_points;
            }
            
            if (tri3_impl::intersects_line_segment(p21, p22, p23, p11, p13, &inter_point))
            {
                i_points.push_back(inter_point);
                if (i_points.size() > 1)
                    return i_points;
            }

            return std::nullopt;
        }

        /// temporary put here some functions for tetrahedron and circumsphere - use 4 points3 
        inline static fp tetrahedron_volume(const point3<fp>& p1, const point3<fp>& p2, const point3<fp>& p3, const point3<fp>& p4)
        {
            auto a = p1.vec() - p4.vec();
            auto b = p2.vec() - p4.vec();
            auto c = p3.vec() - p4.vec();
            auto v = dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(a, dimensional_vector3<fp, sixit::units::length_unit::dim>::cross(b, c));
            return sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::square_meter>(mathf::abs(v)) / 6.0f;
        }

        inline static fp circumsphere_radius(const point3<fp>& p1, const point3<fp>& p2, const point3<fp>& p3, const point3<fp>& p4)
        {
            auto p14 = p1.vec() - p4.vec();
            auto p24 = p2.vec() - p4.vec();
            auto p34 = p3.vec() - p4.vec();

            auto v6 = mathf::abs(dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(
                                    p14, dimensional_vector3<fp, sixit::units::length_unit::dim>::cross(p24, p34)));

            auto a = p14.magnitude();
            auto b = p24.magnitude();
            auto c = p34.magnitude();

            auto a1 = (p3.vec() - p2.vec()).magnitude();
            auto b1 = (p3.vec() - p1.vec()).magnitude();
            auto c1 = (p2.vec() - p1.vec()).magnitude();

            auto p = (a * a1 + b * b1 + c * c1) * sixit::units::create_dimensionless_scalar<fp>(0.5f);
            auto nominator = mathf::sqrt(p * (p - a * a1) * (p - b * b1) * (p - c * c1));
            auto r = nominator / v6;
            return sixit::dmath::fp_traits<fp>::isfinite(r) ? sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::square_meter>(r) : mathf::max_float;
        }
    };

}; // namespace low_level

namespace low_level 
{
    // aliases 
    using ls3_implf = ls3_impl<float>;
    using ls2_implf = ls2_impl<float>;
    using ls1_implf = ls1_impl<float>;
    using tri2_implf = tri2_impl<float>;
    using tri3_implf = tri3_impl<float>;
}

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_line_common_impl_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Volodymyr Melnychuk

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