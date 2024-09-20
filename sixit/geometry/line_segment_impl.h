/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_line_segment_impl_h_included
#define sixit_geometry_line_segment_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/low_level/line_common_impl.h"
#include "sixit/geometry/arc.h"
#include "sixit/geometry/cbezier.h"
#include "sixit/geometry/qbezier.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    bool line_segment3<fp>::operator==(const line_segment3<fp>& o) const
    {
        return low_level::ls3_impl<fp>::is_equal(line_segment3<fp>::p1(), line_segment3<fp>::p2(), o.p1(), o.p2());
    }

    template <typename fp>
    line3<fp> line_segment3<fp>::line() const
    {
        return low_level::ls3_impl<fp>::line(line_segment3<fp>::p1(), line_segment3<fp>::p2());
    }

    template <typename fp>
    direction3<fp> line_segment3<fp>::direction() const
    {
        return low_level::ls3_impl<fp>::direction(line_segment3<fp>::p1(), line_segment3<fp>::p2());
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> line_segment3<fp>::length() const
    {
        return low_level::ls3_impl<fp>::length(line_segment3<fp>::p1(), line_segment3<fp>::p2());
    }

    template <typename fp>
    point3<fp> line_segment3<fp>::interpolate(fp t) const
    {
        return low_level::ls3_impl<fp>::interpolate(line_segment3<fp>::p1(), line_segment3<fp>::p2(), t);
    }

    template <typename fp>
    std::tuple<enum XYZ, fp> line_segment3<fp>::largest_coordinate() const
    {
        low_level::vector3 d = line_segment3<fp>::p2().vec() - line_segment3<fp>::p1().vec();
        if (fabs(d.x()) > fabs(d.y()) && fabs(d.x()) > fabs(d.z()))
            return std::tuple<enum XYZ, fp>{XYZ::X, fabs(d.x())};
        if (fabs(d.y()) > fabs(d.z()))
            return std::tuple<enum XYZ, fp>{XYZ::Y, fabs(d.y())};
        return std::tuple<enum XYZ, fp>{XYZ::Z, fabs(d.z())};
    }

    template <typename fp>
    std::pair<point3<fp>, rotation3<fp>> line_segment3<fp>::track(fp x) const
    {
        return low_level::ls3_impl<fp>::track(line_segment3<fp>::p1(), line_segment3<fp>::p2(), x);
    }

    template <typename fp>
    bounds3<fp> line_segment3<fp>::bounds() const
    {
        return low_level::ls3_impl<fp>::bounds(line_segment3<fp>::p1(), line_segment3<fp>::p2());
    }

    template <typename fp>
    bool line_segment2<fp>::operator==(const line_segment2<fp>& o) const
    {
        return low_level::ls2_impl<fp>::is_equal(line_segment2<fp>::p1(), line_segment2<fp>::p2(), o.p1(), o.p2());
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>  line_segment2<fp>::_ltr(const point2<fp>& p) const
    {
        if (line_segment2<fp>::p1() == p || line_segment2<fp>::p2() == p)//TR-2030
            return sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>::zero();
        return ray2(line_segment2<fp>::p1(), line_segment2<fp>::p2())._ltr(p);
    }

    template <typename fp>
    bool line_segment2<fp>::overlays(const line_segment2<fp>& ls2) const
    {
        // get dominant direction for 1D space, as index (x:0, y:1)
        fp d_x = low_level::mathf::abs( line_segment2<fp>::p1().vec().x - line_segment2<fp>::p2().vec().x );
        fp d_y = low_level::mathf::abs( line_segment2<fp>::p1().vec().y - line_segment2<fp>::p2().vec().y );
        int index = d_x >= d_y ? 0 : 1;

        // now mix and max fps are taken on 1D line
        fp min1 = low_level::mathf::min( line_segment2<fp>::p1().vec()[index], line_segment2<fp>::p2().vec()[index] );
        fp max1 = low_level::mathf::max( line_segment2<fp>::p1().vec()[index], line_segment2<fp>::p2().vec()[index] );
        fp min2 = low_level::mathf::min( ls2.p1().vec()[index], ls2.p2().vec()[index] );
        fp max2 = low_level::mathf::max( ls2.p1().vec()[index], ls2.p2().vec()[index] );

        return max1 > min2 && min1 < max2;
    }

    template <typename fp>
    bool line_segment2<fp>::_strict_intersects_as_ray(const line_segment2& ls2) const
    {
        if (!_intersects_as_ray(ls2))
            return false;
        auto d2 = _ltr(ls2.p2());

        if (d2 == sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>::zero())
            return false;
        return true;
    }

    template <typename fp>
    bool line_segment2<fp>::_intersects_as_ray(const line_segment2& ls2) const
    {
        //p + r*t
        //q + s*u
        auto d1 = _ltr(ls2.p1());
        auto d2 = _ltr(ls2.p2());

        if ((d1 < decltype(d1)::zero()) == (d2 < decltype(d2)::zero()))
            return false;
        if ((d1 > decltype(d1)::zero()) == (d2 > decltype(d2)::zero()))
            return false;

        auto r = line_segment2<fp>::p2().vec() - line_segment2<fp>::p1().vec();//consider it as a ray
        auto s = ls2.p2().vec() - ls2.p1().vec();

        auto dev = r.x * s.y - r.y * s.x;

        low_level::dimensional_vector2 qp = ls2.p1().vec() - line_segment2<fp>::p1().vec();

        auto t = (qp.x * s.y - qp.y * s.x) / dev;
        return sixit::geometry::low_level::mathf::isfinite(t) && t >= decltype(t)::zero() ? true : false;
    }

    template <typename fp>
    bool line_segment2<fp>::strict_intersects(const line_segment2<fp>& ls2) const
    {
        return _strict_intersects_as_ray(ls2) && ls2._strict_intersects_as_ray(*this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersects(const line_segment2<fp>& ls2) const
    {
        return _intersects_as_ray(ls2) && ls2._intersects_as_ray(*this);
    }

    template <typename fp>
    std::optional<point2<fp>> line_segment2<fp>::strict_intersection(const line_segment2<fp>& ls2) const
    {
        if (!strict_intersects(ls2))
            return {};

        auto r = line_segment2<fp>::p2().vec() - line_segment2<fp>::p1().vec();//consider it as a ray
        auto s = ls2.p2().vec() - ls2.p1().vec();

        auto dev_r_s = r.x * s.y - r.y * s.x;

        //auto dev_s_r = s.y * r.x  - s.x * r.y;
        //if (dev_s_r == decltype(dev_s_r)::zero())
        //    return {};

        const auto& p = line_segment2<fp>::p1().vec();
        const auto& q = ls2.p1().vec();

        auto qp = q - p;
       // low_level::vector2 pq = p - q;

        auto t = (qp.x * s.y - qp.y * s.x) / dev_r_s;
       // auto u = (pq.x * r.y - pq.y * r.x) / dev_s_r;

        //if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f)//we already checked crossing
        if (sixit::geometry::low_level::mathf::isfinite(t))
            return point2<fp>(p + r * t);
        else
            return {};
    }

    template <typename fp>
    bool line_segment2<fp>::point_in_segment(const point2<fp>& p) const
    {
        fp fp_0(0.f);
        if (_ltr(p) != fp_0)
            return false;

        const low_level::vector2<fp>& v1 = line_segment2<fp>::p1().vec();
        if (p == v1)
            return false;

        const low_level::vector2<fp>& v2 = line_segment2<fp>::p2().vec();
        if (p == v2)
            return false;

        low_level::vector2<fp> v2v1 = v2 - v1;
        fp proj = low_level::vector2<fp>::dot(v2v1, (p.vec() - v1));

        return (proj > fp_0 && proj < v2v1.sqr_magnitude());
    }

    template <typename fp>
    std::array<typename std::optional<line_segment2<fp>>, 2> line_segment2<fp>::cover_with(const triangle2<fp>& triangle) const
    {
        // Get segment start and end points.
        const point2<fp>& segment_start = line_segment2<fp>::p1();
        const point2<fp>& segment_end = line_segment2<fp>::p2();

        // Check if points are exactly the edges
        const bool start_is_triangle_point = segment_start == triangle.vertex(0) ||
            segment_start == triangle.vertex(1) || segment_start == triangle.vertex(2);

        const bool end_is_triangle_point = segment_end == triangle.vertex(0) ||
            segment_end == triangle.vertex(1) || segment_end == triangle.vertex(2);

        if (start_is_triangle_point && end_is_triangle_point)
            return {};

        // Get triangle edges.
        const line_segment2 triangle_edge_1 = triangle.edge(0);
        const line_segment2 triangle_edge_2 = triangle.edge(1);
        const line_segment2 triangle_edge_3 = triangle.edge(2);

        auto length_1 = triangle_edge_1.length();
        if (triangle.vertex(0) == triangle.vertex(1) || triangle.vertex(0) == triangle.vertex(2) || triangle.vertex(1) == triangle.vertex(2))
        {
            return { line_segment2(segment_start, segment_end)};
        }

        // Orientation of segment's start point to triangle edges.
        const auto d1_start = triangle_edge_1._ltr(segment_start);
        const auto d2_start = triangle_edge_2._ltr(segment_start);
        const auto d3_start = triangle_edge_3._ltr(segment_start);

        // Orientation of segment's end point to triangle edges.
        const auto d1_end = triangle_edge_1._ltr(segment_end);
        const auto d2_end = triangle_edge_2._ltr(segment_end);
        const auto d3_end = triangle_edge_3._ltr(segment_end);

        // Check if segment start and end points are inside/on triangle.
        const bool start_point_inside = start_is_triangle_point ||
            ((d1_start >= decltype(d1_start)::zero() && d2_start >= decltype(d2_start)::zero() && d3_start >= decltype(d3_start)::zero()) || 
             (d1_start <= decltype(d1_start)::zero() && d2_start <= decltype(d2_start)::zero() && d3_start <= decltype(d3_start)::zero()));

        const bool end_point_inside = end_is_triangle_point ||
            ((d1_end >= decltype(d1_start)::zero() && d2_end >= decltype(d2_start)::zero() && d3_end >= decltype(d3_start)::zero()) ||
             (d1_end <= decltype(d1_start)::zero() && d2_end <= decltype(d2_start)::zero() && d3_end <= decltype(d3_start)::zero()));

        if (start_point_inside && end_point_inside)
        {
            return {}; // Segment is fully covered by triangle.
        }

        // Looking for intersections.
        std::array<std::optional<point2<fp>>, 2> intersections = {std::nullopt, std::nullopt};

        for (int i = 0; i < triangle.n_edges(); ++i)
        {
            const line_segment2<fp> edge = triangle.edge(i);
            if (const auto inters = intersection(edge))
            {
                if (!intersections[0].has_value())
                {
                    intersections[0] = inters;
                }
                else if (!intersections[1].has_value() && intersections[0] != inters)
                {
                    intersections[1] = inters;
                }
            }
        }

        // No intersections were found.
        if (!intersections[0].has_value() && !intersections[1].has_value())
        {
            return {*this, std::nullopt};
        }

        // One intersection was found.
        if (intersections[0].has_value() && !intersections[1].has_value())
        {
            if (start_point_inside)
            {
                if (intersections[0].value() == segment_end)
                    return {};

                const line_segment2 segment(intersections[0].value(), segment_end);
                return {segment, std::nullopt};
            }
            else if (end_point_inside)
            {
                if (intersections[0].value() == segment_start)
                    return {};

                const line_segment2 segment{ segment_start, intersections[0].value() };
                return {segment, std::nullopt};
            }

            return {};
        }

        // Two intersections were found.
        std::array<std::optional<line_segment2<fp>>, 2> result;

        // Calculate distances from the start point.
        const auto distance1 = segment_start.distance(intersections[0].value());
        const auto distance2 = segment_start.distance(intersections[1].value());

        // Determine which intersection is more to the left or right.
        const point2<fp>& closerToStart = distance1 < distance2 ? intersections[0].value() : intersections[1].value();
        const point2<fp>& closerToEnd = distance1 < distance2 ? intersections[1].value() : intersections[0].value();

        // Initialize the result array directly with the segments.
        if (segment_start == closerToStart)
            result[0] = std::nullopt;
        else
            result[0] = line_segment2<fp>{ segment_start, closerToStart };

        if(closerToEnd == segment_end)
            result[1] = std::nullopt;
        else
            result[1] = line_segment2<fp>{ closerToEnd, segment_end };

        return result;
    }

    template <typename fp>
    line2<fp> line_segment2<fp>::line() const
    {
        return low_level::ls2_impl<fp>::line(line_segment2<fp>::p1(), line_segment2<fp>::p2());
    }

    template <typename fp>
    direction2<fp> line_segment2<fp>::direction() const
    {
        return low_level::ls2_impl<fp>::direction(line_segment2<fp>::p1(), line_segment2<fp>::p2());
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> line_segment2<fp>::length() const
    {
        return low_level::ls2_impl<fp>::length(line_segment2<fp>::p1(), line_segment2<fp>::p2());
    }

    template <typename fp>
    bounds2<fp> line_segment2<fp>::bounds() const
    {
        return low_level::ls2_impl<fp>::bounds(line_segment2<fp>::p1(), line_segment2<fp>::p2());
    }

    template <typename fp>
    point2<fp> line_segment2<fp>::track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
    {
        auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
        return point2(line_segment2<fp>::p1().vec() * (fp_1 - t) + line_segment2<fp>::p2().vec() * t);
    }

    template <typename fp>
    direction2<fp> line_segment2<fp>::track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>) const
    {
        return direction();
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const
    {
        return other->intersection(intersections, *this);
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const
    {
        auto result = other.strict_intersection(*this);
        if (result.has_value())
        {
            intersections.push_back(result.value());
        }
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const
    {
        auto result = strict_intersection(other);
        if (result.has_value())
        {
            intersections.push_back(result.value());
        }
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const arc2<fp>& other) const
    {
        other.intersection(intersections, *this);
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const qbezier2<fp>& other) const
    {
        other.intersection(intersections, *this);
    }

    template <typename fp>
    void line_segment2<fp>::intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const
    {
        other.intersection(intersections, *this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const
    {
        return other->intersect(*this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const line2<fp>& other) const
    {
        return other.strict_intersects(*this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const line_segment2& other) const
    {
        return strict_intersects(other);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const arc2<fp>& other) const
    {
        return other.intersect(*this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const qbezier2<fp>& other) const
    {
        return other.intersect(*this);
    }

    template <typename fp>
    bool line_segment2<fp>::intersect(const cbezier2<fp>& other) const
    {
        return other.intersect(*this);
    }

    template <typename fp>
    std::optional<point2<fp>> line_segment2<fp>::intersection(const line_segment2<fp>& other) const
    {
        const low_level::dimensional_vector2 r = line_segment2<fp>::p2().vec() - line_segment2<fp>::p1().vec();
        const low_level::dimensional_vector2 s = other.p2().vec() - other.p1().vec();

        const auto r_cross_s = r.x * s.y - r.y * s.x;

        const auto t_numerator = (other.p1().vec().x - line_segment2<fp>::p1().vec().x) * s.y - (other.p1().vec().y - line_segment2<fp>::p1().vec().y) * s.x;
        const auto u_numerator = (other.p1().vec().x - line_segment2<fp>::p1().vec().x) * r.y - (other.p1().vec().y - line_segment2<fp>::p1().vec().y) * r.x;

        const auto t = t_numerator / r_cross_s;
        const auto u = u_numerator / r_cross_s; 
        if (!sixit::geometry::low_level::mathf::isfinite(t) || !sixit::geometry::low_level::mathf::isfinite(u))
            return std::nullopt;

        // Check if the intersection point lies on both segments.
        if (t >= sixit::units::create_dimensionless_scalar<fp>(0.f) && t <= sixit::units::create_dimensionless_scalar<fp>(1.f) && 
                u >= sixit::units::create_dimensionless_scalar<fp>(0.f) && u <= sixit::units::create_dimensionless_scalar<fp>(1.f))
        {
            return point2(line_segment2<fp>::p1().vec().x + t * r.x, line_segment2<fp>::p1().vec().y + t * r.y);
        }

        return std::nullopt;
    }

    template <typename fp>
    bool line_segment1<fp>::operator==(const line_segment1<fp>& o) const
    {
        return low_level::ls1_impl<fp>::is_equal(line_segment1<fp>::p1(), line_segment1<fp>::p2(), o.p1(), o.p2());
    }

    template <typename fp>
    line1<fp> line_segment1<fp>::line() const
    {
        return low_level::ls1_impl<fp>::line(line_segment1<fp>::p1(), line_segment1<fp>::p2());
    }

    template <typename fp>
    direction1<fp> line_segment1<fp>::direction() const
    {
        return low_level::ls1_impl<fp>::direction(line_segment1<fp>::p1(), line_segment1<fp>::p2());
    }

    template <typename fp>
    typename line_segment1<fp>::class_value line_segment1<fp>::length() const
    {
        return low_level::ls1_impl<fp>::length(line_segment1<fp>::p1(), line_segment1<fp>::p2());
    }

    template <typename fp>
    std::pair<point1<fp>, rotation1<fp>> line_segment1<fp>::track(fp x) const
    {
        return low_level::ls1_impl<fp>::track(line_segment1<fp>::p1(), line_segment1<fp>::p2(), x);
    }

    template <typename fp>
    bounds1<fp> line_segment1<fp>::bounds() const
    {
        return low_level::ls1_impl<fp>::bounds(line_segment1<fp>::p1(), line_segment1<fp>::p2());
    }

    template <typename fp>
    bool line_segment1<fp>::overlays(const line_segment1<fp>& ls1) const
    {
        // compare mix and max fps on 1D line
        line_segment1<fp>::class_value min1 = low_level::mathf::min(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec() );
        line_segment1<fp>::class_value max1 = low_level::mathf::max(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec() );
        line_segment1<fp>::class_value min2 = low_level::mathf::min( ls1.p1().vec(), ls1.p2().vec() );
        line_segment1<fp>::class_value max2 = low_level::mathf::max( ls1.p1().vec(), ls1.p2().vec() );

        return max1 > min2 && min1 < max2;
    }

    template <typename fp>
    ray3<fp>::ray3(const point3<fp>& p, const direction3<fp>& v)
    : origin(p), direction(v)
    {}

    template <typename fp>
    ray3<fp>::ray3(const point3<fp>& p1, const point3<fp>& p2)
    : origin(p1), direction(p1, p2)
    {}

    template <typename fp>
    line3<fp> ray3<fp>::line() const
    {
        point3<fp> p2 = origin.vec() + direction.vec();
        return line3(origin, p2);
    }

    template <typename fp>
    point3<fp> ray3<fp>::track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> p) const
    {
        assert(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p) >= fp(0.f));
        low_level::dimensional_vector3 d = direction.vec() * p;
        return point3(origin.vec() + d);
    }

    template <typename fp>
    ray2<fp>::ray2(const point2<fp>& p, const direction2<fp>& v)
    : origin_data(p), direction_data(v)
    {}

    template <typename fp>
    ray2<fp>::ray2(const point2<fp>& p1, const point2<fp>& p2)
    : origin_data(p1), direction_data(p1, p2)
    {}

    template <typename fp>
    line2<fp> ray2<fp>::line() const
    {
        point2<fp> p2 = origin_data.vec() + direction_data.vec();
        return line2(origin_data, p2);
    }

    template <typename fp>
    point2<fp> ray2<fp>::track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> p) const
    {
        low_level::dimensional_vector2 d = direction_data.vec() * p;
        return point2<fp>(origin_data.vec() + d);
    }

    template <typename fp>
    direction2<fp> ray2<fp>::direction() const
    {
        return direction_data;
    }

    template <typename fp>
    point2<fp> ray2<fp>::origin() const
    {
        return origin_data;
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>  line2<fp>::_ltr(const point2<fp>& p) const
    {
        const low_level::dimensional_vector2 v1 = line2<fp>::p2().vec() - line2<fp>::p1().vec();
        const low_level::dimensional_vector2 v2 = p.vec() - line2<fp>::p1().vec();
        auto d = (v1.x * v2.y - v1.y * v2.x);
        return d;
    }

    template <typename fp>
    bool line2<fp>::strict_intersects(const line_segment2<fp>& ls2) const
    {
        auto d1 = _ltr(ls2.p1());
        auto d2 = _ltr(ls2.p2());
        if (d1 == decltype(d1)::zero() && d2 == decltype(d2)::zero())
            return true;
        if (d1 == decltype(d1)::zero())
            return true;
        if (d2 == decltype(d2)::zero())
            return false;
        if ((d1 < decltype(d1)::zero()) != (d2 < decltype(d2)::zero()))
            return true;
        return false;
    }

    template <typename fp>
    bool line2<fp>::intersects(const line_segment2<fp>& ls2) const
    {
        auto d1 = _ltr(ls2.p1());
        auto d2 = _ltr(ls2.p2());
        if (d1 == decltype(d1)::zero() || d2 == decltype(d2)::zero())
            return true;
        if ((d1 < decltype(d1)::zero()) != (d2 < decltype(d2)::zero()))
            return true;
        return false;
    }

    template <typename fp>
    std::optional<point2<fp>> line2<fp>::strict_intersection(const line_segment2<fp>& ls2) const
    {
        if (!strict_intersects(ls2))
            return {};

        auto r = line2<fp>::p2().vec() - line2<fp>::p1().vec();
        auto s = ls2.p2().vec() - ls2.p1().vec();

        auto dev = r.x * s.y - r.y * s.x;

        const auto& p = line2<fp>::p1().vec();
        const auto& q = ls2.p1().vec();

        auto qp = q - p;

        auto t = (qp.x * s.y - qp.y * s.x) / dev;
        if (sixit::geometry::low_level::mathf::isfinite(t))
            return point2<fp>(p + r * t);
        else
            return {};
    }

    /**
     * Signed area of parallelorgam formed by v1, v2, where: `v1 = normalized(end - start);` and `v2 = end - p;`
     * note that the `v1` is normalized
     * positive means clockwise rotation v1->v2, negative - counter-clockwise.
     */
    template <typename fp>
	sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> ray2<fp>::_ltr(const point2<fp>& p) const
	{
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        if (p == origin_data)
            return fp_0;

		const auto& v1 = direction_data.vec() * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
		const auto& v2 = p.vec() - origin_data.vec();
		sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> d = (v1.x * v2.y - v1.y * v2.x);
		return d;
	}

    template <typename fp>
    bool ray2<fp>::strict_intersects(const line_segment2<fp>& ls2) const
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        //p + r*t
        //q + s*u

        if (!intersects(ls2))
            return false;

        auto d2 = _ltr(ls2.p2());

        if (d2 == fp_0)
            return false;
        return true;
    }

    template <typename fp>
    bool ray2<fp>::intersects(const line_segment2<fp>& ls2) const
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        auto d1 = _ltr(ls2.p1());
        auto d2 = _ltr(ls2.p2());

        if (d1 != fp_0 && d2 != fp_0)
        {
            if ((d1 < fp_0) == (d2 < fp_0))
                return false;
            if ((d1 > fp_0) == (d2 > fp_0))
                return false;
        }

        low_level::dimensional_vector2 r = direction_data.vec() * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
        low_level::dimensional_vector2 s = ls2.p2().vec() - ls2.p1().vec();

        auto dev = r.x * s.y - r.y * s.x;

        low_level::dimensional_vector2 p = origin_data.vec();
        low_level::dimensional_vector2 q = ls2.p1().vec();

        low_level::dimensional_vector2 qp = q - p;

        auto t = (qp.x * s.y - qp.y * s.x) / dev;
        //fp t = (qp.x * s.y - qp.y * s.x);//Possible optimization, not needed to devide, we compare to zero, check sign
        return sixit::geometry::low_level::mathf::isfinite(t) && t >= decltype(t)::zero() ? true : false;
    }

    template <typename fp>
    std::optional<point2<fp>> ray2<fp>::strict_intersection(const line_segment2<fp>& ls2) const
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        if (!strict_intersects(ls2))
            return {};

        low_level::dimensional_vector2 r = direction_data.vec();
        low_level::dimensional_vector2 s = ls2.p2().vec() - ls2.p1().vec();

        auto dev = r.x * s.y - r.y * s.x;

        low_level::dimensional_vector2 p = origin_data.vec();
        low_level::dimensional_vector2 q = ls2.p1().vec();

        low_level::dimensional_vector2 qp = q - p;

        auto t = (qp.x * s.y - qp.y * s.x) / dev;
        if (sixit::geometry::low_level::mathf::isfinite(t))
            return point2<fp>(p + r * t);
        else
            return {};
    }

    template <typename fp>
    void bounds3<fp>::expand_by(const point3<fp>& p)
    {
        minn.vec().x() = low_level::mathf::min(minn.vec().x(), p.vec().x());
        minn.vec().y() = low_level::mathf::min(minn.vec().y(), p.vec().y());
        minn.vec().z() = low_level::mathf::min(minn.vec().z(), p.vec().z());

        maxx.vec().x() = low_level::mathf::max(maxx.vec().x(), p.vec().x());
        maxx.vec().y() = low_level::mathf::max(maxx.vec().y(), p.vec().y());
        maxx.vec().z() = low_level::mathf::max(maxx.vec().z(), p.vec().z());
    }

    template <typename fp>
    void bounds3<fp>::expand_by(const bounds3<fp>& other)
    {
        minn.vec().x() = low_level::mathf::min(minn.vec().x(), other.minn.vec().x());
        minn.vec().y() = low_level::mathf::min(minn.vec().y(), other.minn.vec().y());
        minn.vec().z() = low_level::mathf::min(minn.vec().z(), other.minn.vec().z());

        maxx.vec().x() = low_level::mathf::max(maxx.vec().x(), other.maxx.vec().x());
        maxx.vec().y() = low_level::mathf::max(maxx.vec().y(), other.maxx.vec().y());
        maxx.vec().z() = low_level::mathf::max(maxx.vec().z(), other.maxx.vec().z());
    }

    template <typename fp>
    bool bounds3<fp>::intersects(const bounds3<fp>& o) const
    {
        low_level::dimensional_vector3 min = minn.vec();
        low_level::dimensional_vector3 max = maxx.vec();
        low_level::dimensional_vector3 o_min = o.minn.vec();
        low_level::dimensional_vector3 o_max = o.maxx.vec();
        return min.x() < o_max.x() && o_min.x() < max.x() && min.y() < o_max.y() && o_min.y() < max.y() && min.x() < o_max.z() && o_min.z() < max.z();
    }

    template <typename fp>
    bool bounds3<fp>::is_exactly_on_boundary(const point3<fp>& p) const
    {
        low_level::dimensional_vector3 vmin = minn.vec();
        low_level::dimensional_vector3 vmax = maxx.vec();
        low_level::dimensional_vector3 vp = p.vec();
        namespace m = low_level::mathf;
        bool test0 = vp.x() == vmin.x() || vp.y() == vmin.y() || vp.z() == vmin.z();
        bool test1 = vp.x() == vmax.x() || vp.y() == vmax.y() || vp.z() == vmax.z();
        return test0 || test1;
    }

    template <typename fp>
    bool bounds3<fp>::is_inside(const point3<fp>& p) const
    {
        low_level::dimensional_vector3 vmin = minn.vec();
        low_level::dimensional_vector3 vmax = maxx.vec();
        low_level::dimensional_vector3 vp = p.vec();
        return vmin.x() < vp.x() && vp.x() < vmax.x() && vmin.y() < vp.y() && vp.y() < vmax.y() && vmin.z() < vp.z() && vp.z() < vmax.z();
    }

    template <typename fp>
    void bounds2<fp>::expand_by(const point2<fp>& p)
    {
        minn.vec().x = low_level::mathf::min(minn.vec().x, p.vec().x);
        minn.vec().y = low_level::mathf::min(minn.vec().y, p.vec().y);

        maxx.vec().x = low_level::mathf::max(maxx.vec().x, p.vec().x);
        maxx.vec().y = low_level::mathf::max(maxx.vec().y, p.vec().y);
    }

    template <typename fp>
    void bounds2<fp>::expand_by(const bounds2<fp>& other)
    {
        minn.vec().x = low_level::mathf::min(minn.vec().x, other.minn.vec().x);
        minn.vec().y = low_level::mathf::min(minn.vec().y, other.minn.vec().y);

        maxx.vec().x = low_level::mathf::max(maxx.vec().x, other.maxx.vec().x);
        maxx.vec().y = low_level::mathf::max(maxx.vec().y, other.maxx.vec().y);
    }


    template <typename fp>
    bool bounds2<fp>::intersects(const bounds2<fp>& o) const
    {
        const auto& min = minn.vec();
        const auto& max = maxx.vec();
        const auto& o_min = o.minn.vec();
        const auto& o_max = o.maxx.vec();

        return !(max.x < o_min.x || min.x > o_max.x || max.y < o_min.y || min.y > o_max.y);
    }

    template <typename fp>
    bool bounds2<fp>::is_inside(const point2<fp>& p) const
    {
        const auto& vmin = minn.vec();
        const auto& vmax = maxx.vec();
        const auto& vp = p.vec();
        return vmin.x < vp.x && vp.x < vmax.x && vmin.y < vp.y && vp.y < vmax.y;
    }

    template <typename fp>
    bool bounds2<fp>::is_exactly_on_boundary(const point2<fp>& p) const
    {
        low_level::vector2<fp> vmin = minn.vec();
        low_level::vector2<fp> vmax = maxx.vec();
        low_level::vector2<fp> vp = p.vec();
        namespace m = low_level::mathf;
        bool test0 = vp.x() == vmin.x() || vp.y() == vmin.y();
        bool test1 = vp.x() == vmax.x() || vp.y() == vmax.y();
        return test0 || test1;
    }

    template <typename fp>
    polygon2<fp> bounds2<fp>::to_polygon() const
    {
        auto min = minn.vec();
        auto max = maxx.vec();
        return polygon2({ {min.x, min.y}, {max.x, min.y}, {max.x, max.y}, {min.x, max.y}, });
    }

    template <typename fp>
    void bounds1<fp>::expand_by(const point1<fp>& p)
    {
        minn.vec() = low_level::mathf::min(minn.vec(), p.vec());

        maxx.vec() = low_level::mathf::max(maxx.vec(), p.vec());
    }

    template <typename fp>
    void bounds1<fp>::expand_by(const bounds1<fp>& other)
    {
        minn.vec() = low_level::mathf::min(minn.vec(), other.minn.vec());

        maxx.vec() = low_level::mathf::max(maxx.vec(), other.maxx.vec());
    }

    template <typename fp>
    bool bounds1<fp>::intersects(const bounds1<fp>& o) const
    {
        fp min = minn.vec();
        fp max = maxx.vec();
        fp o_min = o.minn.vec();
        fp o_max = o.maxx.vec();
        return min < o_max && o_min < max;
    }

    template <typename fp>
    bool bounds1<fp>::is_inside(const point1<fp>& p) const
    {
        fp vmin = minn.vec();
        fp vmax = maxx.vec();
        fp vp = p.vec();
        return vmin < vp && vp < vmax;
    }

    template <typename fp>
    bool bounds1<fp>::is_exactly_on_boundary(const point1<fp>& p) const
    {
        fp vmin = minn.vec();
        fp vmax = maxx.vec();
        fp vp = p.vec();

        namespace m = low_level::mathf;
        // TODO: Is this corrent behavior in this case?
        bool test0 = vp == vmin || vp == vmax;
        return test0; 
    }

    // todo: use for ortho plane from line ...
    template <typename fp>
    plane3<fp> triangle3<fp>::plane() const
    {
        return low_level::tri3_impl<fp>::plane(this->p1(), this->p2(), this->p3());
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> triangle3<fp>::square() const
    {
        return low_level::tri3_impl<fp>::square(this->p1(), this->p2(), this->p3());
    }

    template <typename fp>
    direction3<fp> triangle3<fp>::normal() const
    {
        return low_level::tri3_impl<fp>::normal(this->p1(), this->p2(), this->p3());
    }

    template <typename fp>
    point3<fp> triangle3<fp>::barycenter() const
    {
        return low_level::tri3_impl<fp>::barycenter(this->p1(), this->p2(), this->p3());
    }

    template <typename fp>
    point3<fp> triangle3<fp>::vertex(int idx) const
    {
        assert(idx >= 0 && idx < this->points.size());
        return this->points[idx];
    }

    template <typename fp>
    line_segment3<fp> triangle3<fp>::edge(int idx) const
    {
        if (idx == 0)
        {
            return {this->points[0], this->points[1]};
        }
        else if (idx == 1)
        {
            return {this->points[1], this->points[2]};
        }
        else if (idx == 2)
        {
            return {this->points[2], this->points[0]};
        }
        assert(false);
        return line_segment3(point3<fp>(), point3<fp>());
    }

    template <typename fp>
    triangle3<fp> triangle3<fp>::face(int idx) const
    {
        if (idx == 0)
        {
            triangle3 tr = *this;
            return tr;
        }
        assert(false);
        return *this;
    }

    template <typename fp>
    ray3<fp> triangle3<fp>::bisector(int idx) const
    {
        return low_level::tri3_impl<fp>::bisector(this->p1(), this->p2(), this->p3(), idx);
    }

    template <typename fp>
    bounds3<fp> triangle3<fp>::bounds() const
    {
        return low_level::tri3_impl<fp>::bounds(this->p1(), this->p2(), this->p3());
    }

    template <typename fp>
    triple<line_segment3<fp>> triangle3<fp>::edges() const
    {
        return triple<line_segment3<fp>>(this->edge(0), this->edge(1), this->edge(2));
    }

    template <typename fp>
    triple<point3<fp>> triangle3<fp>::vertices() const
    {
        return triple<point3<fp>>(this->vertex(0), this->vertex(1), this->vertex(2));
    }

    template <typename fp>
    bounds2<fp> triangle2<fp>::bounds() const
    {
        return low_level::tri2_impl<fp>::bounds(
            this->p1(),
            this->p2(),
            this->p3());
    }

    template <typename fp>
    point2<fp> triangle2<fp>::vertex(int idx) const
    {
        assert(idx >= 0 && idx < this->points.size());
        return this->points[idx];
    }

    template <typename fp>
    line_segment2<fp> triangle2<fp>::edge(int idx) const
    {
        if (idx == 0)
        {
            return { this->points[0], this->points[1]};
        }
        else if (idx == 1)
        {
            return { this->points[1], this->points[2]};
        }

        assert(idx == 2);
        return {this->points[2], this->points[0]};
    }

    template <typename fp>
    triangle2<fp> triangle2<fp>::face([[maybe_unused]] int idx) const
    {
        assert(idx == 0);
        return *this;
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> triangle2<fp>::square() const
    {
        return low_level::tri2_impl<fp>::square(triangle2<fp>::p1(), triangle2<fp>::p2(), triangle2<fp>::p3());
    }

    template <typename fp>
    int triangle2<fp>::normal() const
    {
        return low_level::tri2_impl<fp>::normal(triangle2<fp>::p1(), triangle2<fp>::p2(), triangle2<fp>::p3());
    }

    template <typename fp>
    point2<fp> triangle2<fp>::barycenter() const
    {
        return low_level::tri2_impl<fp>::barycenter(triangle2<fp>::p1(), triangle2<fp>::p2(), triangle2<fp>::p3());
    }

    template <typename fp>
    bool triangle2<fp>::is_inside(const point2<fp> &p) const
    {
        return low_level::tri2_impl<fp>::is_inside_basic(triangle2<fp>::p1(), triangle2<fp>::p2(), triangle2<fp>::p3(), p);
    }

    template <typename fp>
    bool triangle2<fp>::is_inside_combinable(const point2<fp>& p) const
    {
        return low_level::tri2_impl<fp>::is_inside_combinable(triangle2<fp>::p1(), triangle2<fp>::p2(), triangle2<fp>::p3(), p);
    }

    template <typename fp>
    triple<line_segment2<fp>> triangle2<fp>::edges() const
    {
        return triple<line_segment2<fp>>(edge(0), edge(1), edge(2));
    }

    template <typename fp>
    triple<point2<fp>> triangle2<fp>::vertices() const
    {
        return triple<point2<fp>>(vertex(0), vertex(1), vertex(2));
    }

    // template <typename fp, typename TSetPixel>
    // SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
    // void supercover_line(const line_segment2<fp> &ls2, TSetPixel& SET_PIXEL)
    template <typename fp>
    template <typename TSetPixel>
    SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
    inline void line_segment2<fp>::supercover_line(TSetPixel& SET_PIXEL) const
    {
        const point2<fp>& p1 = line_segment2<fp>::p1();
        const point2<fp>& p2 = line_segment2<fp>::p2();

        low_level::dimensional_vector2 v1 = p1.vec();
        low_level::dimensional_vector2 v2 = p2.vec();
        fp x0 = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(v1.x);
        fp y0 = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(v1.y);
        fp x1 = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(v2.x);
        fp y1 = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(v2.y);

        fp dx = x1 - x0;
        fp dy = y1 - y0;
        fp x = x0;
        fp y = y0;
        bool x_dominate = low_level::mathf::abs(dx) > low_level::mathf::abs(dy);

        if (x_dominate)
        {
            fp sx = dx > fp(0.f) ? fp(1.0f) : fp(-1.0f);
            int x_num = dx > fp(0.f) ? low_level::mathf::fp2int(low_level::mathf::trunc(dx + fp(0.5f))) : low_level::mathf::fp2int(low_level::mathf::trunc(-dx + fp(0.5f)));
            fp sy = dy / dx * sx;

            int y_sign = sy >= fp(0.f) ? 1 : -1;

            fp x_rest = dx > fp(0.f) ? low_level::mathf::ceil(x0) - x0 : x0 - low_level::mathf::floor(x0);//always the same
            fp y_rest_max = low_level::mathf::abs(x_rest * sy);
            for (int xi = 0; xi < x_num; ++xi)
            {
                SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x)), low_level::mathf::fp2int(low_level::mathf::trunc(y)));
                int prev_y = low_level::mathf::fp2int(low_level::mathf::trunc(y));
                fp y_rest = dy > fp(0.f) ? low_level::mathf::ceil(y) - y : y - low_level::mathf::floor(y);
                x = x + sx;
                y = y + sy;
                if (prev_y != low_level::mathf::fp2int(low_level::mathf::trunc(y)))//not covered part detected
                {
                    if (y_rest < y_rest_max) //up left
                        SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x - sx)), low_level::mathf::fp2int(low_level::mathf::trunc(y)));
                    else  //down right
                        SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x)), low_level::mathf::fp2int(low_level::mathf::trunc(y - fp(float(y_sign)))));
                }
            }
        }
        else
        {
            fp sy = dy > fp(0.f) ? fp(1.0f) : fp(-1.0f);
            int y_num = dy > fp(0.f) ? low_level::mathf::fp2int(low_level::mathf::trunc(dy + fp(0.5f))) : low_level::mathf::fp2int(low_level::mathf::trunc(-dy + fp(0.5f)));
            fp sx = dx / dy * sy;

            int x_sign = sx >= fp(0.f) ? 1 : -1;

            fp y_rest = dy > fp(0.f) ? low_level::mathf::ceil(y0) - y0 : y0 - low_level::mathf::floor(y0);
            fp x_rest_max = low_level::mathf::abs(y_rest * sx);
            for (int yi = 0; yi < y_num; ++yi)
            {
                SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x)), low_level::mathf::fp2int(low_level::mathf::trunc(y)));
                int prev_x = low_level::mathf::fp2int(low_level::mathf::trunc(x));
                fp x_rest = dx > fp(0.f) ? low_level::mathf::ceil(x) - x : x - low_level::mathf::floor(x);
                x = x + sx;
                y = y + sy;
                if (prev_x != low_level::mathf::fp2int(low_level::mathf::trunc(x)))//not covered part detected
                {
                    if (x_rest < x_rest_max) //up left
                        SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x)), low_level::mathf::fp2int(low_level::mathf::trunc(y - sy)));
                    else  //down right
                        SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x - fp(float(x_sign)))), low_level::mathf::fp2int(low_level::mathf::trunc(y)));
                }
            }
        }
        SET_PIXEL(low_level::mathf::fp2int(low_level::mathf::trunc(x1)), low_level::mathf::fp2int(low_level::mathf::trunc(y1)));
    }

    template <typename fp>
    template<typename TSetPixel>
    inline void line_segment2<fp>::rasterize(TSetPixel& SET_PIXEL) const
    {
        rasterize(line_segment2<fp>::p1(), line_segment2<fp>::p2(), SET_PIXEL);
    }

    template <typename fp>
    template<typename TSetPixel>
    void line_segment2<fp>::rasterize(const point2<fp>& p1, const point2<fp>& p2, TSetPixel& SET_PIXEL)
    {
        //bresenham algorithm
        int x1 = low_level::mathf::fp2int(low_level::mathf::floor(p1.vec().x));
        int y1 = low_level::mathf::fp2int(low_level::mathf::floor(p1.vec().y));
        int x2 = low_level::mathf::fp2int(low_level::mathf::floor(p2.vec().x));
        int y2 = low_level::mathf::fp2int(low_level::mathf::floor(p2.vec().y));

        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            SET_PIXEL(x1, y1);

            if (x1 == x2 && y1 == y2)
                break;

            int err2 = err << 1;
            if (err2 > -dy)
            {
                err = err - dy;
                x1 = x1 + sx;
            }
            if (err2 < dx)
            {
                err = err + dx;
                y1 = y1 + sy;
            }
        }
    }
    
    template <typename fp>
    template<typename TSetPixelAA>
    inline void line_segment2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const
    {
        rasterize_aa(line_segment2<fp>::p1(), line_segment2<fp>::p2(), SET_PIXEL_AA);
    }

    template <typename fp>
    template<typename TSetPixelAA>
    inline void line_segment2<fp>::rasterize_aa(const point2<fp>& p1, const point2<fp>& p2, TSetPixelAA& SET_PIXEL_AA)
    {
        //bresenham algorithm

        int x1 = low_level::mathf::fp2int(low_level::mathf::floor(p1.vec().x));
        int y1 = low_level::mathf::fp2int(low_level::mathf::floor(p1.vec().y));
        int x2 = low_level::mathf::fp2int(low_level::mathf::floor(p2.vec().x));
        int y2 = low_level::mathf::fp2int(low_level::mathf::floor(p2.vec().y));

        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        fp ed = ((dx + dy) == 0 ? fp(1.f) : low_level::mathf::sqrt(fp(float(dx * dx + dy * dy))));

        while (true)
        {
            SET_PIXEL_AA(x1, y1, fp(1.f) - fp(float(std::abs(err - dx + dy))) / ed);

            if (x1 == x2 && y1 == y2)
                break;

            int err2 = err;
            int x1_copy = x1; 

            if ((err2 * 2) >= -dx)
            {
                if (fp(float(err2 + dy)) < ed)
                {
                    SET_PIXEL_AA(x1, y1 + sy, fp(1.f) - fp(float(err2 + dy)) / ed);
                }
                err = err - dy;
                x1 = x1 + sx;
            }
            if ((err2 * 2) <= dy)
            {
                if (fp(float(dx - err2)) < ed)
                {
                    SET_PIXEL_AA(x1_copy + sx, y1, fp(1.f) - fp(float(dx - err2)) / ed);
                }
                err = err + dx;
                y1 = y1 + sy;
            }
        }    
    }


    template <typename fp>
    template <typename TSetVoxel>//TSetVoxel should have operator () (int x, int y, int z)
    SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
    void line_segment3<fp>::line_voxelize(TSetVoxel& SET_VOXEL)
    {
        const low_level::vector3<fp>& p0 = line_segment3::p1().vec();
        const low_level::vector3<fp>& p1 = line_segment3::p2().vec();

        auto lc = line_segment3::largest_coordinate();

        XYZ axis = std::get<XYZ>(lc);
        int first_coord = 1;
        int second_coord = 2;

        if (axis == XYZ::Y)
        {
            first_coord = 0;
        }
        else if (axis == XYZ::Z)
        {
            first_coord = 0;
            second_coord = 1;
        }

        int first_plane = p0[int(axis)];
        int last_plane = p0[int(axis)];

        fp dx_d_dom = (p1[first_coord] - p0[first_coord]) / std::get<fp>(lc);
        fp dy_d_dom = (p1[second_coord] - p0[second_coord]) / std::get<fp>(lc);

        int prev_x = p0[first_coord];
        int prev_y = p0[second_coord];


        int plane_num = 0;

        auto SET_PIXEL = [&](int x, int y)
        {
            switch (axis)
            {
            case XYZ::X: SET_VOXEL(plane_num, x, y); break;
            case XYZ::Y: SET_VOXEL(x, plane_num, y); break;
            case XYZ::Z: SET_VOXEL(x, y, plane_num); break;
            default: assert(0);
            }
        };

        for (plane_num = first_plane; plane_num <= last_plane; ++plane_num)
        {
           // int prev_y = int(y);

            fp plane_coord = plane_num + 0.5f - p0[int(axis)];
            fp x_c = p0[first_coord]  + plane_coord * dx_d_dom;
            fp y_c = p0[second_coord] + plane_coord * dy_d_dom;

            if (prev_x == int(x_c))
            {

            }

            if (prev_y == int(y_c))
            {

            }



           // line_segment2 ls(point2(x_c_base, y_c_base), point2(x_c_first, y_c_first));
           // supercover_line(ls, SET_PIXEL);
        }

    }

    // template <typename TSetVoxel>//TSetVoxel should have operator () (int x, int y, int z)
    // SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
    // void triangle_voxelize(const triangle3 & trng, TSetVoxel& SET_VOXEL)
    template <typename fp>
    template <typename TSetVoxel>//TSetVoxel should have operator () (int x, int y, int z)
    SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
    void triangle3<fp>::triangle_voxelize(TSetVoxel& SET_VOXEL) const
    {
        const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& p0 = triangle3<fp>::p1().vec();
        const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& p1 = triangle3<fp>::p2().vec();
        const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& p2 = triangle3<fp>::p3().vec();

        line_segment3<fp> ls[3] = { edge(0), edge(1), edge(2) };

       // for (int i = 0; i < 3; ++i)

       // line_segment3 ls0 = trng.edge(0);
      //  line_segment3 ls1 = trng.edge(1);
       // line_segment3 ls2 = trng.edge(2);

      //  XYZ axis = XYZ::X;

        auto lc = ls[0].largest_coordinate();
        auto lc1 = ls[1].largest_coordinate();

        int dom_edge = 0;

        if (std::get<fp>(lc) < std::get<fp>(lc1))
        {
            lc = lc1;
            dom_edge = 1;

       //     XYZ axis = XYZ::Y;
        }
        auto lc2 = ls[2].largest_coordinate();
        if (std::get<fp>(lc) < std::get<fp>(lc2))
        {
            lc = lc2;
            dom_edge = 2;
      //      XYZ axis = XYZ::Z;
        }

        XYZ axis = std::get<XYZ>(lc);

        int first_coord = 1;
        int second_coord = 2;

        if (axis == XYZ::Y)
        {
            first_coord = 0;
        } else if (axis == XYZ::Z)
        {
            first_coord = 0;
            second_coord = 1;
        }

        int plane_num = 0;

        auto SET_PIXEL = [&](int x, int y)
        {
            switch (axis)
            {
                case XYZ::X: SET_VOXEL(plane_num, x, y); break;
                case XYZ::Y: SET_VOXEL(x, plane_num, y); break;
                case XYZ::Z: SET_VOXEL(x, y, plane_num); break;
                default: assert(0);
            }
        };

        low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> p_sorted[3];// = { p0, 01, p2 };
        p_sorted[0] = p0;
        p_sorted[1] = p1;
        p_sorted[2] = p2;
        auto pnt_less = [&](const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& left, 
                            const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& right) -> bool
        {
            if (left[int(axis)] < right[int(axis)])
                return true;
            return false;
        };

        std::sort(p_sorted, p_sorted + 3, pnt_less);

       // fp dx_d_dom_base = (p_sorted[0][first_coord] - p_sorted[2][first_coord]) / (p_sorted[0][int(axis)] - p_sorted[2][int(axis)]);
      //  fp dy_d_dom_base = (p_sorted[0][second_coord] - p_sorted[2][second_coord]) / (p_sorted[0][int(axis)] - p_sorted[2][int(axis)]);

        fp dx_d_dom_base = (p_sorted[2][first_coord] -  p_sorted[0][first_coord])  / std::get<fp>(lc);
        fp dy_d_dom_base = (p_sorted[2][second_coord] - p_sorted[0][second_coord]) / std::get<fp>(lc);

        fp dx_d_dom_first = (p_sorted[1][first_coord] -  p_sorted[0][first_coord])  / (p_sorted[1][int(axis)] - p_sorted[0][int(axis)]);
        fp dy_d_dom_first = (p_sorted[1][second_coord] - p_sorted[0][second_coord]) / (p_sorted[1][int(axis)] - p_sorted[0][int(axis)]);

        fp dx_d_dom_sec = (p_sorted[2][first_coord] - p_sorted[1][first_coord])   / (p_sorted[2][int(axis)] - p_sorted[1][int(axis)]);
        fp dy_d_dom_sec = (p_sorted[2][second_coord] - p_sorted[1][second_coord]) / (p_sorted[2][int(axis)] - p_sorted[1][int(axis)]);

        int first_plane = p_sorted[0][int(axis)];
        int mid_plane = p_sorted[1][int(axis)];
        int last_plane = p_sorted[2][int(axis)];

        for (plane_num = first_plane; plane_num <= mid_plane - 1; ++plane_num)
        {
            fp plane_coord = plane_num + 0.5f - p_sorted[0][int(axis)];
            fp x_c_base = p_sorted[0][first_coord]  + plane_coord * dx_d_dom_base;
            fp y_c_base = p_sorted[0][second_coord] + plane_coord * dy_d_dom_base;
            fp x_c_first = p_sorted[0][first_coord]  + plane_coord * dx_d_dom_first;
            fp y_c_first = p_sorted[0][second_coord] + plane_coord * dy_d_dom_first;

            line_segment2<fp> ls(
                point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x_c_base), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y_c_base)), 
                point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x_c_first), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y_c_first)));
            ls.supercover_line(SET_PIXEL);
        }
        for (plane_num = mid_plane; plane_num <= last_plane; ++plane_num)
        {
            fp plane_coord = plane_num + 0.5f - p_sorted[0][int(axis)];
            fp x_c_base = plane_coord * dx_d_dom_base;
            fp y_c_base = plane_coord * dy_d_dom_base;
            fp x_c_first = plane_coord * dx_d_dom_first;
            fp y_c_first = plane_coord * dy_d_dom_first;

            line_segment2<fp> ls(
                point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x_c_base), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y_c_base)), 
                point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x_c_first), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y_c_first)));
            ls.supercover_line(SET_PIXEL);
        }

      /*  if (p0[int(axis)] < p1[int(axis)] && p0[int(axis)] < p1[int(axis)])
        {
            p_sorted[0] = p0;
            if (p1[int(axis)] < p2[int(axis)])
            {
                p_sorted[1] = p1;
                p_sorted[2] = p2;
            }
            else
            {
                p_sorted[2] = p1;
                p_sorted[1] = p2;
            }
        }*/

        if (std::get<XYZ>(lc) == XYZ::X)
        {
//            first_plane
        }

       /* template <typename TSetVox>
        struct SET_PIXEL
        {
            XYZ plane = XYZ::X;
            int plane_num = 0;
            TSetVox& _SET_VOXEL;// = SET_VOXEL;
           // char coord_map[2] = { 0, 1 };
            operator () (int x, int y)
            {
                switch (plane)
                {
                    case XYZ::X: _SET_VOXEL(plane_num, x, y); break;
                    case XYZ::Y: _SET_VOXEL(x, plane_num, y); break;
                    case XYZ::Z: _SET_VOXEL(x, y, plane_num); break;
                default: assert(0);
                }
            }
        };*/
       // lc1.

    }
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_line_segment_impl_h_included

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