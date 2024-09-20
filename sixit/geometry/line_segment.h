/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_line_segment_h_included
#define sixit_geometry_line_segment_h_included

#include <optional>
#include <tuple>
#include <array>
#include <utility>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/projection3to2noscale.h"
#include "sixit/geometry/projection2to1noscale.h"
#include "sixit/geometry/plane.h"
#include "sixit/geometry/low_level/line_common_impl.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/curve_primitive.h"

//#define _HAS_STD_BYTE 0

//class imp_scene_to_mesh3_converter;

namespace sixit
{
namespace geometry
{
    template<typename T>
    using triple = std::tuple<T,T,T>;

    template<typename T>
    using couple = std::tuple<T,T>;

    template <typename fp>
    struct line_segment3: low_level::two_points_holder<point3<fp>>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment3<fp>>;
        friend class voxelizer;
        friend struct triangle3<fp>;
        friend struct plane3<fp>;
        friend class sixit::graphics::intersectable_mesh;

        // already have these friend declarations in two_points_holder, but still need to duplicate it here for XCode 15
        // template<typename fp1, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void(::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp1>& ls2, TSetPixel& SET_PIXEL);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_PIXEL);

      public:
        template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        void line_voxelize(TSetVoxel& SET_VOXEL);
        inline line_segment3(const point3<fp>& p1, const point3<fp>& p2): low_level::two_points_holder<point3<fp>>{p1, p2} {}
        inline bool operator==(const line_segment3& o) const;
        inline point3<fp> interpolate(fp t) const;
        inline std::tuple<enum XYZ, fp> largest_coordinate() const;
        inline line3<fp> line() const;
        inline direction3<fp> direction() const;
        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length() const;
        inline std::pair<point3<fp>, rotation3<fp>> track(fp x) const;
        inline bounds3<fp> bounds() const;
    };

    template <typename fp>
    struct line_segment2: low_level::two_points_holder<point2<fp>>, curve2_primitive<fp>
    {
        friend struct projection2to1noscale<fp>;
        friend struct polygon2<fp>;
        friend struct ray2<fp>;
        template <typename fp1, XYZ axis> friend struct axis_oriented_ray2;
        friend struct line2<fp>;
        friend struct arc2<fp>;
        friend struct qbezier2<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment2<fp>>;
        friend struct shape2_base<fp>;
        friend struct shape2<fp>;
        friend class grid2<fp>;

        // template<typename fp1, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void (::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp1>& ls2, TSetPixel& SET_PIXEL);
        template <typename fp1>
        friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_renderers_only::xy_dont_use_outside_of_renderer(const line_segment2<fp1>& l);

        inline sixit::units::dimensional_scalar<fp, sixit::units::square_meter::dim> _ltr(const point2<fp>& p) const;
        inline bool _strict_intersects_as_ray(const line_segment2<fp>& ls2) const;
        inline bool _intersects_as_ray(const line_segment2<fp>& ls2) const;

      public:
        inline line_segment2(const sixit::rw::comparsers::constructor_for_rw_tag&): low_level::two_points_holder<point2<fp>>{{}, {}} {}
        inline line_segment2(const point2<fp>& p1, const point2<fp>& p2): low_level::two_points_holder<point2<fp>>{p1, p2} {}
        inline bool operator==(const line_segment2& o) const;

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const curve2_primitive<fp>& other, size_t n = 1) const override
        {
            if (other.get_primitive_type() != get_primitive_type())
                return false;

            // rationale: using dynamic cast to convert other object to its actual type for comparison
            return _for_test_only_approximate_eq(dynamic_cast<const line_segment2<fp>&>(other), n);
        }

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const line_segment2<fp>& other, size_t n = 1) const
        {
            return line_segment2<fp>::p1()._for_test_only_approximate_eq(other.p1(), n)
                && line_segment2<fp>::p2()._for_test_only_approximate_eq(other.p2(), n);
        }

        inline line2<fp> line() const;
        inline direction2<fp> direction() const;
        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length() const;
        inline bounds2<fp> bounds() const;
        inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
        inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const;
        inline bool strict_intersects(const line_segment2<fp>& ls2) const;
        inline bool intersects(const line_segment2<fp>& ls2) const;
        inline bool overlays(const line_segment2<fp>& ls2) const;
        inline std::optional<point2<fp>> strict_intersection(const line_segment2<fp>& ls2) const;
        inline bool point_in_segment(const point2<fp>& p) const;
        inline std::array<std::optional<line_segment2<fp>>, 2> cover_with(const triangle2<fp>& tri) const;

        template <typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        inline void supercover_line(TSetPixel& SET_PIXEL) const;

        // virtual intersection methods with all other curve primitives
        inline typename curve2_primitive<fp>::primitive_type get_primitive_type() const { return curve2_primitive<fp>::primitive_type::line_segment; }
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

        inline std::optional<point2<fp>> intersection(const line_segment2<fp>& other) const;

        // rasterize
        template<typename TSetPixel>
        inline void rasterize(TSetPixel& SET_PIXEL) const;

        template<typename TSetPixel>
        inline static void rasterize(const point2<fp>& p1, const point2<fp>& p2, TSetPixel& SET_PIXEL);

        template<typename TSetPixelAA>
        inline void rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const;

        template<typename TSetPixelAA>
        inline static void rasterize_aa(const point2<fp>& p1, const point2<fp>& p2, TSetPixelAA& SET_PIXEL_AA);

        template<typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            // TODO: refactor to AofSTRUCT after TR-2362
            sixit::rw::begin_struct<"line_segment2">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p1", obj.p1());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::STRUCT>(comparser, "p2", obj.p2());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct line_segment1: low_level::two_points_holder<point1<fp>>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment1<fp>>;
        friend struct projection2to1noscale<fp>;

      public:
        using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;

        inline line_segment1(const point1<fp>& p1, const point1<fp>& p2): low_level::two_points_holder<point1<fp>>{p1, p2} {}
        inline bool operator==(const line_segment1<fp>& o) const;
        inline line1<fp> line() const;
        inline direction1<fp> direction() const;
        inline class_value length() const;
        inline std::pair<point1<fp>, rotation1<fp>> track(fp x) const;
        inline bounds1<fp> bounds() const;
        inline bool overlays(const line_segment1<fp>& ls1) const;

        template <typename LS1>
        inline bool intersects(const LS1& o) const
        {
            class_value min1 = low_level::mathf::min(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec());
            class_value max1 = low_level::mathf::max(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec());
            class_value min2 = low_level::mathf::min(o.p1().vec(), o.p2().vec());
            class_value max2 = low_level::mathf::max(o.p1().vec(), o.p2().vec());
            return min1 < max2 && min2 < max1;
        }
        template <typename LS1>
        inline std::optional<line_segment1> intersection(const LS1& o) const
        {
            class_value min1 = low_level::mathf::min(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec());
            class_value max1 = low_level::mathf::max(line_segment1<fp>::p1().vec(), line_segment1<fp>::p2().vec());
            class_value min2 = low_level::mathf::min(o.p1().vec(), o.p2().vec());
            class_value max2 = low_level::mathf::max(o.p1().vec(), o.p2().vec());
            if (min1 < max2 && min2 < max1)
            {
                return line_segment1(low_level::mathf::max(min1, min2), low_level::mathf::min(max1, max2));
            }
            else
            {
                return {};
            }
        }
    };

    template <typename fp>
    struct triangle3: low_level::three_points_holder<point3<fp>>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::triangle3<fp>>;
        friend struct projection3to2noscale<fp>;
        friend struct plane3<fp>;
        friend sixit::graphics::intersectable_mesh;
        // friend imp_scene_to_mesh3_converter;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct indexed_triangle3;
        using counterpart_2d = triangle2<fp>;

        // already have these friend declarations in two_points_holder, but still need to duplicate it here for XCode 15
        // template<typename fp1, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void(::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp1>& ls2, TSetPixel& SET_PIXEL);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_PIXEL);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::line_voxelize)(const ::sixit::geometry::line_segment3& ls3, TSetVoxel& SET_VOXEL);

      public:
        inline triangle3(const point3<fp>& p1, const point3<fp>& p2, const point3<fp>& p3): low_level::three_points_holder<point3<fp>>{p1, p2, p3} {}
        inline plane3<fp> plane() const;
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
        inline direction3<fp> normal() const;
        inline point3<fp> barycenter() const;

        template <typename TSetVoxel> 
        SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        void triangle_voxelize(TSetVoxel& SET_PIXEL) const;

        inline bool intersects(const line_segment3<fp>& o) const
        {
            return low_level::tri3_impl<fp>::intersects_line_segment(this->p1(), this->p2(), this->p3(), o.p1(), o.p2());
        }

        inline bool intersects(const plane3<fp>& o) const
        {
            // max normal direction' projection, 3D to 2D
            line3<fp> n3(o.point, o.point.vec() + o.normal.vec() * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f));
            projection3to2noscale proj32 (o.orthogonal_plane(n3));

            point2 d0 (proj32.transform(o.point));
            point2 d1 (proj32.transform(this->p1()));
            point2 d2 (proj32.transform(this->p2()));
            point2 d3 (proj32.transform(this->p3()));

            // project triangle points to 1D, along plane' normal
            line_segment2<fp> n2 (proj32.transform(o.point), proj32.transform(o.point.vec() + o.normal.vec() * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f)));
            projection2to1noscale proj21(n2);
            point1<fp> t0 (proj21.transform(d0));
            point1<fp> t1 (proj21.transform(d1));
            point1<fp> t2 (proj21.transform(d2));
            point1<fp> t3 (proj21.transform(d3));

            int s1 = low_level::mathf::sign(t0.x - t1.x);
            int s2 = low_level::mathf::sign(t0.x - t2.x);
            int s3 = low_level::mathf::sign(t0.x - t3.x);

            // check if all triangle points lie on the same side of the plane
            if (s1 != s2 || s2 != s3)
                return true;

            return false;
        }

        template<typename T3>
        inline bool intersects(const T3& o) const
        {
            return low_level::tri3_impl<fp>::intersects(this->p1(), this->p2(), this->p3(), o.p1(), o.p2(), o.p3());
        }

        template<typename T3>
        inline std::optional<line_segment3<fp>> intersection(const T3& o) const
        {
            auto intersection = low_level::tri3_impl<fp>::intersection(this->p1(), this->p2(), this->p3(), o.p1(), o.p2(), o.p3());
            if (intersection.has_value())
                return line_segment3(intersection.value()[0], intersection.value()[1]);
            return std::nullopt;
        }
        // inline std::optional<line_segment3> intersection(const triangle3& other) const;

        std::vector<point2<fp>> intersection_points(const point3<fp>& p0, const point3<fp>& p1, const point3<fp>& p2, projection3to2noscale<fp> proj, line2<fp> inter_line) const
        {
            std::vector<point2<fp>> inter_points;
            line_segment2<fp> edges[3] = {
                line_segment2<fp> ( proj.transform(p0), proj.transform(p1) ),
                line_segment2<fp>( proj.transform(p1), proj.transform(p2) ),
                line_segment2<fp>( proj.transform(p2), proj.transform(p0) )
            };

            for (const auto& edge : edges)
            {
                std::optional<point2<fp>> inter_point = inter_line.strict_intersection(edge);
                if (inter_point)
                    inter_points.push_back(*inter_point);
            }

            return inter_points;
        }
        template<typename T3>
        inline bool intersects_3rd(const T3& o) const // temporal 3rd impl. of triangles intersection
        {
            projection3to2noscale<fp> proj32_ta (plane());
            std::optional<line3<fp>> intersection = plane().intersection(o.plane());
            if (!intersection)
                return false;

            line3<fp> inter_line = *intersection;
            line2<fp> inter_line_2d = proj32_ta.transform(inter_line);

            std::vector<point2<fp>> inter_points_a = intersection_points(this->p1(), this->p2(), this->p3(), proj32_ta, inter_line_2d);
            if (inter_points_a.size() < 2) return false;
            std::vector<point2<fp>> inter_points_b = intersection_points(o.p1(), o.p2(), o.p3(), proj32_ta, inter_line_2d);
            if (inter_points_b.size() < 2) return false;

            // project segments to 1D
            projection2to1noscale proj21 ( line_segment2<fp>(inter_points_a[0], inter_points_a[1]) );
            line_segment1 s1a ( proj21.transform( inter_points_a[0] ), proj21.transform( inter_points_a[1] ) );
            line_segment1 s1b ( proj21.transform( inter_points_b[0] ), proj21.transform( inter_points_b[1] ) );

            return s1a.overlays(s1b);
        }

        inline bounds3<fp> bounds() const;

        inline plane3<fp> orthogonal_plane(int edge_number) const;

        inline size_t n_vertices() const
        {
            return 3;
        }
        inline point3<fp> vertex(int idx) const;

        inline size_t n_edges() const
        {
            return 3;
        }
        inline line_segment3<fp> edge(int idx) const;

        inline size_t n_faces() const
        {
            return 1;
        }
        inline triangle3 face(int idx) const;

        inline ray3<fp> bisector(int idx) const;

        inline triple<line_segment3<fp>> edges() const;
        inline triple<point3<fp>> vertices() const;

        inline bool operator ==(const triangle3& other) const
        {
            return vertex(0) == other.vertex(0) && vertex(1) == other.vertex(1) && vertex(2) == other.vertex(2);
        }
    };

    template <typename fp>
    struct triangle2: low_level::three_points_holder<point2<fp>>
    {
        using counterpart_3d = triangle3<fp>;
        friend struct polygon2<fp>;
        friend struct barycentric<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::triangle2<fp>>;

      public:
        inline triangle2(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p3): low_level::three_points_holder<point2<fp>>{p1, p2, p3} {}
        inline bounds2<fp> bounds() const;
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
        inline int normal() const;
        inline point2<fp> barycenter() const;
        inline bool is_inside(const point2<fp>& p) const;

        inline bool is_inside_combinable(const point2<fp>& p) const;

        inline size_t n_vertices() const
        {
            return 3;
        }
        inline point2<fp> vertex(int idx) const;
        inline size_t n_edges() const
        {
            return 3;
        }
        inline line_segment2<fp> edge(int idx) const;
        inline size_t n_faces() const
        {
            return 1;
        }
        inline triangle2<fp> face(int idx) const;

        inline triple<line_segment2<fp>> edges() const;
        inline triple<point2<fp>> vertices() const;

        inline bool operator ==(const triangle2<fp>& other) const
        {
            return vertex(0) == other.vertex(0) && vertex(1) == other.vertex(1) && vertex(2) == other.vertex(2);
        }
    };

}; // namespace geometry

namespace geometry 
{
    // aliases 
    using line_segment3f = line_segment3<float>;
    using line_segment2f = line_segment2<float>;
    using line_segment1f = line_segment1<float>;
    using triangle3f = triangle3<float>;
    using triangle2f = triangle2<float>;
}


}; // namespace sixit

#endif //sixit_geometry_line_segment_h_included

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
