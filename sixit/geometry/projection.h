/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_projection_h_included
#define sixit_geometry_projection_h_included

#include "sixit/geometry/point.h"
#include "sixit/geometry/plane.h"
#include "sixit/geometry/points_span.h"
#include "sixit/geometry/transforms.h"
#include "sixit/core/units.h"

#include <vector>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    inline point3<fp> _orthogonal_projection_helper(const point3<fp>& p, const plane3<fp>& target_plane)
    {
        auto distance_to_plane = target_plane.distance(p);
        low_level::dimensional_vector3 projection_vector = target_plane.normal.vec() * distance_to_plane;
        low_level::dimensional_vector3 projected_point_vector = p.vec() - projection_vector;

        return point3<fp>(projected_point_vector);
    }

    template <typename T, typename fp>
    inline T orthogonal_projection3(const T& o, const plane3<fp>& target_plane)
    {
        using vlo = std::vector<point3<fp>>;
        vlo points_span = o.template _get_projection_points<vlo>();
        for (size_t i = 0; i < points_span.size(); ++i)
        {
            points_span[i] = _orthogonal_projection_helper<fp>(points_span[i], target_plane);
        }
        return T(points_span);
    }

    template <typename fp>
    struct projection3 // orthogonal
    {
        friend struct projection3to2<fp>;
        friend struct projection3to2ex<fp>;
        friend struct projection3to2noscale<fp>;

      private:
        plane3<fp> plane;

      public:
        inline projection3(const plane3<fp> &plane) : plane(plane) {}
        inline projection3() : plane(direction3<fp>(
            sixit::units::create_dimensionless_scalar<fp>(0.f),
            sixit::units::create_dimensionless_scalar<fp>(0.f),
            sixit::units::create_dimensionless_scalar<fp>(1.f)), point3<fp>()) {}

        inline point3<fp> project(const point3<fp> &p) const
        {
            const auto distance_to_plane = plane.distance(p);
            const fp parity = plane.side(p) == true ? 1.0f : -1.0f;
            low_level::dimensional_vector3 projection_vector = plane.normal.vec() * distance_to_plane * 
            sixit::units::create_dimensionless_scalar<fp>(parity);
            low_level::dimensional_vector3 projected_point_vector = p.vec() + projection_vector;

            return point3(projected_point_vector);
        }

        inline affine_transform3<fp> affine_transform() const
        {
            low_level::dimensional_vector3 n = plane.normal.vec();

            low_level::matrix4x4<fp> projection_matrix(
                fp(1.f) - n.x() * n.x(), -n.x() * n.y(), -n.x() * n.z(), fp(0.f),
                -n.y() * n.x(), fp(1.f) - n.y() * n.y(), -n.y() * n.z(), fp(0.f),
                -n.z() * n.x(), -n.z() * n.y(), fp(1.f) - n.z() * n.z(), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(1.f));

            return geometry::affine_transform3(projection_matrix);
        }

        template <typename T>
        inline T project(const T& o) const
        {
            using vlo = std::vector<point3<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = project(points_span[i]);
            }
            return T(points_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection3">(comparser, obj);
            sixit::rw::read_write<&projection3::plane, "plane", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }

    private:
        inline low_level::matrix4x4<fp> matrix4x4() const
        {
            low_level::dimensional_vector3 up(
                sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                sixit::units::create_dimensionless_scalar<fp>(0.0f));

            low_level::dimensional_vector3 right(
                sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                sixit::units::create_dimensionless_scalar<fp>(0.0f));
            low_level::dimensional_vector3 view_dir = plane.normal.vec().normalized();
            if (view_dir == up)
            {
                up = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(view_dir, right);
                up.normalize();
            }
            else
            {
                right = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(up, view_dir);
                right.normalize();

                up = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(view_dir, right);
                up.normalize();
            }

            low_level::matrix4x4 m = low_level::matrix4x4<fp>::identity();
            m[0][0] = right.x();
            m[0][1] = right.y();
            m[0][2] = right.z();
            m[1][0] = up.x();
            m[1][1] = up.y();
            m[1][2] = up.z();
            m[2][0] = -view_dir.x();
            m[2][1] = -view_dir.y();
            m[2][2] = -view_dir.z();
            return m;
        }
    };

    template <typename fp>
    struct projection2 // orthogonal
    {
    friend struct projection2to1<fp>;

      using class_value = typename point2<fp>::class_value;

      private:
        line2<fp> line;
        low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> normal;

      public:


        inline projection2(const line2<fp> &line) : line(line) 
        {
            low_level::dimensional_vector2 dir = line.p2().vec() - line.p1().vec();
            normal = low_level::dimensional_vector2(-dir.y, dir.x).normalized();
        }
        inline projection2() : line(point2<fp>(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f),
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f)), point2<fp>()) {}

        inline point2<fp> project(const point2<fp> &p) const
        {
            low_level::dimensional_vector2 vector_to_point = p.vec() - line.p1().vec();

            auto distance = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(vector_to_point, normal);
            low_level::dimensional_vector2 direction = normal * distance;
            low_level::dimensional_vector2 projected_vector = p.vec() - direction;
            return point2(projected_vector);
        }

        inline point2<fp> mat_mul(fp mat[], point2<fp> p) const
        {
            return point2(p.vec().x * sixit::units::create_dimensionless_scalar<fp>(mat[0]) + 
            p.vec().y * sixit::units::create_dimensionless_scalar<fp>(mat[1]), 
            p.vec().x * sixit::units::create_dimensionless_scalar<fp>(mat[2]) + 
            p.vec().y * sixit::units::create_dimensionless_scalar<fp>(mat[3])); 
        }
        
        inline affine_transform2<fp> affine_transform() const
        {
            low_level::dimensional_vector2 a = line.p1().vec();

            low_level::matrix3x3 mat(
                sixit::units::extract_dim_less_scalar(normal.y), 
                sixit::units::extract_dim_less_scalar(-normal.x), fp(0.0f), 
                sixit::units::extract_dim_less_scalar(normal.x), 
                sixit::units::extract_dim_less_scalar(normal.y), fp(0.0f), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(a.x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(a.y), fp(1.0f));

            low_level::matrix3x3 drop_y(fp(1.0f), fp(0.0f), fp(0.0f), fp(0.0f), fp(0.0f), fp(0.0f), fp(0.0f), fp(0.0f), fp(1.0f));
            low_level::matrix3x3 inv = mat.inverse();

            low_level::matrix3x3 aff = (inv * drop_y) * mat;
            return geometry::affine_transform2(aff);
        }

        template <typename T>
        inline T project(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = project(points_span[i]);
            }
            return T(points_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection2">(comparser, obj);
            sixit::rw::read_write<&projection2::line, "line", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection2::normal, "normal", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct projection3to2
    {
      private:
        plane3<fp> plane;
        direction3<fp> axis_x;
        direction3<fp> axis_y;
        point2<fp> offset;

      public:
        projection3to2() : plane(direction3<fp>(
            sixit::units::create_dimensionless_scalar<fp>(0.f), 
            sixit::units::create_dimensionless_scalar<fp>(0.f), 
            sixit::units::create_dimensionless_scalar<fp>(1.f)), point3<fp>())
        {
            axis_x = direction3<fp>(
                            sixit::units::create_dimensionless_scalar<fp>(1.f),
                            sixit::units::create_dimensionless_scalar<fp>(0.f),
                            sixit::units::create_dimensionless_scalar<fp>(0.f));
            axis_y = direction3<fp>(
                            sixit::units::create_dimensionless_scalar<fp>(0.f),
                            sixit::units::create_dimensionless_scalar<fp>(1.f),
                            sixit::units::create_dimensionless_scalar<fp>(0.f));
            offset = point2<fp>();
        }

        inline projection3to2(const plane3<fp>& pl) : plane(pl)
        {
            const auto& normal = plane.normal.vec();
            fp cx = abs(normal.x());
            fp cy = abs(normal.y());
            fp cz = abs(normal.z());
            direction3<fp> axis0;
            if (cx <= cy && cx <= cz)
                axis0 = direction3<fp>(
                    sixit::units::create_dimensionless_scalar<fp>(1.f),
                    sixit::units::create_dimensionless_scalar<fp>(0.f),
                    sixit::units::create_dimensionless_scalar<fp>(0.f));
            else if (cy <= cx && cy <= cz)
                axis0 = direction3<fp>(
                    sixit::units::create_dimensionless_scalar<fp>(0.f),
                    sixit::units::create_dimensionless_scalar<fp>(1.f),
                    sixit::units::create_dimensionless_scalar<fp>(0.f));
            else
                axis0 = direction3<fp>(
                    sixit::units::create_dimensionless_scalar<fp>(0.f),
                    sixit::units::create_dimensionless_scalar<fp>(0.f),
                    sixit::units::create_dimensionless_scalar<fp>(1.f));
            axis_y = geometry::low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(plane.normal.vec(), axis0.vec());
            axis_x = geometry::low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(axis_y.vec(), plane.normal.vec());
            offset = project(plane.point);
        }

        inline point2<fp> project(const point3<fp>& p) const
        {
            // projection on 2D plane
            fp proj_x = geometry::low_level::vector3<fp>::dot(p.vec(), axis_x.vec());
            fp proj_y = geometry::low_level::vector3<fp>::dot(p.vec(), axis_y.vec());
            return point2(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(proj_x), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(proj_y));
        }

        inline point3<fp> resurrect(const point2<fp>& p) const
        {
            low_level::dimensional_vector2 p2 = p.vec() - offset.vec();
            return point3<fp>(plane.point.vec() + 
            axis_x.vec() * p2.x + axis_y.vec() * p2.y);
        }

        inline affine_transform3<fp> affine_transform() const
        {
            // create matrix from X,Y axes
            low_level::matrix4x4 m(
            axis_x.vec().x(), axis_x.vec().y(), axis_x.vec().z(), fp(0.f),
            axis_y.vec().x(), axis_y.vec().y(), axis_y.vec().z(), fp(0.f),
            fp(0.f),          fp(0.f),          fp(0.f),          fp(0.f),
            fp(0.f),          fp(0.f),          fp(0.f),          fp(1.f)
            );
            return geometry::affine_transform3(m);
        }

        // class T should have a defined projectionT to use this template
        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo2 = std::vector<point2<fp>>;
            using vlo3 = std::vector<point3<fp>>;

            vlo3 points3_span = o.template _get_projection_points<vlo3>();
            vlo2 points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                fp proj_x = geometry::low_level::vector3<fp>::dot(points3_span[i].vec(), axis_x.vec());
                fp proj_y = geometry::low_level::vector3<fp>::dot(points3_span[i].vec(), axis_y.vec());
                points2_span.push_back(
                    point2(
                        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(proj_x), 
                        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(proj_y)));
            }

            return typename T::projectionT(points2_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection3to2">(comparser, obj);
            sixit::rw::read_write<&projection3to2::plane, "plane", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection3to2::axis_x, "axis_x", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection3to2::axis_y, "axis_y", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection3to2::offset, "offset", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct projection2to1
    {
      private:
        line2<fp> line;
        int coord = 0;         // index of the component to keep
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> scale = sixit::units::create_dimensionless_scalar<fp>(1.f);

      public:
        projection2to1() : line(
            point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f)), 
            point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f))) {}
        inline projection2to1(const line2<fp>& ln) : line(ln) 
        {
            // Dropping the most orthogonal coordinate for a "convenient" coordinate system
            low_level::dimensional_vector2 line_dir = (ln.p2().vec() - ln.p1().vec()).normalized();
            auto cx = sixit::geometry::low_level::mathf::abs(line_dir.x);    // not orthogonal values
            auto cy = sixit::geometry::low_level::mathf::abs(line_dir.y);    // not orthogonal values
            coord = cx > cy ? 0 : 1;
            scale += cx > cy ? cy : cx; // or (1-cx) | (1-cy) ?
        }

        inline point1<fp> project(const point2<fp>& p) const
        {
            return point1<fp>(p.vec()[coord] * scale);
        }

        inline affine_transform2<fp> affine_transform() const
        {
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x = coord == 0 ? scale : sixit::units::create_dimensionless_scalar<fp>(0.f);
            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y = coord == 1 ? scale : sixit::units::create_dimensionless_scalar<fp>(0.f);

            return geometry::affine_transform2(low_level::matrix3x3(
                sixit::units::extract_dim_less_scalar(x), fp(0.f), fp(0.f),
                fp(0.f), sixit::units::extract_dim_less_scalar(y), fp(0.f),
                fp(0.f), fp(0.f), fp(1.f)));
        }

        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo1 = std::vector<point1<fp>>;
            using vlo2 = std::vector<point2<fp>>;

            vlo2 points2_span = o.template _get_projection_points<vlo2>();
            vlo1 points1_span;
            points1_span.reserve(points2_span.size());

            for (size_t i = 0; i < points2_span.size(); ++i)
            {
                points1_span.push_back(project(points2_span[i]));
            }

            return typename T::projectionT(points1_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection2to1">(comparser, obj);
            sixit::rw::read_write<&projection2to1::line, "line", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection2to1::coord, "coord", sixit::rw::I32>(comparser);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "scale", obj.scale);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct projection3to2ex
    {
      private:
        low_level::matrix4x4<fp> transformation_matrix;    
        int map[2] = { 0, 0};         // indices of the components to keep

      public:
        inline projection3to2ex(){}
        inline projection3to2ex(const plane3<fp>& target_plane, const direction3<fp>& custom_x_axis)
        {
            const auto& normal = target_plane.normal.vec();
            fp cx = sixit::geometry::low_level::mathf::abs(normal.x());
            fp cy = sixit::geometry::low_level::mathf::abs(normal.y());
            fp cz = sixit::geometry::low_level::mathf::abs(normal.z());
            low_level::vector3 val = low_level::vector3<fp>::one();

            unsigned int c3;

            // Dropping the most orthogonal coordinate for a "convenient" coordinate system
            if (cx >= cy && cx >= cz) // drop x
            {
                val.x() = 0;
                c3 = 0;
                map[0] = 1;
                map[1] = 2;
            }
            else if (cy >= cx && cy >= cz) // drop y
            {
                val.y() = 0;
                c3 = 1;
                map[0] = 0;
                map[1] = 2;
            }
            else // drop z
            {
                val.z() = 0;
                c3 = 2;
                map[0] = 0;
                map[1] = 1;
            }

            low_level::matrix4x4 mat(
                val.x(), fp(0.f), fp(0.f), fp(0.f),
                fp(0.f), val.y(), fp(0.f), fp(0.f),
                fp(0.f), fp(0.f), val.z(), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(1.f));

            low_level::matrix4x4 rot_matrix = low_level::matrix4x4<fp>::identity();
            auto axis_y = low_level::vector3<fp>::cross(normal, custom_x_axis.vec());
            axis_y.normalize();
            auto axis_x = low_level::vector3<fp>::cross(axis_y, normal);
            axis_x.normalize();

            if (!sixit::dmath::fp_traits<fp>::isfinite(axis_x[0]) || !sixit::dmath::fp_traits<fp>::isfinite(axis_x[1]) || 
                !sixit::dmath::fp_traits<fp>::isfinite(axis_x[2]) || !sixit::dmath::fp_traits<fp>::isfinite(axis_x[3]) ||
                !sixit::dmath::fp_traits<fp>::isfinite(axis_y[0]) || !sixit::dmath::fp_traits<fp>::isfinite(axis_y[1]) || 
                !sixit::dmath::fp_traits<fp>::isfinite(axis_y[2]) || !sixit::dmath::fp_traits<fp>::isfinite(axis_y[3]))
            {
                transformation_matrix = mat;
                return;
            }

            rot_matrix[map[0]] = axis_x;
            rot_matrix[map[1]] = axis_y;
            rot_matrix[c3] = normal;

            transformation_matrix = mat * rot_matrix.inverse();
        }

        inline point2<fp> project(const point3<fp>& p) const
        {
            low_level::vector3<fp> tmp_vec(p.vec().x(), p.vec().y(), p.vec().z());
            tmp_vec = transformation_matrix.multiply_point3x4(tmp_vec);

            return point2(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp_vec[map[0]]), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp_vec[map[1]]));
        }

        inline affine_transform3<fp> affine_transform() const
        {
            return geometry::affine_transform3(transformation_matrix);
        }

        // class T should have a defined projectionT to use this template
        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo2 = std::vector<point2<fp>>;
            using vlo3 = std::vector<point3<fp>>;

            vlo3 points3_span = o.template _get_projection_points<vlo3>();
            vlo2 points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                points2_span.push_back(project(points3_span[i]));
            }

            return typename T::projectionT(points2_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection3to2ex">(comparser, obj);
            sixit::rw::read_write<&projection3to2ex::transformation_matrix, "matrix", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "map0", obj.map[0]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "map1", obj.map[1]);
            sixit::rw::end_struct(comparser);
        }
    };
    
    template <typename fp>
    struct projection2to1ex
    {
      private:
        low_level::matrix3x3<fp> transformation_matrix;
        int coord = 0; // index of the component to keep

      public:
        projection2to1ex() {}
        inline projection2to1ex(const line2<fp>& target_line, const direction2<fp>& x_dir)
        {
            low_level::dimensional_vector2 line_dir = (target_line.p2().vec() - target_line.p1().vec()).normalized();
            auto cx = geometry::low_level::mathf::abs(line_dir.x);
            auto cy = geometry::low_level::mathf::abs(line_dir.y);
            fp mat_x;
            fp mat_y;

            if (cx >= cy)
            {
                auto dir2 = direction2<fp>(
                    sixit::units::create_dimensionless_scalar<fp>(1.f), 
                    sixit::units::create_dimensionless_scalar<fp>(0.f));

                mat_x = low_level::mathf::sign(low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::dot(dir2.vec(), x_dir.vec()));
                mat_y = 0;
                coord = 0;
            }
            else
            {
                auto dir2 = direction2<fp>(
                    sixit::units::create_dimensionless_scalar<fp>(0.f), 
                    sixit::units::create_dimensionless_scalar<fp>(1.f));

                mat_x = 0;
                mat_y = low_level::mathf::sign(low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::dot(dir2.vec(), x_dir.vec()));
                coord = 1;
            }
            
            transformation_matrix = low_level::matrix3x3<fp>(
                mat_x,   fp(0.f), fp(0.f),
                fp(0.f), mat_y,   fp(0.f),
                fp(0.f), fp(0.f), fp(1.f));
        }

        inline point1<fp> project(const point2<fp>& p) const
        {
            sixit::geometry::low_level::vector2<fp> tmp = {
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().x),
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().y)};

            tmp = transformation_matrix.multiply_point2x3(tmp);

            return point1<fp>(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp[coord]));
        }

        inline affine_transform2<fp> affine_transform() const
        {
            return geometry::affine_transform2(transformation_matrix);
        }

        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo1 = std::vector<point1<fp>>;
            using vlo2 = std::vector<point2<fp>>;

            vlo2 points2_span = o.template _get_projection_points<vlo2>();
            vlo1 points1_span;
            points1_span.reserve(points2_span.size());

            for (size_t i = 0; i < points2_span.size(); ++i)
            {
                points1_span.push_back(project(points2_span[i]));
            }

            return typename T::projectionT(points1_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection2to1ex">(comparser, obj);
            sixit::rw::read_write<&projection2to1ex::transformation_matrix, "matrix", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&projection2to1ex::coord, "coord", sixit::rw::F32>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };    

    template <typename fp>
    struct axis_x_projection
    {
      public:
        inline point2<fp> project(const point3<fp>& p) const
        {
            return point2(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().y()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().z()));
        }

        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            std::vector<point3<fp>> points3_span = o.template _get_projection_points<std::vector<point3<fp>>>();

            vlo points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                points2_span.push_back(project(points3_span[i]));
            }

            return typename T::projectionT(points2_span);
        }

        inline affine_transform3<fp> affine_transform() const
        {
            return geometry::affine_transform3(low_level::matrix4x4(
                fp(0.f), fp(0.f), fp(0.f), fp(0.f), // drop coordinate x,  
                fp(0.f), fp(1.f), fp(0.f), fp(0.f),
                fp(0.f), fp(0.f), fp(1.f), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(1.f)));
        }
    };

    template <typename fp>
    struct axis_y_projection
    {
      public:
        inline point2<fp> project(const point3<fp>& p) const
        {
            return point2(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().z()));
        }

        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            std::vector<point3<fp>> points3_span = o.template _get_projection_points<std::vector<point3<fp>>>();

            vlo points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                points2_span.push_back(project(points3_span[i]));
            }

            return typename T::projectionT(points2_span);
        }

        inline affine_transform3<fp> affine_transform() const
        {
            return geometry::affine_transform3(low_level::matrix4x4(
                fp(1.f), fp(0.f), fp(0.f), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(0.f),// drop coordinate y
                fp(0.f), fp(0.f), fp(1.f), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(1.f)));
        }
    };

    template <typename fp>
    struct axis_z_projection
    {
      public:
        inline point2<fp> project(const point3<fp>& p) const
        {
            return point2(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().y()));
        }

        template <typename T>
        inline typename T::projectionT project(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            std::vector<point3<fp>> points3_span = o.template _get_projection_points<std::vector<point3<fp>>>();

            vlo points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                points2_span.push_back(project(points3_span[i]));
            }

            return typename T::projectionT(points2_span);
        }

        inline affine_transform3<fp> affine_transform() const
        {
            return geometry::affine_transform3(low_level::matrix4x4(
                fp(1.f), fp(0.f), fp(0.f), fp(0.f),
                fp(0.f), fp(1.f), fp(0.f), fp(0.f),
                fp(0.f), fp(0.f), fp(0.f), fp(0.f), // drop coordinate z
                fp(0.f), fp(0.f), fp(0.f), fp(1.f)));
        }
    };

}; // namespace geometry

namespace geometry 
{
    // aliases 
    using projection3f = projection3<float>;
    using projection2f = projection2<float>;
    using projection3to2f = projection3to2<float>;
    using projection2to1f = projection2to1<float>;
    using projection3to2exf = projection3to2ex<float>;
    using projection2to1exf = projection2to1ex<float>;
    using axis_x_projectionf = axis_x_projection<float>;
    using axis_y_projectionf = axis_y_projection<float>;
    using axis_z_projectionf = axis_z_projection<float>;
}

}; // namespace sixit

#endif //sixit_geometry_projection_h_included

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
