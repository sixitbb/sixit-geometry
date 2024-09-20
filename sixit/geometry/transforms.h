/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_transforms_h_included
#define sixit_geometry_transforms_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/point.h"

#include <tuple>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct affine_transform3
    {
        friend struct affine_transform3to2<fp>;
        friend struct mirror_transform3<fp>;
        friend struct projection3<fp>;
        friend struct projection3to2<fp>;
        friend struct projection3to2ex<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct axis_x_projection<fp>;
        friend struct axis_y_projection<fp>;
        friend struct axis_z_projection<fp>;

      private:
        low_level::matrix4x4<fp> mat_;

        inline explicit affine_transform3(const low_level::matrix4x4<fp>& mat) : mat_(mat) {}

      public:
        inline affine_transform3() : mat_(fp(0.f)) {}

        inline point3<fp> transform(const point3<fp>& p) const
        {
            low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> vv(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().y()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().z()), 
                typename low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::position_type_t());  //tmp solution for  point3 missing the 4th coordinate as 1

            low_level::vector3<fp> r = mat_.multiply_point3x4(vv);
            return point3<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.y()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.z()));
        }

        inline affine_transform3 transform(const affine_transform3& other) const
        {
            low_level::matrix4x4 result_mat = mat_ * other.mat_;
            return affine_transform3(result_mat);
        }

        template <typename T>
        inline T transform(const T& o) const
        {
            using vlo = std::vector<point3<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = transform(points_span[i]);
            }
            return T(points_span);
        }

        inline affine_transform3 inverse()
        {
            low_level::matrix4x4 inverted_mat = mat_.inverse();
            return affine_transform3(inverted_mat);
        }

        inline static affine_transform3 translation(const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& offset)
        {
            low_level::matrix4x4 translation_matrix = low_level::matrix4x4<fp>::translate(offset);
            return affine_transform3(translation_matrix);
        }

        inline static affine_transform3 rotation(const low_level::quaternion<fp>& rotation)
        {
            low_level::matrix4x4 rotation_matrix = low_level::matrix4x4<fp>::rotate(rotation);
            return affine_transform3(rotation_matrix);
        }

        inline static affine_transform3 rotation(const rotation3<fp>& rotation)
        {
            low_level::matrix4x4 rotation_matrix = low_level::matrix4x4<fp>::rotate(rotation.q);
            return affine_transform3(rotation_matrix);
        }

        inline static affine_transform3 scale(const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& factors)
        {
            low_level::matrix4x4 scale_matrix = low_level::matrix4x4<fp>::scale(factors);
            return affine_transform3(scale_matrix);
        }

        inline static affine_transform3 translation(const point3<fp>& offset)  //temporary for test
        {   return translation(offset.vec());}

        inline static affine_transform3 scale(const point3<fp>& factors)  //temporary for test
        {   return scale(factors.vec());}

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"affine_transform3">(comparser, obj);
            sixit::rw::read_write<&affine_transform3::mat_, "matrix", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct affine_transform2
    {
        friend struct affine_transform2to1<fp>;
        friend struct projection3to2<fp>;
        friend struct projection2<fp>;
        friend struct projection2to1<fp>;
        friend struct projection2to1ex<fp>;
        friend struct mirror_transform2<fp>;
        friend struct axis_x_projection<fp>;
        friend struct axis_y_projection<fp>;
        friend struct axis_z_projection<fp>;

      private:
        inline explicit affine_transform2(const low_level::matrix3x3<fp>& mat) : mat_(mat) {}

        low_level::matrix3x3<fp> mat_;

      public:
        inline affine_transform2() : mat_(low_level::matrix3x3<fp>::identity()) {}  

        inline point2<fp> transform(const point2<fp>& p) const
        {
            // auto m = mat_.stripes;
            // return point2( m[0][0] * p.vec().x + m[0][1] * p.vec().y + m[0][2],
                           // m[1][0] * p.vec().x + m[1][1] * p.vec().y + m[1][2] );
            sixit::geometry::low_level::vector2<fp> tmp(
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().x),
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().y));
            tmp = mat_.multiply_point2x3(tmp);

            return point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.x), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.y));
        }

        inline affine_transform2 transform(const affine_transform2& other) const
        {
            low_level::matrix3x3 result_mat = mat_ * other.mat_;
            return affine_transform2(result_mat);
        }

        template <typename T>
        inline T transform(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = transform(points_span[i]);
            }
            return T(points_span);
        }

        inline affine_transform2 inverse()
        {
            low_level::matrix3x3<fp> inverted_mat = mat_.inverse();
            return affine_transform2(inverted_mat);
        }

        inline static affine_transform2 translation(const low_level::vector2<fp>& offset)
        {
            low_level::matrix3x3<fp> translation_matrix = low_level::matrix3x3<fp>::translate(offset);
            return affine_transform2(translation_matrix);
        }

        inline static affine_transform2<fp> rotation(const low_level::quaternion<fp>& rotation)
        {
            low_level::matrix3x3<fp> rotation_matrix = low_level::matrix3x3<fp>::rotate(rotation);
            return affine_transform2(rotation_matrix);
        }

        inline static affine_transform2<fp> rotation(const rotation3<fp>& rotation)
        {
            low_level::matrix3x3<fp> rotation_matrix = low_level::matrix3x3<fp>::rotate(rotation.q);
            return affine_transform2(rotation_matrix);
        }

        inline static affine_transform2 scale(const low_level::vector2<fp>& factors)
        {
            low_level::matrix3x3<fp> scale_matrix = low_level::matrix3x3<fp>::scale(factors);
            return affine_transform2(scale_matrix);
        }


        inline static affine_transform2 scale(const point2<fp>& factors)  //temporary for test
        {   
            sixit::geometry::low_level::vector2<fp> tmp{
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(factors.vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(factors.vec().y)};
            return scale(tmp);    }

        inline static affine_transform2 translation(const point2<fp>& offset)  //temporary for test
        {   
            sixit::geometry::low_level::vector2<fp> tmp{
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(offset.vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(offset.vec().y)};
            return translation(tmp); 
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"affine_transform2">(comparser, obj);
        sixit::rw::read_write<&affine_transform2::mat_, "matrix", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct mirror_transform3
    {
      private:
        plane3<fp> mirror_plane;

      public:
        inline mirror_transform3() : mirror_plane(plane3(
            direction3<fp>(
                sixit::units::create_dimensionless_scalar<fp>(0.f),
                sixit::units::create_dimensionless_scalar<fp>(0.f),
                sixit::units::create_dimensionless_scalar<fp>(1.f)), 
            point3<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f)))) {}
        inline mirror_transform3(const plane3<fp>& plane) : mirror_plane(plane) {}

        inline point3<fp> transform(const point3<fp>& p) const
        {
            low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> vv(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().y()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec().z()), 
                typename low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::position_type_t());  //tmp solution for  point3 missing the 4th coordinate as 1

            auto distance = mirror_plane.distance(vv);
            auto scalar_multiplier = sixit::units::create_dimensionless_scalar<fp>(2.f) * distance;

            low_level::dimensional_vector3 mirrored_vec = vv - (mirror_plane.normal.vec() * scalar_multiplier);
            return point3<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(mirrored_vec.x()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(mirrored_vec.y()), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(mirrored_vec.z()));
        }

        inline affine_transform3<fp> to_affine_transform3() const
        {
            low_level::matrix4x4 m = low_level::matrix4x4<fp>::identity();
            auto normal = mirror_plane.normal.vec();

            // construct the reflected matrix
            for (int i = 0; i < 3; ++i)  
                for (int j = 0; j < 3; ++j) 
                    m[i][j] = -2.0f * normal[i] * normal[j];

            for (int i = 0; i < 3; ++i)
            {
                m[i][3] = mirror_plane.point.vec()[i];
                m[3][i] = 0.0f;
            }

            m[3][3] = 1.0f;
            return affine_transform3(m);
        }

        template <typename T>
        inline T transform(const T& o) const
        {
            using vlo = std::vector<point3<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = transform(points_span[i]);
            }
            return T(points_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"mirror_transform3">(comparser, obj);
            sixit::rw::read_write<&mirror_transform3::mirror_plane, "plane", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct affine_transform3to2
    {
      private:
        affine_transform3<fp> transform_;

        inline explicit affine_transform3to2(const low_level::matrix4x4<fp>& mat) : transform_(mat) {}

      public:
        inline affine_transform3to2() : transform_(affine_transform3<fp>()) {}
        inline explicit affine_transform3to2(const affine_transform3<fp>& transform3) : transform_(transform3.mat_) {}

        inline point2<fp> transform(const point3<fp>& p) const
        {
            point3 transformed_point = transform_.transform(p);

            fp x_abs = sixit::geometry::low_level::mathf::abs(transformed_point.vec().x());
            fp y_abs = sixit::geometry::low_level::mathf::abs(transformed_point.vec().y());
            fp z_abs = sixit::geometry::low_level::mathf::abs(transformed_point.vec().z());

            // Dropping the axis with the smallest absolute value.
            if (x_abs <= y_abs && x_abs <= z_abs)
                return point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().y()), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().z()));
            else if (y_abs <= x_abs && y_abs <= z_abs)
                return point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().x()), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().z()));
            else
                return point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().x()), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(transformed_point.vec().y()));
        }

        inline affine_transform3<fp> to_affine_transform3() const
        {
            return transform_;
        }

        template <typename T>
        inline typename T::projectionT transform(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            std::vector<point3<fp>> points3_span = o.template _get_projection_points<std::vector<point3<fp>>>();

            vlo points2_span;
            points2_span.reserve(points3_span.size());

            for (size_t i = 0; i < points3_span.size(); ++i)
            {
                points2_span.push_back(transform(points3_span[i]));
            }

            return typename T::projectionT(points2_span);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"affine_transform3to2">(comparser, obj);
            sixit::rw::read_write<&affine_transform3to2::transform_, "transform", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct affine_transform2to1
    {
    private:

        inline explicit affine_transform2to1(const low_level::matrix3x3<fp>& mat) : transform_(mat) {}
        affine_transform2<fp> transform_;

    public:

        inline explicit affine_transform2to1(const affine_transform2<fp>& transform2) : transform_(transform2.mat_) {}    //test

        inline point1<fp> transform(const point2<fp>& p) const
        {
            point2<fp> transformed_point = transform_.transform(p);

            if (geometry::low_level::mathf::abs(transformed_point.vec().x) > geometry::low_level::mathf::abs(transformed_point.vec().y))
                return point1<fp>(transformed_point.vec().x);
            else
                return point1<fp>(transformed_point.vec().y);
        }

        inline affine_transform2<fp> to_affine_transform2()
        {
            return transform_;
        }

        template <typename T>
        inline typename T::projectionT transform(const T& o) const
        {
            using vlo = std::vector<point1<fp>>;
            std::vector<point2<fp>> points2_span = o.template _get_projection_points<std::vector<point2<fp>>>();
            
            vlo points1_span;
            points1_span.reserve(points2_span.size());
            
            for (size_t i = 0; i < points2_span.size(); ++i)
            {
                points1_span.push_back(transform(points2_span[i]));
            }

            return typename T::projectionT(points1_span);
        }
    };

    template <typename fp>
    struct mirror_transform2
    {
      private:
        line2<fp> mirror_line;

      public:
        inline mirror_transform2(const line2<fp>& line) : mirror_line(line) {}

        inline point2<fp> transform(const point2<fp>& p) const
        {
            auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
            low_level::dimensional_vector2 line_vec = mirror_line.p2().vec() - mirror_line.p1().vec();
            low_level::dimensional_vector2 normal_vec = low_level::dimensional_vector2<fp, sixit::units::meter::dim>(-line_vec.y, line_vec.x).normalized(); // rotating by 90 degrees
            low_level::dimensional_vector2 relative_vec = p.vec() - mirror_line.p1().vec();
            auto projection_length = fp_2 * low_level::dimensional_vector2<fp, sixit::units::meter::dim>::dot(relative_vec, normal_vec); // double of the projection length
            low_level::dimensional_vector2 mirrored_vec = p.vec() - normal_vec * projection_length;

            return point2(mirrored_vec); // Return the mirrored point;
        }

        inline affine_transform2<fp> to_affine_transform2()
        {
            low_level::dimensional_vector2 line = mirror_line.p2().vec() - mirror_line.p1().vec();
            low_level::dimensional_vector2 tmp_normal(-line.y, line.x);
            
            low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> normal;
            if (tmp_normal.magnitude() > sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero())
                normal = tmp_normal.normalized();
            else
                normal = low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::zero(); // tmp_normal / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f);

            //construct matrix
            low_level::matrix3x3 m(
                1 - 2 * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(normal[0] * normal[0]), 
                -2 * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(normal[0] * normal[1]), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(mirror_line.p1().vec()[0]),
                -2 * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(normal[1] * normal[0]), 
                1 - 2 * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(normal[1] * normal[1]), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(mirror_line.p1().vec()[1]),
                0.0f,0.0f, 1.0f);

            return affine_transform2(m);
        }

        template <typename T>
        inline T transform(const T& o) const
        {
            using vlo = std::vector<point2<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = transform(points_span[i]);
            }
            return T(points_span);
        }
    };

}; // namespace geometry

namespace geometry 
{
    // aliases 
    using affine_transform3f = affine_transform3<float>;
    using affine_transform2f = affine_transform2<float>;
    using mirror_transform3f = mirror_transform3<float>;
    using affine_transform3to2f = affine_transform3to2<float>;
    using affine_transform2to1f = affine_transform2to1<float>;
    using mirror_transform2f = mirror_transform2<float>;
}


}; // namespace sixit

#endif //sixit_geometry_transforms_h_included

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
