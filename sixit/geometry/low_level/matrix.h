/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_matrix_h_included
#define sixit_geometry_low_level_matrix_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/_internal/simd/mat4.h"
#include "sixit/core/units.h"
#include "sixit/rw/comparsers/json_comparser.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    struct matrix4x4: private gpu::mat4<fp>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::low_level::matrix4x4<fp>>;
        friend struct quaternion<fp>;
        friend struct vector2<fp>;
        friend struct vector3<fp>;
        friend struct ::sixit::geometry::projection_perspective3<fp>;
        friend struct ::sixit::geometry::inverse_projection_perspective3<fp>;
        friend struct ::sixit::geometry::projection3<fp>;
        friend struct ::sixit::geometry::projection3to2<fp>;
        friend struct ::sixit::geometry::projection3to2ex<fp>;
        friend struct ::sixit::geometry::projection3to2noscale<fp>;
        friend struct ::sixit::geometry::affine_transform3<fp>;
        friend struct ::sixit::geometry::mirror_transform3<fp>;
        friend struct ::sixit::geometry::axis_x_projection<fp>;
        friend struct ::sixit::geometry::axis_y_projection<fp>;
        friend struct ::sixit::geometry::axis_z_projection<fp>;
        friend struct ::sixit::geometry::trs3<fp>;
        friend struct ::sixit::graphics::trs3_animation;
        template <typename fp1>
        friend matrix4x4<fp1> low_level::for_renderers_only::get_camera_projection_matrix4x4_dont_use_outside_of_renderer(const projection_perspective3<fp1>& proj,
            const point3<fp1>& position, const direction3<fp1>& direction, const meters<fp1>& focus_distance);
        //friend matrix4x4<float> low_level::for_renderers_only::get_camera_projectionmatrix4x4_dont_use_outside_of_renderer(const projection_perspective3<float>& proj,
        //    const point3<float>& position, const direction3<float>& direction, const meters<float>& focus_distance);

      private:
        inline matrix4x4(const gpu::mat4<fp>& o);
        inline static gpu::mat4<fp> translate_aux(const gpu::vec4p<fp>& vec);
        inline static gpu::mat4<fp> scale_aux(const gpu::vec4p<fp>& vec);
        inline static gpu::mat4<fp> rotate_aux(const gpu::quat4<fp>& quat);

        inline matrix4x4();
        inline matrix4x4(fp m00, fp m01, fp m02, fp m03, fp m10, fp m11, fp m12, fp m13,
            fp m20, fp m21, fp m22, fp m23, fp m30, fp m31, fp m32, fp m33);
        matrix4x4(const matrix4x4& o) = default;
        matrix4x4(matrix4x4&& o) = default;
        matrix4x4& operator=(const matrix4x4&) = default;
        matrix4x4& operator=(matrix4x4&&) = default;
        inline fp& operator()(int row, int col);
        inline fp operator()(int row, int col) const;
        inline vector3<fp> multiply_point3x4(const vector3<fp>& position) const;
        inline vector3<fp> multiply_vector(const vector3<fp>& direction) const;
        inline static matrix4x4 create_from_quaternion(const quaternion<fp>& quat);
        inline static matrix4x4 identity();
        inline static matrix4x4 zero();
        inline matrix4x4 inverse() const;
        inline fp determinant() const;
        inline const matrix4x4 operator*(const matrix4x4& o) const;
        inline matrix4x4& operator*=(const matrix4x4& o);
        inline const matrix4x4 operator+(const matrix4x4& o) const;
        inline matrix4x4& operator+=(const matrix4x4& o);
        inline const matrix4x4 operator-(const matrix4x4& o) const;
        inline matrix4x4& operator-=(const matrix4x4& o);
        inline const matrix4x4 operator/(fp o) const;
        inline matrix4x4& operator/=(fp o);
        inline const matrix4x4 operator*(fp o) const;
        inline matrix4x4& operator*=(fp o);
        inline static matrix4x4 translate(const vector3<fp>& vec3);
        inline static matrix4x4 scale(const vector3<fp>& vec3);
        inline static matrix4x4 rotate(const quaternion<fp>& quat);
        template <sixit::units::physical_dimension dim>
        inline static matrix4x4 rotate(const dimensional_quaternion<fp, dim>& quat);
        static matrix4x4 trs(const vector3<fp>& translate, const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& rotation, 
                                const vector3<fp>& scale);
        inline static matrix4x4 perspective(fp fov, fp aspect, fp near, fp far);

        inline static matrix4x4 compose(const gpu::vec4p<fp>& t, const gpu::quat4<fp>& r, const gpu::vec4p<fp>& s);
        inline static void decompose(const matrix4x4<fp>& m, low_level::vector3<fp>& t, gpu::quat4<fp>& r, low_level::vector3<fp>& s);

    public:
        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"matrix4x4">(comparser, obj);
            if constexpr (std::is_same<decltype(obj.mat[0].x()), sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>>() || 
                            std::is_same<decltype(obj.mat[0].x()), sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>>())
            {
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m00", obj.mat[0].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m01", obj.mat[0].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m02", obj.mat[0].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m03", obj.mat[0].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m10", obj.mat[1].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m11", obj.mat[1].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m12", obj.mat[1].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m13", obj.mat[1].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m20", obj.mat[2].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m21", obj.mat[2].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m22", obj.mat[2].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m23", obj.mat[2].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m30", obj.mat[3].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m31", obj.mat[3].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m32", obj.mat[3].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m33", obj.mat[3].w());
            } 
            else {
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m00", obj.mat[0].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m01", obj.mat[0].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m02", obj.mat[0].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m03", obj.mat[0].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m10", obj.mat[1].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m11", obj.mat[1].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m12", obj.mat[1].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m13", obj.mat[1].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m20", obj.mat[2].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m21", obj.mat[2].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m22", obj.mat[2].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m23", obj.mat[2].w());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m30", obj.mat[3].x());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m31", obj.mat[3].y());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m32", obj.mat[3].z());
                sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m33", obj.mat[3].w());
            }
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct matrix3x3 
    {
        friend struct quaternion<fp>;
        template<typename fp1, sixit::units::physical_dimension>
        friend struct dimensional_vector2;
        friend struct vector2<fp>;
        friend struct vector3<fp>;
        friend struct ::sixit::geometry::projection2<fp>;
        friend struct ::sixit::geometry::projection2to1ex<fp>;
        friend struct ::sixit::geometry::projection2to1<fp>;
        friend struct ::sixit::geometry::projection3to2<fp>;
        friend struct ::sixit::geometry::projection_perspective2<fp>;
        friend struct ::sixit::geometry::trs2<fp>;
        friend struct ::sixit::geometry::affine_transform2<fp>;
        friend struct ::sixit::geometry::mirror_transform2<fp>;
        friend struct ::sixit::geometry::axis_x_projection<fp>;
        friend struct ::sixit::geometry::axis_y_projection<fp>;
        friend struct ::sixit::geometry::axis_z_projection<fp>;

      private:
        std::array<std::array<fp, 3>, 3> mat_;
        inline static matrix3x3 translate_aux(const vector2<fp>& vec);
        inline static matrix3x3 scale_aux(const vector2<fp>& vec);
        inline static matrix3x3 rotate_aux(const quaternion<fp>& quat);
        inline matrix3x3();
        inline matrix3x3(fp m00, fp m01, fp m02, fp m10, fp m11, fp m12, fp m20, fp m21, fp m22);
        inline matrix3x3(const matrix3x3<fp>& o) = default;
        inline matrix3x3(matrix3x3<fp>&& o) = default;
        inline matrix3x3& operator=(const matrix3x3<fp>&) = default;
        inline matrix3x3& operator=(matrix3x3<fp>&&) = default;
        inline fp& operator()(int row, int col);
        inline fp operator()(int row, int col) const;
        inline vector2<fp> multiply_point2x3(const vector2<fp>& position) const;
        inline vector2<fp> multiply_vector(const vector2<fp>& direction);
        inline static matrix3x3<fp> create_from_quaternion(const quaternion<fp>& quat);
        inline static matrix3x3<fp> identity();
        inline static matrix3x3<fp> zero();
        inline matrix3x3<fp> inverse();
        inline fp determinant();
        inline matrix3x3<fp> operator*(const matrix3x3<fp>& o) const;
        inline matrix3x3<fp>& operator*=(const matrix3x3<fp>& o);
        inline matrix3x3<fp> operator+(const matrix3x3<fp>& o) const;
        inline matrix3x3<fp>& operator+=(const matrix3x3<fp>& o);
        inline matrix3x3<fp> operator-(const matrix3x3<fp>& o) const;
        inline matrix3x3<fp>& operator-=(const matrix3x3<fp>& o);
        inline matrix3x3<fp> operator/(fp o) const;
        inline matrix3x3<fp>& operator/=(fp o);
        inline matrix3x3<fp> operator*(fp o) const;
        inline matrix3x3<fp>& operator*=(fp o);
        inline static matrix3x3 translate(const vector2<fp>& vec);
        inline static matrix3x3 scale(const vector2<fp>& vec);
        inline static matrix3x3 rotate(const quaternion<fp>& quat);
        template <sixit::units::physical_dimension dim>
        inline static matrix3x3 rotate(const dimensional_quaternion<fp, dim>& quat);
        inline static matrix3x3 trs(const vector2<fp>& translate, const quaternion<fp>& rotation, const vector2<fp>& scale);
        inline static matrix3x3 perspective(fp fov, fp aspect, fp near, fp far);

    public:
        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"matrix3x3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m00", obj.mat_[0][0]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m01", obj.mat_[0][1]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m02", obj.mat_[0][2]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m10", obj.mat_[1][0]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m11", obj.mat_[1][1]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m12", obj.mat_[1][2]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m20", obj.mat_[2][0]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m21", obj.mat_[2][1]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "m22", obj.mat_[2][2]);
            sixit::rw::end_struct(comparser);
        }
    };
}; // namespace low_level
}; // namespace geometry

namespace geometry::low_level
{
    // aliases 
    using matrix4x4f = matrix4x4<float>;
    using matrix3x3f = matrix3x3<float>;
}

}; // namespace sixit

#endif //sixit_geometry_low_level_matrix_h_included

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
