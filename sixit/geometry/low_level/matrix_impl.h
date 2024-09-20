/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_matrix_impl_h_included
#define sixit_geometry_low_level_matrix_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/low_level/quaternion_impl.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/low_level/matrix.h"


namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    matrix4x4<fp>::matrix4x4(): gpu::mat4<fp>(0.0f) {}

    template <typename fp>
    matrix4x4<fp>::matrix4x4(fp m00, fp m01, fp m02, fp m03, fp m10, fp m11, fp m12, fp m13,
        fp m20, fp m21, fp m22, fp m23, fp m30, fp m31, fp m32, fp m33):
            gpu::mat4<fp>(m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33) {}

    template <typename fp>
    matrix4x4<fp>::matrix4x4(const gpu::mat4<fp>& o):
        gpu::mat4<fp>(o) {}

    template <typename fp>
    gpu::mat4<fp> matrix4x4<fp>::translate_aux(const gpu::vec4p<fp>& vec)
    {
        return gpu::mat4<fp>::translate(vec);
    }

    template <typename fp>
    gpu::mat4<fp> matrix4x4<fp>::scale_aux(const gpu::vec4p<fp>& vec)
    {
        return gpu::mat4<fp>::scale(vec);
    }

    template <typename fp>
    gpu::mat4<fp> matrix4x4<fp>::rotate_aux(const gpu::quat4<fp>& quat)
    {
        return gpu::mat4<fp>::rotate(quat);
    }

    template <typename fp>
    fp& matrix4x4<fp>::operator()(int row, int col) {
        return (*this)[row][col];
    }
    
    template <typename fp>
    fp matrix4x4<fp>::operator()(int row, int col) const {
        return (*this)[row][col];
    }

    template <typename fp>
    vector3<fp> matrix4x4<fp>::multiply_point3x4(const vector3<fp>& position) const
    {
        return *this * position;
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::identity()
    {
        return gpu::mat4<fp>::identity();
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::zero()
    {
        return gpu::mat4<fp>(0.0f);
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::inverse() const
    {
        return gpu::mat4<fp>::inverse(*this);
    }

    template <typename fp>
    fp matrix4x4<fp>::determinant() const
    {
        return gpu::mat4<fp>::determinant(*this);
    }

    template <typename fp>
    const matrix4x4<fp> matrix4x4<fp>::operator*(const matrix4x4<fp>& o) const
    {
        return gpu::operator*(*this, o);
    }

    template <typename fp>
    matrix4x4<fp>& matrix4x4<fp>::operator*=(const matrix4x4<fp>& o)
    {
        *this = gpu::operator*(*this, o);
        return *this;
    }

    template <typename fp>
    const matrix4x4<fp> matrix4x4<fp>::operator+(const matrix4x4<fp>& o) const
    {
        return gpu::operator+(*this, o);
    }

    template <typename fp>
    matrix4x4<fp>& matrix4x4<fp>::operator+=(const matrix4x4<fp>& o)
    {
        *this = gpu::operator+(*this, o);
        return *this;
    }

    template <typename fp>
    const matrix4x4<fp> matrix4x4<fp>::operator-(const matrix4x4<fp>& o) const
    {
        return gpu::operator-(*this, o);
    }

    template <typename fp>
    matrix4x4<fp>& matrix4x4<fp>::operator-=(const matrix4x4<fp>& o)
    {
        *this = gpu::operator-(*this, o);
        return *this;
    }

    template <typename fp>
    const matrix4x4<fp> matrix4x4<fp>::operator/(fp o) const
    {
        return gpu::operator/(*this, o);
    }

    template <typename fp>
    matrix4x4<fp>& matrix4x4<fp>::operator/=(fp o)
    {
        *this = gpu::operator/(*this, o);
        return *this;
    }

    template <typename fp>
    const matrix4x4<fp> matrix4x4<fp>::operator*(fp o) const
    {
        return gpu::operator*(*this, o);
    }

    template <typename fp>
    matrix4x4<fp>& matrix4x4<fp>::operator*=(fp o)
    {
        *this = gpu::operator*(*this, o);
        return *this;
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::translate(const vector3<fp>& vec3)
    {
        return (matrix4x4) matrix4x4<fp>::translate_aux((gpu::vec4p<fp>)vec3);
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::scale(const vector3<fp>& vec3)
    {
        return (matrix4x4) matrix4x4<fp>::scale_aux((gpu::vec4p<fp>)vec3);
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::rotate(const quaternion<fp>& quat)
    {
       return (matrix4x4) matrix4x4<fp>::rotate_aux((gpu::quat4<fp>)quat);
    }

    template <typename fp>
    template <sixit::units::physical_dimension dim>
    matrix4x4<fp> matrix4x4<fp>::rotate(const dimensional_quaternion<fp, dim>& quat)
    {
        return (matrix4x4) matrix4x4<fp>::rotate_aux((gpu::quat4<fp>)quat);
    }

    template <typename fp>
    inline matrix4x4<fp> matrix4x4<fp>::trs(const vector3<fp>& translate, 
                                            const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& rotation, const vector3<fp>& scale)
    {
        return (matrix4x4) gpu::mat4<fp>::compose(translate, (gpu::quat4<fp>)rotation, scale);
    }

    template <typename fp>
    vector3<fp> matrix4x4<fp>::multiply_vector(const vector3<fp>& direction) const
    {
        return gpu::operator*(*this, direction);
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::create_from_quaternion(const quaternion<fp>& quat)
    {
        return gpu::mat4<fp>::rotate((gpu::quat4<fp>)quat);
    }

    template <typename fp>
    matrix4x4<fp> matrix4x4<fp>::perspective(fp fov, fp aspect, fp znear, fp zfar)
    {
        return (matrix4x4) gpu::mat4<fp>::perspective(fov, aspect, znear, zfar);
    }

    template <typename fp>
    inline matrix4x4<fp> matrix4x4<fp>::compose(const gpu::vec4p<fp>& t, const gpu::quat4<fp>& r, const gpu::vec4p<fp>& s)
    {
        return matrix4x4<fp>(gpu::mat4<fp>::compose(t, r, s));
    }

    template <typename fp>
    inline void matrix4x4<fp>::decompose(const matrix4x4<fp>& m, low_level::vector3<fp>& t, gpu::quat4<fp>& r, low_level::vector3<fp>& s)
    {
        gpu::mat4<fp>::decompose(m, t, r, s);
    }

    template <typename fp>
    matrix3x3<fp>::matrix3x3()
    {}

    template <typename fp>
    matrix3x3<fp>::matrix3x3(fp m00_, fp m01_, fp m02_, fp m10_, fp m11_, fp m12_, fp m20_, fp m21_, fp m22_) :
        mat_({ {{{m00_,m01_,m02_}}, {{m10_,m11_,m12_}}, {{m20_,m21_,m22_}} } })
    {}

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::translate_aux(const vector2<fp>& vec)
    {
        matrix3x3<fp> m = identity();
        m(2, 0) = vec[0];
        m(2, 1) = vec[1];
        return m;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::scale_aux(const vector2<fp>& vec)
    {
        matrix3x3<fp> m = identity();
        m(0, 0) = vec[0];
        m(1, 1) = vec[1];
        return m;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::rotate_aux(const quaternion<fp>& quat)
    {
        matrix3x3<fp> m = identity();
        m(0, 0) = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
        m(0, 1) = -2.0f * (quat.x * quat.y - quat.z * quat.w);
        m(1, 0) = -2.0f * (quat.x * quat.y + quat.z * quat.w);
        m(1, 1) = 1.0f - 2.0f * (quat.x * quat.x + quat.z * quat.z);
        return m;
    }

    template <typename fp>
    fp& matrix3x3<fp>::operator()(int row, int col)
    {
        return mat_[row][col];
    }

    template <typename fp>
    fp matrix3x3<fp>::operator()(int row, int col) const
    {
        return mat_[row][col];
    }

    template <typename fp>
    vector2<fp> matrix3x3<fp>::multiply_point2x3(const vector2<fp>& position) const
    {
        vector2<fp> v;
        v.x = mat_[0][0] * position[0] + mat_[1][0] * position[1] + mat_[2][0];
        v.y = mat_[0][1] * position[0] + mat_[1][1] * position[1] + mat_[2][1];
        return v;
    }

    template <typename fp>
    vector2<fp> matrix3x3<fp>::multiply_vector(const vector2<fp>& direction)
    {
        vector2<fp> v;
        v.x = mat_[0][0] * direction[0] + mat_[1][0] * direction[1];
        v.y = mat_[0][1] * direction[0] + mat_[1][1] * direction[1];
        return v;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::create_from_quaternion(const quaternion<fp>& quat)
    {
        return matrix3x3<fp>::rotate_aux(quat);
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::identity()
    {
        return matrix3x3<fp>{
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::zero()
    {
        return matrix3x3<fp>{
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        };
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::inverse()
    {
        matrix3x3<fp> m;
        fp det = determinant();
        m(0, 0) = (mat_[1][1] * mat_[2][2] - mat_[2][1] * mat_[1][2]) / det;
        m(0, 1) = (mat_[1][2] * mat_[2][0] - mat_[0][1] * mat_[2][2]) / det;
        m(0, 2) = (mat_[0][1] * mat_[1][2] - mat_[1][1] * mat_[0][2]) / det;
        m(1, 0) = (mat_[2][0] * mat_[1][2] - mat_[1][0] * mat_[2][2]) / det;
        m(1, 1) = (mat_[0][0] * mat_[2][2] - mat_[0][2] * mat_[2][0]) / det;
        m(1, 2) = (mat_[1][0] * mat_[0][2] - mat_[0][0] * mat_[1][2]) / det;
        m(2, 0) = (mat_[1][0] * mat_[2][1] - mat_[1][1] * mat_[2][0]) / det;
        m(2, 1) = (mat_[0][1] * mat_[2][0] - mat_[0][0] * mat_[2][1]) / det;
        m(2, 2) = (mat_[0][0] * mat_[1][1] - mat_[1][0] * mat_[0][1]) / det;
        return m;
    }

    template <typename fp>
    fp matrix3x3<fp>::determinant()
    {
        return mat_[0][0] * mat_[1][1] * mat_[2][2] + mat_[0][1] * mat_[1][2] * mat_[2][0]
            + mat_[0][2] * mat_[1][0] * mat_[2][1] - mat_[0][2] * mat_[1][1] * mat_[2][0]
            - mat_[0][0] * mat_[1][2] * mat_[2][1] - mat_[0][1] * mat_[1][0] * mat_[2][2];
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::operator*(const matrix3x3<fp>& o) const
    {
        matrix3x3<fp> m;
        for ( int i = 0; i < m.mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = 0.0f;
                for ( int k = 0; k < mat_.size(); ++k) {
                    m(i, j) = m(i, j) + mat_[i][k] * o(k, j);
                }
            }
        return m;
    }

    template <typename fp>
    matrix3x3<fp>& matrix3x3<fp>::operator*=(const matrix3x3<fp>& o)
    {
        matrix3x3<fp> m;
        for ( int i = 0; i < m.mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = 0.0f;
                for ( int k = 0; k < mat_.size(); ++k) {
                    m(i, j) = m(i, j) + mat_[i][k] * o(k, j);
                }
            }
        *this = m;
        return *this;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::operator+(const matrix3x3<fp>& o) const
    {
        matrix3x3<fp> m;
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = mat_[i][j] + o(i, j);
            }
        return m;
    }

    template <typename fp>
    matrix3x3<fp>& matrix3x3<fp>::operator+=(const matrix3x3<fp>& o)
    {
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                mat_[i][j] += o(i, j);
            }
        return *this;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::operator-(const matrix3x3<fp>& o) const
    {
        matrix3x3 m;
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = mat_[i][j] - o(i, j);
            }
        return m;
    }

    template <typename fp>
    matrix3x3<fp>& matrix3x3<fp>::operator-=(const matrix3x3<fp>& o)
    {
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                mat_[i][j] -= o(i, j);
            }
        return *this;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::operator/(fp o) const
    {
        matrix3x3<fp> m;
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = mat_[i][j] / o;
            }
        return m;
    }

    template <typename fp>
    matrix3x3<fp>& matrix3x3<fp>::operator/=(fp o)
    {
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                mat_[i][j] /= o;
            }
        return *this;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::operator*(fp o) const
    {
        matrix3x3<fp> m;
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                m(i, j) = mat_[i][j] * o;
            }
        return m;
    }

    template <typename fp>
    matrix3x3<fp>& matrix3x3<fp>::operator*=(fp o)
    {
        for ( int i = 0; i < mat_.size(); ++i)
            for ( int j = 0; j < mat_.size(); ++j)
            {
                mat_[i][j] *= o;
            }
        return *this;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::translate(const vector2<fp>& vec)
    {
        return matrix3x3<fp>::translate_aux(vec);
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::scale(const vector2<fp>& vec)
    {
        return matrix3x3<fp>::scale_aux(vec);
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::rotate(const quaternion<fp>& quat)
    {
        return rotate_aux(quat);
    }

    template <typename fp>
    template <sixit::units::physical_dimension dim>
    matrix3x3<fp> matrix3x3<fp>::rotate(const dimensional_quaternion<fp, dim>& quat)
    {
        quaternion mtq(quat.w, quat.x, quat.y, quat.z);
        return (matrix3x3) matrix3x3::rotate_aux(mtq);
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::trs(const vector2<fp>& translate, const quaternion<fp>& rotation, const vector2<fp>& scale)
    {
        quaternion mrot(rotation.w, rotation.x, rotation.y, rotation.z);
        matrix3x3 mat = identity();
        mat *= matrix3x3::scale_aux(scale);
        mat *= matrix3x3::rotate_aux(mrot);
        mat *= matrix3x3::translate_aux(translate);
        return mat;
    }

    template <typename fp>
    matrix3x3<fp> matrix3x3<fp>::perspective(fp fov, fp aspect, fp znear, fp zfar)
    {
        fp tan_half_fov = sixit::geometry::low_level::mathf::tan(fov / fp(2.0f));
        matrix3x3 m;

        m(0, 0) = 1.0f / (aspect * tan_half_fov);
        m(1, 1) = 1.0f / (tan_half_fov);
        m(2, 2) = -(zfar + znear) / (zfar - znear);
        return m;
    }
}; // namespace low_level
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_matrix_impl_h_included

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
