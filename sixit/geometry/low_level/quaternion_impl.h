/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_quaternion_impl_h_included
#define sixit_geometry_low_level_quaternion_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    quaternion<fp>::quaternion(const gpu::quat4<fp>& o): quaternion(o.w, o.x, o.y, o.z) {}
    
    template <typename fp>
    quaternion<fp>::quaternion(const matrix3x3<fp>& m)
    {
        *this = create_from_matrix(m);
    }
    
    template <typename fp>
    quaternion<fp>::quaternion(): gpu::quat4<fp>() {}

    template <typename fp>
    quaternion<fp>::quaternion(const gpu::mat4<fp>& m)
    {
        gpu::vec4p<fp> t, s;
        gpu::mat4<fp>::decompose(m, t, (gpu::quat4<fp>&) *this, s);
    }

    template <typename fp>
    quaternion<fp>::quaternion(fp w, fp x, fp y, fp z): gpu::quat4<fp>(x, y, z, w) {}

    template <typename fp>
    quaternion<fp>::quaternion(fp x, fp y, fp z) // form euler angles
    {
        x = x * mathf::deg_2_rad;
        y = y * mathf::deg_2_rad;
        z = z * mathf::deg_2_rad;
        *this = (quaternion) sixit::geometry::gpu::quat4<fp>::from_euler(x, y, z);
    }

    template <typename fp>
    quaternion<fp>::operator gpu::mat4<fp>() const
    {
        gpu::vec4p<fp> t(0.0f), s(1.0f);
        return gpu::mat4<fp>::compose(t, *this, s);
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::create_from_matrix(const gpu::mat4<fp>& mat)
    {
        gpu::vec4p<fp> t, s;
        gpu::quat4<fp> r;
        gpu::mat4<fp>::decompose(mat, t, r, s);
        return (quaternion) r;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::create_from_matrix(const matrix3x3<fp>& mat)
    {
        gpu::mat4 m4(
                mat(0,0), mat(0,1), mat(0,2), fp(0.0f),
                mat(1,0), mat(1,1), mat(1,2), fp(0.0f),
                mat(2,0), mat(2,1), mat(2,2), fp(0.0f),
                fp(0.0f), fp(0.0f), fp(0.0f), fp(1.0f));
        return create_from_matrix(m4);
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::axis_angle(const vector3<fp>& axis, fp angle)
    {
        return (quaternion) gpu::quat4<fp>::from_axis_angle(axis, angle * mathf::deg_2_rad);
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::euler_angles(fp x, fp y, fp z)
    {
        return quaternion(x, y, z);
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::identity()
    {
        return quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    template <typename fp>
    vector3<fp> quaternion<fp>::euler_angles() const
    {
        vector3<fp> angles;
        float x = this->x();
        float y = this->y();
        float z = this->z();
        float w = this->w();
        fp check = 2.0f * (-y * z + w * x);
        if (check < -1.0f)
        {
            angles.x() = -mathf::pi / 2.0f;
            angles.y() = 0.0f;
            fp sinp = 2.0f * (x * z - w * y);
            fp cosp = 1.0f - 2.0f * (y * y + z * z);
            angles.z() = -mathf::atan2(sinp, cosp);
        }
        else if (check > 1.0f)
        {
            angles.x() = mathf::pi / 2.0f;
            angles.y() = 0.0f;
            fp sinp = 2.0f * (x * z - w * y);
            fp cosp = 1.0f - 2.0f * (y * y + z * z);
            angles.z() = mathf::atan2(sinp, cosp);
        }
        else
        {
            angles.x() = mathf::asin(check);
            angles.y() = mathf::atan2(2.0f * (x * z + w * y), 1.0f - 2.0f * (x * x + y * y));
            angles.z() = mathf::atan2(2.0f * (x * y + w * z), 1.0f - 2.0f * (x * x + z * z));
        }
        angles *= mathf::rad_2_deg;
        angles.x() = mathf::fmod(angles.x(), 360.0f);
        angles.y() = mathf::fmod(angles.y(), 360.0f);
        angles.z() = mathf::fmod(angles.z(), 360.0f);
        if (angles.x() < 0.0f)
        {
            angles.x() += 360.0f;
        }
        if (angles.y() < 0.0f)
        {
            angles.y() += 360.0f;
        }
        if (angles.z() < 0.0f)
        {
            angles.z() += 360.0f;
        }
        if (mathf::abs(angles.x() - 360.0f) <= 0.f)
        {
            angles.x() = 0.0f;
        }
        if (mathf::abs(angles.y() - 360.0f) <= 0.f)
        {
            angles.y() = 0.0f;
        }
        if (mathf::abs(angles.z() - 360.0f) <= 0.f)
        {
            angles.z() = 0.0f;
        }
        return angles;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::normalized() const
    {
        quaternion q(gpu::quat4<fp>::normalize(*this));
        return q;
    }

    template <typename fp>
    void quaternion<fp>::set(fp w_, fp x_, fp y_, fp z_)
    {
        this->x = x_;
        this->y = y_;
        this->z = z_;
        this->w = w_;
    }

    template <typename fp>
    void quaternion<fp>::normalize()
    {
        *this = (quaternion) gpu::quat4<fp>::normalize(*this);
    }

    template <typename fp>
    fp quaternion<fp>::angle(const quaternion<fp>& a, const quaternion<fp>& b)
    {
        fp dot = gpu::quat4<fp>::dot(a, b);
        dot = mathf::min(mathf::abs(dot), fp(1.0f));
        if (dot > 1.0f)
        {
            return 0.0f;
        }
        return 2.0f * mathf::acos(dot) * mathf::rad_2_deg;
    }
    
    template <typename fp>
    fp quaternion<fp>::dot(const quaternion<fp>& a, const quaternion<fp>& b)
    {
        return gpu::quat4<fp>::dot(a, b);
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::inverse(const quaternion<fp>& a)
    {
        return (quaternion) gpu::quat4<fp>::inverse(a);
    }

    // TODO: test this
    template <typename fp>
    quaternion<fp> quaternion<fp>::look_rotation(const vector3<fp>& fwdDir, const vector3<fp>& upDir)
    {
        vector3 fwd = fwdDir.normalized();
        vector3 right = vector3<fp>::cross(upDir, fwd).normalized();
        vector3 up = vector3<fp>::cross(right, fwd);
        fp m00 = right.x();
        fp m01 = right.y();
        fp m02 = right.z();
        fp m10 = up.x();
        fp m11 = up.y();
        fp m12 = up.z();
        fp m20 = fwd.x();
        fp m21 = fwd.y();
        fp m22 = fwd.z();
        fp num8 = (m00 + m11) + m22;
        quaternion q = quaternion::identity();
        if (num8 > 0.0f)
        {
            fp num = mathf::sqrt(num8 + 1.0f);
            q.w = num * 0.5f;
            num = 0.5f / num;
            q.x = (m12 - m21) * num;
            q.y = (m20 - m02) * num;
            q.z = (m01 - m10) * num;
            return q;
        }
        if ((m00 >= m11) && (m00 >= m22))
        {
            fp num7 = mathf::sqrt(((1.0f + m00) - m11) - m22);
            fp num4 = 0.5f / num7;
            q.x = 0.5f * num7;
            q.y = (m01 + m10) * num4;
            q.z = (m02 + m20) * num4;
            q.w = (m12 - m21) * num4;
            return q;
        }
        if (m11 > m22)
        {
            fp num6 = mathf::sqrt(((1.0f + m11) - m00) - m22);
            fp num3 = 0.5f / num6;
            q.x = (m10 + m01) * num3;
            q.y = 0.5f * num6;
            q.z = (m21 + m12) * num3;
            q.w = (m20 - m02) * num3;
            return q;
        }
        fp num5 = mathf::sqrt(((1.0f + m22) - m00) - m11);
        fp num2 = 0.5f / num5;
        q.x = (m20 + m02) * num2;
        q.y = (m21 + m12) * num2;
        q.z = 0.5f * num5;
        q.w = (m01 - m10) * num2;
        return q;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::from_to_rotation(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& from_direction, 
                                                    const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& to_direction)
    {
        quaternion result;
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> fd = from_direction.normalized();
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> td = to_direction.normalized();
        
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> half = fd + td;
        auto half_magntude = half.magnitude();
        if (half_magntude > sixit::units::create_dimensionless_scalar<fp>(0.f))
        {
            half = half / half_magntude;
            vector3 cross = vector3<fp>::cross(fd, half);
            result.set(vector3<fp>::dot(fd, half), cross.x(), cross.y(), cross.z());
        }
        else
        {
            // Opposite vectors
            fp vx = abs(fd.x());
            fp vy = abs(fd.y());
            fp vz = abs(fd.z());
            vector3<fp> ortogonal = vx < vy ? (vx < vz ? vector3<fp>::right() : vector3<fp>::forward()) : (vy < vz ? vector3<fp>::up() : vector3<fp>::forward());
            ortogonal = vector3<fp>::cross(fd, ortogonal).normalized();

            result.set(0, ortogonal.x(), ortogonal.y(), ortogonal.z());
        }

        return result;
    }

    // template <typename fp>
    // inline quaternion quaternion::LerpUnclamped(const quaternion& min, const quaternion& max, fp p)
    // {
    //     fp dot = quaternion::dot(min, max);
    //     if (mathf::abs(dot) > 1.0f - mathf::epsilon)
    //     {
    //         return quaternion(min);
    //     }

    //     fp half = mathf::acos(dot);
    //     fp sinHalf = mathf::sqrt(1.0f - dot * dot);
    //     if (mathf::abs(sinHalf) <  mathf::epsilon)
    //     {
    //         return mathlib_internal::operator+(mathlib_internal::operator*(min.vec(), 0.5f), mathlib_internal::operator*(max.vec(), 0.f));
    //     }
    //     fp ratioa = mathf::sin((1.0f - p) * half) / sinHalf;
    //     fp ratiob = mathf::sin(p * half) / sinHalf;
    //     quaternion r;
    //     r.w = min.w * ratioa + max.w * ratiob;
    //     r.x = min.x * ratioa + max.x * ratiob;
    //     r.y = min.y * ratioa + max.y * ratiob;
    //     r.z = min.z * ratioa + max.z * ratiob;
    //     return r;
    // }

    template <typename fp>
    quaternion<fp> quaternion<fp>::operator*(const quaternion<fp>& o)
    {
        return (quaternion) gpu::operator*(*this, o);
    }

    template <typename fp>
    quaternion<fp>& quaternion<fp>::operator*=(const quaternion<fp>& o)
    {
        *this = (quaternion) gpu::operator*(*this, o);
        return *this;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::operator+(const quaternion<fp>& o)
    {
        return (quaternion) gpu::operator+(*this, o);
    }

    template <typename fp>
    quaternion<fp>& quaternion<fp>::operator+=(const quaternion<fp>& o)
    {
        *this = (quaternion) gpu::operator+(*this, o);
        return *this;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::operator-(const quaternion<fp>& o)
    {
        return (quaternion) gpu::operator-(*this, o);
    }

    template <typename fp>
    quaternion<fp>& quaternion<fp>::operator-=(const quaternion<fp>& o)
    {
        *this = (quaternion) gpu::operator-(*this, o);
        return *this;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::operator*(fp o)
    {
        return (quaternion) gpu::operator*(*this, o);
    }

    template <typename fp>
    quaternion<fp>& quaternion<fp>::operator*=(fp o)
    {
        *this = (quaternion) gpu::operator*(*this, o);
        return *this;
    }

    template <typename fp>
    quaternion<fp> quaternion<fp>::operator/(fp o)
    {
        return (quaternion) gpu::operator/(*this, o);
    }

    template <typename fp>
    quaternion<fp>& quaternion<fp>::operator/=(fp o)
    {
        *this = (quaternion) gpu::operator/(*this, o);
        return *this;
    }

    template <typename fp>
    [[nodiscard]] bool quaternion<fp>::_for_test_only_approximate_eq(const quaternion<fp>& other, size_t n) const
    {
        return impl::_for_test_only_approximate_eq<fp, 4>(this->quat, other.quat, n);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(const gpu::quat4<fp>& o): dimensional_quaternion(
                                                            sixit::units::create_dimensionless_scalar<fp>(o.w()), 
                                                            sixit::units::create_dimensionless_scalar<fp>(o.x()), 
                                                            sixit::units::create_dimensionless_scalar<fp>(o.y()), 
                                                            sixit::units::create_dimensionless_scalar<fp>(o.z())) {}
    
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(const matrix3x3<fp>& m)
    {
        *this = create_from_matrix(m);
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(): gpu::quat4<fp>() {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(const gpu::mat4<fp>& m)
    {
        gpu::vec4p<fp> t, s;
        gpu::mat4<fp>::decompose(m, t, (gpu::quat4<fp>&) *this, s);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> w, 
                                                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                                                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y, 
                                                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> z): gpu::quat4<fp>(
                                                                sixit::units::extract_dim_less_scalar(x), sixit::units::extract_dim_less_scalar(y), 
                                                                sixit::units::extract_dim_less_scalar(z), sixit::units::extract_dim_less_scalar(w)) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::dimensional_quaternion(
                                        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                                        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y, 
                                        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> z) // form euler angles
    {
        x *= sixit::units::create_dimensionless_scalar<fp>(mathf::deg_2_rad);
        y *= sixit::units::create_dimensionless_scalar<fp>(mathf::deg_2_rad);
        z *= sixit::units::create_dimensionless_scalar<fp>(mathf::deg_2_rad);
        *this = (dimensional_quaternion) sixit::geometry::gpu::quat4<fp>::from_euler(
            sixit::units::extract_dim_less_scalar(x), sixit::units::extract_dim_less_scalar(y), sixit::units::extract_dim_less_scalar(z));
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>::operator gpu::mat4<fp>() const
    {
        gpu::vec4p<fp> t(0.0f), s(1.0f);
        return gpu::mat4<fp>::compose(t, *this, s);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::create_from_matrix(const gpu::mat4<fp>& mat)
    {
        gpu::vec4p<fp> t, s;
        gpu::quat4<fp> r;
        gpu::mat4<fp>::decompose(mat, t, r, s);
        return (dimensional_quaternion) r;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::create_from_matrix(const matrix3x3<fp>& mat)
    {
        gpu::mat4 m4(
                mat(0,0), mat(0,1), mat(0,2), fp(0.0f),
                mat(1,0), mat(1,1), mat(1,2), fp(0.0f),
                mat(2,0), mat(2,1), mat(2,2), fp(0.0f),
                fp(0.0f), fp(0.0f), fp(0.0f), fp(1.0f));
        return create_from_matrix(m4);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::axis_angle(const dimensional_vector3<fp, dim>& axis, 
                                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle)
    {
        return (dimensional_quaternion) gpu::quat4<fp>::from_axis_angle(axis, sixit::units::extract_dim_less_scalar(angle) * mathf::deg_2_rad);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::euler_angles(
                                            sixit::units::dimensional_scalar<fp, dim> x, 
                                            sixit::units::dimensional_scalar<fp, dim> y, 
                                            sixit::units::dimensional_scalar<fp, dim> z)
    {
        return dimensional_quaternion(x, y, z);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::identity()
    {
        return dimensional_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_quaternion<fp, dim>::euler_angles() const
    {
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> angles;
        float x = this->x();
        float y = this->y();
        float z = this->z();
        float w = this->w();
        fp check = 2.0f * (-y * z + w * x);
        if (check < -1.0f)
        {
            angles.x() = -mathf::pi / 2.0f;
            angles.y() = 0.0f;
            fp sinp = 2.0f * (x * z - w * y);
            fp cosp = 1.0f - 2.0f * (y * y + z * z);
            angles.z() = -mathf::atan2(sinp, cosp);
        }
        else if (check > 1.0f)
        {
            angles.x() = mathf::pi / 2.0f;
            angles.y() = 0.0f;
            fp sinp = 2.0f * (x * z - w * y);
            fp cosp = 1.0f - 2.0f * (y * y + z * z);
            angles.z() = mathf::atan2(sinp, cosp);
        }
        else
        {
            angles.x() = mathf::asin(check);
            angles.y() = mathf::atan2(2.0f * (x * z + w * y), 1.0f - 2.0f * (x * x + y * y));
            angles.z() = mathf::atan2(2.0f * (x * y + w * z), 1.0f - 2.0f * (x * x + z * z));
        }
        angles *= sixit::units::create_dimensionless_scalar<fp>(fp(mathf::rad_2_deg));
        angles.x() = mathf::fmod(angles.x(), 360.0f);
        angles.y() = mathf::fmod(angles.y(), 360.0f);
        angles.z() = mathf::fmod(angles.z(), 360.0f);
        if (angles.x() < 0.0f)
        {
            angles.x() += 360.0f;
        }
        if (angles.y() < 0.0f)
        {
            angles.y() += 360.0f;
        }
        if (angles.z() < 0.0f)
        {
            angles.z() += 360.0f;
        }
        if (mathf::abs(angles.x() - 360.0f) <= 0.f)
        {
            angles.x() = 0.0f;
        }
        if (mathf::abs(angles.y() - 360.0f) <= 0.f)
        {
            angles.y() = 0.0f;
        }
        if (mathf::abs(angles.z() - 360.0f) <= 0.f)
        {
            angles.z() = 0.0f;
        }
        return angles;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, sixit::units::simple_scalar::dim> dimensional_quaternion<fp, dim>::normalized() const
    {
        dimensional_quaternion<fp, sixit::units::simple_scalar::dim> q(gpu::quat4<fp>::normalize(*this));
        return q;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_quaternion<fp, dim>::set(
                                        sixit::units::dimensional_scalar<fp, dim> w_, 
                                        sixit::units::dimensional_scalar<fp, dim> x_, 
                                        sixit::units::dimensional_scalar<fp, dim> y_, 
                                        sixit::units::dimensional_scalar<fp, dim> z_)
    {
        this->x() = x_.value;
        this->y() = y_.value;
        this->z() = z_.value;
        this->w() = w_.value;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> dimensional_quaternion<fp, dim>::angle(
                                        const dimensional_quaternion<fp, dim>& a, 
                                        const dimensional_quaternion<fp, dim>& b)
    {
        fp dot = gpu::quat4<fp>::dot(a, b);
        dot = mathf::min(mathf::abs(dot), fp(1.0f));
        if (dot > 1.0f)
        {
            return 0.0f;
        }
        return sixit::units::create_dimensionless_scalar<fp>(2.0f * mathf::acos(dot) * mathf::rad_2_deg);
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, dim * dim> dimensional_quaternion<fp, dim>::dot(
                                        const dimensional_quaternion<fp, dim>& a, 
                                        const dimensional_quaternion<fp, dim>& b)
    {
        return gpu::quat4<fp>::dot(a, b);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::inverse(const dimensional_quaternion<fp, dim>& a)
    {
        return (dimensional_quaternion) gpu::quat4<fp>::inverse(a);
    }

    // TODO: test this
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::look_rotation(const dimensional_vector3<fp, dim>& fwdDir, const dimensional_vector3<fp, dim>& upDir)
    {
        dimensional_vector3 fwd = fwdDir.normalized();
        dimensional_vector3 right = dimensional_vector3<fp, dim>::cross(upDir, fwd).normalized();
        dimensional_vector3 up = dimensional_vector3<fp, dim>::cross(right, fwd);
        fp m00 = right.x();
        fp m01 = right.y();
        fp m02 = right.z();
        fp m10 = up.x();
        fp m11 = up.y();
        fp m12 = up.z();
        fp m20 = fwd.x();
        fp m21 = fwd.y();
        fp m22 = fwd.z();
        fp num8 = (m00 + m11) + m22;
        dimensional_quaternion<fp, dim> q = dimensional_quaternion<fp, dim>::identity();
        if (num8 > 0.0f)
        {
            fp num = mathf::sqrt(num8 + 1.0f);
            q.w = num * 0.5f;
            num = 0.5f / num;
            q.x = (m12 - m21) * num;
            q.y = (m20 - m02) * num;
            q.z = (m01 - m10) * num;
            return q;
        }
        if ((m00 >= m11) && (m00 >= m22))
        {
            fp num7 = mathf::sqrt(((1.0f + m00) - m11) - m22);
            fp num4 = 0.5f / num7;
            q.x = 0.5f * num7;
            q.y = (m01 + m10) * num4;
            q.z = (m02 + m20) * num4;
            q.w = (m12 - m21) * num4;
            return q;
        }
        if (m11 > m22)
        {
            fp num6 = mathf::sqrt(((1.0f + m11) - m00) - m22);
            fp num3 = 0.5f / num6;
            q.x = (m10 + m01) * num3;
            q.y = 0.5f * num6;
            q.z = (m21 + m12) * num3;
            q.w = (m20 - m02) * num3;
            return q;
        }
        fp num5 = mathf::sqrt(((1.0f + m22) - m00) - m11);
        fp num2 = 0.5f / num5;
        q.x = (m20 + m02) * num2;
        q.y = (m21 + m12) * num2;
        q.z = 0.5f * num5;
        q.w = (m01 - m10) * num2;
        return q;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::from_to_rotation(
                                                    const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& from_direction, 
                                                    const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& to_direction)
    {
        dimensional_quaternion<fp, dim> result;
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> fd = from_direction.normalized();
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> td = to_direction.normalized();
        
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> half = fd + td;
        auto half_magntude = half.magnitude();
        if (half_magntude > sixit::units::create_dimensionless_scalar<fp>(0.f))
        {
            half = half / half_magntude;
            dimensional_vector3 cross = dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(fd, half);
            result.set(dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(fd, half), cross.dim_x(), cross.dim_y(), cross.dim_z());
        }
        else
        {
            // Opposite vectors
            fp vx = low_level::mathf::abs(fd.x());
            fp vy = low_level::mathf::abs(fd.y());
            fp vz = low_level::mathf::abs(fd.z());
            dimensional_vector3<fp, sixit::units::simple_scalar::dim> ortogonal = vx < vy
                                 ? (vx < vz ? dimensional_vector3<fp, sixit::units::simple_scalar::dim>::right() : dimensional_vector3<fp, sixit::units::simple_scalar::dim>::forward()) 
                                 : (vy < vz ? dimensional_vector3<fp, sixit::units::simple_scalar::dim>::up() : dimensional_vector3<fp, sixit::units::simple_scalar::dim>::forward());
            ortogonal = dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(fd, ortogonal).normalized();

            result.set(sixit::units::create_dimensionless_scalar<fp>(0.f), 
                        sixit::units::create_dimensionless_scalar<fp>(ortogonal.x()), 
                        sixit::units::create_dimensionless_scalar<fp>(ortogonal.y()), 
                        sixit::units::create_dimensionless_scalar<fp>(ortogonal.z()));
        }

        return result;
    }

    // template <typename fp>
    // inline quaternion quaternion::LerpUnclamped(const quaternion& min, const quaternion& max, fp p)
    // {
    //     fp dot = quaternion::dot(min, max);
    //     if (mathf::abs(dot) > 1.0f - mathf::epsilon)
    //     {
    //         return quaternion(min);
    //     }

    //     fp half = mathf::acos(dot);
    //     fp sinHalf = mathf::sqrt(1.0f - dot * dot);
    //     if (mathf::abs(sinHalf) <  mathf::epsilon)
    //     {
    //         return mathlib_internal::operator+(mathlib_internal::operator*(min.vec(), 0.5f), mathlib_internal::operator*(max.vec(), 0.f));
    //     }
    //     fp ratioa = mathf::sin((1.0f - p) * half) / sinHalf;
    //     fp ratiob = mathf::sin(p * half) / sinHalf;
    //     quaternion r;
    //     r.w = min.w * ratioa + max.w * ratiob;
    //     r.x = min.x * ratioa + max.x * ratiob;
    //     r.y = min.y * ratioa + max.y * ratiob;
    //     r.z = min.z * ratioa + max.z * ratiob;
    //     return r;
    // }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    dimensional_quaternion<fp, dim * dim1> dimensional_quaternion<fp, dim>::operator*(const dimensional_quaternion<fp, dim1>& o)
    {
        return (dimensional_quaternion<fp, dim * dim1>) gpu::operator*(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>& dimensional_quaternion<fp, dim>::operator*=(const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& o)
    {
        *this = (dimensional_quaternion<fp, dim>) gpu::operator*(*this, o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::operator+(const dimensional_quaternion<fp, dim>& o)
    {
        return (dimensional_quaternion<fp, dim>) gpu::operator+(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>& dimensional_quaternion<fp, dim>::operator+=(const dimensional_quaternion<fp, dim>& o)
    {
        *this = (dimensional_quaternion<fp, dim>) gpu::operator+(*this, o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim> dimensional_quaternion<fp, dim>::operator-(const dimensional_quaternion<fp, dim>& o)
    {
        return (dimensional_quaternion<fp, dim>) gpu::operator-(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>& dimensional_quaternion<fp, dim>::operator-=(const dimensional_quaternion<fp, dim>& o)
    {
        *this = (dimensional_quaternion<fp, dim>) gpu::operator-(*this, o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    dimensional_quaternion<fp, dim * dim1> dimensional_quaternion<fp, dim>::operator*(sixit::units::dimensional_scalar<fp, dim1> o)
    {
        return (dimensional_quaternion<fp, dim * dim1>) gpu::operator*(*this, o.value);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>& dimensional_quaternion<fp, dim>::operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        *this = (dimensional_quaternion<fp, dim>) gpu::operator*(*this, o.value);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    dimensional_quaternion<fp, dim / dim1> dimensional_quaternion<fp, dim>::operator/(sixit::units::dimensional_scalar<fp, dim1> o)
    {
        return (dimensional_quaternion<fp, dim / dim1>) gpu::operator/(*this, o.value);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_quaternion<fp, dim>& dimensional_quaternion<fp, dim>::operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        *this = (dimensional_quaternion<fp, dim>) gpu::operator/(*this, o.value);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    [[nodiscard]] bool dimensional_quaternion<fp, dim>::_for_test_only_approximate_eq(const dimensional_quaternion<fp, dim>& other, size_t n) const
    {
        return impl::_for_test_only_approximate_eq<fp, fp, 4>(this->const_ptr(), other.const_ptr(), n);
    }





}; // namespace low_level
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_quaternion_impl_h_included

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
