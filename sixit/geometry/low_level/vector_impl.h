/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_vector_impl_h_included
#define sixit_geometry_low_level_vector_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/_internal/simd/mat4.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    vector2<fp>::vector2() {
        fp zero(0.f);
        set(zero, zero);
    }

    template <typename fp>
    vector2<fp>::vector2(const vector2<fp>& o): x(o.x), y(o.y) {
    }

    template <typename fp>
    vector2<fp>::vector2(const vector3<fp>& o): x(o.x()), y(o.y()) {
    }

    template <typename fp>
    vector2<fp>::vector2(fp x, fp y) : x(x), y(y) {
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator+=(const vector2<fp>& o)
    {
        x = x + o.x;
        y = y + o.y;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator+(const vector2<fp>& o) const
    {
        return vector2(x + o.x, y + o.y);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator+=(fp o)
    {
        x = x + o;
        y = y + o;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator+(fp o) const
    {
        return vector2(x + o, y + o);
    }
    
    template <typename fp>
    vector2<fp>& vector2<fp>::operator-=(const vector2<fp>& o)
    {
        x = x - o.x;
        y = y - o.y;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator-(const vector2<fp>& o) const
    {
        return vector2(x - o.x, y - o.y);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator-=(fp o)
    {
        x = x - o;
        y = y - o;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator-(fp o) const
    {
        return vector2(x - o, y - o);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator*=(const vector2<fp>& o)
    {
        x = x * o.x;
        y = y * o.y;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator*(const vector2<fp>& o) const
    {
        return vector2(x * o.x, y * o.y);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator*=(fp o)
    {
        x = x * o;
        y = y * o;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator*(const fp o) const
    {
        auto xx = x * o;
        auto yy = y * o;
        return vector2(xx, yy);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator/=(const vector2<fp>& o)
    {
        x = x / o.x;
        y = y / o.y;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator/(const vector2<fp>& o) const
    {
        return vector2(x/o.x, y/o.y);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator/=(fp o)
    {
        x = x / o;
        y = y / o;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator/(fp o) const
    {
        auto xx = x / o;
        auto yy = y / o;
        return vector2(xx, yy);
    }

    template <typename fp>
    inline fp vector2<fp>::operator[](int index) const
    {
        return index == 0 ? x : y;
    }

    template <typename fp>
    inline vector2<fp> vector2<fp>::operator-() const
    {
        return vector2<fp>(-x, -y);
    }

    template <typename fp>
    fp vector2<fp>::magnitude() const
    {
        return mathf::sqrt(x * x + y * y);
    }

    template <typename fp>
    auto vector2<fp>::sqr_magnitude() const
    {
        return x * x + y * y;
    }

    template <typename fp>
    void vector2<fp>::normalize()
    {
        *this = normalized();
    }


    template <typename fp>
    void vector2<fp>::set(fp x_, fp y_)
    {
        this->x = x_;
        this->y = y_;
    }

    /*template <typename fp>
    vector2<fp> vector2<fp>::up()
    {
        return vector2<fp>{fp(0.f), fp(1.f)};
    }

    template <typename fp>
    vector2<fp> vector2<fp>::down()
    {
        return vector2<fp>{fp(0.f), fp(-1.f)};
    }

    template <typename fp>
    vector2<fp> vector2<fp>::left()
    {
        return vector2<fp>{fp(-1.f), fp(0.f)};
    }

    template <typename fp>
    vector2<fp> vector2<fp>::right()
    {
        return vector2<fp>{fp(1.f), fp(0.f)};
    }

    template <typename fp>
    vector2<fp> vector2<fp>::one()
    {
        return vector2<fp>{fp(1.f), fp(1.f)};
    }*/

    template <typename fp>
    vector2<fp> vector2<fp>::zero()
    {
        return vector2<fp>{fp(0.f), fp(0.f)};
    }

    template <typename fp>
    fp vector2<fp>::angle(const vector2<fp>& a, const vector2<fp>& b)
    {
        fp angle = mathf::atan2(a.y, a.x) - mathf::atan2(b.y, b.x);
        if (angle > mathf::pi) angle = angle - mathf::pi * 2;
        if (angle < -mathf::pi) angle = angle + mathf::pi * 2;
        return angle * mathf::rad_2_deg;
    }

    template <typename fp>
    fp vector2<fp>::dot(const vector2<fp>& a, const vector2<fp>& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    template <typename fp>
    fp vector2<fp>::normalized() const
    {
        fp m = magnitude();
        auto xx = x / m;
        auto yy = y / m;
        return vector2(xx, yy);
    }

    template <typename fp>
    inline vector2<fp> vector2<fp>::max(const vector2<fp>& a, const vector2<fp>& b)
    {
        return vector2<fp>{mathf::max(a.x, b.x), mathf::max(a.y, b.y)};
    }

    template <typename fp>
    inline vector2<fp> vector2<fp>::min(const vector2<fp>& a, const vector2<fp>& b)
    {
        return vector2<fp>{mathf::min(a.x, b.x), mathf::min(a.y, b.y)};
    }

    template <typename fp>
    vector2<fp> vector2<fp>::scale(const vector2<fp>& a, fp b)
    {
        return a * b;
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator*=(const matrix4x4<fp>& o)
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        this->set(vec3.x(), vec3.y());
        this->x = vec3.x();
        this->y = vec3.y();
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator*(const matrix4x4<fp>& o) const
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        return vector2(vec3);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator*=(const matrix3x3<fp>& o)
    {
        fp x_ = o(0, 0) * x + o(1, 0) * y + o(2, 0);
        fp y_ = o(0, 1) * x + o(1, 1) * y + o(2, 1);
        fp z_ = o(0, 2) * x + o(1, 2) * y + o(2, 2);
        x = x_ / z_;
        y = y_ / z_;
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator*(const matrix3x3<fp>& o) const
    {
        fp x_ = o(0, 0) * x + o(1, 0) * y + o(2, 0);
        fp y_ = o(0, 1) * x + o(1, 1) * y + o(2, 1);
        fp z_ = o(0, 2) * x + o(1, 2) * y + o(2, 2);
        return vector2(x_ / z_, y_ / z_);
    }

    template <typename fp>
    vector2<fp>& vector2<fp>::operator*=(const quaternion<fp>& o) //weird issue
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        this->set(vec3.x(), vec3.y());
        return *this;
    }

    template <typename fp>
    const vector2<fp> vector2<fp>::operator*(const quaternion<fp>& o) const
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        return vector2(vec3);
    }
    
    template <typename fp>
    void vector2<fp>::scale(fp b)
    {
        *this = *this * b;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>::dimensional_vector2()
    {
        sixit::units::dimensional_scalar<fp, dim> zero = sixit::units::dimensional_scalar<fp, dim>::zero();
        set(zero, zero);
    }
        
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>::dimensional_vector2(const dimensional_vector2<fp, dim>& o): x(o.x), y(o.y) {}
 
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>::dimensional_vector2(sixit::units::dimensional_scalar<fp, dim> x, 
                                                        sixit::units::dimensional_scalar<fp, dim> y) : x(x), y(y) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>::dimensional_vector2(const dimensional_vector3<fp, dim>& o) : x(o.x), y(o.y) {}

    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_vector2<fp, dim>::set(sixit::units::dimensional_scalar<fp, dim> x_, 
                                        sixit::units::dimensional_scalar<fp, dim> y_)
    {
        this->x = x_;
        this->y = y_;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator+=(const dimensional_vector2<fp, dim>& o)
    {
        x += o.x;
        y += o.y;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator+(const dimensional_vector2<fp, dim>& o) const
    {
        return dimensional_vector2(x + o.x, y + o.y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator+=(sixit::units::dimensional_scalar<fp, dim> o)
    {
        x += o;
        y += o;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator+(sixit::units::dimensional_scalar<fp, dim> o) const
    {
        return dimensional_vector2(x + o, y + o);
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator-=(const dimensional_vector2<fp, dim>& o)
    {
        x -= o.x;
        y -= o.y;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator-(const dimensional_vector2<fp, dim>& o) const
    {
        return dimensional_vector2(x - o.x, y - o.y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator-=(sixit::units::dimensional_scalar<fp, dim> o)
    {
        x -= o;
        y -= o;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator-(sixit::units::dimensional_scalar<fp, dim> o) const
    {
        return dimensional_vector2(x - o, y - o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator*=(const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& o)
    {
        x *= o.x;
        y *= o.y;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    inline const dimensional_vector2<fp, dim * dim_o> dimensional_vector2<fp, dim>::operator*(const dimensional_vector2<fp, dim_o>& o) const
    {
        return dimensional_vector2(x * o.x, y * o.y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        x *= o;
        y *= o;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    const dimensional_vector2<fp, dim * dim_o> dimensional_vector2<fp, dim>::operator*(const sixit::units::dimensional_scalar<fp, dim_o> o) const
    {
        return dimensional_vector2<fp, dim * dim_o>(x * o, y * o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator/=(const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& o)
    {
        x /= o.x;
        y /= o.y;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    const dimensional_vector2<fp, dim / dim_o> dimensional_vector2<fp, dim>::operator/(const dimensional_vector2<fp, dim_o>& o) const
    {
        return dimensional_vector2(x/o.x, y/o.y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        x /= o;
        y /= o;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    inline const dimensional_vector2<fp, dim / dim_o> dimensional_vector2<fp, dim>::operator/(const sixit::units::dimensional_scalar<fp, dim_o> o) const
    {
        return dimensional_vector2<fp, dim / dim_o>(x / o, y / o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline sixit::units::dimensional_scalar<fp, dim> dimensional_vector2<fp, dim>::operator[](int index) const
    {
        return index == 0 ? x : y;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator-() const
    {
        return dimensional_vector2(-x, -y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, dim> dimensional_vector2<fp, dim>::magnitude() const
    {
        return mathf::sqrt(x * x + y * y);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, dim * dim> dimensional_vector2<fp, dim>::sqr_magnitude() const
    {
        return x * x + y * y;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_vector2<fp, dim>::normalize()
    {
        *this = normalized();
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, sixit::units::simple_scalar::dim> dimensional_vector2<fp, dim>::normalized() const
    {
            auto m = magnitude();
            return dimensional_vector2<fp, sixit::units::simple_scalar::dim>(x / m, y / m);
    }

    /*template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::up()
    {
        return dimensional_vector2<fp, dim>{element_t::zero(), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::down()
    {
        return dimensional_vector2<fp, dim>{element_t::zero(), element_t(fp(-1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::left()
    {
        return dimensional_vector2<fp, dim>{element_t(fp(-1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::right()
    {
        return dimensional_vector2<fp, dim>{element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::one()
    {
        return dimensional_vector2<fp, dim>{element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }*/

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::zero()
    {
        return dimensional_vector2<fp, dim>{element_t::zero(), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> dimensional_vector2<fp, dim>::angle(
                                        const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b)
    {
        fp angle = (mathf::atan2(a.y, a.x) - mathf::atan2(b.y, b.x)).value;
        if (angle > fp(mathf::pi)) angle = angle - mathf::pi * 2;
        if (angle < fp(-mathf::pi)) angle = angle + mathf::pi * 2;
        return sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>(angle * mathf::rad_2_deg);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    sixit::units::dimensional_scalar<fp, dim * dim_o> dimensional_vector2<fp, dim>::dot(
                                        const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim_o>& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::max(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b)
    {
        return dimensional_vector2<fp, dim>{mathf::max(a.x, b.x), mathf::max(a.y, b.y)};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::min(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b)
    {
        return dimensional_vector2<fp, dim>{mathf::min(a.x, b.x), mathf::min(a.y, b.y)};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim_o>
    dimensional_vector2<fp, dim * dim_o> dimensional_vector2<fp, dim>::scale(const dimensional_vector2<fp, dim>& a, sixit::units::dimensional_scalar<fp, dim_o> b) 
    {
        return a * b;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator*=(const matrix4x4<fp>& o)
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        this->set(vec3.x(), vec3.y());
        this->x = vec3.x();
        this->y = vec3.y();
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator*(const matrix4x4<fp>& o) const
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        return vector2(vec3);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator*=(const matrix3x3<fp>& o)
    {
        fp x_ = o(0, 0) * x + o(1, 0) * y + o(2, 0);
        fp y_ = o(0, 1) * x + o(1, 1) * y + o(2, 1);
        fp z_ = o(0, 2) * x + o(1, 2) * y + o(2, 2);
        x = x_ / z_;
        y = y_ / z_;
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator*(const matrix3x3<fp>& o) const
    {
        fp x_ = o(0, 0) * x + o(1, 0) * y + o(2, 0);
        fp y_ = o(0, 1) * x + o(1, 1) * y + o(2, 1);
        fp z_ = o(0, 2) * x + o(1, 2) * y + o(2, 2);
        return dimensional_vector2<fp, dim>(x_ / z_, y_ / z_);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector2<fp, dim>& dimensional_vector2<fp, dim>::operator*=(const quaternion<fp>& o) //weird issue
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        this->set(vec3.x(), vec3.y());
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector2<fp, dim> dimensional_vector2<fp, dim>::operator*(const quaternion<fp>& o) const
    {
        vector3<fp> vec3;
        vec3.set(this->x, this->y, fp(0.0f));
        vec3 *= o;
        return dimensional_vector2<fp, dim>(vec3);
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_vector2<fp, dim>::scale(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b)
    {
        *this = *this * b;
    }


    template <typename fp>
    vector3<fp>::vector3(const gpu::vec4p<fp>& o): gpu::vec4p<fp>(o) {}
    
    template <typename fp>
    vector3<fp>::vector3(const gpu::vec4base<fp>& o) : gpu::vec4p<fp>(o) {}

    template <typename fp>
    vector3<fp>::vector3():gpu::vec4p<fp>(0.0f) {}

    template <typename fp>
    vector3<fp>::vector3(fp x, fp y, fp z): gpu::vec4p<fp>(x, y, z) {}

    template <typename fp>
    vector3<fp>::vector3(fp x, fp y, fp z, position_type_t): gpu::vec4p<fp>(x, y, z, 1.0f) {}

    template <typename fp>
    vector3<fp>::vector3(fp x, fp y, fp z, normal_type_t): gpu::vec4p<fp>(x, y, z, 0.0f) {}

    template <typename fp>
    vector3<fp>::vector3(fp x, fp y, fp z, fp w): gpu::vec4p<fp>(x, y, z, w) {}

    template <typename fp>
    vector3<fp>::vector3(const vector2<fp>& o): gpu::vec4p<fp>(o.x(), o.y(), 0) {}

    template <typename fp>
    vector3<fp>::operator vector2<fp>() const
    {
        return vector2<fp>(fp(vector3<fp>::x()), fp(vector3<fp>::y()));
    }

    template <typename fp>
    bool vector3<fp>::operator==(const vector3<fp>& o) const
    {
        return vector3<fp>::x() == o.x() && vector3<fp>::y() == o.y() && vector3<fp>::z() == o.z();
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator+=(const vector3<fp>& o)
    {
        gpu::vec4p<fp>::operator +=(o);
        return *this;
    }
    
    template <typename fp>
    const vector3<fp> vector3<fp>::operator+(const vector3<fp>& o) const
    {
        return gpu::operator+(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator+=(fp o)
    {
        gpu::vec4p<fp>::operator+=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator+(fp o) const
    {
        return gpu::operator+(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator-=(const vector3<fp>& o)
    {
        gpu::vec4p<fp>::operator-=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator-(const vector3<fp>& o) const
    {
        return gpu::operator-(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator-=(fp o)
    {
        gpu::vec4p<fp>::operator-=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator-(fp o) const
    {
        return gpu::operator-(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator*=(const vector3<fp>& o)
    {
        gpu::vec4p<fp>::operator*=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator*(const vector3<fp>& o) const
    {
        return gpu::operator*(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator*=(fp o)
    {
        gpu::vec4p<fp>::operator*=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator*(fp o) const
    {
        return gpu::operator*(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator/=(const vector3<fp>& o)
    {
        gpu::vec4p<fp>::operator/=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator/(const vector3<fp>& o) const
    {
        return gpu::operator/(*this, o);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator/=(fp o)
    {
        gpu::vec4p<fp>::operator/=(o);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator/(fp o) const
    {
        return gpu::operator/(*this, o);
    }
    
    template <typename fp>
    fp vector3<fp>::magnitude() const
    {
        return gpu::vec4p<fp>::magnitude();
    }

    template <typename fp>
    fp vector3<fp>::sqr_magnitude() const
    {
        return gpu::vec4p<fp>::sqr_magnitude();
    }

    template <typename fp>
    void vector3<fp>::normalize()
    {
        gpu::vec4p<fp>::normalize();
    }

    template <typename fp>
    vector3<fp> vector3<fp>::normalized() const
    {
        return gpu::vec4p<fp>::normalized();
    }

    template <typename fp>
    void vector3<fp>::set(fp x_, fp y_, fp z_)
    {
        this->x() = x_;
        this->y() = y_;
        this->z() = z_;
    }

    /*template <typename fp>
    vector3<fp> vector3<fp>::forward()
    {
        return vector3{0, 0, 1};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::back()
    {
        return vector3{0, 0, -1};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::up()
    {
        return vector3{0, 1, 0};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::down()
    {
        return vector3{0, -1, 0};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::left()
    {
        return vector3{-1, 0, 0};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::right()
    {
        return vector3{1, 0, 0};
    }

    template <typename fp>
    vector3<fp> vector3<fp>::one()
    {
        return vector3{1, 1, 1};
    }*/

    template <typename fp>
    vector3<fp> vector3<fp>::zero()
    {
        return vector3{0, 0, 0};
    }

    template <typename fp>
    fp vector3<fp>::angle(const vector3<fp>& a, const vector3<fp>& b)
    {
        fp denominator = mathf::sqrt(a.sqr_magnitude() * b.sqr_magnitude());
        fp div = vector3::dot(a, b) / denominator;

        if (!sixit::dmath::fp_traits<fp>::isfinite(div))
            return 0.f;

        fp dot = mathf::clamp(div, -1.0f, 1.0f);
        return mathf::acos(dot) * mathf::rad_2_deg;
    }

    template <typename fp>
    inline fp vector3<fp>::dot(const vector3<fp>& a, const vector3<fp>& b)
    {
        return gpu::vec4p<fp>::dot(a, b);
    }

    template <typename fp>
    inline vector3<fp> vector3<fp>::cross(const vector3<fp>& a, const vector3<fp>& b)
    {
        return gpu::vec4base<fp>::cross(a, b);
    }

    template <typename fp>
    inline vector3<fp> vector3<fp>::max(const vector3<fp>& a, const vector3<fp>& b)
    {
        return gpu::vec4p<fp>::max(a, b);
    }

    template <typename fp>
    inline vector3<fp> vector3<fp>::min(const vector3<fp>& a, const vector3<fp>& b)
    {
        return gpu::vec4p<fp>::min(a, b);
    }

    template <typename fp>
    vector3<fp> vector3<fp>::scale(const vector3<fp>& a, fp b)
    {
        return gpu::vec4p<fp>::scale3(a, b);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator*=(const matrix4x4<fp>& o)
    {
        *this = gpu::operator*((const gpu::mat4<fp>&) o, *this);
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator*(const matrix4x4<fp>& o) const
    {
        return gpu::operator*((const gpu::mat4<fp>&) o, *this);
    }

    template <typename fp>
    vector3<fp>& vector3<fp>::operator*=(const quaternion<fp>& o)
    {
        vector3<fp> t(vector3<fp>::x(), vector3<fp>::y(), vector3<fp>::z(), 0);
        vector3<fp> q(o.x,o.y,o.z,0);
        *this += cross(q, cross(q, *this) + t * o.w) * 2.0f;
        return *this;
    }

    template <typename fp>
    const vector3<fp> vector3<fp>::operator*(const quaternion<fp>& o) const
    {
        vector3<fp> t(vector3<fp>::x(), vector3<fp>::y(), vector3<fp>::z(), 0);
        vector3<fp> q(o.x,o.y,o.z,0);
        return *this + cross(q, cross(q,*this) + t * o.w) * 2.0f;
        // old impl: return *this + cross(cross(*this,q) + t * o.w, q) * 2.0f;
    }

    template <typename fp>
    void vector3<fp>::scale(fp b_)
    {
        gpu::vec4p<fp>::scale3(b_);
    }


    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(const gpu::vec4p<fp>& o): gpu::vec4p<fp>(o) {}
    
    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(const gpu::vec4base<fp>& o) : gpu::vec4p<fp>(o) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3():gpu::vec4p<fp>(0.0f) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z): gpu::vec4p<fp>(x.value, y.value, z.value) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z, position_type_t): gpu::vec4p<fp>(x.value, y.value, z.value, 1.0f) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z, normal_type_t): gpu::vec4p<fp>(x.value, y.value, z.value, 0.0f) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z, sixit::units::dimensional_scalar<fp, dim> w): gpu::vec4p<fp>(x.value, y.value, z.value, w.value) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::dimensional_vector3(const dimensional_vector2<fp, dim>& o): gpu::vec4p<fp>(o.x(), o.y(), 0) {}

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>::operator dimensional_vector2<fp, dim>() const
    {
        return _dimnesional_vector2<fp, dim>(dimensional_vector3<fp,dim>::x()), dimensional_vector3<fp, dim>::y();
    }

    template <typename fp, sixit::units::physical_dimension dim>
    bool dimensional_vector3<fp, dim>::operator==(const dimensional_vector3<fp, dim>& o) const
    {
        return dimensional_vector3<fp, dim>::x() == o.x() && 
                dimensional_vector3<fp, dim>::y() == o.y() && 
                dimensional_vector3<fp, dim>::z() == o.z();
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator+=(const dimensional_vector3<fp, dim>& o)
    {
        gpu::vec4p<fp>::operator +=(o);
        return *this;
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::operator+(const dimensional_vector3<fp, dim>& o) const
    {
        return gpu::operator+(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator+=(sixit::units::dimensional_scalar<fp, dim> o)
    {
        gpu::vec4p<fp>::operator+=(o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::operator+(sixit::units::dimensional_scalar<fp, dim> o) const
    {
        return gpu::operator+(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator-=(const dimensional_vector3<fp, dim>& o)
    {
        gpu::vec4p<fp>::operator-=(o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::operator-(const dimensional_vector3<fp, dim>& o) const
    {
        return gpu::operator-(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator-=(sixit::units::dimensional_scalar<fp, dim> o)
    {
        gpu::vec4p<fp>::operator-=(o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::operator-(sixit::units::dimensional_scalar<fp, dim> o) const
    {
        return gpu::operator-(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator*=(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& o)
    {
        gpu::vec4p<fp>::operator*=(o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    const dimensional_vector3<fp, dim * dim1> dimensional_vector3<fp, dim>::operator*(const dimensional_vector3<fp, dim1>& o) const
    {
        return gpu::operator*(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        gpu::vec4p<fp>::operator*=(o.value);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    const dimensional_vector3<fp, dim * dim1> dimensional_vector3<fp, dim>::operator*(sixit::units::dimensional_scalar<fp, dim1> o) const
    {
        return gpu::operator*(*this, o.value);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator/=(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& o)
    {
        gpu::vec4p<fp>::operator/=(o);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    const dimensional_vector3<fp, dim / dim1> dimensional_vector3<fp, dim>::operator/(const dimensional_vector3<fp, dim1>& o) const
    {
        return gpu::operator/(*this, o);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        gpu::vec4p<fp>::operator/=(o.value);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    const dimensional_vector3<fp, dim / dim1> dimensional_vector3<fp, dim>::operator/(sixit::units::dimensional_scalar<fp, dim1> o) const
    {
        return gpu::operator/(*this, o.value);
    }
    
    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, dim> dimensional_vector3<fp, dim>::magnitude() const
    {
        return sixit::units::dimensional_scalar<fp, dim>({ gpu::vec4p<fp>::magnitude(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, dim * dim> dimensional_vector3<fp, dim>::sqr_magnitude() const
    {
        return sixit::units::dimensional_scalar<fp, dim * dim>({ gpu::vec4p<fp>::sqr_magnitude(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, sixit::units::simple_scalar::dim> dimensional_vector3<fp, dim>::normalized() const
    {
        return gpu::vec4p<fp>::normalized();
    }

    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_vector3<fp, dim>::set(sixit::units::dimensional_scalar<fp, dim> x_, 
                                            sixit::units::dimensional_scalar<fp, dim> y_, 
                                            sixit::units::dimensional_scalar<fp, dim> z_)
    {
        this->x() = x_;
        this->y() = y_;
        this->z() = z_;
    }

    /*template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::forward()
    {
        return dimensional_vector3{element_t::zero(), element_t::zero(), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::back()
    {
        return dimensional_vector3{element_t::zero(), element_t::zero(), element_t(fp(-1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::up()
    {
        return dimensional_vector3{element_t::zero(), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::down()
    {
        return dimensional_vector3{element_t::zero(), element_t(fp(-1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::left()
    {
        return dimensional_vector3{element_t(fp(-1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero(), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::right()
    {
        return dimensional_vector3{element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t::zero(), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::one()
    {
        return dimensional_vector3{element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()), element_t(fp(1.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp())};
    }*/

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::zero()
    {
        return dimensional_vector3{element_t::zero(), element_t::zero(), element_t::zero()};
    }

    template <typename fp, sixit::units::physical_dimension dim>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> dimensional_vector3<fp, dim>::angle(
        const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim>& b)
    {
        auto denominator = mathf::sqrt(a.sqr_magnitude() * b.sqr_magnitude());
        if (denominator < mathf::epsilon)
        {
            return 0.0f;
        }

        auto dot = mathf::clamp(dimensional_vector3::dot(a, b) / denominator, fp(-1.0f), fp(1.0f));
        return mathf::acos(dot) * mathf::rad_2_deg;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    inline sixit::units::dimensional_scalar<fp, dim * dim1> dimensional_vector3<fp, dim>::dot(
                                const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim1>& b)
    {
        return sixit::units::dimensional_scalar<fp, dim* dim1>({ gpu::vec4p<fp>::dot(a, b), sixit::units::internal_constructor_of_dimensional_scalar_from_fp()});
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    inline dimensional_vector3<fp, dim * dim1> dimensional_vector3<fp, dim>::cross(
                                    const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim1>& b)
    {
        return gpu::vec4base<fp>::cross(a, b);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::max(
                                    const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim>& b)
    {
        return gpu::vec4p<fp>::max(a, b);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::min(
                                    const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim>& b)
    {
        return gpu::vec4p<fp>::min(a, b);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    dimensional_vector3<fp, dim * dim1> dimensional_vector3<fp, dim>::scale(
                                    const dimensional_vector3<fp, dim>& a, sixit::units::dimensional_scalar<fp, dim1> b)
    {
        return gpu::vec4p<fp>::scale3(a, b);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator*=(const matrix4x4<fp>& o)
    {
        *this = gpu::operator*((const gpu::mat4<fp>&) o, *this);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    const dimensional_vector3<fp, dim> dimensional_vector3<fp, dim>::operator*(const matrix4x4<fp>& o) const
    {
        return gpu::operator*((const gpu::mat4<fp>&) o, *this);
    }

    template <typename fp, sixit::units::physical_dimension dim>
    dimensional_vector3<fp, dim>& dimensional_vector3<fp, dim>::operator*=(const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& o)
    {

        dimensional_vector3<fp, dim> t(dim_x(), dim_y(), dim_z(), sixit::units::dimensional_scalar<fp, dim>::zero());
        dimensional_vector3<fp, sixit::units::simple_scalar::dim> q(sixit::units::create_dimensionless_scalar<fp>(o.x()), sixit::units::create_dimensionless_scalar<fp>(o.y()), sixit::units::create_dimensionless_scalar<fp>(o.z()), sixit::units::create_dimensionless_scalar<fp>(fp(0.f)));
        *this += cross(q, cross(q, *this) + t * sixit::units::create_dimensionless_scalar<fp>(o.w())) * sixit::units::create_dimensionless_scalar<fp>(2.0f);
        return *this;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    template <sixit::units::physical_dimension dim1>
    const dimensional_vector3<fp, dim * dim1> dimensional_vector3<fp, dim>::operator*(const dimensional_quaternion<fp, dim1>& o) const
    {
        dimensional_vector3<fp, dim> t(dim_x(), dim_y(), dim_z(), sixit::units::dimensional_scalar<fp, dim>::zero());
        dimensional_vector3<fp, dim1> q(o.dim_x(), o.dim_y(), o.dim_z(), sixit::units::dimensional_scalar<fp, dim1>::zero());
        return *this * sixit::geometry::low_level::mathf::make_constant_from<sixit::units::dimensional_scalar<fp, dim1>, 1.f>() + cross(q, cross(q, *this) + t * o.dim_w()) * sixit::units::create_dimensionless_scalar<fp>(fp(2.0f));
        // old impl: return *this + cross(cross(*this,q) + t * o.w, q) * 2.0f;
    }

    template <typename fp, sixit::units::physical_dimension dim>
    void dimensional_vector3<fp, dim>::scale(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b_)
    {
        gpu::vec4p<fp>::scale3(b_);
    }
}; // namespace low_level
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_vector_impl_h_included

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
