/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_levelcomplex_number_rotation_impl_h_included
#define sixit_geometry_low_levelcomplex_number_rotation_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template<typename fp>
    complex_number_rotation<fp>::complex_number_rotation()
    {

    }

    template<typename fp>
    complex_number_rotation<fp>::complex_number_rotation(
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> _a, 
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> _b) : a(_a), b(_b)
    {

    }

    template<typename fp>
    inline complex_number_rotation<fp> complex_number_rotation<fp>::by_angle(
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle)
    {
        return complex_number_rotation(low_level::mathf::cos(
            angle * sixit::units::create_dimensionless_scalar<fp>(mathf::deg_2_rad)), 
            low_level::mathf::sin(angle * sixit::units::create_dimensionless_scalar<fp>(mathf::deg_2_rad)));
    }

    template<typename fp>
    inline complex_number_rotation<fp> complex_number_rotation<fp>::identity()
    {
        return complex_number_rotation(sixit::units::create_dimensionless_scalar<fp>(fp(1.0f)), sixit::units::create_dimensionless_scalar<fp>(fp(0.0f)));//rot by 0 deg
    }

    template<typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> complex_number_rotation<fp>::angle() const
    {
        return atan2(b, a);
    }

    template<typename fp>
    void complex_number_rotation<fp>::set(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> _a, 
                                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> _b)
    {
        a = _a;
        b = _b;
    }

    template<typename fp>
    void complex_number_rotation<fp>::normalize()
    {
        fp norm = dot(*this, *this);
        a /= norm;
        b /= norm;
    }

    template<typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> complex_number_rotation<fp>::dot(const complex_number_rotation<fp>& r1, const complex_number_rotation<fp>& r2)
    {
        return r1.a * r2.a + r1.b * r2.b;
    }

    template<typename fp>
    inline complex_number_rotation<fp> complex_number_rotation<fp>::inverse(const complex_number_rotation<fp>& r)
    {
        return complex_number_rotation(r.a, -r.b);//consinered normalized
    }

    template<typename fp>
    inline complex_number_rotation<fp> complex_number_rotation<fp>::from_to_rotation(
        const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& from_direction, 
        const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& to_direction)
    {
        return complex_number_rotation<fp>
        (
            to_direction.x * from_direction.x + to_direction.y * from_direction.y,
            to_direction.y * from_direction.x - to_direction.x * from_direction.y
        );
    }

    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator*(const complex_number_rotation<fp>& o)
    {
        return complex_number_rotation(a*o.a - b*o.b, a * o.b + b * o.a);
    }

    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator*(const complex_number_rotation<fp>& o) const
    {
        return complex_number_rotation(a*o.a - b*o.b, a * o.b + b * o.a);
    }

    template<typename fp>
    complex_number_rotation<fp>& complex_number_rotation<fp>::operator*=(const complex_number_rotation<fp>& o)
    {
        *this = operator*(o);
        return *this;
    }

    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator+(const complex_number_rotation<fp>& o)
    {
        return complex_number_rotation(a + o.a, b + o.b);
    }

    template<typename fp>
    complex_number_rotation<fp>& complex_number_rotation<fp>::operator+=(const complex_number_rotation<fp>& o)
    {
        *this = operator+(o);
        return *this;
    }

    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator-(const complex_number_rotation<fp>& o)
    {
        return complex_number_rotation(a - o.a, b - o.b);
    }

    template<typename fp>
    complex_number_rotation<fp>& complex_number_rotation<fp>::operator-=(const complex_number_rotation<fp>& o)
    {
        *this = operator-(o);
        return *this;
    }

    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator*(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        return complex_number_rotation(a * o, b * o);
    }

    template<typename fp>
    complex_number_rotation<fp>& complex_number_rotation<fp>::operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        *this = operator*(o);
        return *this;
    }
    
    template<typename fp>
    complex_number_rotation<fp> complex_number_rotation<fp>::operator/(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        return complex_number_rotation(a / o, b / o);
    }

    template<typename fp>
    complex_number_rotation<fp>& complex_number_rotation<fp>::operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o)
    {
        *this = operator/(o);
        return *this;
    }

};
};
};

#endif //sixit_geometry_low_levelcomplex_number_rotation_impl_h_included

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