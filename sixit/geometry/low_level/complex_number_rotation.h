/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_complex_number_rotation_h_included
#define sixit_geometry_low_level_complex_number_rotation_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/matrix.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
template <typename fp>
struct complex_number_rotation
{
    friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation2<fp>>;

    template <typename fp1>
    friend struct ::sixit::geometry::rotation2;
    friend struct ::sixit::geometry::trs2<fp>;

  private:
    inline complex_number_rotation();
    inline complex_number_rotation(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a, sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b);

    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a;
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b;

    static complex_number_rotation by_angle(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle);

    static complex_number_rotation identity();
    inline sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle() const;
    inline complex_number_rotation normalized() const;
    inline void set(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> a, 
                      sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b);
    inline void normalize();
    inline static sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> dot(
                          const complex_number_rotation<fp>& a, const complex_number_rotation<fp>& b);
    static complex_number_rotation<fp> inverse(const complex_number_rotation<fp>& a);

    // TODO: test this
    static complex_number_rotation<fp> from_to_rotation(
              const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& from_direction, 
              const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& to_direction);

    inline complex_number_rotation<fp> operator*(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp> operator*(const complex_number_rotation<fp>& other) const;
    inline complex_number_rotation<fp>& operator*=(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp> operator+(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp>& operator+=(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp> operator-(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp>& operator-=(const complex_number_rotation<fp>& o);
    inline complex_number_rotation<fp> operator*(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
    inline complex_number_rotation<fp>& operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
    inline complex_number_rotation<fp> operator/(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
    inline complex_number_rotation<fp>& operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
};

}; // namespace low_level

namespace low_level 
{
    // aliases 
    using complex_number_rotationf = complex_number_rotation<float>;
}

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_leve_lcomplex_number_rotation_h_included

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
