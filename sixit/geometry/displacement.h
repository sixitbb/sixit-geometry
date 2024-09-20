/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_displacement_h_included
#define sixit_geometry_displacement_h_included

#include "sixit/geometry/units.h"

#include <cmath>

namespace sixit
{
namespace geometry
{

template <typename fp>
class displacement3
{
  public:
    explicit displacement3(meters<fp> x, meters<fp> y, meters<fp> z) : x_(x), y_(y), z_(z) { }
    explicit displacement3(centimeters<fp> x, centimeters<fp> y, centimeters<fp> z) : x_(meters(x)), y_(meters(y)), z_(meters(z)) { }
    explicit displacement3(inches<fp> x, inches<fp> y, inches<fp> z) : x_(meters(x)), y_(meters(y)), z_(meters(z)) { }

    template <typename DistanceUnit = meters<fp>>
    DistanceUnit distance() const
    {
        const fp x = static_cast<fp>(x_);
        const fp y = static_cast<fp>(y_);
        const fp z = static_cast<fp>(z_);

        const meters<fp> dist_in_meters = meters(std::hypot(x, y, z));

        return DistanceUnit(dist_in_meters);
    }

  private:
    // Getter functions for users to retrieve values in desired distance units
    template <typename T>
    T get_x() const { return T(x_); }

    template <typename T>
    T get_y() const { return T(y_); }

    template <typename T>
    T get_z() const { return T(z_); }

    // Setter functions to set values in desired distance units
    template <typename T>
    void set_x(const T& x) { x_ = meters<fp>(x); }

    template <typename T>
    void set_y(const T& y) { y_ = meters<fp>(y); }

    template <typename T>
    void set_z(const T& z) { z_ = meters<fp>(z); }

  private:
    meters<fp> x_, y_, z_;
};

}; // namespace geometry

namespace geometry 
{
  using displacement3f = displacement3<float>;
}

}; // namespace sixit

#endif //sixit_geometry_displacement_h_included

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