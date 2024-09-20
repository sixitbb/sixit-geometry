/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_for_importer_h_included
#define sixit_geometry_low_level_for_importer_h_included

#include <tuple>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/rotation.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
namespace for_importer_only
{

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const point3<fp>& p)
{
    return std::make_tuple(p.vec().x(), p.vec().y(), p.vec().z());
}

template <typename fp>
inline static point3<fp> make_point3_dont_use_outside_of_importer(fp x, fp y, fp z)
{
    return point3<fp>(
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(z));
}

template <typename fp>
inline static point2<fp> make_point2_dont_use_outside_of_importer(fp x, fp y)
{
    return point2<fp>(
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y));
}

template <typename fp>
inline static direction3<fp> make_direction3_dont_use_outside_of_importer(fp x, fp y, fp z)
{
    return direction3<fp>(
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(x), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(y), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(z));
}

template <typename fp>
inline static tangent<fp> make_tangent_dont_use_outside_of_importer(fp x, fp y, fp z, handedness w)
{
    return {x, y, z, w};
}

template <typename fp>
inline static std::tuple<fp, fp, fp, handedness> xyzw_dont_use_outside_of_importer(const tangent<fp>& t)
{
    return std::make_tuple(t.dir3_.vec().x(), t.dir3_.vec().y(), t.dir3_.vec().z(), t.handedness_);
}

template <typename fp>
inline static rotation3<fp> make_rotation3_dont_use_outside_of_importer(fp x, fp y, fp z, fp w)
{
    rotation3<fp> rot;
    rot.q = dimensional_quaternion<fp, sixit::units::simple_scalar::dim>(
        sixit::units::create_dimensionless_scalar<fp>(w), 
        sixit::units::create_dimensionless_scalar<fp>(x), 
        sixit::units::create_dimensionless_scalar<fp>(y), 
        sixit::units::create_dimensionless_scalar<fp>(z));
    return rot;
}

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const geometry::translation3<fp>& tr3)
{
    return std::make_tuple(tr3.vec.x(), tr3.vec.y(), tr3.vec.z());
}

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const geometry::scale3<fp>& sc3)
{
    return std::make_tuple(sc3.vec.x(), sc3.vec.y(), sc3.vec.z());
}

template <typename fp>
inline static std::tuple<fp, fp, fp, fp> xyzw_dont_use_outside_of_importer(const geometry::rotation3<fp>& rt3)
{
    return std::make_tuple(rt3.q.x(), rt3.q.y(), rt3.q.z(), rt3.q.w());
}

}
}
}
}

#endif //sixit_geometry_low_level_for_importer_h_included

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