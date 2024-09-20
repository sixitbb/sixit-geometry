/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_for_renderers_h_included
#define sixit_geometry_low_level_for_renderers_h_included

#include <tuple>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/low_level/matrix.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
namespace for_renderers_only
{

template <typename fp>
inline static gpu::mat4<fp> matrix_dont_use_outside_of_renderer(const trs3<fp>& trs)
{
#ifdef TRS_AS_MATRIX4X4
    return trs3.mat;
#else
    return gpu::mat4<fp>::compose(trs.translation.vec, trs.rotation.q, trs.scale.vec);
#endif
	
}

//need to be review
template <typename fp>
inline static matrix4x4<fp> get_camera_projection_matrix4x4_dont_use_outside_of_renderer(const projection_perspective3<fp>& proj,
    const point3<fp>& position, const direction3<fp>& direction, const sixit::geometry::meters<fp>& focus_distance)
{
    point3 center_direction = direction3<fp>::move_point(position, direction, focus_distance);
    std::tuple<fp, fp, fp> center_xyz = xyz_dont_use_outside_of_renderer(center_direction);
    gpu::vec4p center(std::get<0>(center_xyz), std::get<1>(center_xyz), std::get<2>(center_xyz), 1.0f);
    gpu::vec4p up = gpu::vec4p(0.0f, 1.0f, 0.0f);
    std::tuple<fp, fp, fp> pos_xyz = xyz_dont_use_outside_of_renderer(position);
    gpu::vec4p eye_position(std::get<0>(pos_xyz), std::get<1>(pos_xyz), std::get<2>(pos_xyz), 1.0f);
    matrix4x4<fp> view_matrix = matrix4x4<fp>::look_at(eye_position, center, up);

    return proj.mat * view_matrix;
}

// possibly needed camera trs as well
template <typename fp>
inline static bounds3<fp> bounds3_projected_dont_use_outside_of_renderer(const trs3<fp>& trs, const bounds3<fp>& bounds)
{
    auto mmin = trs.transform(bounds.minn);
    auto mmax = trs.transform(bounds.maxx);
    return bounds3(mmin, mmax);
}

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const point3<fp>& p)
{
    return std::make_tuple(p.vec().x(), p.vec().y(), p.vec().z());
}

template <typename fp>
inline static std::tuple<fp, fp, fp, fp> xyzw_dont_use_outside_of_renderer(const rotation3<fp>& r)
{
    // const low_level::quat
    return std::make_tuple(r.q.x, r.q.y, r.q.z, r.q.w);
}

template <typename fp>
inline static std::tuple<fp, fp> xy_dont_use_outside_of_renderer(const point2<fp>& p)
{
    return std::make_tuple(
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().x), 
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().y));
}

template <typename fp>
inline static std::tuple<fp, fp, fp, fp> xy_dont_use_outside_of_renderer(const line_segment2<fp>& l)
{
    return std::make_tuple(
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(l.p1().vec().x), 
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(l.p1().vec().y), 
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(l.p2().vec().x), 
        sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(l.p2().vec().y));
}

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const direction3<fp>& d)
{
    return std::make_tuple(d.vec().x(), d.vec().y(), d.vec().z());
}

template <typename fp>
inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const tangent<fp>& t)
{
    return std::make_tuple(t.dir3_.vec().x(), t.dir3_.vec().y(), t.dir3_.vec().z());
}

template <typename fp>
inline static point3<fp> make_point3_dont_use_outside_of_renderer(fp x, fp y, fp z)
{
    return point3<fp>(
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(z));
}

template <typename fp>
inline static point2<fp> make_point2_dont_use_outside_of_renderer(fp x, fp y)
{
    return point2<fp>(
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
        sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y));
}

template <typename fp>
inline static rotation3<fp> make_rotation3_dont_use_outside_of_renderer(fp x, fp y, fp z, fp w)
{
    rotation3<fp> rot;
    rot.q = quaternion(w, x, y, z);
    return rot;
}

template <typename fp>
inline static tangent<fp> make_tangent_dont_use_outside_of_renderer(fp x, fp y, fp z, handedness w)
{
    tangent<fp> tang(x, y, z, w);
    return tang;
}
}
}
}
}

#endif //sixit_geometry_low_level_for_renderers_h_included

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
