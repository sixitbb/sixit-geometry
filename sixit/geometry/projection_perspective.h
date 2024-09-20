/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_projection_perspective_h_included
#define sixit_geometry_projection_perspective_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/matrix.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/points_span.h"
#include "sixit/core/units.h"

#include <vector>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct projection_perspective3
    {
        friend struct inverse_projection_perspective3<fp>;
        template <typename fp1>
        friend low_level::matrix4x4<fp1> low_level::for_renderers_only::get_camera_projection_matrix4x4_dont_use_outside_of_renderer(const projection_perspective3<fp1>& proj,
            const point3<fp1>& position, const direction3<fp1>& direction, const meters<fp1>& focus_disctance);

      private:
        low_level::matrix4x4<fp> mat;

      public:
        inline projection_perspective3(fp fov, fp aspect, fp znear, fp zfar)
        {
            mat = low_level::matrix4x4<fp>::perspective(fov, aspect, znear, zfar);
        }

        inline point3<fp> project(const point3<fp>& p)
        {
            low_level::vector3<fp> tmp(p.vec().x(), p.vec().y(), p.vec().z());
            low_level::vector3 pp = mat.multiply_point3x4(tmp);
            pp = pp / pp.w();
            return {
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(pp.x()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(pp.y()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(pp.z())
            };
        }

        template <typename T>
        T project(const T& o)
        {
            using vlo = std::vector<point3<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = project(points_span[i]);
            }
            return T(points_span);
        }
    };

    template <typename fp>
    struct projection_perspective2
    {
      private:
        low_level::matrix3x3<fp> mat;

      public:
        inline projection_perspective2(fp fov, fp aspect, fp znear, fp zfar)
        {
            mat = low_level::matrix3x3<fp>::perspective(fov, aspect, znear, zfar);
        }

        inline point2<fp> project(const point2<fp>& p)
        {
            sixit::geometry::low_level::vector2<fp> tmp(
                                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().x), 
                                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().y));
            tmp = mat.multiply_point2x3(tmp);

            return point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.x),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.y));
        }

        template <typename T>
        inline T project(const T& o)
        {
            using vlo = std::vector<point2<fp>>;
            vlo points_span = o.template _get_projection_points<vlo>();
            for (size_t i = 0; i < points_span.size(); ++i)
            {
                points_span[i] = project(points_span[i]);
            }
            return T(points_span);
        }
    };

    template <typename fp>
    struct inverse_projection_perspective3
    {
      private:
        low_level::matrix4x4<fp> mat_inverse;
      public:
        inverse_projection_perspective3(const projection_perspective3<fp>& projection_)
            : mat_inverse(projection_.mat.inverse())
        {}

        line3<fp> unproject(const point2<fp>& p)
        {
            auto point = point3<fp>(
                p.vec().x, p.vec().y, 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f));
            
            low_level::vector3<fp> tmp(point.vec().x(), point.vec().y(), point.vec().z(), low_level::vector3<fp>::position_type);
            tmp = mat_inverse.multiply_point3x4(tmp);
            return line3(point3<fp>(), point3<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.x()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.y()),
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(tmp.z())
            ));
        }
    };

} // namespace geometry

namespace geometry 
{
    // aliases 
    using projection_perspective3f = projection_perspective3<float>;
    using projection_perspective2f = projection_perspective2<float>;
    using inverse_projection_perspective3f = inverse_projection_perspective3<float>;
}

} // namespace sixit

#endif //sixit_geometry_projection_perspective_h_included

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
