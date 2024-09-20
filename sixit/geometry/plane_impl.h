/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_plane_impl_h_included
#define sixit_geometry_plane_impl_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"

#pragma warning(push)
#pragma warning(disable : 4244 4456)
#include "3rdparty/jama/jama_svd.h"
#pragma warning(pop)
//#include <cmath>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    plane3<fp>::plane3(const direction3<fp>& normal, const point3<fp>& point)
        : normal(normal), point(point)
    {}

    template <typename fp>
    std::tuple<fp, fp, fp, fp> plane3<fp>::abcd() const
    {
        fp a = normal.vec().x();
        fp b = normal.vec().y();
        fp c = normal.vec().z();
        fp d = -low_level::vector3<fp>::dot(normal.vec(), point.vec());
        return std::make_tuple(a, b, c, d);
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> plane3<fp>::distance(const point3<fp>& p) const
    {
        return low_level::mathf::abs(low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(normal.vec(), p.vec() - point.vec()));
    }

    template <typename fp>
    bool plane3<fp>::side(const point3<fp>& p) const
    {
        low_level::vector3 r = point.vec() - p.vec();
        r.normalize();
        return low_level::vector3<fp>::dot(r, normal.vec()) > fp(0.0f);
    }

    // todo: test this
    template <typename fp>
    plane3<fp> plane3<fp>::orthogonal_plane(const line3<fp>& line) const
    {
        low_level::dimensional_vector3 d1 = line.p1().vec() - point.vec();
        low_level::dimensional_vector3 d2 = line.p2().vec() - point.vec();
        low_level::dimensional_vector3 normal_ = {
            sixit::units::create_dimensionless_scalar<fp>(d1.y() * d2.z() - d1.x() * d2.y()),
            sixit::units::create_dimensionless_scalar<fp>(d1.z() * d2.x() - d1.x() * d2.z()),
            sixit::units::create_dimensionless_scalar<fp>(d1.x() * d2.y() - d1.y() * d2.x())
        };
        normal_.normalize();
        plane3 res = plane3(direction3(
            sixit::units::create_dimensionless_scalar<fp>(-normal_.y()), 
            sixit::units::create_dimensionless_scalar<fp>(normal_.x()), 
            sixit::units::create_dimensionless_scalar<fp>(normal_.z())), line.p1());
        return res;
    }

    template <typename fp>
    line3<fp> plane3<fp>::orthogonal_line(const point3<fp>& p1) const
    {
        point3 p2 = point3(p1.vec() + normal.vec());
        return line3(p1, p2);
    }

    //std::optional<line3> plane3::intersection(const plane3& o) const
    //{
    //    low_level::vector3 direction = low_level::vector3::cross(normal.vec(), o.normal.vec());
    //    if (direction.sqr_magnitude() <= low_level::mathf::epsilon)
    //    {
    //        return {};
    //    }

    //    direction.normalize();

    //    low_level::vector3 diff = o.point.vec() - point.vec();
    //    float d1 = low_level::vector3::dot(normal.vec(), diff);
    //    float d2 = low_level::vector3::dot(o.normal.vec(), diff);
    //    float denom = low_level::vector3::dot(normal.vec(), o.normal.vec());
    //    low_level::vector3 p1 = o.normal.vec() * d1;
    //    low_level::vector3 p2 = normal.vec() * d2;
    //    return line3(point3((p1 - p2) / denom), point3(direction));
    //}
    //
    //inline std::optional<line3> plane3::intersection_gems(const plane3& plane) const
    //{
    //    static_assert( std::numeric_limits<float>::is_iec559 );
    //    //https://mathworld.wolfram.com/Plane-PlaneIntersection.html
    //    //https://stackoverflow.com/questions/6408670/line-of-intersection-between-two-planes
    //    //check if the planes are not parallel

    //    low_level::vector3 line_dir = low_level::vector3::cross(normal.vec(), plane.normal.vec());
    //    auto sq_norm = line_dir.sqr_magnitude();
    //    if (sq_norm <= low_level::mathf::epsilon)//discussion point
    //    {
    //        return {};
    //    }

    //    //find distances d1, d2 to the origin to get Hessian normal form
    //    float d1 = -low_level::vector3::dot(normal.vec(), point.vec());
    //    float d2 = -low_level::vector3::dot(plane.normal.vec(), plane.point.vec());
    //    
    //    //find basic directions
    //    low_level::vector3 line_to_plane2 = low_level::vector3::cross(line_dir, plane.normal.vec());
    //    low_level::vector3 plane1_to_line = low_level::vector3::cross(normal.vec(), line_dir);

    //    low_level::vector3 p1 = (line_to_plane2 * d1 + plane1_to_line * d2) / sq_norm;
    //    low_level::vector3 p2 = p1 + line_dir;

    //    return line3(p1, p2);
    //}

    template <typename fp>
    inline std::optional<line3<fp>> plane3<fp>::intersection(const plane3<fp>& plane) const
    {
        static_assert(std::numeric_limits<float>::is_iec559);
        //https://mathworld.wolfram.com/Plane-PlaneIntersection.html
        //check if the planes are not parallel

        low_level::dimensional_vector3 line_dir = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(normal.vec(), plane.normal.vec());
        // auto sq_norm = line_dir.sqr_magnitude();

        //if (sq_norm <= low_level::mathf::epsilon)//discussion point
        //{
         //   return {};
       // }

        //find distances d1, d2 to the origin to get Hessian normal form
        auto d1 = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(normal.vec(), point.vec());
        auto d2 = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(plane.normal.vec(), plane.point.vec());

        int max_coord = 2;
        if (low_level::mathf::abs(line_dir[0]) > low_level::mathf::abs(line_dir[1]) &&
              low_level::mathf::abs(line_dir[0]) > low_level::mathf::abs(line_dir[2]))
            max_coord = 0;
        else if (low_level::mathf::abs(line_dir[1]) > low_level::mathf::abs(line_dir[0]) && 
              low_level::mathf::abs(line_dir[1]) > low_level::mathf::abs(line_dir[2]))
            max_coord = 1;

        low_level::dimensional_vector3 n1 = normal.vec();
        low_level::dimensional_vector3 n2 = plane.normal.vec();

        int map_coords[3][2] = { {1, 2}, {0, 2}, {0, 1} };
        auto max_map = map_coords[max_coord];

        int coord[2] = { 0, 1 };
        auto a = sixit::units::create_dimensionless_scalar<fp>(n1[max_map[0]]);
        auto b = sixit::units::create_dimensionless_scalar<fp>(n1[max_map[1]]);
        auto c = sixit::units::create_dimensionless_scalar<fp>(n2[max_map[0]]);
        auto d = sixit::units::create_dimensionless_scalar<fp>(n2[max_map[1]]);

        auto det = a * d - b * c;//shouldn't be zero, if the planes are not parallel

        auto x1 =  ( d * d1 - b * d2) / det;
        auto x2 =  (-c * d1 + a * d2) / det;

        
        if (!sixit::dmath::mathf::isfinite(x1) || !sixit::dmath::mathf::isfinite(x2))
            return {};

        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> res;
        res[max_coord]  = 0.0f;
        res[max_map[0]] = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(x1);
        res[max_map[1]] = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(x2);

        return line3<fp>(point3<fp>(res), point3<fp>(res + line_dir));
    }

    template<typename VLO> plane3<float> best_fit_plane3_vlo(const VLO& vlo)
    {
        assert(vlo.get_size() >= 3);

        Array2D<float> matrix(vlo.get_size(), 3);

        low_level::vector3 avg = vlo[0].vector;

        for (size_t i = 1; i < vlo.get_size(); ++i)
            avg += vlo[i].vector;
        avg /= float(vlo.get_size());

        for (size_t i = 0; i < vlo.get_size(); ++i)
        {
            auto sft = vlo[i].vector - avg;
            matrix[i][0] = sft[0];
            matrix[i][1] = sft[1];
            matrix[i][2] = sft[2];
        }

        JAMA::SVD svd(matrix);

        Array2D<float> V;
        svd.getV(V);

        direction3<float> normal(
            sixit::units::create_dimensionless_scalar<float>(V[0][2]), 
            sixit::units::create_dimensionless_scalar<float>(V[1][2]), 
            sixit::units::create_dimensionless_scalar<float>(V[2][2]));
        point3 point(avg.x(), avg.y(), avg.z());
        //direction3 normal;// (V[0][2], V[1][2], V[2][2]);
        //point3 point;// (avg.x, avg.y, avg.z);
        return plane3(normal, point);
    }

    template<typename PCloud> plane3<float> best_fit_plane3(const PCloud& pcloud)
    {
        //assert(vlo.get_size() >= 3);

        Array2D<float> matrix(narrow_cast<int>(pcloud.get_size()), 3);
        Array1D<float> v(3);

        Array1D<float> c(3);
        point3 centroid = pcloud.centroid();
        c[0] = centroid.vector[0];
        c[1] = centroid.vector[1];
        c[2] = centroid.vector[2];

        for (int i = 0; i < pcloud.get_size(); ++i)
        {
            const auto& pnt = pcloud.get_point(i);
            v[0] = pnt.vector[0];
            v[1] = pnt.vector[1];
            v[2] = pnt.vector[2];

            auto sft = v - c;
            matrix[i][0] = sft[0];
            matrix[i][1] = sft[1];
            matrix[i][2] = sft[2];
        }

        JAMA::SVD svd(matrix);

        // Array2D<float> U;
         //svd.getU(U);
        Array2D<float> V;
        svd.getV(V);
        // direction3 normal(0, 0, 0);
        // point3 point(0, 0, 0);


        direction3<float> normal(
            sixit::units::create_dimensionless_scalar<float>(V[0][2]), 
            sixit::units::create_dimensionless_scalar<float>(V[1][2]), 
            sixit::units::create_dimensionless_scalar<float>(V[2][2]));
      //  point3 point(centroid);

        return plane3(normal, centroid);
    }
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_plane_impl_h_included

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