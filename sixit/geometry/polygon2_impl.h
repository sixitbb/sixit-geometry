/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_polygon2_impl_h_included
#define sixit_geometry_polygon2_impl_h_included

#include "sixit/geometry/polygon2.h"
#include "sixit/geometry/low_level/polygon_common_impl.h"

#include <numeric>

namespace sixit
{
    namespace geometry
    {
        template <typename fp>
        polygon2<fp>::polygon2(const triangle2<fp>& tri)
        {
            add_vertex(tri.p1());
            add_vertex(tri.p2());
            add_vertex(tri.p3());
        }

        template <typename fp>
        void polygon2<fp>::add_vertex(const point2<fp>& p)
        {
            // todo: add checks here
            vertices.push_back(p);
        }

        template <typename fp>
        int64_t polygon2<fp>::n_vertices() const
        {
            return ssize(vertices);
        }

        template <typename fp>
        point2<fp> polygon2<fp>::vertex(int64_t idx) const
        {
            assert(idx < n_vertices());
            return vertices[idx];
        }

        template <typename fp>
        size_t polygon2<fp>::n_edges() const
        {
            return n_vertices();
        }

        template <typename fp>
        line_segment2<fp> polygon2<fp>::edge(size_t idx) const
        {
            assert(idx < n_edges());
            point2<fp> f_p = vertex(idx);
            point2<fp> s_p;
            if (idx + 1 >= n_edges())
            {
                s_p = vertex(0);
            }
            else
            {
                s_p = vertex(idx + 1);
            }
            return line_segment2<fp>(f_p, s_p);
        }

        template <typename fp>
        size_t polygon2<fp>::n_faces() const
        {
            // todo: check this
            size_t ver = n_vertices();
            if (ver == 0)
            {
                return 0;
            }
            return ver - 2;
        }

        template <typename fp>
        triangle2<fp> polygon2<fp>::face(size_t idx) const
        {
            assert(idx < n_faces());
            std::vector<triangle2<fp>> tris;
            to_triangles(tris);
            return tris[idx];
        }

        template <typename fp>
        ray2<fp> polygon2<fp>::bisector(int64_t idx) const
        {
            assert(idx >= 0 && idx < n_vertices());
            const low_level::dimensional_vector2 p1 = vertex(idx == 0 ? n_vertices() - 1 : idx - 1).vec();
            const low_level::dimensional_vector2 p2 = vertex(idx).vec();
            const low_level::dimensional_vector2 p3 = vertex(idx == n_vertices() - 1 ? 0 : idx + 1).vec();
            low_level::dimensional_vector2 d1 = (p1 - p2).normalized();
            low_level::dimensional_vector2 d3 = (p3 - p2).normalized();
            low_level::dimensional_vector2 dir(d1 + d3);

            if (low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::zero() == dir)
            {
                dir = (p3 - p1) / sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
            }

            dir *= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(fp(normal()));
            return ray2<fp>(p2, direction2<fp>(dir.x, dir.y));
        }
        
        template <typename fp>
        int polygon2<fp>::vertex_concavity(int vertex_index) const {
            int64_t last_id = n_vertices() - 1;
            assert(vertex_index <= last_id);
            int sign = normal();
            point2<fp> p = vertex(vertex_index);
            point2<fp> p0, p1;
            if (vertex_index == 0) {
                p0 = vertex(last_id);
                p1 = vertex(1);
            }
            else if (vertex_index == last_id) {
                p0 = vertex(last_id - 1);
                p1 = vertex(0);
            }
            else {
                p0 = vertex(vertex_index - 1);
                p1 = vertex(vertex_index + 1);
            }
            ray2 ray(p0, p1);
            auto ltr = ray._ltr(p);
            if (((ltr < sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) && (sign < 0)) || 
                ((ltr > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) && (sign > 0))) {
                return 1;
            }
            else if (ltr == sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) return 0;
            else return -1;
        }

        template <typename fp>
        sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> polygon2<fp>::square() const
        {
            return low_level::polygon2_impl<fp>::square(vertices);
        }

        template <typename fp>
        bounds2<fp> polygon2<fp>::bounds() const
        {
            return low_level::polygon2_impl<fp>::bounds(vertices);
        }

        /*void polygon2::to_triangles(std::vector<triangle2>& res) const
        {
            auto tris = low_level::polygon2_impl::triangulate(vertices);
            std::copy(tris.begin(), tris.end(), std::back_inserter(res));
        }*/

        template <typename fp>
        void polygon2<fp>::to_triangles(std::vector<triangle2<fp>>& res) const
        {
            assert(vertices.size() >= 3);

            std::vector<size_t> indices;
            indices.resize(vertices.size());
            std::iota(indices.begin(), indices.end(), 0);

            int norm = normal();

            triangle2<fp> tri(vertices[0], vertices[0], vertices[0]);
            size_t i1, i2, i3;

            auto get_tri = [
                &tri,
                    &i1, &i2, &i3,
                    &vertices = std::as_const(vertices),
                    &indices = std::as_const(indices)
            ](size_t i)
            {
                if (i == 0)
                    i1 = indices[indices.size() - 1];
                else
                    i1 = indices[i - 1];
                i2 = indices[i];
                if (i == indices.size() - 1)
                    i3 = indices[0];
                else
                    i3 = indices[i + 1];
                tri = triangle2<fp>(vertices[i1], vertices[i2], vertices[i3]);
            };

                size_t ii = 0;
                while (indices.size() > 3)
                {
                    if (ii >= indices.size())
                        ii = 0;

                    get_tri(ii);
                    auto tr_n = tri.normal();
                    if (tr_n == 0)//degenerate triangle
                    {
                        res.push_back(tri);
                        indices.erase(indices.begin() + ii);
                    }
                    else if (tr_n != norm)
                    {
                        ++ii;
                        continue;
                    }
                    else
                    {
                        size_t i = 0;
                        for (; i < indices.size(); ++i)
                        {
                            auto ind = indices[i];
                            if (ind == i1 || ind == i2 || ind == i3)
                                continue;
                            if (tri.is_inside_combinable(vertices[ind]))
                                break;
                        }
                        if (i != indices.size())
                        {
                            ++ii;
                            continue;
                        }
                        else
                        {
                            res.push_back(tri);
                            indices.erase(indices.begin() + ii);
                        }
                    }
                }
                res.push_back(triangle2<fp>(vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]));
        }


        template <typename fp>
        int polygon2<fp>::normal() const
        {
            return low_level::polygon2_impl<fp>::normal(vertices);
        }

        template <typename fp>
        bool polygon2<fp>::is_inside(const polygon2<fp>& other) const
        {
            return std::all_of(other.vertices.begin(), other.vertices.end(), [this](const point2<fp>& p) { return is_inside(p); });
        }

        template <typename fp>
        template <typename P2>
        bool polygon2<fp>::is_inside(const P2& point) const
        {
            auto vec = point.vec();
            bool inside = false;
            low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> vi;
            low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> vj = vertex(n_vertices() - 1).vec();
            for (int64_t i = 0; i < n_vertices(); i++)
            {
                vi = vertex(i).vec();

                if ((vi.y > vec.y) != (vj.y > vec.y) && (vec.x < (vj.x - vi.x) * (vec.y - vi.y) / (vj.y - vi.y) + vi.x))
                {
                    inside = !inside;
                }
                vj = vi;
            }
            return inside;
        }

        template <typename fp>
        bool polygon2<fp>::is_inside_combinable(const point2<fp>& point) const
        {
            std::vector<triangle2<fp>> triangles;
            to_triangles(triangles);
            return std::any_of(triangles.begin(), triangles.end(), [&point](const triangle2<fp>& tri) { return tri.is_inside_combinable(point); });
        }

        // implementation base on ray tracing
        //bool polygon2::is_inside_combinable(const point2& point) const
        //{
        //    ray2 ray(point, direction2::up());
        //    size_t n = n_vertices();
        //    int num = 0;
        //    for (size_t i = 0; i < n; ++i)
        //    {
        //        point2 p_c = vertex(i);
        //        point2 p_n = i == n - 1 ? vertex(0) : vertex(i + 1);
        //        fp d1 = ray._ltr(p_c);
        //        fp d2 = ray._ltr(p_n);
        //        if ((d1 == 0.0f && d2 < 0) || (d2 == 0.0f && d1 < 0) || (d1 == 0.0f && d2 == 0.0f))
        //            continue;
        //        else if ((d1 == 0.0f && d2 > 0) || (d2 == 0.0f && d1 > 0) || ((d1 < 0.0f) != (d2 < 0.0f)))
        //        {
        //            low_level::vector2 r = ray.direction_data.vec();
        //            low_level::vector2 s = p_n.vec() - p_c.vec();

        //            fp dev = r.x * s.y - r.y * s.x;
        //            if (dev == 0.0f)
        //                continue;

        //            //checking if we are not behind the start
        //            low_level::vector2 p = point.vec();
        //            low_level::vector2 q = p_c.vec();

        //            low_level::vector2 qp = q - p;

        //            //fp t = (qp.x * s.y - qp.y * s.x) / dev;
        //            fp t = (qp.x * s.y - qp.y * s.x);//optimization, do not need to devide, we compare to zero, check sign
        //            if ((t >= 0.0f) == (dev > 0))//we cross the ray
        //                ++num;
        //        }
        //    }
        //    return (num & 1) != 0;
        //}


        template <typename fp>
        bool polygon2<fp>::strict_intersects(const polygon2<fp>& other) const
        {
            for (size_t i = 0; i < n_edges(); i++)
            {
                line_segment2 seg = edge(i);
                for (size_t j = 0; j < other.n_edges(); j++)
                {
                    if (seg.strict_intersects(other.edge(j)))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        template <typename fp>
        inline std::vector<point2<fp>> polygon2<fp>::strict_intersections(const polygon2<fp>& other) const
        {
            std::vector<point2<fp>> vec;
            for (size_t i = 0; i < n_edges(); i++)
            {
                line_segment2 seg = edge(i);
                for (size_t j = 0; j < other.n_edges(); j++)
                {
                    std::optional<point2<fp>> p = seg.strict_intersection(other.edge(j));
                    if (p.has_value())
                    {
                        vec.push_back(p.value());
                    }
                }
            }
            return vec;
        }

        template <typename fp>
        template <typename LS2>
        std::vector<point2<fp>> polygon2<fp>::strict_intersections(const LS2& ls) const
        {
            std::vector<point2<fp>> vec;
            for (size_t i = 0; i < n_edges(); i++)
            {
                std::optional<point2<fp>> p = edge(i).strict_intersection(ls);
                if (p.has_value())
                {
                    vec.push_back(p.value());
                }
            }
            return vec;
        }

        template <typename fp>
        template <typename P2>
        bool polygon2<fp>::is_point_in_edge(const P2& point) const
        {
            for (size_t i = 0; i < n_edges(); i++)
            {
                if (edge(i).point_in_segment(point))
                {
                    return true;
                }
            }
            return false;
        }
    };
};

#endif //sixit_geometry_polygon2_impl_h_included

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