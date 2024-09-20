/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_shape2_h_included
#define sixit_geometry_shape2_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/polygon2.h"

#include <vector>
#include <set>
#include <list>
#include <array>

namespace sixit
{
namespace geometry
{
    enum class ownership
    {
        shape,
        none,
        undefined,
        border
    };

    struct edge_ownership
    {
        ownership current;
        ownership other;
    };

    template <typename fp>
    struct shape_edge
    {
        shape_edge() {}

        inline shape_edge(ownership left, ownership right, line_segment2<fp> segment)
            : left({ left, ownership::undefined }), right({ right, ownership::undefined }), segment(segment)
        {
        }
        edge_ownership left = edge_ownership({ ownership::undefined, ownership::undefined });
        edge_ownership right = edge_ownership({ ownership::undefined, ownership::undefined });
        line_segment2<fp> segment = line_segment2<fp>(point2<fp>(), point2<fp>());
    };

    template <typename fp>
    struct shape2_base
    {
        polygon2<fp> polygon;
        std::vector<shape2_base<fp>> holes;

        inline shape2_base() : polygon(polygon2<fp>::underlying_vertices_type) {}
        inline shape2_base(const polygon2<fp>& polygon, const std::vector<shape2_base<fp>>& holes = {});

        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
        inline void get_vertices(std::vector<point2<fp>>& vec) const;

        inline bool is_inside_combinable(const point2<fp>& point) const;
        inline bool is_inside(const shape2_base<fp>& shape) const;
        inline bool is_inside(const polygon2<fp>& p) const;
        inline bool strict_intersects(const shape2_base<fp>& shape) const;
        inline bool strict_intersects(const polygon2<fp>& p) const;
        inline std::vector<point2<fp>> strict_intersections(const shape2_base<fp>& shape) const;
        inline std::vector<point2<fp>> strict_intersections(const polygon2<fp>& p) const;

        inline int point_depth(const point2<fp>& p) const;
        inline bool is_point_in_surface(const point2<fp>& p) const;
        inline void get_shape_edges(std::list<shape_edge<fp>>& edges, bool is_hole) const;

        inline bounds2<fp> bounds() const
        {
            return polygon.bounds();
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"shape2_base">(comparser, obj);
            sixit::rw::read_write<&shape2_base::polygon, "polygon", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&shape2_base::holes, "holes", sixit::rw::VofSTRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct shape2
    {
        private:
        std::vector<shape2_base<fp>> shapes_base;
    public:
        inline shape2() {}
        inline shape2(const std::vector<shape2_base<fp>>& shapes_base);

        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;

        inline bool is_inside_combinable(const point2<fp>& point) const;
        inline void to_triangles(std::vector<triangle2<fp>>& tris) const;

        inline bool strict_intersects(const shape2<fp>& shape) const;
        inline bool strict_intersects(const polygon2<fp>& p) const;

        inline void _or(const triangle2<fp>& tri);
        inline void _and(const triangle2<fp>& tri);
        inline void _subtract(const triangle2<fp>& tri);
        inline void _xor(const triangle2<fp>& tri);

        inline void _or(const shape2<fp>& other);
        inline void _and(const shape2<fp>& other);
        inline void _subtract(const shape2<fp>& other);
        inline void _xor(const shape2<fp>& other);

        inline std::vector<shape2_base<fp>> get_shapes_base() const { return shapes_base; }

        inline bounds2<fp> bounds() const
        {
            auto bb = shapes_base[0].bounds();
            for (size_t i = 1; i < shapes_base.size(); i++)
                bb.expand_by(shapes_base[i].bounds());

            return bb;
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"shape2">(comparser, obj);
            sixit::rw::read_write<&shape2::shapes_base, "shapes_base", sixit::rw::VofSTRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }

        inline static std::vector<polygon2<fp>> edges_to_polygons(std::vector<line_segment2<fp>>& segments);
        inline static std::vector<polygon2<fp>> edges_to_polygons2(std::vector<line_segment2<fp>>& segments);
        inline static shape2 polygons_to_shape(std::vector<polygon2<fp>>& polygons);

    private:

        // private structures for shapes operations
        typedef std::pair<fp, fp> vertex_key;
        typedef std::pair<vertex_key, vertex_key> edge_key;

        enum class edge_crossing_state
        {
            disjoined,
            joined,
            overlapped,
            crossed
        };

        struct edge_crossing
        {
            typename std::list<shape_edge<fp>>::iterator it;
            std::array<fp, 2> ltr_distance;
            edge_crossing_state cross_state;
        };

        struct edges_pair_intersection
        {
            edge_crossing a;
            edge_crossing b;
            edge_crossing_state cross_state;
            point2<fp> intersection_point;
        };

        struct logic_operation_data
        {
            std::list<shape_edge<fp>> a_edges;
            std::list<shape_edge<fp>> b_edges;
            std::map<vertex_key, ownership> a_vertices;
            std::map<vertex_key, ownership> b_vertices;

            std::map<shape2<fp>::edge_key, shape_edge<fp>> a_edges_map;
            std::map<shape2<fp>::edge_key, shape_edge<fp>> b_edges_map;

            // for debug purposes
            std::map<vertex_key, uint32_t> vertex_to_index;
        };

        typedef std::function<ownership(ownership, ownership)> logic_function;
        inline static ownership logic_or(ownership own, ownership other)
        {
            return (own == ownership::shape || other == ownership::shape) ? ownership::shape : ownership::none;
        }
        inline static ownership logic_and(ownership own, ownership other)
        {
            return (own == ownership::shape && other == ownership::shape) ? ownership::shape : ownership::none;
        }
        inline static ownership logic_subtract(ownership own, ownership other)
        {
            return own == ownership::shape ? (other == ownership::shape ? ownership::none : ownership::shape) : ownership::none;
        }
        inline static ownership logic_xor(ownership own, ownership other)
        {
            return ((own == ownership::shape && other == ownership::none) || (own == ownership::none && other == ownership::shape)) ? ownership::shape : ownership::none;
        }

        // 
        inline std::vector<polygon2<fp>> polygons_from_logic_function(const shape2<fp>& other, logic_function op);

        inline static void create_edge_intersections(logic_operation_data& op_data);
        inline static edges_pair_intersection edges_pair_crossing(typename std::list<shape_edge<fp>>::iterator& a_it, typename std::list<shape_edge<fp>>::iterator& b_it);
        inline static sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> ltr_distance(point2<fp> p, const line_segment2<fp>& ls, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> ls_length);
        inline static edge_crossing_state make_crossing_state(const std::array<fp, 2>& ltrs);
        inline static edge_crossing_state make_crossing_state(edge_crossing_state a, edge_crossing_state b);
        inline static void process_edges_pair_crossing(edges_pair_intersection& intersection, logic_operation_data& op_data);
        inline static void process_edges_overlapping(std::list<shape_edge<fp>>& edges, std::map<vertex_key, ownership>& op_vertices, typename std::list<shape_edge<fp>>::iterator& it, const point2<fp>& p1, const point2<fp>& p2);
        inline static void mark_vertex_by_ltr(std::map<vertex_key, ownership>& vertices, vertex_key key, fp v_ltr, const shape_edge<fp>& ref_edge);
        inline static bool point_in_edge(const line_segment2<fp>& ls, const point2<fp>& p);
        inline static void divide_edge_by_point(std::list<shape_edge<fp>>& edges, typename std::list<shape_edge<fp>>::iterator& it, const point2<fp>& p);
        inline static void assign_labels_to_vertices(std::map<vertex_key, ownership>& a_vertices, const shape2& b_shape);
        inline static void assign_labels_to_edges(std::map<edge_key, shape_edge<fp>>& a_edges, std::map<edge_key, shape_edge<fp>>& b_edges, std::map<vertex_key, ownership>& a_vertices);
        inline static void assign_labels(shape_edge<fp>& a_edge, std::map<edge_key, shape_edge<fp>>& b_edges, std::map<vertex_key, ownership>& a_vertices);
        inline static ownership joined_edge_ownership(shape_edge<fp>& a_edge, std::map<edge_key, shape_edge<fp>>& b_edges);
        inline static ownership joined_edge_ownership(const point2<fp>& join_point, const point2<fp>& other_point, std::map<edge_key, shape_edge<fp>>& b_edges);
        inline static bool is_right_side_of_edges_joint(const line_segment2<fp>& e0, const line_segment2<fp>& e1, const point2<fp>& p);
        inline static shape_edge<fp> pop_right_out_edge(std::vector<shape_edge<fp>>& out_edges, const shape_edge<fp>& in_edge);
        inline static void remove_coincident_edges(logic_operation_data& op_data);

        inline static vertex_key make_key(const point2<fp>& p) { return { 
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().x), 
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(p.vec().y) }; }
        inline static edge_key make_key(const line_segment2<fp>& ls)
        {
            return { 
                {
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p1().vec().x), 
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p1().vec().y)
                }, 
                {
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p2().vec().x), 
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p2().vec().y)
                } };
        }
        inline static edge_key make_opposite_key(const line_segment2<fp>& ls)
        {
            return { 
                {
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p2().vec().x), 
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p2().vec().y)
                }, 
                {
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p1().vec().x), 
                    sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ls.p1().vec().y)
                } };
        }
        inline static bool greater(const point2<fp>& p1, const point2<fp>& p2)
        {
            if (p1.vec().x > p2.vec().x)
                return true;

            if (p1.vec().x < p2.vec().x)
                return false;

            return (p1.vec().y > p2.vec().y);
        }
        inline static sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> _ltr(const point2<fp>& p1, const point2<fp>& p2, const point2<fp>& p)
        {
            if (greater(p1, p2))
            {
                low_level::dimensional_vector2 vv = p2.vec() - p1.vec();
                low_level::dimensional_vector2 p_v = p.vec() - p1.vec();
                return vv.y * p_v.x - vv.x * p_v.y;
            }
            else
            {
                // reverse
                low_level::dimensional_vector2 vv = p1.vec() - p2.vec();
                low_level::dimensional_vector2 p_v = p.vec() - p2.vec();
                return - (vv.y * p_v.x - vv.x * p_v.y);
            }
        }

        inline static std::vector<line_segment2<fp>> merge_edges(const std::map<edge_key, shape_edge<fp>>& own, const std::map<edge_key, shape_edge<fp>>& other, logic_function op);

        // refactored edges_to_polygons2
        inline static bool check_edges_integrity(const std::vector<line_segment2<fp>>& segments);
        inline static std::vector<point2<fp>> build_poly_contour(std::vector<line_segment2<fp>>& chain, std::vector<line_segment2<fp>>& segments);
        inline static line_segment2<fp> pop_right_next_segment(std::vector<line_segment2<fp>>& segments, const line_segment2<fp>& current_segment);

        // for debuging
        inline static void output_debug_merged_edges(const std::vector<line_segment2<fp>>& edges, std::map<vertex_key, uint32_t>& vertex_to_index);
        inline static void output_debug_dimension_range(std::vector<line_segment2<fp>>& edges);
        inline static void output_debug_indexed_edges(std::map<vertex_key, uint32_t>& vertex_to_index, const std::list<shape_edge<fp>>& a_edges, const std::list<shape_edge<fp>>& b_edges);
        inline static void debug_vertex_to_index(std::map<vertex_key, uint32_t>& vertex_to_index, const std::list<shape_edge<fp>>& a_edges, const std::list<shape_edge<fp>>& b_edges);

        // TEMPORARY keep it - finding polygons from edges - need to be reviewed
        typedef std::pair<fp, fp> debug_point2;
        typedef std::pair<debug_point2, debug_point2> debug_edge;
        inline static std::map<debug_point2, std::vector<debug_point2>> build_graph(const std::vector<line_segment2<fp>>& edges);
        inline static std::vector<debug_point2> find_cycle(const std::map<debug_point2, std::vector<debug_point2>>& adjacency, const debug_point2& start, size_t max_depth);
    };
};

namespace geometry 
{
    // aliases 
    using shape_edgef = shape_edge<float>;
    using shape2_basef = shape2_base<float>;
    using shape2f = shape2<float>;
}

};

#endif //sixit_geometry_shape2_h_included

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
