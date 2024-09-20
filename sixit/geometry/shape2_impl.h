/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_shape2_impl_h_included
#define sixit_geometry_shape2_impl_h_included

#include "sixit/geometry/shape2.h"

#include <queue>
#include <vector>
#include <iomanip>

namespace sixit
{
namespace geometry
{
    // shape2_base
    template <typename fp>
    shape2_base<fp>::shape2_base(const polygon2<fp>& polygon, const std::vector<shape2_base<fp>>& holes) : polygon(polygon), holes(holes)
    {
        //holes must be inside the polygon.
        size_t count = holes.size();
        for (size_t i = 0; i < count; i++)
        {
            [[maybe_unused]] const shape2_base& shape_i = holes[i];
            assert(is_inside(shape_i));

            for (size_t j = 0; j < count; j++)
            {
                //holes should not overlap nor intersect each other
                [[maybe_unused]] const shape2_base& shape_j = holes[j];
                assert(!shape_i.strict_intersects(shape_j));
                assert(!shape_i.is_inside(shape_j));
                assert(!shape_j.is_inside(shape_i));
            }
        }
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> shape2_base<fp>::square() const
    {
        auto holes_val = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        for (const shape2_base& s : holes)
        {
            holes_val += s.square();
        }
        return polygon.square() - holes_val;
    }

    template <typename fp>
    void shape2_base<fp>::get_vertices(std::vector<point2<fp>>& vec) const
    {
        //should vertices be inverted for children?
        vec = std::vector<point2<fp>>(polygon.vertices.begin(), polygon.vertices.end());

        for (const auto& hole : holes)
            hole.get_vertices(vec);
    }

    template <typename fp>
    bool shape2_base<fp>::is_inside_combinable(const point2<fp>& point) const
    {
        return polygon.is_inside_combinable(point);
    }

    template <typename fp>
    bool shape2_base<fp>::is_inside(const shape2_base<fp>& shape) const
    {
        return polygon.is_inside(shape.polygon);
    }

    template <typename fp>
    bool shape2_base<fp>::is_inside(const polygon2<fp>& p) const
    {
        return polygon.is_inside(p);
    }

    template <typename fp>
    bool shape2_base<fp>::strict_intersects(const shape2_base<fp>& shape) const
    {
        return polygon.strict_intersects(shape.polygon);//should use strict_intersection() function from TR-2030
    }

    template <typename fp>
    bool shape2_base<fp>::strict_intersects(const polygon2<fp>& p) const
    {
        return polygon.strict_intersects(p);//should use strict_intersection() function from TR-2030
    }

    template <typename fp>
    std::vector<point2<fp>> shape2_base<fp>::strict_intersections(const shape2_base<fp>& shape) const
    {
        return polygon.strict_intersections(shape.polygon);
    }

    template <typename fp>
    std::vector<point2<fp>> shape2_base<fp>::strict_intersections(const polygon2<fp>& p) const
    {
        return polygon.strict_intersections(p);
    }

    template <typename fp>
    int shape2_base<fp>::point_depth(const point2<fp>& p) const
    {
        for (const auto& h : holes)
        {
            int depth = h.point_depth(p);
            if (depth >= 0)
            {
                return depth + 1;
            }
        }

        bool b1 = polygon.is_inside_combinable(p);
        return b1 ? 0 : -1;
    }

    template <typename fp>
    bool shape2_base<fp>::is_point_in_surface(const point2<fp>& p) const
    {
        int depth = point_depth(p);
        return depth >= 0 && depth % 2 == 0;
    }

    template <typename fp>
    void shape2_base<fp>::get_shape_edges(std::list<shape_edge<fp>>& edges, bool is_hole) const
    {
        for (const auto& hole : holes)
        {
            hole.get_shape_edges(edges, !is_hole);
        }

        bool reverse = is_hole ? polygon.normal() < 0 : polygon.normal() > 0;
        std::vector<point2<fp>> vertices = reverse ? std::vector<point2<fp>>(polygon.vertices.rbegin(), polygon.vertices.rend()) :
            std::vector<point2<fp>>(polygon.vertices.begin(), polygon.vertices.end());

        ownership left = ownership::none;
        ownership right = ownership::shape;

        for (size_t i = 0; i < vertices.size() - 1; i++)
            edges.push_back(shape_edge(left, right, line_segment2<fp>(vertices[i], vertices[i + 1])));

        edges.push_back(shape_edge(left, right, line_segment2<fp>(vertices.back(), vertices.front())));
    }


    // shape
    template <typename fp>
    shape2<fp>::shape2(const std::vector<shape2_base<fp>>& shapes_base) : shapes_base(shapes_base)
    {
        //shapes does not intersect, does not overlap. Edges or points can overlap
        for (size_t i = 0; i < shapes_base.size(); i++)
        {
            [[maybe_unused]] const shape2_base<fp>& shape_i = shapes_base[i];
            for (size_t j = i; j < shapes_base.size(); j++)
            {
                [[maybe_unused]] const shape2_base<fp>& shape_j = shapes_base[j];
                assert(!shape_i.strict_intersects(shape_j));
                assert(!shape_i.is_inside(shape_j));
                assert(!shape_j.is_inside(shape_i));
            }
        }
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> shape2<fp>::square() const
    {
        auto val = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.0f);
        for (const shape2_base<fp>& s : shapes_base)
        {
            val += s.square();
        }
        return val;
    }

    template <typename fp>
    bool shape2<fp>::is_inside_combinable(const point2<fp>& point) const
    {
        for (size_t i = 0; i < shapes_base.size(); i++)
            if (shapes_base[i].polygon.is_inside_combinable(point))
                return true;

        return false;
    }

    template <typename fp>
    void shape2<fp>::to_triangles(std::vector<triangle2<fp>>& tris) const
    {
        std::vector<point2<fp>> vertices;
        for (const shape2_base<fp>& s : shapes_base)
        {
            s.get_vertices(vertices);
        }
        polygon2<fp> p({ vertices });
        return p.to_triangles(tris);
    }

    template <typename fp>
    bool shape2<fp>::strict_intersects(const shape2<fp>& shape) const
    {
        for (size_t i = 0; i < shapes_base.size(); i++)
        {
            const shape2_base<fp>& si = shapes_base[i];
            for (size_t j = i; j < shape.shapes_base.size(); j++)
            {
                if (!si.strict_intersects(shape.shapes_base[j]))
                    return false;
            }
        }
        
        return true;

    }

    template <typename fp>
    bool shape2<fp>::strict_intersects(const polygon2<fp>& p) const
    {
        return std::any_of(shapes_base.begin(), shapes_base.end(), [&p](const shape2_base<fp>& s) { return s.strict_intersects(p); });
    }

    template <typename fp>
    void shape2<fp>::_or(const triangle2<fp>& tri)
    {
        _or(shape2<fp>({ polygon2<fp>(tri) }));
    }

    template <typename fp>
    void shape2<fp>::_and(const triangle2<fp>& tri)
    {
        _and(shape2<fp>({ polygon2<fp>(tri) }));
    }

    template <typename fp>
    void shape2<fp>::_subtract(const triangle2<fp>& tri)
    {
        _subtract(shape2<fp>({ polygon2<fp>(tri) }));
    }

    template <typename fp>
    void shape2<fp>::_xor(const triangle2<fp>& tri)
    {
        _xor(shape2<fp>({ polygon2<fp>(tri) }));
    }

    template <typename fp>
    void shape2<fp>::_or(const shape2<fp>& other)
    {
        auto polygons = polygons_from_logic_function(other, logic_or);         
        auto result = shape2<fp>::polygons_to_shape(polygons);
        *this = result;
    }

    template <typename fp>
    void shape2<fp>::_and(const shape2<fp>& other)
    {
        auto polygons = polygons_from_logic_function(other, logic_and);
        auto result = shape2<fp>::polygons_to_shape(polygons);
        *this = result;
    }

    template <typename fp>
    void shape2<fp>::_subtract(const shape2<fp>& other)
    {
        auto polygons = polygons_from_logic_function(other, logic_subtract);
        auto result = shape2::polygons_to_shape(polygons);
        *this = result;
    }

    template <typename fp>
    void shape2<fp>::_xor(const shape2<fp>& other)
    {
        auto polygons = polygons_from_logic_function(other, logic_xor);
        auto result = shape2::polygons_to_shape(polygons);
        *this = result;
    }

    template <typename fp>
    void shape2<fp>::assign_labels_to_vertices(std::map<vertex_key, ownership>& a_vertices, const shape2<fp>& b_shape)
    {
        auto point_in_surface = [&b_shape](const point2<fp>& p) -> bool
            {
                for (const auto& base : b_shape.shapes_base)
                        if (base.is_point_in_surface(p))
                            return true;
                    return false;
            };

        for (auto& it : a_vertices)
        {
            if (it.second == ownership::undefined)
            {
                it.second = point_in_surface(point2(
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(it.first.first), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(it.first.second))) ? ownership::shape : ownership::none;
            }
        }
    }

    template <typename fp>
    void shape2<fp>::assign_labels_to_edges(std::map<edge_key, shape_edge<fp>>& a_edges, std::map<edge_key, shape_edge<fp>>& b_edges, std::map<vertex_key, ownership>& a_vertices)
    {
        for (auto it = a_edges.begin(); it != a_edges.end(); ++it)
        {
            assign_labels(it->second, b_edges, a_vertices);
        }
    }

    template <typename fp>
    void shape2<fp>::remove_coincident_edges(logic_operation_data& op_data)
    {
        for (auto it = op_data.a_edges_map.begin(); it != op_data.a_edges_map.end(); ++it)
        {
            const auto& a_edge = it->second;

            auto a_key = make_key(a_edge.segment);
            if (op_data.b_edges_map.contains(a_key))
                op_data.b_edges_map.erase(a_key);

            auto opposite_a_key = make_opposite_key(a_edge.segment);
            if (op_data.b_edges_map.contains(opposite_a_key))
                op_data.b_edges_map.erase(opposite_a_key);
        }
    }

    template <typename fp>
    void shape2<fp>::assign_labels(shape_edge<fp>& a_edge, std::map<edge_key, shape_edge<fp>>& b_edges, std::map<vertex_key, ownership>& a_vertices)
    {
        // check if other_edges contains the same edge
        auto a_key = make_key(a_edge.segment);
        if (b_edges.contains(a_key))
        {
            const shape_edge<fp>& coincident_b = b_edges[a_key];
            a_edge.left.other = coincident_b.left.current;
            a_edge.right.other = coincident_b.right.current;
            //b_edges.erase(a_key);
            return;
        }

        // check if other_edges contains the opposite edge
        auto opposite_a_key = make_opposite_key(a_edge.segment);
        if (b_edges.contains(opposite_a_key))
        {
            const shape_edge<fp>& opposite_b = b_edges[opposite_a_key];
            a_edge.left.other = opposite_b.right.current;
            a_edge.right.other = opposite_b.left.current;
            //b_edges.erase(opposite_a_key);
            return;
        }

        // common case - get state from vertices
        ownership o1 = a_vertices[make_key(a_edge.segment.p1())];
        ownership o2 = a_vertices[make_key(a_edge.segment.p2())];

        // impossible combinations
        assert(!(o1 == ownership::shape && o2 == ownership::none));
        assert(!(o2 == ownership::shape && o1 == ownership::none));
        assert(!(o2 == ownership::undefined || o1 == ownership::undefined));

        // combine
        ownership o = ownership::undefined;
        if (o1 == ownership::shape || o2 == ownership::shape)
            o = ownership::shape;
        else
            o = (o1 == ownership::none || o2 == ownership::none) ? ownership::none : joined_edge_ownership(a_edge, b_edges);

        a_edge.left.other = o;
        a_edge.right.other = o;
    }

    template <typename fp>
    bool shape2<fp>::is_right_side_of_edges_joint(const line_segment2<fp>& e0, const line_segment2<fp>& e1, const point2<fp>& p)
    {
        auto joint_turn = _ltr(e0.p1(), e0.p2(), e1.p2());
        auto e0_side = _ltr(e0.p1(), e0.p2(), p);
        auto e1_side = _ltr(e1.p1(), e1.p2(), p);
        bool joint_turn_right = joint_turn >= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        bool right_side = joint_turn_right ? (e0_side > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) && 
                                                (e1_side > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) : 
                                                (e0_side > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f)) || 
                                                (e1_side > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f));

        // for check
        //bool left_side = joint_turn_right ? (e0_side < 0) || (e1_side < 0) : (e0_side < 0) && (e1_side < 0);
        //if (left_side == right_side)
        //{
        //    std::cout << "is_right_side_of_edges_joint" << std::endl;
        //}

        return right_side;
    }

    template <typename fp>
    shape_edge<fp> shape2<fp>::pop_right_out_edge(std::vector<shape_edge<fp>>& out_edges, const shape_edge<fp>& in_edge)
    {
        typename std::vector<shape_edge<fp>>::iterator r_it = out_edges.begin();
        typename std::vector<shape_edge<fp>>::iterator it = r_it;
        for (it++; it != out_edges.end(); it++)
        {
            if (is_right_side_of_edges_joint(in_edge.segment, r_it->segment, it->segment.p2()))
                r_it = it;
        }

        shape_edge<fp> out_edge = *r_it;
        out_edges.erase(r_it);
        return out_edge;
    }

    template <typename fp>
    ownership shape2<fp>::joined_edge_ownership(const point2<fp>& join_point, const point2<fp>& other_point, std::map<edge_key, shape_edge<fp>>& b_edges)
    {
        auto in_selector = [&join_point](const auto& kv) -> bool { return kv.second.segment.p2() == join_point; };
        auto out_selector = [&join_point](const auto& kv) -> bool { return kv.second.segment.p1() == join_point; };

        auto select_edges = [&b_edges](auto p) {
            std::vector<shape_edge<fp>> es;
            for (const auto& kv : b_edges) if (p(kv)) es.push_back(kv.second);
            return es; };

        std::vector<shape_edge<fp>> out_edges = select_edges(out_selector);
        std::vector<shape_edge<fp>> in_edges = select_edges(in_selector);

        assert(out_edges.size() == in_edges.size());
        // assert(out_edges.size() != 0);

        ownership edge_ownership = ownership::undefined;
        while (in_edges.size() > 0)
        {
            shape_edge<fp> in_edge = in_edges.back();
            in_edges.pop_back();

            auto out_edge = pop_right_out_edge(out_edges, in_edge);

            bool is_right_side = is_right_side_of_edges_joint(in_edge.segment, out_edge.segment, other_point);
            if (is_right_side)
                return in_edge.right.current;
            else
                edge_ownership = in_edge.left.current;
        }

        return edge_ownership;
    }

    template <typename fp>
    ownership shape2<fp>::joined_edge_ownership(shape_edge<fp>& a_edge, std::map<edge_key, shape_edge<fp>>& b_edges)
    {
        auto o1 = joined_edge_ownership(a_edge.segment.p1(), a_edge.segment.p2(), b_edges);

        // for debug check
        //auto o2 = joined_edge_ownership(a_edge.segment.p2(), a_edge.segment.p1(), b_edges);
        //if (o1 != o2)
        //{
        //    o1 = joined_edge_ownership(a_edge.segment.p1(), a_edge.segment.p2(), b_edges);
        //    o2 = joined_edge_ownership(a_edge.segment.p2(), a_edge.segment.p1(), b_edges);
        //    std::cout << "joined_edge_ownership error" << std::endl;
        //}

        return o1;
    }

    template <typename fp>
    typename shape2<fp>::edge_crossing_state shape2<fp>::make_crossing_state(const std::array<fp, 2>& ltrs)
    {
        if (ltrs[0] < fp(0.0f))
            return ltrs[1] < fp(0.0f) ? edge_crossing_state::disjoined : (ltrs[1] > fp(0.0f) ? edge_crossing_state::crossed : edge_crossing_state::joined);
        if (ltrs[0] > fp(0.0f))
            return ltrs[1] > fp(0.0f) ? edge_crossing_state::disjoined : (ltrs[1] < fp(0.0f) ? edge_crossing_state::crossed : edge_crossing_state::joined);

        return ltrs[1] > fp(0.0f) ? edge_crossing_state::joined : (ltrs[1] < fp(0.0f) ? edge_crossing_state::joined : edge_crossing_state::overlapped);
    }

    template <typename fp>
    typename shape2<fp>::edge_crossing_state shape2<fp>::make_crossing_state(edge_crossing_state a, edge_crossing_state b)
    {
        if (a == edge_crossing_state::disjoined || b == edge_crossing_state::disjoined)
            return edge_crossing_state::disjoined;

        if (a == edge_crossing_state::overlapped || b == edge_crossing_state::overlapped)
            return edge_crossing_state::overlapped;

        if (a == edge_crossing_state::crossed && b == edge_crossing_state::crossed)
            return edge_crossing_state::crossed;

        return edge_crossing_state::joined;
    }

    template <typename fp>
    typename shape2<fp>::edges_pair_intersection shape2<fp>::edges_pair_crossing(typename std::list<shape_edge<fp>>::iterator& a_it, typename std::list<shape_edge<fp>>::iterator& b_it)
    {
        edges_pair_intersection intersection;
        intersection.a.it = a_it;
        intersection.b.it = b_it;

        const auto& a_ls = a_it->segment;
        auto a_ls_length = a_ls.length();
        const auto& b_ls = b_it->segment;
        auto b_ls_length = b_ls.length();

        intersection.a.ltr_distance = { 
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ltr_distance(a_ls.p1(), b_ls, b_ls_length)),  
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ltr_distance(a_ls.p2(), b_ls, b_ls_length)) };
        intersection.b.ltr_distance = { 
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ltr_distance(b_ls.p1(), a_ls, a_ls_length)),  
            sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(ltr_distance(b_ls.p2(), a_ls, a_ls_length)) };
        intersection.a.cross_state = make_crossing_state(intersection.a.ltr_distance);
        intersection.b.cross_state = make_crossing_state(intersection.b.ltr_distance);
        intersection.cross_state = make_crossing_state(intersection.a.cross_state, intersection.b.cross_state);

        if (intersection.cross_state == edge_crossing_state::disjoined)
            return intersection;

        if ((intersection.a.cross_state == edge_crossing_state::joined) && (intersection.b.cross_state == edge_crossing_state::joined))
            return intersection;

        if (intersection.cross_state == edge_crossing_state::crossed)
        {
            // both crossed - create new cross point
            if (a_ls_length < b_ls_length)
            {
                auto sum = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::abs(intersection.a.ltr_distance[0]) + low_level::mathf::abs(intersection.a.ltr_distance[1]));
                auto t = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::abs(intersection.a.ltr_distance[0])) / sum;
                intersection.intersection_point = point2(a_ls.p1().vec() * (sixit::units::create_dimensionless_scalar<fp>(1.0f) - t) + a_ls.p2().vec() * t);
            }
            else
            {
                auto sum = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::abs(intersection.b.ltr_distance[0]) + low_level::mathf::abs(intersection.b.ltr_distance[1]));
                auto t = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::abs(intersection.b.ltr_distance[0])) / sum;
                intersection.intersection_point = point2(b_ls.p1().vec() * (sixit::units::create_dimensionless_scalar<fp>(1.0f) - t) + b_ls.p2().vec() * t);
            }

            // check intersection point for coincidenting with existing edges and update the cross states if so
            if (intersection.intersection_point == a_ls.p1() || intersection.intersection_point == a_ls.p2())
                intersection.a.cross_state = edge_crossing_state::joined;
            if (intersection.intersection_point == b_ls.p1() || intersection.intersection_point == b_ls.p2())
                intersection.b.cross_state = edge_crossing_state::joined;
            intersection.cross_state = make_crossing_state(intersection.a.cross_state, intersection.b.cross_state);
            return intersection;
        }

        if (intersection.a.cross_state == edge_crossing_state::crossed)
        {
            // a crossed (b joined)
            intersection.intersection_point = (intersection.b.ltr_distance[0] == fp(0.0f)) ? b_ls.p1() : b_ls.p2();
        }
        else
        if (intersection.b.cross_state == edge_crossing_state::crossed)
        {
            // b crossed (a joined)
            intersection.intersection_point = (intersection.a.ltr_distance[0] == fp(0.0f)) ? a_ls.p1() : a_ls.p2();
        }
        else
        if (intersection.cross_state == edge_crossing_state::overlapped)
        {
            intersection.a.cross_state = edge_crossing_state::overlapped;
            intersection.b.cross_state = edge_crossing_state::overlapped;
            intersection.a.ltr_distance = { 0.0f, 0.0f };
            intersection.b.ltr_distance = { 0.0f, 0.0f };
        }

        return intersection;
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> shape2<fp>::ltr_distance(point2<fp> p, const line_segment2<fp>& ls, 
                                                                                        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> ls_length)
    {
        const low_level::dimensional_vector2 v1 = ls.p2().vec() - ls.p1().vec();
        const low_level::dimensional_vector2 p_v1 = p.vec() - ls.p1().vec();
        const low_level::dimensional_vector2 p_v2 = p.vec() - ls.p2().vec();
        auto d_v1 = p_v1.magnitude();
        auto d_v2 = p_v2.magnitude();
        return ((d_v1 < d_v2) ? (v1.y * p_v1.x - v1.x * p_v1.y) : (v1.y * p_v2.x - v1.x * p_v2.y)) / ls_length;
    }

    template <typename fp>
    void shape2<fp>::mark_vertex_by_ltr(std::map<vertex_key, ownership>& vertices, vertex_key key, fp v_ltr, const shape_edge<fp>& ref_edge)
    {
        if (vertices[key] != ownership::border)
            vertices[key] = (v_ltr < fp(0.0f)) ? ref_edge.left.current : ((v_ltr > fp(0.0f)) ? ref_edge.right.current : ownership::border);
    }

    template <typename fp>
    bool shape2<fp>::point_in_edge(const line_segment2<fp>& ls, const point2<fp>& p)
    {
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p1 = ls.p1().vec();
        if (p == p1)
            return false;

        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p2 = ls.p2().vec();
        if (p == p2)
            return false;

        low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p2p1 = p2 - p1;
        auto proj = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(p2p1, (p.vec() - p1));

        return (proj > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f) && proj < p2p1.sqr_magnitude());
    }
    
    template <typename fp>
    void shape2<fp>::divide_edge_by_point(std::list<shape_edge<fp>>& edges, typename std::list<shape_edge<fp>>::iterator& it, const point2<fp>& p)
    {
        edges.insert(std::next(it), shape_edge<fp>(it->left.current, it->right.current, line_segment2<fp>(p, it->segment.p2())));
        it->segment.p2() = p;
    }

    template <typename fp>
    void shape2<fp>::process_edges_overlapping(std::list<shape_edge<fp>>& edges, std::map<vertex_key, ownership>& op_vertices, typename std::list<shape_edge<fp>>::iterator& it, const point2<fp>& p1, const point2<fp>& p2)
    {
        if (point_in_edge(it->segment, p1))
        {
            divide_edge_by_point(edges, it, p1);
            op_vertices[make_key(p1)] = ownership::border;
        }

        if (point_in_edge(it->segment, p2))
        {
            divide_edge_by_point(edges, it, p2);
            op_vertices[make_key(p2)] = ownership::border;
        }
    }

    template <typename fp>
    void shape2<fp>::process_edges_pair_crossing(edges_pair_intersection& intersection, logic_operation_data& op_data)
    {
        // if both joined
        if (intersection.a.cross_state == edge_crossing_state::joined && intersection.b.cross_state == edge_crossing_state::joined)
        {
            // mark vertex from shape_a on border of shape_b
            if (intersection.a.ltr_distance[0] == fp(0.0f))
                op_data.a_vertices[make_key(intersection.a.it->segment.p1())] = ownership::border;
            else
                op_data.a_vertices[make_key(intersection.a.it->segment.p2())] = ownership::border;

            // mark vertex from shape_b on border of shape_a
            if (intersection.b.ltr_distance[0] == fp(0.0f))
                op_data.b_vertices[make_key(intersection.b.it->segment.p1())] = ownership::border;
            else
                op_data.b_vertices[make_key(intersection.b.it->segment.p2())] = ownership::border;

            return; // edges just connected - have a common vertex - nothing to do
        }

        if (intersection.cross_state == edge_crossing_state::overlapped)
        {
            process_edges_overlapping(op_data.a_edges, op_data.b_vertices, intersection.a.it, intersection.b.it->segment.p1(), intersection.b.it->segment.p2());
            process_edges_overlapping(op_data.b_edges, op_data.a_vertices, intersection.b.it, intersection.a.it->segment.p1(), intersection.a.it->segment.p2());
            return;
        }

        // if any crossing
        if (intersection.a.cross_state == edge_crossing_state::crossed)
        {
            divide_edge_by_point(op_data.a_edges, intersection.a.it, intersection.intersection_point);

            // mark vertices
            op_data.a_vertices[make_key(intersection.intersection_point)] = ownership::border;
            mark_vertex_by_ltr(op_data.b_vertices, make_key(intersection.b.it->segment.p1()), intersection.b.ltr_distance[0], *(intersection.a.it));
            mark_vertex_by_ltr(op_data.b_vertices, make_key(intersection.b.it->segment.p2()), intersection.b.ltr_distance[1], *(intersection.a.it));
        }

        if (intersection.b.cross_state == edge_crossing_state::crossed)
        {
            divide_edge_by_point(op_data.b_edges, intersection.b.it, intersection.intersection_point);

            // mark vertices
            op_data.b_vertices[make_key(intersection.intersection_point)] = ownership::border;
            mark_vertex_by_ltr(op_data.a_vertices, make_key(intersection.a.it->segment.p1()), intersection.a.ltr_distance[0], *(intersection.b.it));
            mark_vertex_by_ltr(op_data.a_vertices, make_key(intersection.a.it->segment.p2()), intersection.a.ltr_distance[1], *(intersection.b.it));
        }
    }

    template <typename fp>
    void shape2<fp>::create_edge_intersections(logic_operation_data& op_data)
    {
        //intersections a-b
        for (auto a_it = op_data.a_edges.begin(); a_it != op_data.a_edges.end(); ++a_it)
        {
            for (auto b_it = op_data.b_edges.begin(); b_it != op_data.b_edges.end(); ++b_it)
            {
                auto intersection = edges_pair_crossing(a_it, b_it);
                if (intersection.cross_state != edge_crossing_state::disjoined)
                {
                    process_edges_pair_crossing(intersection, op_data);
                }
            }
        }
    }

    template <typename fp>
    std::vector<polygon2<fp>> shape2<fp>::polygons_from_logic_function(const shape2<fp>& other, logic_function op)
    {
        logic_operation_data op_data;

        //create shape edges
        for (const auto& s: shapes_base)
            s.get_shape_edges(op_data.a_edges, false);

        for (const auto& o : other.shapes_base)
            o.get_shape_edges(op_data.b_edges, false);

        //  vertex maps
        for (const auto& e : op_data.a_edges)
            op_data.a_vertices.insert_or_assign({ 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.segment.p1().vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.segment.p1().vec().y) }, ownership::undefined);
        for (const auto& e : op_data.b_edges)
            op_data.b_vertices.insert_or_assign({ 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.segment.p1().vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.segment.p1().vec().y) }, ownership::undefined);

        create_edge_intersections(op_data);

        for (auto e : op_data.a_edges)
            op_data.a_edges_map.insert({ shape2::make_key(e.segment), e });

        for (auto e : op_data.b_edges)
            op_data.b_edges_map.insert({ shape2::make_key(e.segment), e });

        assign_labels_to_vertices(op_data.a_vertices, other);
        assign_labels_to_vertices(op_data.b_vertices, *this);
        assign_labels_to_edges(op_data.a_edges_map, op_data.b_edges_map, op_data.a_vertices);
        assign_labels_to_edges(op_data.b_edges_map, op_data.a_edges_map, op_data.b_vertices);
        remove_coincident_edges(op_data);

        // output debug_dimension_range(const std::vector<line_segment2>& edges)
        {
            std::vector<line_segment2<fp>> all_edges;
            for (const auto& p : op_data.a_edges_map)
                all_edges.push_back(p.second.segment);
            for (const auto& p : op_data.b_edges_map)
                all_edges.push_back(p.second.segment);

            // for debug check
            // output_debug_dimension_range(all_edges);
        }

        // merge edges and perform the operation
        std::vector<line_segment2<fp>> merged_edges = merge_edges(op_data.a_edges_map, op_data.b_edges_map, op);

        // for debugging
        //bool is_topology_ok = check_edges_integrity(merged_edges);
        //if (!is_topology_ok)
        //{
        //    // output debug info
        //    // op_data.a_edges, op_data.b_edges contains info after create_edge_intersections
        //    debug_vertex_to_index(op_data.vertex_to_index, op_data.a_edges, op_data.b_edges);
        //    output_debug_indexed_edges(op_data.vertex_to_index, op_data.a_edges, op_data.b_edges);
        //    output_debug_merged_edges(merged_edges, op_data.vertex_to_index);
        //}

        std::vector<polygon2<fp>> polygons = edges_to_polygons(merged_edges);
        return polygons;
    }

    template <typename fp>
    void shape2<fp>::debug_vertex_to_index(std::map<vertex_key, uint32_t>& vertex_to_index, const std::list<shape_edge<fp>>& a_edges, const std::list<shape_edge<fp>>& b_edges)
    {
        uint32_t v_index = 0;
        for (const auto& e : a_edges)
        {
            vertex_key vk = { e.segment.p1().vec().x, e.segment.p1().vec().y };
            if (!vertex_to_index.contains(vk))
                vertex_to_index[vk] = v_index++;
        }
        for (const auto& e : b_edges)
        {
            vertex_key vk = { e.segment.p1().vec().x, e.segment.p1().vec().y };
            if (!vertex_to_index.contains(vk))
                vertex_to_index[vk] = v_index++;
        }
    }

    template <typename fp>
    void shape2<fp>::output_debug_dimension_range(std::vector<line_segment2<fp>>& edges)
    {
        fp max_length(0.0f);
        for (const auto& ls : edges) 
            max_length = std::max(ls.length(), max_length);

        auto min_length = fp(0.00001f) * max_length;
        auto count_small_edges = std::count_if(edges.begin(), edges.end(), [&min_length](const line_segment2<fp>& ls) {return ls.length() < min_length; });
        std::cout << "number small edges: " << count_small_edges << std::endl;
    }

    template <typename fp>
    void shape2<fp>::output_debug_indexed_edges(std::map<vertex_key, uint32_t>& vertex_to_index, const std::list<shape_edge<fp>>& a_edges, const std::list<shape_edge<fp>>& b_edges)
    {
        std::cout << "output_debug_indexed_edges" << std::endl;

        for (const auto& e : a_edges)
        {
            int32_t i1 = vertex_to_index[make_key(e.segment.p1())];
            int32_t i2 = vertex_to_index[make_key(e.segment.p2())];
            std::cout << i1 << "  " << i2 << std::endl;
        }
        for (const auto& e : b_edges)
        {
            int32_t i1 = vertex_to_index[make_key(e.segment.p1())];
            int32_t i2 = vertex_to_index[make_key(e.segment.p2())];
            std::cout << i1 << "  " << i2 << std::endl;
        }
    }

    template <typename fp>
    void shape2<fp>::output_debug_merged_edges(const std::vector<line_segment2<fp>>& edges, std::map<vertex_key, uint32_t>& vertex_to_index)
    {
        std::cout << "output_debug_merged_edges" << std::endl;

        for (const auto& e : edges)
        {
            int32_t i1 = vertex_to_index[make_key(e.p1())];
            int32_t i2 = vertex_to_index[make_key(e.p2())];
            std::cout << i1 << "  " << i2 << std::endl;
        }
    }

    template <typename fp>
    shape2<fp> shape2<fp>::polygons_to_shape(std::vector<polygon2<fp>>& polygons)
    {
        //find polygons inside other polygons
        std::map<size_t, std::set<size_t>> hierarchy; // child, parents

        for (size_t i = 0; i < polygons.size(); ++i)
        {
            hierarchy.insert({ i, {} });
            for (size_t j = i + 1; j < polygons.size(); ++j)
            {
                if (polygons[i].is_inside(polygons[j]))
                {
                    hierarchy[j].insert(i);
                }
                else if (polygons[j].is_inside(polygons[i]))
                {
                    hierarchy[i].insert(j);
                }
            }
        }

        std::set<size_t> done;
        auto find_parent_child_pair = [&hierarchy, &done]() ->std::pair<size_t, size_t>
        {
            //find a polygon with only 1 parent
            for (auto it = hierarchy.begin(); it != hierarchy.end(); it++)
            {
                if (it->second.size() == 1 && done.find(it->first) == done.end())
                {
                    done.insert(it->first);
                    return { *it->second.begin(),  it->first }; // parent, child
                }
            }
            return { 0, 0 };
        };

        // flatten the hierarchy so each poly has 1 or no parents
        const auto empty_pair = std::pair<size_t, size_t>(0, 0);
        auto parent_child_pair = find_parent_child_pair();
        while (parent_child_pair != empty_pair)
        {
            // if any polygon is child of both (parent and child) erase the parent from it's list
            for (auto& h : hierarchy)
            {
                auto& parents = h.second;
                if (parents.find(parent_child_pair.first) != parents.end() && parents.find(parent_child_pair.second) != parents.end())
                {
                    parents.erase(parent_child_pair.first);
                }
            }
            parent_child_pair = find_parent_child_pair();
        }

        auto find_orphan = [&hierarchy]() -> std::optional<size_t>
        {
            for (auto pair : hierarchy)
                if (pair.second.empty())
                    return pair.first;
            return {};
        };

        // pick a starting polygon. create a shape2_base and add children as holes
        std::vector<shape2_base<fp>> result;
        std::optional<size_t> orphan = find_orphan();
        while (orphan.has_value())
        {
            size_t polygon_idx = orphan.value();
            hierarchy.erase(polygon_idx);
            result.emplace_back(polygons[polygon_idx]);
            shape2_base<fp>* curr_shape = &result.back();

            std::queue<std::tuple<size_t, shape2_base<fp>*, size_t>> shapes_locations;  // polygon_idx is located at parent shape, at hole #
            shapes_locations.push({ polygon_idx, nullptr, 0 });

            while (!shapes_locations.empty())
            {
                for (auto it = hierarchy.begin(); it != hierarchy.end();)
                {
                    if (it->second.size() == 1 && *it->second.begin() == polygon_idx)
                    {
                        // [it] is child of current shape

                        //push to queue, set as hole in current shape
                        shapes_locations.push({ it->first, curr_shape, curr_shape->holes.size()});
                        curr_shape->holes.emplace_back(polygons[it->first]);
                        it = hierarchy.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }

                //pick new element from queue
                shapes_locations.pop();
                if (!shapes_locations.empty())
                {
                    polygon_idx = get<0>(shapes_locations.front());
                    size_t hole_idx = get<2>(shapes_locations.front());
                    curr_shape = &get<1>(shapes_locations.front())->holes[hole_idx];
                }
            }

            orphan = find_orphan();
        }

        return shape2(result);
    }

    template <typename fp>
    std::vector<line_segment2<fp>> shape2<fp>::merge_edges(const std::map<edge_key, shape_edge<fp>>& own, const std::map<edge_key, shape_edge<fp>>& other, logic_function op)
    {
        std::vector<line_segment2<fp>> segments;
        for (auto it = own.begin(); it != own.end(); ++it)
        {
            const auto left = op(it->second.left.current, it->second.left.other);
            const auto right = op(it->second.right.current, it->second.right.other);
            if (left != right)
            {
                if (left == ownership::none)
                    segments.push_back(it->second.segment);
                else
                    segments.push_back(line_segment2<fp>(it->second.segment.p2(), it->second.segment.p1()));
            }
        }

        for (auto it = other.begin(); it != other.end(); ++it)
        {
            const auto left = op(it->second.left.other, it->second.left.current);
            const auto right = op(it->second.right.other, it->second.right.current);
            if (left != right)
            {
                if (left == ownership::none)
                    segments.push_back(it->second.segment);
                else
                    segments.push_back(line_segment2<fp>(it->second.segment.p2(), it->second.segment.p1()));
            }
        }

        return segments;
    }

    template <typename fp>
    bool shape2<fp>::check_edges_integrity(const std::vector<line_segment2<fp>>& segments)
    {
        std::map<vertex_key, int32_t> connections;
        for (const auto& s : segments)
        {
            vertex_key vk1 = make_key(s.p1());
            vertex_key vk2 = make_key(s.p2());

            if (connections.contains(vk1))
                connections[vk1] -= 1;
            else
                connections[vk1] = -1;

            if (connections.contains(vk2))
                connections[vk2] += 1;
            else
                connections[vk2] = 1;

        }

        for (auto p : connections)
        {
            if (p.second != 0)
                return false;
        }

        return true;
    }

    template <typename fp>
    line_segment2<fp> shape2<fp>::pop_right_next_segment(std::vector<line_segment2<fp>>& segments, const line_segment2<fp>& current_segment)
    {
        typename std::vector<line_segment2<fp>>::iterator r_it = segments.end();
        for (typename std::vector<line_segment2<fp>>::iterator it = segments.begin(); it != segments.end(); it++)
        {
            if (current_segment.p2() == it->p1())
            {
                if (r_it == segments.end())
                    r_it = it;
                else
                {
                    if (is_right_side_of_edges_joint(current_segment, *r_it, it->p2()))
                        r_it = it;
                }
            }
        }

        assert(r_it != segments.end());
        line_segment2 next_segment = *r_it;
        segments.erase(r_it);
        return next_segment;
    }

    template <typename fp>
    std::vector<point2<fp>> shape2<fp>::build_poly_contour(std::vector<line_segment2<fp>>& chain, std::vector<line_segment2<fp>>& segments)
    {
        while (true)
        {
            line_segment2 next_segment = pop_right_next_segment(segments, chain.back());
            chain.push_back(next_segment);

            point2 last_point = next_segment.p2();
            auto first_segment = std::find_if(chain.begin(), chain.end(), [&last_point](const line_segment2<fp>& x) { return x.p1() == last_point; });
            if (first_segment != chain.end())
            {
                std::vector<point2<fp>> poly_points;
                for (typename std::vector<line_segment2<fp>>::iterator it = first_segment; it < chain.end(); it++)
                    poly_points.push_back(it->p1());

                chain.erase(first_segment, chain.end());
                return poly_points;
            }
        }

        return {};
    }

    template <typename fp>
    std::vector<polygon2<fp>> shape2<fp>::edges_to_polygons(std::vector<line_segment2<fp>>& segments)
    {
        if (segments.size() < 3)
            return {};

        std::vector<polygon2<fp>> polygons;
        std::vector<line_segment2<fp>> chain;
        while (segments.size() > 2)
        {
            if (chain.size() == 0)
            {
                chain.push_back(segments.back());
                segments.pop_back();
            }

            std::vector<point2<fp>> poly_points = build_poly_contour(chain, segments);
            polygons.push_back(polygon2(poly_points));
        }

        return polygons;
    }

    template <typename fp>
    std::vector<polygon2<fp>> shape2<fp>::edges_to_polygons2(std::vector<line_segment2<fp>>& segments)
    {
        if (segments.size() < 3)
            return {};

        std::vector<std::vector<debug_point2>> result;
        std::map<debug_point2, std::vector<debug_point2>> graph = build_graph(segments);

        auto remove_adjacency = [&graph](const debug_point2& p1, debug_point2& p2)
            {
                if (graph.contains(p1))
                {
                    graph[p1].erase(std::find(graph[p1].begin(), graph[p1].end(), p2));
                    if (graph[p1].empty())
                        graph.erase(p1);
                }
            };

        while (!graph.empty())
        {
            std::vector<debug_point2> cycle;
            size_t shortest_depth = std::numeric_limits<size_t>::max();

            for (size_t i = 0; i < graph.size(); i++)
            {
                auto it = std::next(graph.begin(), i);
                debug_point2 start(it->first);
                auto curr_cycle = find_cycle(graph, start, shortest_depth);
                if (!curr_cycle.empty())
                {
                    shortest_depth = curr_cycle.size() - 1;
                    cycle = std::move(curr_cycle);
                }
            }

            assert(!cycle.empty());
            if (!cycle.empty())
            {
                for (size_t i = 0; i < cycle.size() - 1; i++)
                {
                    remove_adjacency(cycle[i], cycle[i + 1]);
                    remove_adjacency(cycle[i + 1], cycle[i]);
                }
                remove_adjacency(cycle.front(), cycle.back());
                remove_adjacency(cycle.back(), cycle.front());
                result.push_back(std::move(cycle));
            }
        }

        std::vector<polygon2<fp>> polygons;
        polygons.reserve(result.size());
        for (const auto& cycle : result)
        {
            std::vector<point2<fp>> points;
            points.reserve(cycle.size());
            for (const auto& p : cycle)
                points.push_back({ 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.first), 
                    sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.second) });

            polygons.push_back(polygon2<fp>(points));
        }
        return polygons;
    }

    template <typename fp>
    std::map<typename shape2<fp>::debug_point2, std::vector<typename shape2<fp>::debug_point2>> shape2<fp>::build_graph(const std::vector<line_segment2<fp>>& edges)
    {
        std::map<debug_point2, std::vector<debug_point2>> map;
        for (const line_segment2<fp>& e : edges)
        {
            debug_point2 p1(
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.p1().vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.p1().vec().y));
            debug_point2 p2(
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.p2().vec().x), 
                sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(e.p2().vec().y));
            map[p1].push_back(p2);
            map[p2].push_back(p1);
        }
        return map;
    }

    template <typename fp>
    std::vector<typename shape2<fp>::debug_point2> shape2<fp>::find_cycle(const std::map<debug_point2, std::vector<debug_point2>>& adjacency, const debug_point2& start, size_t max_depth)
    {
        std::queue<debug_point2> queue;
        std::set<debug_point2> visited;
        std::set<debug_edge> used_edge;
        std::map<debug_point2, size_t> depth;
        std::map<debug_point2, debug_point2> parent; // point, parent
        visited.insert(start);
        queue.push(start);
        depth[start] = 0;

        auto get_path = [&parent](const debug_point2* p)
            {
                std::vector<debug_point2> path{ *p };
                while (parent.contains(*p))
                {
                    p = &parent[*p];
                    path.push_back(*p);
                }
                return path;
            };

        while (!queue.empty())
        {
            const debug_point2 p = queue.front();
            queue.pop();

            if (depth[p] >= max_depth)
                break;

            for (const auto& q : adjacency.at(p))
            {
                if (visited.contains(q))
                {
                    if (!used_edge.contains({ p,q }) && !used_edge.contains({ q,p }))
                    {
                        if (depth[p] + depth[q] < max_depth)
                        {
                            auto path1 = get_path(&p);
                            auto path2 = get_path(&q);
                            std::reverse(path1.begin(), path1.end());
                            path1.insert(path1.end(), path2.begin(), path2.end() - 1);
                            return path1;
                        }
                    }
                }
                else
                {
                    depth[q] = depth[p] + 1;
                    visited.insert(q);
                    parent[q] = p;
                    queue.push(q);
                    used_edge.insert({ p,q });
                }
            }
        }

        return std::vector<debug_point2>();
    }

};
};

#endif //sixit_geometry_shape2_impl_h_included

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