/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_indexed_polygon2_impl_h_included
#define sixit_geometry_indexed_polygon2_impl_h_included

#include "sixit/geometry/indexed_polygon2.h"
#include "sixit/geometry/indexed.h"
#include "sixit/geometry/low_level/polygon_common_impl.h"
#include "sixit/geometry/indexed_polygon2.h"

#include <numeric>
#include <set>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    indexed_polygon2<fp>::indexed_polygon2(low_level::reference_container<point2<fp>>* ref, const std::vector<size_t>& indices)
        :low_level::indexed_vector<point2<fp>, polygon2<fp>>(ref, indices) {}

    template <typename fp>
    indexed_polygon2<fp>::indexed_polygon2(const indexed_triangle2<fp>& tri)
        :low_level::indexed_vector<point2<fp>, polygon2<fp>>(tri.ref_container, { tri.index.begin(), tri.index.end() }) {}

    template <typename fp>
    std::vector<indexed_point2<fp>> indexed_polygon2<fp>::get_indexed_points() const
    {
        std::vector<indexed_point2<fp>> indexed_points;
        indexed_points.reserve(indexed_polygon2<fp>::index.size());
        for (size_t idx : this->index)
        {
            indexed_points.push_back({ indexed_polygon2<fp>::ref_container, idx });
        }
        return indexed_points;
    }

    template <typename fp>
    int64_t indexed_polygon2<fp>::n_vertices() const
    {
        return ssize(indexed_polygon2<fp>::index);
    }

    template <typename fp>
    indexed_point2<fp> indexed_polygon2<fp>::vertex(int64_t idx) const //do we need this?
    {
        assert(idx < ssize(indexed_polygon2<fp>::index));
        return indexed_point2(indexed_polygon2<fp>::ref_container, indexed_polygon2<fp>::index[idx]);
    }

    template <typename fp>
    size_t indexed_polygon2<fp>::n_edges() const
    {
        return n_vertices();
    }

    template <typename fp>
    indexed_line_segment2<fp> indexed_polygon2<fp>::edge(size_t idx) const
    {
        assert(idx < n_edges());
        size_t f_p = idx;
        size_t sp;
        if (idx + 1 >= n_edges())
        {
            sp = 0;
        }
        else
        {
            sp = idx + 1;
        }

        return indexed_line_segment2(indexed_polygon2<fp>::ref_container, 
                                        indexed_polygon2<fp>::index[f_p], 
                                        indexed_polygon2<fp>::index[sp]);
    }

    template <typename fp>
    ray2<fp> indexed_polygon2<fp>::bisector(int64_t idx) const
    {
        assert(idx >= 0 && idx < n_vertices());
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p1 = vertex(idx == 0 ? n_vertices() - 1 : idx - 1).vec();
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p2 = vertex(idx).vec();
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p3 = vertex(idx == n_vertices() - 1 ? 0 : idx + 1).vec();

        low_level::dimensional_vector2 d1 = (p1 - p2).normalized();
        low_level::dimensional_vector2 d3 = (p3 - p2).normalized();
        low_level::dimensional_vector2 dir(d1 + d3);

        if (dir == low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::zero())
        {
            dir = (p3 - p1) / sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f);
        }

        dir *= sixit::units::create_dimensionless_scalar<fp>(normal());
        return ray2<fp>(p2, direction2(dir.x, dir.y));
    }

    template <typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> indexed_polygon2<fp>::square() const
    {
        std::vector<indexed_triangle2<fp>> tris;
        to_triangles(tris);
        sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> res = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.f);
        for (const auto& t : tris)
        {
            res += t.square();
        }
        return res;
    }

    template <typename fp>
    bounds2<fp> indexed_polygon2<fp>::bounds() const
    {
        return low_level::polygon2_impl<fp>::bounds(indexed_polygon2<fp>::get_points());
    }

    template <typename fp>
    template <typename T2>
    void indexed_polygon2<fp>::to_triangles(std::vector<T2>& res) const
    {
        assert(indexed_polygon2<fp>::index.size() >= 3);

        std::vector<size_t> indices;
        indices.resize(indexed_polygon2<fp>::index.size());
        std::iota(indices.begin(), indices.end(), 0);
        
        int norm = normal();

       // assert(norm != 0);

        indexed_triangle2 tri(indexed_polygon2<fp>::ref_container, indexed_polygon2<fp>::index[0], 
                                indexed_polygon2<fp>::index[0], indexed_polygon2<fp>::index[0]);
        size_t i1, i2, i3;

        auto get_tri = [&](size_t i)
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
            tri = indexed_triangle2(indexed_polygon2<fp>::ref_container, indexed_polygon2<fp>::index[i1], 
                                    indexed_polygon2<fp>::index[i2], indexed_polygon2<fp>::index[i3]);
        };

        auto convert_tri = [](const indexed_triangle2<fp>& tr)
        {
            if constexpr (std::is_same<T2, indexed_triangle2<fp>>::value)
                return tr;
            else
                return tr.get();
        };

        size_t ii = 0;
        while (indices.size() > 3)
        {
            if (ii >= indices.size())
                break;

            get_tri(ii);

            if (tri.normal() != norm && norm != 0)
            {
                ++ii;
                continue;
            }
            else
            {
                size_t i = 0;
                for (; i < indices.size(); ++i)
                {
                    if (i == i1 || i == i2 || i == i3)
                        continue;
                    if (tri.is_inside_combinable(indexed_polygon2<fp>::get_point(indices[i])))
                        break;
                }
                if (i != indices.size())
                {
                    ++ii;
                    continue;
                }
                else
                {
                    res.push_back(convert_tri(tri));
                    indices.erase(indices.begin() + ii);
                    ii = 0;
                }
            }
        }
        res.push_back(convert_tri(indexed_triangle2<fp>(indexed_polygon2<fp>::ref_container, indexed_polygon2<fp>::index[indices[0]], 
                                                    indexed_polygon2<fp>::index[indices[1]], indexed_polygon2<fp>::index[indices[2]])));
    }

    template <typename fp>
    int indexed_polygon2<fp>::normal() const
    {
        return low_level::polygon2_impl<fp>::normal(indexed_polygon2<fp>::get_points()); //or should this use get_indexed_points() instead?
    }

    template <typename fp>
    template <typename P2>
    bool indexed_polygon2<fp>::is_inside(const P2& point) const
    {
        //is there performance improvement by using get_point() instead of vertex().vec() ?
        auto vec = point.vec();
        bool inside = false;
        low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> vi = {
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f),
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f)
        };
        low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> vj = vertex(n_vertices() - 1).vec();
        for (int64_t i = 0; i < n_vertices(); i++)
        {
            vi = vertex(i).vec();
            // y is between the 2 adjacent points &&  x is at the left of the proportional x of the segment
            if ((vi.y > vec.y) != (vj.y > vec.y) && (vec.x < (vj.x - vi.x) * (vec.y - vi.y) / (vj.y - vi.y) + vi.x))
            {
                inside = !inside;
            }
            vj = vi;
        }
        return inside;
    }

    template <typename fp>
    typename indexed_polygon2<fp>::overlap_topology indexed_polygon2<fp>::overlaps(const indexed_triangle2<fp>& tri) const
    {
        assert(indexed_polygon2<fp>::ref_container == tri.ref_container);
        std::map<int64_t, std::vector<int64_t>> common_indices1 = common_indices(*this, tri);
        std::map<int64_t, std::vector<int64_t>> common_indices2 = common_indices(tri, *this);

        for (int64_t i = 0; i < n_vertices(); i++)
        {
            if (common_indices1.find(i) == common_indices1.end() && tri.is_inside(vertex(i).vec()))
            {
                return overlap_topology::overlapping;
            }
        }

        for (int64_t i = 0; i < tri.n_vertices(); i++)
        {
            if (common_indices2.find(i) == common_indices2.end() && is_inside(indexed_polygon2<fp>::tri.vertex(i)))
            {
                return overlap_topology::overlapping;
            }
        }

        //// edge intersection
        bool has_common_edges = false;
        for (size_t i = 0; i < n_edges(); i++)
        {
            size_t idx = indexed_polygon2<fp>::index[i];
            size_t idx_next = indexed_polygon2<fp>::index[i < n_edges() - 1 ? i + 1 : 0];
            indexed_line_segment2 poly_edge = edge(i);

            for (int64_t j = 0; j < tri.n_edges(); j++)
            {
                //do not compare segments that share vertices (always intersect)
                size_t idx_j = tri.index[j];
                size_t idx_next_j = tri.index[j < tri.n_edges() - 1 ? j + 1 : 0];

                if (idx == idx_j || idx_next == idx_j || idx == idx_next_j || idx_next == idx_next_j)
                {
                    if (!has_common_edges && ((idx == idx_j && idx_next == idx_next_j) || (idx_next == idx_j && idx == idx_next_j)))
                    {
                        has_common_edges = true;
                    }
                    continue;
                }

                if (segments_intersect(poly_edge, indexed_polygon2<fp>::tri.edge(j)))
                {
                    return overlap_topology::overlapping;
                }
            }
        }

        bool tri_is_subset = ssize(common_indices2) == tri.n_vertices();
        bool poly_is_subset = ssize(common_indices1) == n_vertices();

        if (poly_is_subset && tri_is_subset)
        {
            return overlap_topology::overlapping;
        }

        if (poly_is_subset || tri_is_subset)  // overlap or share all edges
        {
            // rationale: for both shapes:
            //  obtain the order of the indices that compose the sub polygon
            //  obtain the orientation of its shape (the "normal")
            //  if both index order is the same and both normals have the same orientation then they overlap

            std::vector<size_t> tri_index = { tri.index[0], tri.index[1], tri.index[2] };
            const auto set = poly_is_subset ? indexed_polygon2<fp>::tri_index : indexed_polygon2<fp>::index;
            const auto subset = poly_is_subset ? indexed_polygon2<fp>::index : indexed_polygon2<fp>::tri_index;
            const auto first_match_index = poly_is_subset ? common_indices1.at(0).front() : common_indices2.at(0).front();

            auto is_subset_in_order = [](const std::vector<size_t>& subset, const std::vector<size_t>& set, size_t first_match_set, bool reverse)
            {
                size_t si = 0;
                int64_t count = ssize(set);
                if (reverse)
                {
                    for (int64_t i = count -1; i >= 0; i++)
                    {
                        if (subset[si] == set[(first_match_set + i) % count])
                        {
                            if (++si == subset.size())
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    for (int64_t i = 0; i < count; i++)
                    {
                        if (subset[si] == set[(first_match_set + i) % count])
                        {
                            if (++si == subset.size())
                            {
                                return true;
                            }
                        }
                    }
                }
                return false;
            };

            bool same_index_order = is_subset_in_order(subset, set, first_match_index, false);

            if (!same_index_order)
            {
                bool reverse_index_order = is_subset_in_order(subset, set, first_match_index, true);
                if (!reverse_index_order)
                {
                    //indices does not follow an order. is this a valid polygon?
                    assert(false);
                }
            }

            bool same_orientation = normal() == tri.normal();

            if (same_orientation == same_index_order)
            {
                return overlap_topology::overlapping;
            }
            return overlap_topology::has_common_edge;
        }

        if (has_common_edges)
        {
            return overlap_topology::has_common_edge;
        }

        if (!common_indices1.empty())
        {
            return indexed_polygon2::overlap_topology::has_common_vertex;
        }
        return (indexed_polygon2::overlap_topology)0;
    }

    template <typename fp>
    typename indexed_polygon2<fp>::overlap_topology indexed_polygon2<fp>::overlaps(const indexed_polygon2<fp>& other) const
    {
        assert(indexed_polygon2<fp>::ref_container == other.ref_container);
        std::map<int64_t, std::vector<int64_t>> common_indices1 = common_indices(*this, other);
        std::map<int64_t, std::vector<int64_t>> common_indices2 = common_indices(other, *this);
        
        for (int64_t i = 0; i < n_vertices(); i++)
        {
            if (common_indices1.find(i) == common_indices1.end() && other.is_inside(vertex(i)))
            {
                return overlap_topology::overlapping;
            }
        }

        for (int64_t i = 0; i < other.n_vertices(); i++)
        {
            if (common_indices2.find(i) == common_indices2.end() && is_inside(other.vertex(i)))
            {
                return overlap_topology::overlapping;
            }
        }

        //// edge intersection
        bool has_common_edges = false;
        for (size_t i = 0; i < n_edges(); i++)
        {
            size_t idx = indexed_polygon2<fp>::index[i];
            size_t idx_next = this->index[i < n_edges() - 1 ? i + 1 : 0];
            indexed_line_segment2 poly_edge = indexed_polygon2<fp>::edge(i);

            for (size_t j = 0; j < other.n_edges(); j++)
            {
                //do not compare segments that share vertices (always intersect)
                size_t idx_j = other.index[j];
                size_t idx_next_j = other.index[j < other.n_edges() - 1 ? j + 1 : 0];

                if (idx == idx_j || idx_next == idx_j || idx == idx_next_j || idx_next == idx_next_j)
                {
                    if (!has_common_edges && ((idx == idx_j && idx_next == idx_next_j) || (idx_next == idx_j && idx == idx_next_j)))
                    {
                        has_common_edges = true;
                    }
                    continue;
                }

                if (segments_intersect(poly_edge, other.edge(j)))
                {
                    return overlap_topology::overlapping;
                }
            }
        }

        bool other_is_subset = ssize(common_indices2) == other.n_vertices();
        bool this_is_subset = ssize(common_indices1) == n_vertices();

        if (this_is_subset && other_is_subset)
        {
            return overlap_topology::overlapping;
        }

        if (this_is_subset || other_is_subset) // overlap or share all edges
        {
            // rationale: for both shapes:
            //  obtain the order of the indices that compose the sub polygon
            //  obtain the orientation of its shape (the "normal")
            //  if both index order is the same and both normals have the same orientation then they overlap

            const auto set = this_is_subset ? other.index : this->index;
            const auto subset = this_is_subset ? this->index : other.index;
            const auto first_match_index = this_is_subset ? common_indices1.at(0).front() : common_indices2.at(0).front();

            auto is_subset_in_order = [](const std::vector<size_t>& subset, const std::vector<size_t>& set, size_t first_match_set, bool reverse)
            {
                size_t si = 0;
                int64_t count = ssize(set);
                if (reverse)
                {
                    for (int64_t i = count - 1; i >= 0; i++)
                    {
                        if (subset[si] == set[(first_match_set + i) % count])
                        {
                            if (++si == subset.size())
                            {
                                return true;
                            }
                        }
                    }
                }
                else
                {
                    for (int64_t i = 0; i < count; i++)
                    {
                        if (subset[si] == set[(first_match_set + i) % count])
                        {
                            if (++si == subset.size())
                            {
                                return true;
                            }
                        }
                    }
                }
                return false;
            };

            bool same_index_order = is_subset_in_order(subset, set, first_match_index, false);

            if (!same_index_order)
            {
                bool reverse_index_order = is_subset_in_order(subset, set, first_match_index, true);
                if (!reverse_index_order)
                {
                    //indices does not follow an order. is this a valid polygon?
                    assert(false);
                }
            }

            bool same_orientation = normal() == other.normal();

            if (same_orientation == same_index_order)
            {
                return overlap_topology::overlapping;
            }
            return overlap_topology::has_common_edge;
        }

        if (has_common_edges)
        {
            return overlap_topology::has_common_edge;
        }

        if (!common_indices1.empty())
        {
            return indexed_polygon2::overlap_topology::has_common_vertex;
        }
        return (indexed_polygon2::overlap_topology)0;
    }

    template <typename fp>
    void indexed_polygon2<fp>::join(const indexed_triangle2<fp>& tri)
    {
        //polygons should not have repeated points
        if (overlaps(tri) == overlap_topology::has_common_edge)
        {
            auto tri_common_pts = common_indices(tri, * this);
            if (tri_common_pts.size() == 2) // add missing point
            {
                size_t tri_idx0 = tri.index[0];
                size_t tri_idx1 = tri.index[1];

                bool found_0 = tri_common_pts.find(tri_idx0) == tri_common_pts.end();
                bool found_1 = tri_common_pts.find(tri_idx1) == tri_common_pts.end();
                size_t idx_to_add = !found_0 ? tri_idx0 : (!found_1 ? tri_idx1 : tri.index[2]);

                size_t i = tri_common_pts.begin()->second.front();
                size_t j = tri_common_pts.rbegin()->second.front();
                size_t min = std::min(i, j);
                size_t max = std::max(i, j);
                if (min == 0 && max == indexed_polygon2<fp>::index.size()-1)
                {
                    indexed_polygon2<fp>::index.insert(indexed_polygon2<fp>::index.end(), idx_to_add);
                }
                else
                {
                    indexed_polygon2<fp>::index.insert(indexed_polygon2<fp>::index.begin() + min + 1, idx_to_add);
                }
            }
            else if(tri_common_pts.size() == 3) // delete 2 edges
            {
                bool two_edges_shared = false;

                for (const auto& pair : tri_common_pts)
                {
                    int64_t i = pair.second.front();
                    size_t prev_ref_idx = this->index[0 ? n_vertices() - 1 : i - 1];      // prev cyclic index
                    size_t next_ref_idx = this->index[i == n_vertices() - 1 ? 0 : i + 1]; // next cyclic index

                    if (std::find(tri.index.begin(), tri.index.end(), prev_ref_idx) != tri.index.end()
                        && std::find(tri.index.begin(), tri.index.end(), next_ref_idx) != tri.index.end())
                    {
                        two_edges_shared = true;
                        indexed_polygon2<fp>::index.erase(indexed_polygon2<fp>::index.begin() + i, indexed_polygon2<fp>::index.begin() + i+1);
                        break;
                    }
                }

                if (!two_edges_shared)
                {
                    // 2 shared edges and 1 shared point. should not join into a polygon
                }
            }
        }
    }

    template <typename fp>
    void indexed_polygon2<fp>::join(const indexed_polygon2<fp>& other)
    {
        if (overlaps(other) == overlap_topology::has_common_edge)
        {
            struct edge_info
            {
                size_t ref_idx0, ref_idx1;  // relative to A
                size_t a_begin, a_end;      // indices on poly A
                size_t b_begin, b_end;      // indices on poly B
                bool incremental_order;
                edge_info(size_t ref_idx0, size_t  ref_idx1, size_t a_begin, size_t a_end, size_t b_begin, size_t b_end, bool incremental_order)
                    : ref_idx0(ref_idx0), ref_idx1(ref_idx1), a_begin(a_begin), a_end(a_end), b_begin(b_begin), b_end(b_end), incremental_order(incremental_order) {}
            };

            //collect edges information
            auto common_pts = common_indices(*this, other);
            std::vector< edge_info> edge_infos;
            for (auto& pair : common_pts)
            {
                const int64_t i = pair.first;
                const size_t ref_index_a = indexed_polygon2<fp>::index[i];
                const size_t next_i = i == n_vertices() - 1 ? 0 : i + 1;

                const auto next_it = common_pts.find(next_i);
                if (next_it != common_pts.end())// next point is also common point
                {
                    for (const int64_t& other_i : pair.second)
                    {
                        //foreach commmon point find if the adjacent is also the [i+1]
                        size_t ref_index_b = indexed_polygon2<fp>::index[next_i];
                        const int64_t other_prev_idx = other_i == 0 ? other.n_vertices() - 1 : other_i - 1;
                        const int64_t other_next_idx = other_i == other.n_vertices() - 1 ? 0 : other_i + 1;

                        auto other_vec = next_it->second;
                        if (std::find(other_vec.begin(), other_vec.end(), other_prev_idx) != other_vec.end())
                        {
                            // pair found in reverse order
                            edge_infos.push_back(edge_info(ref_index_a, ref_index_b, i, next_i, other_i, other_prev_idx, false));
                        }
                        if (std::find(other_vec.begin(), other_vec.end(), other_next_idx) != other_vec.end())
                        {
                            // pair found in same order
                            edge_infos.push_back(edge_info(ref_index_a, ref_index_b, i, next_i, other_i, other_next_idx, true));
                        }
                    }
                }
            }

            // merge consecutive edges
            for (size_t i = edge_infos.size(); i > 1; i--)
            {
                auto& curr = edge_infos[i - 1];
                auto& prev = edge_infos[i - 2];
                if (prev.a_end == curr.a_begin)
                {
                    prev.a_end = curr.a_end;
                    prev.b_end = curr.b_end;
                    prev.ref_idx1 = curr.ref_idx1;

                    edge_infos.erase(edge_infos.begin() + i-1);
                }
            }


            if (edge_infos.size() > 1)
            {
                //do not handle more than 1 border. result should be a shape2
                return;
            }

            //calculate segments between edges
            auto get_segment_between_edges = [](const std::vector<size_t>& indices, const edge_info& from, const edge_info& to, bool a_side )
            {
                size_t begin, end;
                if (a_side)
                {
                    begin = from.a_end;
                    end = to.a_begin;
                }
                else
                {
                    begin = to.incremental_order? from.b_end : to.b_begin;
                    end = to.incremental_order ? to.b_begin : from.b_end;
                }

                std::vector<size_t> segment_points;
                if (begin < end)
                {
                    segment_points.insert(segment_points.begin(), indices.begin() + begin, indices.begin() + end + 1);
                }
                else
                {
                    segment_points.insert(segment_points.begin(), indices.begin() + begin, indices.end());
                    segment_points.insert(segment_points.end(), indices.begin(), indices.begin() + end + 1);
                }
                return segment_points;
            };

            // connect segments from different polygons
            std::vector< std::vector<size_t>> segments;
            std::vector<size_t> join_indices;
            for (size_t i = 0; i < edge_infos.size(); i++)
            {
                const edge_info& edge = edge_infos[i];
                const edge_info& next_edge = edge_infos[(i + 1) % edge_infos.size()];

                std::vector<size_t> segment_a = get_segment_between_edges(this->index, edge, next_edge, true);
                std::vector<size_t> segment_b = get_segment_between_edges(other.index, edge, next_edge, false);
                if(edge.incremental_order)
                    std::reverse(segment_b.begin(), segment_b.end());

                std::vector<size_t> complete;
                bool append_a = edge.a_begin != edge.a_end;
                bool append_b = edge.b_begin != edge.b_end;
                if (append_a && append_b)
                {
                    complete = segment_a;
                    complete.insert(complete.end(), segment_b.begin() + 1, segment_b.begin() + segment_b.size() - 1);
                }
                else if(append_a)
                {
                    complete = segment_a;
                }
                else if (append_b)
                {
                    complete = segment_b;
                }
                segments.push_back(complete);
                join_indices.push_back(segment_a.size()-1);
            }

            //if there are multiple segments, this means there are holes. connect them
            std::vector<size_t> result = segments.front();
            if (segments.size() > 1)
            {
                size_t join_index = join_indices.front();
                for (size_t i = 1; i < segments.size(); i++)
                {
                    std::vector<size_t>& seg = segments[i];

                    result.insert(result.begin() + join_index, result[join_index]);     //duplicate join index
                    result.insert(result.begin() + join_index + 1, seg[0]);             // duplicate other join index
                    result.insert(result.begin() + join_index + 1, seg.begin(), seg.end());        

                    join_index += join_indices[i] +1;
                }
            }
            this->index = result;
        }
    }

    template <typename fp>
    template <typename T1, typename T2>
    std::map<int64_t, std::vector<int64_t>> geometry::indexed_polygon2<fp>::common_indices(const T1& a, const T2& b)
    {
        assert(a.ref_container == b.ref_container);
        std::map<int64_t, std::vector<int64_t>> indices; // this , other

        for (int64_t i = 0; i < a.n_vertices(); i++)
        {
            auto idx_a = a.index[i];
            for (int64_t j = 0; j < b.n_vertices(); j++)
            {
                if (idx_a == b.index[j])
                {
                    indices[i].push_back(j);
                }
            }
        }
        return indices;
    }

    template <typename fp>
    bool geometry::indexed_polygon2<fp>::segments_intersect(const indexed_line_segment2<fp>& ls1, const indexed_line_segment2<fp>& ls2)
    {
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& a = ls1.p1().vec();
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& b = ls1.p2().vec();
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& a2 = ls2.p1().vec();
        const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& b2 = ls2.p2().vec();

        if (a == a2 || a == b2 || b == a2 || b == b2)
        {
            return true;
        }

        int normal1 = low_level::tri2_impl<fp>::normal(a, b, a2);
        int normal2 = low_level::tri2_impl<fp>::normal(a, b, b2);
        int normal3 = low_level::tri2_impl<fp>::normal(a2, b2, a);
        int normal4 = low_level::tri2_impl<fp>::normal(a2, b2, b);

        if (normal1 != normal2 && normal3 != normal4)
        {
            return true;
        }

        auto on_segment = [](const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p, 
                                const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& q, 
                                const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& r)
        {
            return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x)
                && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
        };

        // Special cases
        if (normal1 == 0 && on_segment(a, a2, b))
        {
            return true;
        }

        if (normal2 == 0 && on_segment(a, b2, b))
        {
            return true;
        }

        if (normal3 == 0 && on_segment(a2, a, b2))
        {
            return true;
        }

        if (normal4 == 0 && on_segment(a2, b, b2))
        {
            return true;
        }

        return false;  // No intersection
    }

}
};

#endif //sixit_geometry_indexed_polygon2_impl_h_included

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