/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_voxelizer_h_included
#define sixit_geometry_voxelizer_h_included

#include "sixit/geometry/geometry.h"
#include "sixit/geometry/point.h"
#include "sixit/graphics/mesh/base_mesh3.h"
#include <iostream>
#include <vector>
#include <cmath>

#include "grid3.h"

namespace sixit
{
namespace geometry
{

class voxelizer
{
    public:

    struct tri
    {
        int id;
        triangle3f triangle;
    };

    struct tricell
    {
        std::vector<size_t> vertices;
        std::vector<tri> triangles;
    };

    struct voxelization_data
    {
        std::vector<std::vector<std::vector<tricell>>> cells;
        std::unordered_map<int, std::vector<std::tuple<int, int, int>>> triangle_cells;
    };

    // Distributes mesh vertices by grid cells
    voxelization_data voxelize_points(const grid3f& g, const sixit::graphics::gpu_base_mesh3& m, int const res)
    {
        voxelization_data vd;
        vd.cells = std::vector<std::vector<std::vector<tricell>>>(res, std::vector<std::vector<tricell>>(res, std::vector<tricell>(res)));
        
        for (int i = 0; i < m.vertices_.get_size(); i++)
        {
            point3f p = m.vertices_[i];
            grid_cell3 gc = g.point3_to_grid_cell(p);
            int64_t x = gc.x;
            int64_t y = gc.y;
            int64_t z = gc.z;

            if ( x < 0 || x >= res || y < 0 || y >= res || z < 0 || z >= res )
                continue;
            
            vd.cells[x][y][z].vertices.push_back(i);
        }
        return vd;
    }

    // Voxelizes triangles by their bounds, for the whole mesh.
    voxelization_data voxelize_bounds(const grid3f& g, const sixit::graphics::gpu_base_mesh3& m, int const res, point3f margin = point3f())
    {
        voxelization_data vd;
        vd.cells = std::vector<std::vector<std::vector<tricell>>>(res, std::vector<std::vector<tricell>>(res, std::vector<tricell>(res)));
        
        for (int i = 0; i < m.indices_.get_size(); i += 3)
        {
            point3f p0 = m.vertices_[m.indices_[i + 0]];
            point3f p1 = m.vertices_[m.indices_[i + 1]];
            point3f p2 = m.vertices_[m.indices_[i + 2]];
            triangle3f t3 (p0, p1, p2);

            // optimize: limit grid by triangle bounds
            auto tb = bounds3f(p0);
            tb.expand_by(p1);
            tb.expand_by(p2);

            grid_cell3 g_min = g.point3_to_grid_cell(tb.minn.vec() - margin.vec());
            grid_cell3 g_max = g.point3_to_grid_cell(tb.maxx.vec() + margin.vec());
            int64_t min_x = std::max<int64_t>(0, g_min.x);
            int64_t min_y = std::max<int64_t>(0, g_min.y);
            int64_t min_z = std::max<int64_t>(0, g_min.z);
            int64_t max_x = std::min<int64_t>(res-1, g_max.x);
            int64_t max_y = std::min<int64_t>(res-1, g_max.y);
            int64_t max_z = std::min<int64_t>(res-1, g_max.z);
            
            // single cell check
            if (min_x == max_x && min_y == max_y && min_z == max_z)
            {
                vd.cells[min_x][min_y][min_z].triangles.push_back({i, t3});
                vd.triangle_cells[i].emplace_back(std::make_tuple(min_x, min_y, min_z));
                continue;
            }

            for (int64_t z = min_z; z <= max_z; z++)
            {
                for (int64_t y = min_y; y <= max_y; y++)
                {
                    for (int64_t x = min_x; x <= max_x; x++)
                    {
                        vd.cells[x][y][z].triangles.push_back({i, t3});
                        vd.triangle_cells[i].emplace_back( std::make_tuple(x, y, z) );
                    }
                }
            }
        }
        return vd;
    }

    // Voxelizes triangles for the whole mesh.
    voxelization_data voxelize_sw_sd(const grid3f& g, const sixit::graphics::gpu_base_mesh3& m, int const res)
    {
        voxelization_data vd;
        vd.cells = std::vector<std::vector<std::vector<tricell>>>(res, std::vector<std::vector<tricell>>(res, std::vector<tricell>(res)));
        
        for (int i = 0; i < m.indices_.get_size(); i += 3)
        {
            point3f p0 = m.vertices_[m.indices_[i + 0]];
            point3f p1 = m.vertices_[m.indices_[i + 1]];
            point3f p2 = m.vertices_[m.indices_[i + 2]];
            triangle3f t3 (p0, p1, p2);

            // optimize: limit grid by triangle bounds
            auto tb = bounds3f(p0);
            tb.expand_by(p1);
            tb.expand_by(p2);

            grid_cell3 g_min = g.point3_to_grid_cell(tb.minn);
            grid_cell3 g_max = g.point3_to_grid_cell(tb.maxx);
            int64_t min_x = g_min.x;
            int64_t min_y = g_min.y;
            int64_t min_z = g_min.z;
            int64_t max_x = g_max.x;
            int64_t max_y = g_max.y;
            int64_t max_z = g_max.z;

            // single cell check
            if (min_x == max_x && min_y == max_y && min_z == max_z)
            {
                vd.cells[min_x][min_y][min_z].triangles.push_back({i, t3});
                vd.triangle_cells[i].emplace_back(std::make_tuple(min_x, min_y, min_z));
                continue;
            }

            for (int64_t z = min_z; z <= max_z; z++)
            {
                auto cz = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(z));
                for (int64_t y = min_y; y <= max_y; y++)
                {
                    auto cy = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(y));
                    for (int64_t x = min_x; x <= max_x; x++)
                    {
                        auto cx = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(x));
                        point3f tp0((p0.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_), 
                            (p0.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_), 
                            (p0.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp1((p1.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_), 
                            (p1.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_), 
                            (p1.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp2((p2.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_), 
                            (p2.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_), 
                            (p2.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                        if (intersect_cell(tp0, tp1, tp2))
                        {
                            vd.cells[x][y][z].triangles.push_back({i, t3});
                            vd.triangle_cells[i].emplace_back( std::make_tuple(x, y, z) );
                        }
                    }
                }
            }
        }
        return vd;
    }

    // Voxelizes a single triangle.
    voxelization_data voxelize_sw_sd(const grid3f& g, const sixit::geometry::triangle3f& t, int const res)
    {
        voxelization_data vd;
        
        // optimize: limit grid by triangle bounds
        point3f p0 = std::get<0>(t.vertices());
        point3f p1 = std::get<1>(t.vertices());
        point3f p2 = std::get<2>(t.vertices());
        auto tb = bounds3f(p0);
        tb.expand_by(p1);
        tb.expand_by(p2);

        int64_t min_x = std::max<int64_t>(0, g.toFloorGridX(tb.minn.vec().x()));
        int64_t min_y = std::max<int64_t>(0, g.toFloorGridY(tb.minn.vec().y()));
        int64_t min_z = std::max<int64_t>(0, g.toFloorGridZ(tb.minn.vec().z()));
        int64_t max_x = std::min<int64_t>(res-1, g.toFloorGridX(tb.maxx.vec().x()));
        int64_t max_y = std::min<int64_t>(res-1, g.toFloorGridY(tb.maxx.vec().y()));
        int64_t max_z = std::min<int64_t>(res-1, g.toFloorGridZ(tb.maxx.vec().z()));

        // single cell check
        if (min_x == max_x && min_y == max_y && min_z == max_z)
        {
            vd.triangle_cells[0].emplace_back(std::make_tuple(min_x, min_y, min_z));
            return vd;
        }

        for (int64_t z = min_z; z <= max_z; z++)
        {
            auto cz = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(z));
            for (int64_t y = min_y; y <= max_y; y++)
            {
                auto cy = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(y));
                for (int64_t x = min_x; x <= max_x; x++)
                {
                    auto cx = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(x));
                    point3f tp0((p0.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                        (p0.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                        (p0.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                    point3f tp1((p1.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                        (p1.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                        (p1.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                    point3f tp2((p2.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                        (p2.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                        (p2.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                    if (intersect_cell(tp0, tp1, tp2))
                    {
                        vd.triangle_cells[0].emplace_back(std::make_tuple(x, y, z));
                    }
                }
            }
        }
        return vd;
    }

    // Voxelizes line segment.
    std::optional<voxelization_data> voxelize_sw_sd(const grid3f& g, const sixit::geometry::line_segment3f& s, int const res)
    {
        voxelization_data vd;

        // trim the segment by grid bounds
        point3f o = s.get_point(0);
        point3f e = s.get_point(1);
        bounds3f gb = bounds3f(g.origin_, point3f(
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(res-1)), 
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(res-1)), 
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(res-1))));
        std::optional<point3f> s1 = line_box_intersects(o, e, gb.minn, gb.maxx);
        if (!s1.has_value())
            return std::nullopt;
        std::optional<point3f> s2 = line_box_intersects(e, o, gb.minn, gb.maxx);
        if (!s2.has_value())
            return std::nullopt;

        o = s1.value();
        e = s2.value();
        
        direction3f dir(o, e);
        int incX = (dir.vec().x() > 0) ? 1 : -1;
        int incY = (dir.vec().y() > 0) ? 1 : -1;
        int incZ = (dir.vec().z() > 0) ? 1 : -1;

        // origin cell
        int64_t x = g.toFloorGridX(o.vec().x());
        int64_t y = g.toFloorGridY(o.vec().y());
        int64_t z = g.toFloorGridZ(o.vec().z());
        if (x >= 0 && x < res && y >= 0 && y < res && z >= 0 && z < res)
            vd.triangle_cells[0].emplace_back(std::make_tuple(x,y,z));

        // end cell
        int64_t ex = g.toFloorGridX(e.vec().x());
        int64_t ey = g.toFloorGridY(e.vec().y());
        int64_t ez = g.toFloorGridZ(e.vec().z());
        
        if (x == ex && y == ey && z == ez) // single cell
            return vd;

        int64_t cx = std::abs(ex - x);
        int64_t cy = std::abs(ey - y);
        int64_t cz = std::abs(ez - z);

        int64_t cSum = INT_MAX;

        while (cx + cy + cz > 0)
        {
            float dx = distance_to_cell(o.vec().x(), dir.vec().x(), x, g.stepX_, g.origin_.vec().x());
            float dy = distance_to_cell(o.vec().y(), dir.vec().y(), y, g.stepY_, g.origin_.vec().y());
            float dz = distance_to_cell(o.vec().z(), dir.vec().z(), z, g.stepZ_, g.origin_.vec().z());
            float d = std::min(dx, std::min(dy, dz));
            o.vec()+= dir.vec() * d;

            if (dx < dy && dx < dz)
                x = x + incX;
            else if (dy < dz)
                y = y + incY;
            else
                z = z + incZ;

            cx = std::abs(ex - x);
            cy = std::abs(ey - y);
            cz = std::abs(ez - z);

            if (cx + cy + cz > cSum)
                break;

            if (x >= 0 && x < res && y >= 0 && y < res && z >= 0 && z < res)
                vd.triangle_cells[0].emplace_back(std::make_tuple(x,y,z));
            
            cSum = cx + cy + cz;
        }

        // for (auto c : vd.triangle_cells[0])
            // std::cout << "cell: " << std::get<0>(c) << " " << std::get<1>(c) << " " << std::get<2>(c) << std::endl;
        return vd;
    }

    // segment intersection points with the grid bounds
    std::optional<point3f> line_box_intersects(point3f& v1, point3f& v2, point3f& bMin, point3f& bMax)
    {
        float minX = (bMin.vec().x() - v1.vec().x()) / (v2.vec().x() - v1.vec().x());
        float maxX = (bMax.vec().x() - v1.vec().x()) / (v2.vec().x() - v1.vec().x());
        float minY = (bMin.vec().y() - v1.vec().y()) / (v2.vec().y() - v1.vec().y());
        float maxY = (bMax.vec().y() - v1.vec().y()) / (v2.vec().y() - v1.vec().y());
        float minZ = (bMin.vec().z() - v1.vec().z()) / (v2.vec().z() - v1.vec().z());
        float maxZ = (bMax.vec().z() - v1.vec().z()) / (v2.vec().z() - v1.vec().z());

        float tMin = std::max<float>(std::min<float>(minX, maxX),
            std::max<float>( std::min<float>(minY, maxY), std::min<float>(minZ, maxZ) ));
        float tMax = std::min<float>(std::max<float>(minX, maxX),
            std::min<float>( std::max<float>(minY, maxY), std::max<float>(minZ, maxZ) ));

        if (tMax >= 0 && tMin <= tMax)
        {
            float t = std::min<float>(std::max<float>(tMin, 0), 1);
            return point3f(v1.vec().dim_x() + (v2.vec().dim_x() - v1.vec().dim_x()) * sixit::units::create_dimensionless_scalar(t),
                          v1.vec().dim_y() + (v2.vec().dim_y() - v1.vec().dim_y()) * sixit::units::create_dimensionless_scalar(t),
                          v1.vec().dim_z() + (v2.vec().dim_z() - v1.vec().dim_z()) * sixit::units::create_dimensionless_scalar(t));
        }
        return std::nullopt;
    }

    // Voxelizes plane.
    std::optional<voxelization_data> voxelize_sw_sd(const grid3f& g, const sixit::geometry::plane3f& plane, int const res)
    {
        voxelization_data vd;
        
        point3f pC = plane.point;
        direction3f pN = plane.normal;

        // max normal direction' projection
        float ax = abs(pN.vec().x());
        float ay = abs(pN.vec().y());
        float az = abs(pN.vec().z());
        int proj = ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2;
            
        // 1. Whole grid check -- todo -- make optional ??
        point3f min = g.origin_;
        float dx = g.stepX_ * res;
        float dy = g.stepY_ * res;
        float dz = g.stepZ_ * res;
        std::vector<point3f> points = box_points(min, dx, dy, dz);

        if (!plane_intersects(points, pC, pN, proj))
            return std::nullopt;

        for (int z = 0; z <= res-1; z++)
        {
            auto cz = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(z));
            for (int y = 0; y <= res-1; y++)
            {
                auto cy = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(y));
                for (int x = 0; x <= res-1; x++)
                {
                    auto cx = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(x));
                    // create mini-box
                    point3f min (cx, cy, cz);
                    float dx = g.stepX_ * res;
                    float dy = g.stepY_ * res;
                    float dz = g.stepZ_ * res;
                    std::vector<point3f> points = box_points(min, g.stepX_, g.stepY_, g.stepZ_);
                    if (plane_intersects(points, pC, pN, proj))
                    {
                        vd.triangle_cells[0].emplace_back( std::make_tuple(x, y, z) );
                    }
                }
            }
        }
        return vd;
    }

    bool plane_intersects(std::vector<point3f>& points, point3f& pC, direction3f& pN, int proj)
    {
        int sign = 0;
        float pCp = proj == 0 ? pC.vec().x() : proj == 1 ? pC.vec().y() : pC.vec().z();
        for (int i = 0; i < points.size(); i++)
        {
            point3f projP = project_on_line(points[i], pC, pN);
            float projCoord = proj == 0 ? projP.vec().x() : proj == 1 ? projP.vec().y() : projP.vec().z();
            int s = (int) low_level::mathf::sign(pCp - projCoord);
            if (sign != 0 && sign != s)
                return true;
            sign = s;
        }
        return false;
    }

    point3f project_on_line(point3f& p, point3f& l0, direction3f& l1)
    {
        point3f l0P = p.vec() - l0.vec();
        auto dist = l0P.vec().dim_x() * l1.vec().dim_x() + l0P.vec().dim_y() * l1.vec().dim_y() + l0P.vec().dim_z() * l1.vec().dim_z();
        point3f projP = l0.vec() + l1.vec() * dist;
        return projP;
    }

    std::vector<point3f> box_points(const point3f& origin, float const dx, float const dy, float const dz)
    {
        std::vector<point3f> points(8);
        auto bx0 = origin.vec().dim_x();
        auto bx1 = origin.vec().dim_x() + sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(dx);
        auto by0 = origin.vec().dim_y();
        auto by1 = origin.vec().dim_y() + sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(dy);
        auto bz0 = origin.vec().dim_z();
        auto bz1 = origin.vec().dim_z() + sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(dz);
        points[0] = point3f(bx0, by0, bz0);
        points[1] = point3f(bx1, by0, bz0);
        points[2] = point3f(bx0, by1, bz0);
        points[3] = point3f(bx1, by1, bz0);
        points[4] = point3f(bx0, by0, bz1);
        points[5] = point3f(bx1, by0, bz1);
        points[6] = point3f(bx0, by1, bz1);
        points[7] = point3f(bx1, by1, bz1);
        return points;
    }


    // Part 2. Various methods, for voxelization.

    struct cell
    {
        std::vector<size_t> triangles;
    };

    // 1. Diff method.

    // Diff method, single triangle test
    void voxelize_triangle(int const res = 1)
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.f);

        point3f p0 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.2f), 
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f));
        point3f p1 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.5f), 
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.8f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.2f));
        point3f p2 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(4.1f), 
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.7f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.4f));
        point3f v0, v1, v2;
        size_t rDir = 0;
        size_t triangle_id = 4;

        static const float grid_size = 5.0f; // todo - this will be defined by mesh bounds
        grid3 g(point3f(fp_0, fp_0, fp_0), grid_size/res, grid_size/res, grid_size/res);

        std::vector<std::vector<std::vector<cell>>> cells(res, std::vector<std::vector<cell>>(res, std::vector<cell>(res)));
        
        sort_points(p0, p1, p2, rDir, v0, v1, v2);

        std::vector<point3f> points01 = voxelize(v0, v1, rDir, g, cells, triangle_id);

        if (!points01.empty())
        {
            voxelize(v0, v2, rDir, g, cells, triangle_id);
            voxelize(v2, v1, rDir, g, cells, triangle_id);

            std::vector<point3f> points02 = get_points(v0, v2, points01, rDir);
            std::vector<point3f> points21 = get_points(v2, v1, {points01.begin() + points02.size(), points01.end()}, rDir);
            points02.insert(points02.end(), points21.begin(), points21.end());

            if (points01.size() != points02.size())
                std::cout << points01.size() << " / " << points02.size() << " points miscount" << std::endl;

            for (size_t i = 0; i < points01.size() - 1; i++)
            {
                points01[i].vec() = (points01[i].vec() + points01[i + 1].vec()) / 2.0f;
                points02[i].vec() = (points02[i].vec() + points02[i + 1].vec()) / 2.0f;
            }
            points01.pop_back();
            points02.pop_back();

            for (size_t i = 0; i < points02.size(); i++)
            {
                voxelize(points01[i], points02[i], rDir, g, cells, triangle_id);
            }
        }

        grid_count(cells);
    }
    
    // Diff method, whole mesh
    void voxelize_mesh(const bounds3f& b, const std::vector<triangle3f>& triangles, const std::vector<size_t>& ids, int const res)
    {
        static const float grid_size = sixit::units::extract_dim_less_scalar(line_segment3f(b.minn, b.maxx).length()/
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f));
        grid3f g(b.minn, grid_size/res, grid_size/res, grid_size/res);
        std::vector<std::vector<std::vector<cell>>> cells(res, std::vector<std::vector<cell>>(res, std::vector<cell>(res)));

        for (size_t ti = 0; ti < triangles.size(); ti++)
        {
            point3f p0 = std::get<0>(triangles[ti].vertices());
            point3f p1 = std::get<1>(triangles[ti].vertices());
            point3f p2 = std::get<2>(triangles[ti].vertices());
            point3f v0, v1, v2;
            size_t rDir = 0;
            size_t tri_id = ids[ti];

            sort_points(p0, p1, p2, rDir, v0, v1, v2);

            std::vector<point3f> points01 = voxelize(v0, v1, rDir, g, cells, tri_id);

            if (!points01.empty())
            {
                voxelize(v0, v2, rDir, g, cells, tri_id);
                voxelize(v2, v1, rDir, g, cells, tri_id);

                std::vector<point3f> points02 = get_points(v0, v2, points01, rDir);
                std::vector<point3f> points21 = get_points(v2, v1, {points01.begin() + points02.size(), points01.end()}, rDir);
                points02.insert(points02.end(), points21.begin(), points21.end());

                for (size_t i = 0; i < points01.size() - 1; i++)
                {
                    points01[i].vec() = (points01[i].vec() + points01[i + 1].vec()) / 2.0f;
                    points02[i].vec() = (points02[i].vec() + points02[i + 1].vec()) / 2.0f;
                }
                points01.pop_back();
                points02.pop_back();

                for (size_t i = 0; i < points02.size(); i++)
                {
                    voxelize(points01[i], points02[i], rDir, g, cells, tri_id);
                }
            }
        }
    }

    // 2. Schwarz-Seidel method.

    // Schwarz-Seidel method, single triangle
    void voxelize_schwarz_seidel(size_t res = 1)
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.f);

        point3f p0 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.2f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f));
        point3f p1 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.5f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.8f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.2f));
        point3f p2 = point3f(sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(4.1f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.7f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.4f));
        size_t tri_id = 4;

        static const float grid_size = 5.0f; // todo - this will be defined by mesh bounds
        grid3f g(point3f(fp_0, fp_0, fp_0), grid_size/res, grid_size/res, grid_size/res);

        std::vector<std::vector<std::vector<cell>>> cells(res, std::vector<std::vector<cell>>(res, std::vector<cell>(res)));

        // optimize: limit grid by triangle bounds
        auto b = bounds3f(p0);
        b.expand_by(p1);
        b.expand_by(p2);

        size_t count = 0;

        for (int64_t z = g.toFloorGridZ(b.minn.vec().z()); z < g.toCeilGridZ(b.maxx.vec().z()); z++)
        {
            auto cz = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(z));
            for (int64_t y = g.toFloorGridY(b.minn.vec().y()); y < g.toCeilGridY(b.maxx.vec().y()); y++)
            {
                auto cy = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(y));
                for (int64_t x = g.toFloorGridX(b.minn.vec().x()); x < g.toCeilGridX(b.maxx.vec().x()); x++)
                {
                    auto cx = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(x));
                    // triangle3 micro_triangle
                    // (
                        point3f tp0(
                            (p0.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p0.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p0.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp1(
                            (p1.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p1.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p1.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp2(
                            (p2.vec().dim_x() - cx) / sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p2.vec().dim_y() - cy) / sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p2.vec().dim_z() - cz) / sixit::units::create_dimensionless_scalar(g.stepZ_));
                    // );
                    
                    if (intersect_cell( tp0, tp1, tp2 ))
                    {
                        cells[x][y][z].triangles.push_back(tri_id);
                        count++;
                    }
                }
            }
        }

        std::cout << "total cells filled: " << count << std::endl;
    }

    // Schwarz-Seidel method, whole mesh
    void voxelize_mesh_sw_sd(const bounds3f& b, const std::vector<triangle3f>& triangles, const std::vector<size_t>& ids, int const res)
    {
        static const float grid_size = sixit::units::extract_dim_less_scalar(line_segment3f(b.minn, b.maxx).length() /
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f));
        grid3f g(b.minn, grid_size/res, grid_size/res, grid_size/res);
        std::vector<std::vector<std::vector<cell>>> cells(res, std::vector<std::vector<cell>>(res, std::vector<cell>(res)));

        for (size_t ti = 0; ti < triangles.size(); ti++)
        {
            point3f p0 = std::get<0>(triangles[ti].vertices());
            point3f p1 = std::get<1>(triangles[ti].vertices());
            point3f p2 = std::get<2>(triangles[ti].vertices());
            size_t tri_id = ids[ti];
            // optimize: limit grid by triangle bounds
            auto tb = bounds3f(p0);
            tb.expand_by(p1);
            tb.expand_by(p2);

            int64_t min_x = g.toFloorGridX(tb.minn.vec().x());
            int64_t min_y = g.toFloorGridY(tb.minn.vec().y());
            int64_t min_z = g.toFloorGridZ(tb.minn.vec().z());
            // single cell check
            if (
                min_x == g.toFloorGridX(tb.maxx.vec().x()) &&
                min_y == g.toFloorGridY(tb.maxx.vec().y()) &&
                min_z == g.toFloorGridZ(tb.maxx.vec().z())
                )
            {
                cells[min_x][min_y][min_z].triangles.push_back(tri_id);
                continue;
            }
        
            for (int64_t z = min_z; z < g.toCeilGridZ(tb.maxx.vec().z()); z++)
            {
                auto cz = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridZToRealZ(z));
                for (int64_t y = min_y; y < g.toCeilGridY(tb.maxx.vec().y()); y++)
                {
                    auto cy = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridYToRealY(y));
                    for (int64_t x = min_x; x < g.toCeilGridX(tb.maxx.vec().x()); x++)
                    {
                        auto cx = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.gridXToRealX(x));
                        point3f tp0( 
                            (p0.vec().dim_x() - cx)/ sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p0.vec().dim_y() - cy)/ sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p0.vec().dim_z() - cz)/ sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp1( 
                            (p1.vec().dim_x() - cx)/ sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p1.vec().dim_y() - cy)/ sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p1.vec().dim_z() - cz)/ sixit::units::create_dimensionless_scalar(g.stepZ_));
                        point3f tp2( 
                            (p2.vec().dim_x() - cx)/ sixit::units::create_dimensionless_scalar(g.stepX_),
                            (p2.vec().dim_y() - cy)/ sixit::units::create_dimensionless_scalar(g.stepY_),
                            (p2.vec().dim_z() - cz)/ sixit::units::create_dimensionless_scalar(g.stepZ_));
                        if (intersect_cell(tp0, tp1, tp2))
                        {
                            // if (cells[x][y][z].triangles.empty())
                                // std::cout << x << " " << y << " " << z << " : " << tri_id << std::endl;
                            cells[x][y][z].triangles.push_back(tri_id);
                        }
                    }
                }
            }            
        }

        std::cout << g.stepX_ << " " << g.stepY_ << " " << g.stepZ_ << std::endl;

        grid_count(cells);
    }

    // 3. Combined Projections method.

    // Combined Projections method, single triangle
    void voxelize_cells(int const res = 1)
    {
        auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.f);

        point3f p0 = point3f(
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.2f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.1f));
        point3f p1 = point3f(
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.5f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.8f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(3.2f));
        point3f p2 = point3f(
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(4.1f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.7f),
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.4f));
        size_t tri_id = 4;

        static const float grid_size = 5.0f;
        grid3f g(point3f(fp_0, fp_0, fp_0), grid_size/res, grid_size/res, grid_size/res);

        std::vector<std::vector<std::vector<size_t>>> c3D(res, std::vector<std::vector<size_t>>(res, std::vector<size_t>(res)));

        std::vector<std::vector<size_t>> cXY(res, std::vector<size_t>(res));
        std::vector<std::vector<size_t>> cXZ(res, std::vector<size_t>(res));
        std::vector<std::vector<size_t>> cYZ(res, std::vector<size_t>(res));
        point2f pp0;
        point2f pp1;
        point2f pp2;
        
        // XY grid
        std::vector<point2f> bXY = proj_bounds(p0, p1, p2, 2);
        int gx0 = (int) (sixit::units::extract_dim_less_scalar((bXY[0].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
        int gx1 = (int) (sixit::units::extract_dim_less_scalar((bXY[1].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
        int gy0 = (int) (sixit::units::extract_dim_less_scalar((bXY[0].vec()[1] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
        int gy1 = (int) (sixit::units::extract_dim_less_scalar((bXY[1].vec()[1] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
        if (gx0 == gx1 && gy0 == gy1)
            cXY[gx0][gy0] = tri_id;
        else
        {
            pp0 = {p0.vec().dim_x(), p0.vec().dim_y()};
            pp1 = {p1.vec().dim_x(), p1.vec().dim_y()};
            pp2 = {p2.vec().dim_x(), p2.vec().dim_y()};
            std::vector<point2f> s01 = {pp0, pp1};
            std::vector<point2f> s12 = {pp1, pp2};
            std::vector<point2f> s20 = {pp2, pp0};
            fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cXY, g.origin_.vec().x(), g.stepX_, g.origin_.vec().y(), g.stepY_, tri_id);
            fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cXY, g.origin_.vec().y(), g.stepY_, g.origin_.vec().x(), g.stepX_, tri_id);
        }

        // XZ grid
        std::vector<point2f> bXZ = proj_bounds(p0, p1, p2, 1);
        gx0 = (int) (sixit::units::extract_dim_less_scalar((bXZ[0].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
        gx1 = (int) (sixit::units::extract_dim_less_scalar((bXZ[1].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
        gy0 = (int) (sixit::units::extract_dim_less_scalar((bXZ[0].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
        gy1 = (int) (sixit::units::extract_dim_less_scalar((bXZ[1].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
        if (gx0 == gx1 && gy0 == gy1)
            cXZ[gx0][gy0] = tri_id;
        else
        {
            pp0 = {p0.vec().dim_x(), p0.vec().dim_z()};
            pp1 = {p1.vec().dim_x(), p1.vec().dim_z()};
            pp2 = {p2.vec().dim_x(), p2.vec().dim_z()};
            std::vector<point2f> s01 = {pp0, pp1};
            std::vector<point2f> s12 = {pp1, pp2};
            std::vector<point2f> s20 = {pp2, pp0};
            fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cXZ, g.origin_.vec().x(), g.stepX_, g.origin_.vec().z(), g.stepZ_, tri_id);
            fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cXZ, g.origin_.vec().z(), g.stepZ_, g.origin_.vec().x(), g.stepX_,tri_id);
        }

        // YZ grid
        std::vector<point2f> bYZ = proj_bounds(p0, p1, p2, 0);
        gx0 = (int) (sixit::units::extract_dim_less_scalar((bYZ[0].vec()[0] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
        gx1 = (int) (sixit::units::extract_dim_less_scalar((bYZ[1].vec()[0] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
        gy0 = (int) (sixit::units::extract_dim_less_scalar((bYZ[0].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
        gy1 = (int) (sixit::units::extract_dim_less_scalar((bYZ[1].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
        if (gx0 == gx1 && gy0 == gy1)
            cYZ[gx0][gy0] = tri_id;
        else
        {
            pp0 = {p0.vec().dim_y(), p0.vec().dim_z()};
            pp1 = {p1.vec().dim_y(), p1.vec().dim_z()};
            pp2 = {p2.vec().dim_y(), p2.vec().dim_z()};
            std::vector<point2f> s01 = {pp0, pp1};
            std::vector<point2f> s12 = {pp1, pp2};
            std::vector<point2f> s20 = {pp2, pp0};
            fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cYZ, g.origin_.vec().y(), g.stepY_, g.origin_.vec().z(), g.stepZ_, tri_id);
            fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cYZ, g.origin_.vec().z(), g.stepZ_, g.origin_.vec().y(), g.stepY_, tri_id);
        }

        // 3D grid
        for (int x = 0; x < res; x++)
        {
            for (int y = 0; y < res; y++)
            {
                for (int z = 0; z < res; z++)
                {
                    if (cXY[x][y]==tri_id && cXZ[x][z]==tri_id && cYZ[y][z]==tri_id)
                    {
                        c3D[x][y][z] = tri_id;
                    }
                }
            }
        }

    }

    // Combined Projections method, whole mesh
    void voxelize_mesh_cells(const bounds3f& b, const std::vector<triangle3f>& triangles, const std::vector<size_t>& ids, int const res)
    {
        static const float grid_size = sixit::units::extract_dim_less_scalar(line_segment3f(b.minn, b.maxx).length() /
            sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f));
        grid3f g(b.minn, grid_size/res, grid_size/res, grid_size/res);
        std::vector<std::vector<std::vector<cell>>> cells(res, std::vector<std::vector<cell>>(res, std::vector<cell>(res)));
        std::vector<std::vector<size_t>> cXY(res, std::vector<size_t>(res));
        std::vector<std::vector<size_t>> cXZ(res, std::vector<size_t>(res));
        std::vector<std::vector<size_t>> cYZ(res, std::vector<size_t>(res));

        point2f pp0;
        point2f pp1;
        point2f pp2;

        for (size_t ti = 0; ti < triangles.size(); ti++)
        {
            point3f p0 = std::get<0>(triangles[ti].vertices());
            point3f p1 = std::get<1>(triangles[ti].vertices());
            point3f p2 = std::get<2>(triangles[ti].vertices());
            size_t tri_id = ids[ti];

            // XY grid
            std::vector<point2f> bXY = proj_bounds(p0, p1, p2, 2);
            int gx0 = (int) (sixit::units::extract_dim_less_scalar((bXY[0].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
            int gx1 = (int) (sixit::units::extract_dim_less_scalar((bXY[1].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
            int gy0 = (int) (sixit::units::extract_dim_less_scalar((bXY[0].vec()[1] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
            int gy1 = (int) (sixit::units::extract_dim_less_scalar((bXY[1].vec()[1] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
            if (gx0 == gx1 && gy0 == gy1)
                cXY[gx0][gy0] = tri_id;
            else
            {
                pp0 = {p0.vec().dim_x(), p0.vec().dim_y()};
                pp1 = {p1.vec().dim_x(), p1.vec().dim_y()};
                pp2 = {p2.vec().dim_x(), p2.vec().dim_y()};
                std::vector<point2f> s01 = {pp0, pp1};
                std::vector<point2f> s12 = {pp1, pp2};
                std::vector<point2f> s20 = {pp2, pp0};
                fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cXY, g.origin_.vec().x(), g.stepX_, g.origin_.vec().y(), g.stepY_, tri_id);
                fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cXY, g.origin_.vec().y(), g.stepY_, g.origin_.vec().x(), g.stepX_, tri_id);
            }

            // XZ grid
            std::vector<point2f> bXZ = proj_bounds(p0, p1, p2, 1);
            gx0 = (int) (sixit::units::extract_dim_less_scalar((bXZ[0].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
            gx1 = (int) (sixit::units::extract_dim_less_scalar((bXZ[1].vec()[0] - g.origin_.vec().dim_x()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepX_)));
            gy0 = (int) (sixit::units::extract_dim_less_scalar((bXZ[0].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
            gy1 = (int) (sixit::units::extract_dim_less_scalar((bXZ[1].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
            if (gx0 == gx1 && gy0 == gy1)
                cXZ[gx0][gy0] = tri_id;
            else
            {
                pp0 = {p0.vec().dim_x(), p0.vec().dim_z()};
                pp1 = {p1.vec().dim_x(), p1.vec().dim_z()};
                pp2 = {p2.vec().dim_x(), p2.vec().dim_z()};
                std::vector<point2f> s01 = {pp0, pp1};
                std::vector<point2f> s12 = {pp1, pp2};
                std::vector<point2f> s20 = {pp2, pp0};
                fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cXZ, g.origin_.vec().x(), g.stepX_, g.origin_.vec().z(), g.stepZ_, tri_id);
                fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cXZ, g.origin_.vec().z(), g.stepZ_, g.origin_.vec().x(), g.stepX_,tri_id);
            }

            // YZ grid
            std::vector<point2f> bYZ = proj_bounds(p0, p1, p2, 0);
            gx0 = (int) (sixit::units::extract_dim_less_scalar((bYZ[0].vec()[0] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
            gx1 = (int) (sixit::units::extract_dim_less_scalar((bYZ[1].vec()[0] - g.origin_.vec().dim_y()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepY_)));
            gy0 = (int) (sixit::units::extract_dim_less_scalar((bYZ[0].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
            gy1 = (int) (sixit::units::extract_dim_less_scalar((bYZ[1].vec()[1] - g.origin_.vec().dim_z()) / sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(g.stepZ_)));
            if (gx0 == gx1 && gy0 == gy1)
                cYZ[gx0][gy0] = tri_id;
            else
            {
                pp0 = {p0.vec().dim_y(), p0.vec().dim_z()};
                pp1 = {p1.vec().dim_y(), p1.vec().dim_z()};
                pp2 = {p2.vec().dim_y(), p2.vec().dim_z()};
                std::vector<point2f> s01 = {pp0, pp1};
                std::vector<point2f> s12 = {pp1, pp2};
                std::vector<point2f> s20 = {pp2, pp0};
                fill_cells(gx0, gx1, 0, 1, s01, s12, s20, cYZ, g.origin_.vec().y(), g.stepY_, g.origin_.vec().z(), g.stepZ_, tri_id);
                fill_cells(gy0, gy1, 1, 0, s01, s12, s20, cYZ, g.origin_.vec().z(), g.stepZ_, g.origin_.vec().y(), g.stepY_, tri_id);
            }            

            // optimize: limit grid by triangle bounds
            auto tb = bounds3f(p0);
            tb.expand_by(p1);
            tb.expand_by(p2);
            int64_t min_x = g.toFloorGridX(tb.minn.vec().x());
            int64_t min_y = g.toFloorGridY(tb.minn.vec().y());
            int64_t min_z = g.toFloorGridZ(tb.minn.vec().z());
            int64_t max_x = g.toCeilGridX(tb.maxx.vec().x());
            int64_t max_y = g.toCeilGridY(tb.maxx.vec().y());
            int64_t max_z = g.toCeilGridZ(tb.maxx.vec().z());

            // combined 3D grid
            for (int64_t x = min_x; x < max_x; x++)
            {
                for (int64_t y = min_y; y < max_y; y++)
                {
                    for (int64_t z = min_z; z < max_z; z++)
                    {
                        if (cXY[x][y]==tri_id && cXZ[x][z]==tri_id && cYZ[y][z]==tri_id)
                        {
                            cells[x][y][z].triangles.push_back(tri_id);
                        }
                    }
                }
            }
        }

        grid_count(cells);
    }

    private:

    // for Sw-Seidel test:
    bool intersect_cell (point3f& p0, point3f& p1, point3f& p2)
    {
        auto fp_0_5 = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(0.5f);
        point3f e0(p1.vec() - p0.vec());
        point3f e1(p2.vec() - p1.vec());
        point3f e2(p0.vec() - p2.vec());
        direction3f norm = low_level::tri3_implf::normal(p0, p1, p2);        

        // fast box check
        float r = 0.5f * abs(norm.vec().x()) + 0.5f * abs(norm.vec().y()) + 0.5f * abs(norm.vec().z());
        point3f d(fp_0_5 - p0.vec().dim_x(), fp_0_5 - p0.vec().dim_y(), fp_0_5 - p0.vec().dim_z());
        float s = (norm.vec().x() * d.vec().x() + norm.vec().y() * d.vec().y() + norm.vec().z() * d.vec().z());
        if (abs(s) > r)
            return false;

        float cx = norm.vec().x() >= 0.0f ? 1.0f : -1.0f;
        float cy = norm.vec().y() >= 0.0f ? 1.0f : -1.0f;
        float cz = norm.vec().z() >= 0.0f ? 1.0f : -1.0f;

        if (edge_point (e0.vec().y(), p0.vec().x(), e0.vec().x(), p0.vec().y(), cz))
            return false;
        if (edge_point (e1.vec().y(), p1.vec().x(), e1.vec().x(), p1.vec().y(), cz))
            return false;
        if (edge_point (e2.vec().y(), p2.vec().x(), e2.vec().x(), p2.vec().y(), cz))
            return false;
        if (edge_point (e0.vec().x(), p0.vec().z(), e0.vec().z(), p0.vec().x(), cy))
            return false;
        if (edge_point (e1.vec().x(), p1.vec().z(), e1.vec().z(), p1.vec().x(), cy))
            return false;
        if (edge_point (e2.vec().x(), p2.vec().z(), e2.vec().z(), p2.vec().x(), cy))
            return false;
        if (edge_point (e0.vec().z(), p0.vec().y(), e0.vec().y(), p0.vec().z(), cx))
            return false;
        if (edge_point (e1.vec().z(), p1.vec().y(), e1.vec().y(), p1.vec().z(), cx))
            return false;
        if (edge_point (e2.vec().z(), p2.vec().y(), e2.vec().y(), p2.vec().z(), cx))
            return false;

        return true;
    }

    bool edge_point(float& e1, float& p1, float& e2, float& p2, float& c)
    {
        return (e1 * p1 - e2 * p2) * c + std::max(0.0f, -e1 * c) + std::max(0.0f, e2 * c) < 0;
    }

    // for Combined Projections method:
    void fill_cells(int gx0, int gx1, int a0, int a1,
        std::vector<point2f>& s01, std::vector<point2f>& s12, std::vector<point2f>& s20,
        std::vector<std::vector<size_t>>& gridXY,
        float& gridMinX, float& cellSizeX, float& gridMinY, float& cellSizeY, size_t id)
    {
        for (int i = gx0 + 1; i <= gx1; i++)
        {
            std::vector<float> iPoints;
            float x = gridMinX + i * cellSizeX;

            auto x01 = segment_point(s01, x, a0);
            if (x01.has_value())
                iPoints.push_back(*x01);

            auto x12 = segment_point(s12, x, a0);
            if (x12.has_value())
                iPoints.push_back(*x12);

            auto x20 = segment_point(s20, x, a0);
            if (x20.has_value())
                iPoints.push_back(*x20);

            if (iPoints.size() == 2)
            {
                int p1i = grid_index_1D(iPoints[0], gridMinY, cellSizeY);
                int p2i = grid_index_1D(iPoints[1], gridMinY, cellSizeY);

                int minIndex = std::min(p1i, p2i);
                int maxIndex = std::max(p1i, p2i);

                for (int ci = minIndex; ci <= maxIndex; ci++)
                {
                    if (a0 > a1)
                    {
                        gridXY[ci][i-1] = id;
                        gridXY[ci][i] = id;
                    }
                    else
                    {
                        gridXY[i-1][ci] = id;
                        gridXY[i][ci] = id;
                    }
                }
            }
        }
    }

    std::optional<float> segment_point(std::vector<point2f>& s, float c, int axis)
    {
        auto dim_c = sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(c);
        if (axis == 0)
        {
            if ((s[0].vec()[0] <= dim_c && s[1].vec()[0] >= dim_c) || (s[0].vec()[0] >= dim_c && s[1].vec()[0] <= dim_c)) // c is within segment bounds
            {
                auto res = s[0].vec()[1] + (s[1].vec()[1] - s[0].vec()[1]) * (dim_c - s[0].vec()[0]) / (s[1].vec()[0] - s[0].vec()[0]);
                return sixit::units::extract_dim_less_scalar(res /
                    sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f));
            }
        }
        else
        {
            if ((s[0].vec()[1] <= dim_c && s[1].vec()[1] >= dim_c) || (s[0].vec()[1] >= dim_c && s[1].vec()[1] <= dim_c)) // c is within segment bounds
            {
                auto res = s[0].vec()[0] + (s[1].vec()[0] - s[0].vec()[0]) * (dim_c - s[0].vec()[1]) / (s[1].vec()[1] - s[0].vec()[1]);
                return sixit::units::extract_dim_less_scalar(res /
                    sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(1.f));
            }

            
        }
        return std::nullopt;
    }

    int grid_index_1D(float point, float origin, float size)
    {
        return static_cast<int> ((point - origin) / size);
    }

    std::vector<point2f> proj_bounds(point3f v0, point3f v1, point3f v2, int proj)
    {
        point2f v20, v21, v22;

        switch (proj)
        {
            case 0: // YZ
                v20 = { v0.vec().dim_y(), v0.vec().dim_z() };
                v21 = { v1.vec().dim_y(), v1.vec().dim_z() };
                v22 = { v2.vec().dim_y(), v2.vec().dim_z() };
                break;

            case 1: // XZ
                v20 = { v0.vec().dim_x(), v0.vec().dim_z() };
                v21 = { v1.vec().dim_x(), v1.vec().dim_z() };
                v22 = { v2.vec().dim_x(), v2.vec().dim_z() };
                break;

            default: // XY
                v20 = { v0.vec().dim_x(), v0.vec().dim_y() };
                v21 = { v1.vec().dim_x(), v1.vec().dim_y() };
                v22 = { v2.vec().dim_x(), v2.vec().dim_y() };
                break;
        }

        auto minX = std::min(v20.vec()[0], std::min(v21.vec()[0], v22.vec()[0]));
        auto minY = std::min(v20.vec()[1], std::min(v21.vec()[1], v22.vec()[1]));
        auto maxX = std::max(v20.vec()[0], std::max(v21.vec()[0], v22.vec()[0]));
        auto maxY = std::max(v20.vec()[1], std::max(v21.vec()[1], v22.vec()[1]));
        return { point2f(minX, minY), point2f(maxX, maxY) };
    }

    // for Diff method:
    // sorts triangle points to take the longest side as v0v1, and its projection as rDir direction 
    void sort_points(const point3f& p0, const point3f& p1, const point3f& p2, size_t& rDir, point3f& v0, point3f& v1, point3f& v2)
    {
        float dx01 = std::abs(p0.vec().x() - p1.vec().x());
        float dy01 = std::abs(p0.vec().y() - p1.vec().y());
        float dz01 = std::abs(p0.vec().z() - p1.vec().z());

        float dx02 = std::abs(p0.vec().x() - p2.vec().x());
        float dy02 = std::abs(p0.vec().y() - p2.vec().y());
        float dz02 = std::abs(p0.vec().z() - p2.vec().z());

        float dx12 = std::abs(p1.vec().x() - p2.vec().x());
        float dy12 = std::abs(p1.vec().y() - p2.vec().y());
        float dz12 = std::abs(p1.vec().z() - p2.vec().z());

        float dx = std::max(dx01, std::max(dx02, dx12));
        float dy = std::max(dy01, std::max(dy02, dy12));
        float dz = std::max(dz01, std::max(dz02, dz12));

        rDir = (dx > dy && dx > dz) ? 0 : (dy > dz) ? 1 : 2;

        point3f minPoint, maxPoint, midPoint;

        float pDir0 = (rDir == 0) ? p0.vec().x() : (rDir == 1) ? p0.vec().y() : p0.vec().z();
        float pDir1 = (rDir == 0) ? p1.vec().x() : (rDir == 1) ? p1.vec().y() : p1.vec().z();
        float pDir2 = (rDir == 0) ? p2.vec().x() : (rDir == 1) ? p2.vec().y() : p2.vec().z();

        if (pDir0 <= pDir1 && pDir0 <= pDir2)
            minPoint = p0;
        else if (pDir1 <= pDir2)
            minPoint = p1;
        else
            minPoint = p2;

        if (pDir0 >= pDir1 && pDir0 >= pDir2)
            maxPoint = p0;
        else if (pDir1 >= pDir2)
            maxPoint = p1;
        else
            maxPoint = p2;

        midPoint = (p0 != minPoint && p0 != maxPoint) ? p0 : (p1 != minPoint && p1 != maxPoint) ? p1 : p2;

        v0 = minPoint;
        v1 = maxPoint;
        v2 = midPoint;
    }
    
    // fills Grid and returns the meaning (in rDir direction) points
    std::vector<point3f> voxelize(const point3f& o, const point3f& e, size_t& rDir, grid3f& grid,
        std::vector<std::vector<std::vector<cell>>>& cells, size_t& tri_id)
    {
        std::vector<point3f> points;
        direction3f dir(o, e);
        // cell increment depends on end-origin direction
        int incX = (dir.vec().x() > 0) ? 1 : -1;
        int incY = (dir.vec().y() > 0) ? 1 : -1;
        int incZ = (dir.vec().z() > 0) ? 1 : -1;
        // current cell
        int64_t x = grid.toFloorGridX(o.vec().x());
        int64_t y = grid.toFloorGridY(o.vec().y());
        int64_t z = grid.toFloorGridZ(o.vec().z());

        auto& triangles = cells[x][y][z].triangles;
        if (triangles.empty() || triangles.back() != tri_id)
            triangles.push_back(tri_id);

        // end cell
        int64_t ex = grid.toFloorGridX(e.vec().x());
        int64_t ey = grid.toFloorGridY(e.vec().y());
        int64_t ez = grid.toFloorGridZ(e.vec().z());
        
        if (x == ex && y == ey && z == ez) // single cell
            return points;

        int64_t cx = std::abs(ex - x);
        int64_t cy = std::abs(ey - y);
        int64_t cz = std::abs(ez - z);

        point3f p = o;
        int64_t cSum = INT_MAX;

        while (cx + cy + cz > 0)
        {
            float dx = distance_to_cell(p.vec().x(), dir.vec().x(), x, grid.stepX_, grid.origin_.vec().x());
            float dy = distance_to_cell(p.vec().y(), dir.vec().y(), y, grid.stepY_, grid.origin_.vec().y());
            float dz = distance_to_cell(p.vec().z(), dir.vec().z(), z, grid.stepZ_, grid.origin_.vec().z());
            float d = std::min(dx, std::min(dy, dz));
            p.vec()+= dir.vec() * d;

            if (dx < dy && dx < dz)
            {
                x = x + incX;
                if (rDir == 0)
                    points.push_back(p);
            }
            else if (dy < dz)
            {
                y = y + incY;
                if (rDir == 1)
                    points.push_back(p);
            }
            else
            {
                z = z + incZ;
                if (rDir == 2)
                    points.push_back(p);
            }

            cx = std::abs(ex - x);
            cy = std::abs(ey - y);
            cz = std::abs(ez - z);

            if (cx + cy + cz > cSum)
                break;

            auto& triangles = cells[x][y][z].triangles;
            if (triangles.empty() || triangles.back() != tri_id)
                triangles.push_back(tri_id);

            cSum = cx + cy + cz;
        }
        return points;
    }

    // distance to the cell size, from point in ortho (x,y, or z) dir
    float distance_to_cell(float p, float dir, int64_t c, float gStep, float gOrigin)
    {
        if (dir == 0)
            return std::numeric_limits<float>::max();
        if (dir > 0)
            return (gOrigin + (c + 1) * gStep - p) / dir;
        else
            return (gOrigin + c * gStep - p) / dir;
    }

    // points (for triangle side) lying in 2D plane defined by rDir,
    // and on the same rDir coordinate as the given tracePoints
    std::vector<point3f> get_points(const point3f& origin, const point3f& end, const std::vector<point3f>& tracePoints, size_t rDir)
    {
        direction3f dir (origin, end);
        std::vector<point3f> points;
        float ep = rDir == 0 ? end.vec().x() : rDir == 1 ? end.vec().y() : end.vec().z();
        float o = rDir == 0 ? origin.vec().x() : rDir == 1 ? origin.vec().y() : origin.vec().z();
        float d = rDir == 0 ? dir.vec().x() : rDir == 1 ? dir.vec().y() : dir.vec().z();

        for (const point3f &tp : tracePoints)
        {
            float p = (rDir == 0) ? tp.vec().x() : (rDir == 1) ? tp.vec().y() : tp.vec().z();
            if (p > ep)
                break;
            float dt = (p - o) / d;
            point3f interPoint = origin.vec() + dir.vec() * sixit::units::for_import_only_make_dimensional_scalar<float, sixit::units::meter>(dt);
            points.push_back(interPoint);
        }
        return points;
    }

    void voxelize_2D(const point3f& o, const point3f& e, int rDir, grid3f& grid, std::vector<grid_cell3>& flock)
    {
        direction3f dir(o, e);
        int incX = (dir.vec().x() > 0) ? 1 : -1;
        int incY = (dir.vec().y() > 0) ? 1 : -1;

        int64_t x = grid.toFloorGridX(o.vec().x());
        int64_t y = grid.toFloorGridY(o.vec().y());

        grid_cell3 cell {x, y, 0};
        flock.push_back(cell);

        int64_t ex = grid.toFloorGridX(e.vec().x());
        int64_t ey = grid.toFloorGridY(e.vec().y());
        
        if (x == ex && y == ey) // single cell
            return;

        int64_t cx = std::abs(ex - x);
        int64_t cy = std::abs(ey - y);

        point3f p = o;
        int64_t cSum = INT_MAX;

        while (cx + cy > 0)
        {
            float dx = distance_to_cell(p.vec().x(), dir.vec().x(), x, grid.stepX_, grid.origin_.vec().x()); // swap grid.step_
            float dy = distance_to_cell(p.vec().y(), dir.vec().y(), y, grid.stepY_, grid.origin_.vec().y());
            float d = std::min(dx, dy);
            p.vec()+= dir.vec() * d;

            x = x + (dx <= dy ? incX : 0);
            y = y + (dy < dx  ? incY : 0);

            cx = std::abs(ex - x);
            cy = std::abs(ey - y);

            if (cx + cy > cSum)
                break;

            grid_cell3 cell {x, y, 0};
            flock.push_back(cell);

            cSum = cx + cy;
        }
    }

    void grid_count(const std::vector<std::vector<std::vector<cell>>> cells)
    {
        size_t count = 0;
        for (const auto& level : cells)
        {
            for (const auto& row : level)
            {
                for (const auto& cell : row)
                {
                    count += cell.triangles.size();
                }
            }
        }
        std::cout << "grid cells filled: " << count << std::endl;
    }
};

}; // namespace geometry
}; // namespace sixit

#endif // sixit_geometry_voxelizer_h_included

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
