/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry__grid3_h_included
#define sixit_geometry__grid3_h_included

#include "sixit/geometry/low_level/forward_declaration.h"

#include <iostream>

namespace sixit
{
namespace geometry
{

struct grid_cell3 // Represent a cell in a 3D grid addressed by it's x, y, and z indices in the whole mesh grid
{
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
};

template <typename fp>
class grid3 // represents grid positioning information and answers basic questions about relations of objects to grid cells
{
    friend class voxelizer;
    friend sixit::graphics::intersectable_mesh;
    friend class grid3_with_bounds<fp>; // TODO: remove as soon as reworked
    // TODO: list actual friends here

  public:
    grid3(point3<fp> origin, fp stepX, fp stepY, fp stepZ) : origin_(origin), 
        stepX_(stepX > fp(0.f) ? stepX : fp(1.0f)), stepY_(stepY > fp(0.f) ? stepY : fp(1.0f)), stepZ_(stepZ > fp(0.f) ? stepZ : fp(1.0f)) { }
    grid3(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originX, 
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originY, 
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originZ, 
        fp stepX, fp stepY, fp stepZ) : origin_(originX, originY, originZ),
        stepX_(stepX > fp(0.f) ? stepX : fp(1.0f)), stepY_(stepY > fp(0.f) ? stepY : fp(1.0f)), stepZ_(stepZ > fp(0.f) ? stepZ : fp(1.0f)) { }

    grid_cell3 point3_to_grid_cell( const point3<fp>& real ) const { return { toFloorGridX( real.vec().x() ), toFloorGridY( real.vec().y() ), toFloorGridZ( real.vec().z() ) }; }
    // TODO: add triangle3_to_grid_cells() that returns an iterator over related cells

  private: // TODO: revise necessity of each call below and remove unused
    // Functions to convert grid coordinates to real-world coordinates
    fp gridXToRealX( int64_t gridX ) const { return origin_.vec().x() + gridX * stepX_; }
    fp gridYToRealY( int64_t gridY ) const { return origin_.vec().y() + gridY * stepY_; }
    fp gridZToRealZ( int64_t gridZ ) const { return origin_.vec().z() + gridZ * stepZ_; }

    // Functions to convert real-world coordinates to the nearest floor grid coordinates
    int64_t toFloorGridX( fp realX ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( ( realX - origin_.vec().x() ) / stepX_ )); }
    int64_t toFloorGridY( fp realY ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( ( realY - origin_.vec().y() ) / stepY_ )); }
    int64_t toFloorGridZ( fp realZ ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( ( realZ - origin_.vec().z() ) / stepZ_ )); }

    // Functions to convert real-world coordinates to the nearest ceil grid coordinates
    int64_t toCeilGridX( fp realX ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( ( realX - origin_.vec().x() ) / stepX_ )); }
    int64_t toCeilGridY( fp realY ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( ( realY - origin_.vec().y() ) / stepY_ )); }
    int64_t toCeilGridZ( fp realZ ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( ( realZ - origin_.vec().z() ) / stepZ_ )); }

    // Functions to convert real-world coordinates to the nearest floor step multiples
    int64_t toFloorDX( fp realX ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( realX / stepX_ )); }
    int64_t toFloorDY( fp realY ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( realY / stepY_ )); }
    int64_t toFloorDZ( fp realZ ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::floor( realZ / stepZ_ )); }

    // Functions to convert real-world coordinates to the nearest ceil step multiples
    int64_t toCeilDX( fp realX ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( realX / stepX_ )); }
    int64_t toCeilDY( fp realY ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( realY / stepY_ )); }
    int64_t toCeilDZ( fp realZ ) const { return sixit::dmath::fp_traits<fp>::fp2int64(low_level::mathf::ceil( realZ / stepZ_ )); }

  private:
    point3<fp> origin_;   // Origin of the grid
    fp stepX_ = fp(0.f); // Step size along X axis
    fp stepY_ = fp(0.f); // Step size along Y axis
    fp stepZ_ = fp(0.f); // Step size along Z axis
};

struct grid_bounds3
{
    grid_cell3 min; // Minimum grid coordinates where voxels are set
    grid_cell3 max; // Maximum grid coordinates where voxels are set

    void clear()
    {
        min.x = std::numeric_limits<int64_t>::max();
        min.y = std::numeric_limits<int64_t>::max();
        min.z = std::numeric_limits<int64_t>::max();
        max.x = std::numeric_limits<int64_t>::min();
        max.y = std::numeric_limits<int64_t>::min();
        max.z = std::numeric_limits<int64_t>::min();
    }
};

template<typename fp>
class grid3_with_bounds : public grid3<fp> // TODO: rename and rework accordingly
{
  public:
    grid3_with_bounds(point3<fp> origin, fp stepX, fp stepY, fp stepZ) : grid3<fp>(origin, stepX, stepY, stepZ) 
    {
        gridBounds_.clear();
    }
    grid3_with_bounds(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originX,
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originY,
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> originZ, 
        fp stepX, fp stepY, fp stepZ) : grid3<fp>(originX, originY, originZ, stepX, stepY, stepZ) 
    {
        gridBounds_.clear();
    }

    // Function to expand the grid to accommodate a new voxel without copying blocks
    // Remember to use it whenever set a new voxel to ensure gridBounds_ reflects the correct boundaries of your grid
    void expandGridToNewPoint( const point3<fp>& real ) { expandGridToNewCell(grid3<fp>::point3_to_grid_cell( real ) ); }
    void expandGridToNewPoint(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> realX,
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> realY,
        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> realZ) { expandGridToNewPoint( point3<fp>( realX, realY, realZ ) ); }
    void expandGridToNewCell( const grid_cell3& cell )
    {
        // Update the minimum grid coordinates where voxels are set
        gridBounds_.min.x = std::min(gridBounds_.min.x, cell.x);
        gridBounds_.min.y = std::min(gridBounds_.min.y, cell.y);
        gridBounds_.min.z = std::min(gridBounds_.min.z, cell.z);

        // Update the maximum grid coordinates where voxels are set
        gridBounds_.max.x = std::max(gridBounds_.max.x, cell.x + 1);
        gridBounds_.max.y = std::max(gridBounds_.max.y, cell.y + 2); // Note: we add one more point to allow super-green on top of the scene
        gridBounds_.max.z = std::max(gridBounds_.max.z, cell.z + 1);
    }

    // Functions to check if a given grid cell is within the bounds of the current grid
    bool isGridCellWithinBounds( int64_t x, int64_t y, int64_t z ) const
    {
        return (x >= gridBounds_.min.x && x <= gridBounds_.max.x &&
                y >= gridBounds_.min.y && y <= gridBounds_.max.y &&
                z >= gridBounds_.min.z && z <= gridBounds_.max.z);
    }
    bool isGridCellWithinBounds( const grid_cell3& cell ) const { return isGridCellWithinBounds( cell.x, cell.y, cell.z ); }

    // Functions to get the dimensions of the grid in grid cell units
    int64_t getWidth() const { return gridBounds_.max.x < gridBounds_.min.x ? 1 : gridBounds_.max.x - gridBounds_.min.x + 1; }
    int64_t getHeight() const { return gridBounds_.max.y < gridBounds_.min.y ? 1 : gridBounds_.max.y - gridBounds_.min.y + 1; }
    int64_t getDepth() const { return gridBounds_.max.z < gridBounds_.min.z ? 1 : gridBounds_.max.z - gridBounds_.min.z + 1; }

    // Function to clear the grid bounds
    void clearGrid()
    {
        gridBounds_.clear();
    }

  private:
    grid_bounds3 gridBounds_; // Minimum and maximum grid coordinates where voxels are set
};

}; // namespace geometry

namespace geometry
{
  // aliases
  using grid3_with_boundsf = grid3_with_bounds<float>;

  using grid3f = grid3<float>;
}// namespace geometry

}; // namespace sixit

#endif //sixit_geometry__grid3_h_included

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
