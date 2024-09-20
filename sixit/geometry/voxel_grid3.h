/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/
// reserved.

#ifndef sixit_geometry_voxel_grid3_h_included
#define sixit_geometry_voxel_grid3_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/grid3.h"
#include "sixit/core/containers.h"

namespace sixit
{
namespace geometry
{

template<typename VoxelDataT>
class voxel_grid3
{
    static_assert(std::is_default_constructible_v<VoxelDataT>, "VoxelDataT must have a default constructor");

    using VoxelDeque1D = containers::index_preserving_deque<VoxelDataT>;
    using VoxelDeque2D = containers::index_preserving_deque<VoxelDeque1D>;
    using VoxelDeque3D = containers::index_preserving_deque<VoxelDeque2D>;

  public:
    voxel_grid3(int voxelSize = 1) : voxelSize_(voxelSize)
    {
        assert(voxelSize_ > 0 && "voxelSize must be greater than 0");
    }

    voxel_grid3(const voxel_grid3&) = delete;
    voxel_grid3(voxel_grid3&& other) noexcept : voxels_(std::move(other.voxels_)), voxelSize_(other.voxelSize_)
    {
        other.voxelSize_ = 1;
    }

    voxel_grid3& operator=(const voxel_grid3& other) = delete;
    voxel_grid3& operator=(voxel_grid3&& other) noexcept
    {
        if (this != &other)
        {
            voxels_ = std::move(other.voxels_);
            voxelSize_ = other.voxelSize_;
            other.voxelSize_ = 1;
        }
        return *this;
    }

    voxel_grid3 clone() const
    {
        voxel_grid3 copy(voxelSize_);
        copy.voxels_ = voxels_;
        return copy;
    }

    // Functions to set the voxel data to specific grid cell
    void setVoxel(int64_t x, int64_t y, int64_t z, VoxelDataT&& voxelData) { setVoxel(grid_cell3{x, y, z}, std::move(voxelData)); }
    void setVoxel(const grid_cell3& cell, VoxelDataT&& voxelData)
    {
        grid_cell3 scalableCell = scaleToVoxelSize(cell);
        expandVoxelsToNewCell(scalableCell);
        voxels_[scalableCell.x][scalableCell.y][scalableCell.z] = std::move(voxelData);
    }

    // Functions to get the voxel data at a specific grid cell
    const VoxelDataT& getVoxel(int64_t x, int64_t y, int64_t z) const { return getVoxel(grid_cell3{x, y, z}); }
    const VoxelDataT& getVoxel(const grid_cell3& cell) const
    {
        grid_cell3 scalableCell = scaleToVoxelSize(cell);
        assert(isVoxelCellWithinBounds(scalableCell) && "Voxel cell is out of bounds.");
        return voxels_[scalableCell.x][scalableCell.y][scalableCell.z];
    }

    // Removing all set voxels
    void clearVoxels()
    {
        voxels_.clear();
    }

  private:
    // Function to expand the deque in both directions from a given coordinate
    template <typename DequeT, typename ElementT>
    void expandAxis(DequeT& deque, int64_t coordinate)
    {
        if (deque.empty())
            deque.emplace_back(ElementT());

        auto expandFront = [&deque](int64_t expandTo) {
            for (int64_t i = deque.min_idx(); i > expandTo; i--)
                deque.emplace_front(ElementT());
        };

        auto expandBack = [&deque](int64_t expandTo) {
            for (int64_t i = deque.max_idx() - 1; i < expandTo; i++)
                deque.emplace_back(ElementT());
        };

        for (int64_t i = coordinate; i < deque.max_idx(); ++i)
            expandFront(i);

        for (int64_t i = coordinate; i >= deque.min_idx(); --i)
            expandBack(i);
    }

    // Function to expand the voxels to a new cell in 3D space
    void expandVoxelsToNewCell(const grid_cell3& cell)
    {
        expandAxis<VoxelDeque3D, VoxelDeque2D>(voxels_, cell.x); // Expand along x-axis to new cell

        // Iterate over x dimension
        for (int64_t x = voxels_.min_idx(); x < voxels_.max_idx(); ++x)
        {
            expandAxis<VoxelDeque2D, VoxelDeque1D>(voxels_[x], cell.y); // Expand along y-axis to new cell

            // Iterate over y dimension
            for (int64_t y = voxels_[x].min_idx(); y < voxels_[x].max_idx(); ++y)
            {
                expandAxis<VoxelDeque1D, VoxelDataT>(voxels_[x][y], cell.z); // Expand along z-axis to new cell
            }
        }
    }

    // Checks if a specified grid cell is within the range of the voxel data grid.
    bool isVoxelCellWithinBounds(const grid_cell3& cell) const
    {
        return cell.x >= voxels_.min_idx() && cell.x < voxels_.max_idx() && cell.y >= voxels_[cell.x].min_idx() &&
               cell.y < voxels_[cell.x].max_idx() && cell.z >= voxels_[cell.x][cell.y].min_idx() &&
               cell.z < voxels_[cell.x][cell.y].max_idx();
    }

    grid_cell3 scaleToVoxelSize(const grid_cell3& cell) const
    {
        const size_t xTail = cell.x & (voxelSize_ - 1);
        const size_t yTail = cell.y & (voxelSize_ - 1);
        const size_t zTail = cell.z & (voxelSize_ - 1);

        const int64_t xScaled = cell.x / voxelSize_ + (xTail ? 1 : 0);
        const int64_t yScaled = cell.y / voxelSize_ + (yTail ? 1 : 0);
        const int64_t zScaled = cell.z / voxelSize_ + (zTail ? 1 : 0);

        return {cell.x / voxelSize_, cell.y / voxelSize_, cell.z / voxelSize_};
    }

  private:
    VoxelDeque3D voxels_;
    int voxelSize_ = 1;
};

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_voxel_grid3_h_included

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