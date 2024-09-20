/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_triangle2_stack_h_included
#define sixit_geometry_triangle2_stack_h_included

#include "sixit/core/_internal/containers/index_preserving_deque.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/grid2.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/points_span.h"
#include "sixit/geometry/shape2.h"
//#include "sixit/core/containers.h"
#include "sixit/geometry/sixit_mathf.h"

namespace sixit
{
namespace geometry
{
    enum FastAddMode
    {
        SlowerAddFasterSearch,
        FasterAddSlowerSearch,
        SlowerAddFasterSquare
    };

    enum GridFormat
    {
        Const,
        Resizable
    };

    template <typename fp>
    struct cell_data
    {
        std::vector<size_t> triangles_touching_indices;
        fp square = 0.0f;
    };

#if SIXIT_LWA_OPTIONAL_CONCEPT
    template <typename T>
    concept vlo_grid2_cell = sixit::vlo<T> && std::same_as<sixit::internal::get_value_type<T>, grid2_cell>;
#else
#pragma message("SIXIT_WARNING: SIXIT_LWA_OPTIONAL_CONCEPT is not defined, sixit concepts unavaliable")
#endif

    template <class Triangle, typename fp>
    class triangle2_stack : public grid2<fp>
    {
    public:
        using triangle_comparator = std::function<bool(const Triangle&, const Triangle&)>; 
        using triangle_data_type = decltype(std::declval<Triangle>().get_data());

        inline triangle2_stack(fp grid_step_x = 1.0f, fp grid_step_y = 1.0f, FastAddMode as_mode = SlowerAddFasterSearch) : 
            grid2<fp>(grid_step_x, grid_step_y), add_search_mode_(as_mode), grid_storage_mode_(Resizable), bounds_(point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.0f), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.0f)))
        {
            operation_flags_.square_calculated = (as_mode == SlowerAddFasterSquare);
            operation_flags_.sorted = true;
        }
        
        triangle2_stack(fp min_x, fp min_y, fp max_x, fp max_y, fp grid_step_x = 1.0f, 
        fp grid_step_y = 1.0f, FastAddMode as_mode = SlowerAddFasterSearch, GridFormat grid_sorage_mode = Const) : 
            grid2<fp>(grid_step_x, grid_step_y), add_search_mode_(as_mode), grid_storage_mode_(grid_sorage_mode), 
            bounds_(point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(min_x), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(min_y)), 
                    point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(max_x), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(max_y)))
        {
            int left_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(grid2<fp>::to_grid_x(min_x))) - 1;
            int bottom_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(grid2<fp>::to_grid_y(min_y))) - 1;
            int right_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::ceil(grid2<fp>::to_grid_x(max_x))) + 1;
            int top_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::ceil(grid2<fp>::to_grid_y(max_y))) + 1;
            grid2_container tmp(left_border, bottom_border, right_border, top_border);
            std::swap(grid_data_, tmp);
            operation_flags_.bounds_calculated = true;
            operation_flags_.sorted = true;
            operation_flags_.square_calculated = (as_mode == SlowerAddFasterSquare);
        }
        
        triangle2_stack(const std::vector<Triangle>& triangles, fp grid_step_x = 1.0f, fp grid_step_y = 1.0f, 
        FastAddMode as_mode = SlowerAddFasterSearch, GridFormat grid_sorage_mode = Const) : 
            grid2<fp>(grid_step_x, grid_step_y), add_search_mode_(as_mode), grid_storage_mode_(grid_sorage_mode), bounds_(point2<fp>(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f)))
        {
            auto min_x = triangles.begin()->get_triangle2().bounds().minn.vec().x;
            auto min_y = triangles.begin()->get_triangle2().bounds().minn.vec().y;
            auto max_x = triangles.begin()->get_triangle2().bounds().maxx.vec().x;
            auto max_y = triangles.begin()->get_triangle2().bounds().maxx.vec().y;

            for (const Triangle& tri : triangles) 
            {   
                bounds2<fp> bounds = tri.get_triangle2().bounds();
                if (bounds.minn.vec().x < min_x) min_x = bounds.minn.vec().x;
                if (bounds.minn.vec().y < min_y) min_y = bounds.minn.vec().y;
                if (bounds.maxx.vec().x > max_x) max_x = bounds.maxx.vec().x;
                if (bounds.maxx.vec().y > max_y) max_y = bounds.maxx.vec().y;
            }

            int left_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(min_x)) - 1;
            int bottom_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(min_y)) - 1;
            int right_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::ceil(max_x)) + 1;
            int top_border = sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::ceil(max_y)) + 1;

            grid2_container tmp(left_border, bottom_border, right_border, top_border + 1);
            std::swap(grid_data_, tmp);

            bounds_= bounds2<fp>(point2<fp>(min_x, min_y), point2<fp>(max_x, max_y));
            operation_flags_.bounds_calculated = true;

            for (const auto& triangle : triangles)
                add(triangle);
        }
                
        void add(const Triangle& tri)
        {
            operation_flags_.bounds_calculated = false || grid_storage_mode_ == Const;

            size_t triangle_index;
            if (removed_tri_indices_.empty())
            {
                triangle_index = triangles_.size(); // Index of the newly added triangle
                triangles_.push_back(tri);
                sorted_indices_.push_back(triangle_index);
            }
            else
            {
                // triangles has been deleted before. Reuse the same index with new triangle
                triangle_index = *removed_tri_indices_.begin();
                triangles_[triangle_index] = tri;
                removed_tri_indices_.erase(removed_tri_indices_.begin());
            }

            const std::vector<grid2_cell> affected_cells = add_search_mode_ == FasterAddSlowerSearch
                ? grid2<fp>::triangle2_bounds_to_grid_cells(tri.get_triangle2())
                : grid2<fp>::triangle2_to_grid_cells(tri.get_triangle2());

            if (add_search_mode_ == SlowerAddFasterSquare)
            {
                for (const auto& cell : affected_cells)
                {
                    cell_data<fp>& data = grid_storage_mode_ == Const ? grid_data_.at(cell) :
                                                                        grid_data_.at_with_verification(cell); 
                    operation_flags_.sorted &= data.triangles_touching_indices.empty() || !(tri < triangles_[data.triangles_touching_indices.back()]);
                    data.triangles_touching_indices.push_back(triangle_index);
                    calculate_cell_square(cell);
                }
            }
            else
            {
                for (const auto& cell : affected_cells)
                {
                    cell_data<fp>& data = grid_storage_mode_ == Const ? grid_data_.at(cell) :
                                                                        grid_data_.at_with_verification(cell);
                    operation_flags_.sorted &= data.triangles_touching_indices.empty() || !(tri < triangles_[data.triangles_touching_indices.back()]);
                    data.triangles_touching_indices.push_back(triangle_index);
                }

                operation_flags_.square_calculated = false;
            }
        }

        template<typename... Args>
        inline void add(Args... args)
        {
            add(Triangle(args...));
        }

        void remove(const Triangle& tri)
        {
            // Find the triangle in the triangles_ vector
            auto it = std::find(triangles_.begin(), triangles_.end(), tri);

            while (it != triangles_.end())
            {
                // Get the index of the triangle to remove
                size_t triangle_index = std::distance(triangles_.begin(), it);

                if (std::find(removed_tri_indices_.begin(), removed_tri_indices_.end(), triangle_index) != removed_tri_indices_.end())
                {
                    it = std::find(++it, triangles_.end(), tri);
                    continue;
                }

                // Get the cells related to the triangle
                const std::vector<grid2_cell> related_cells = add_search_mode_ == FasterAddSlowerSearch
                    ? grid2<fp>::triangle2_bounds_to_grid_cells(tri.get_triangle2())
                    : grid2<fp>::triangle2_to_grid_cells(tri.get_triangle2());

                for (const grid2_cell& cell : related_cells)
                {
                    cell_data<fp>& data = grid_data_.at(cell);

                    // Remove the triangle index from the cell data
                    auto it_touching = std::find(data.triangles_touching_indices.begin(), data.triangles_touching_indices.end(), triangle_index);

                    // Update or remove the cell data accordingly
                    if (it_touching != data.triangles_touching_indices.end())
                    {
                        data.triangles_touching_indices.erase(it_touching);

                        if (add_search_mode_ == SlowerAddFasterSquare)
                        {
                            calculate_cell_square(cell);
                        }
                        else
                        {
                            operation_flags_.square_calculated = false;
                        }
                    }
                    else
                    {
                        assert(false && "triangle not found in cell");
                    }
                }

                // reuse the index when another triangle is added
                // do not remove triangle : avoid already existing indices in cells to missmatch triangles indices.
                removed_tri_indices_.insert(triangle_index);
                sorted_indices_.erase(std::remove(sorted_indices_.begin(), sorted_indices_.end(), triangle_index), sorted_indices_.end());
                operation_flags_.bounds_calculated = false || grid_storage_mode_ == Const;

                break;
            }
        }

        inline fp cell_square(const grid2_cell& cell)
        {
            if (!operation_flags_.square_calculated)
                calculate_cell_square(cell);

            return grid_data_.at(cell).square;
        }

        inline void calculate_all_cell_squares()
        {
            if (!operation_flags_.square_calculated)
                grid_data_.foreach_cell([this](const grid2_cell& cell) { calculate_cell_square(cell); });
            operation_flags_.square_calculated = true;
        }

        inline fp get_square()
        {
            fp result = 0;
            calculate_all_cell_squares();
            grid_data_.foreach_cell([this, &result](const grid2_cell& cell) { result = result + cell_square(cell); });
            return result;
        }

        inline size_t size() const
        {
            return sorted_indices_.size();
        }

        bounds2<fp> bounds() const
        {
            if (!operation_flags_.bounds_calculated)
            {
                if (!triangles_.empty())
                {
                    // Starting with the bounds of the first triangle
                    bounds_ = triangles_.front().get_triangle2().bounds();

                    for (const auto& item : triangles_)
                    {
                        const auto& triangle = item.get_triangle2();
                        auto b = triangle.bounds();
                        bounds_.expand_by(b.minn);
                        bounds_.expand_by(b.maxx);
                    }
                }
                else
                {
                    bounds_ = bounds2(point2<fp>(fp(0.0f), fp(0.0f)));
                }
                operation_flags_.bounds_calculated = true;
            }
            
            return bounds_;
        }
        
        void sort(triangle_comparator comp = [](const Triangle& l, const Triangle& r) { return l < r; })
        {
            if (operation_flags_.sorted)
                return;

            // Sort the triangles based on triangle_comparator.
            std::stable_sort(sorted_indices_.begin(), sorted_indices_.end(), [this, comp](size_t l, size_t r) { return comp(this->triangles_[l], this->triangles_[r]); });
            
            // Assign new sorted indices to each triangle in the grid cells they affect.
            std::vector<std::vector<std::vector<size_t>*>> storage(sorted_indices_.size());
            grid_data_.foreach_cell([this, comp, &storage](const grid2_cell& cell)
                {
                    auto& triangles_indices = this->grid_data_.at(cell).triangles_touching_indices;
                    for (const auto& idx : triangles_indices)
                        storage[idx].push_back(&triangles_indices);
                    triangles_indices.clear();
                });
            for (const auto& tri_idx : sorted_indices_)
            {
                for (auto& triangles_indices : storage[tri_idx])
                    triangles_indices->push_back(tri_idx);
            }
                
            // Mark the triangles as sorted.
            operation_flags_.sorted = true;
        }

        // -- Iterators
        class cell_iterator
        {
        public:
            using pointer = const Triangle*;
            using reference = const Triangle&;
            using value_type = Triangle;
            using iterator_category = std::forward_iterator_tag;
            using difference_type = typename std::vector<Triangle>::iterator::difference_type;

            inline cell_iterator(triangle2_stack& stack, const grid2_cell& cell, std::optional<triangle_data_type> min_data = std::nullopt, bool end_flag = false)
                : stack_(stack), cell_(cell), current_index_(0), min_data_(min_data)
            {
                if (end_flag)
                {
                    // Set to the size of the array to indicate the end
                    const cell_data<fp>& data = stack_.grid_data_.at(cell);
                    current_index_ = data.triangles_touching_indices.size();
                }
                else
                {
                    // If data is specified, find the index of the next valid (filtered by data) triangle.
                    if (min_data_.has_value())
                    {
                        assert(stack_.operation_flags_.sorted && "Triangles must be sorted before retrieval");
                        advance_to_next_valid_data();
                    }
                }
            }

            inline reference operator*()
            {
                size_t triangle_index = stack_.grid_data_.at(cell_).triangles_touching_indices[current_index_];
                return stack_.triangles_[triangle_index];
            }
            inline pointer operator->()
            {
                size_t triangle_index = stack_.grid_data_.at(cell_).triangles_touching_indices[current_index_];
                return &stack_.triangles_[triangle_index];
            }
            inline bool operator==(const cell_iterator& other) const 
            { 
                return &stack_ == &(other.stack_) && cell_ == other.cell_ && current_index_ == other.current_index_;
            }
            inline bool operator!=(const cell_iterator& other) const 
            { 
                return !(*this == other); 
            }
            inline cell_iterator& operator++()
            {
                ++current_index_;

                if (min_data_.has_value())
                    advance_to_next_valid_data();

                return *this;
            }

            inline size_t get_inside_index() const 
            {
                return stack_.grid_data_.at(cell_).triangles_touching_indices[current_index_];
            }

        private:
            inline void advance_to_next_valid_data()
            {
                const auto& indices_in_cell = stack_.grid_data_.at(cell_).triangles_touching_indices;
                const size_t end = indices_in_cell.size();
                
                while (current_index_ < end)
                {
                    const size_t triangle_index = indices_in_cell[current_index_];
                    const triangle_data_type current_data = stack_.triangles_[triangle_index].get_data();

                    if (min_data_.value() > current_data)
                    {
                        break; // Found a valid index, exit the loop.
                    }

                    if (stack_.operation_flags_.sorted)
                        current_index_ = end;
                    else
                        ++current_index_;
                }
            }

        private:
            const triangle2_stack& stack_;
            const grid2_cell cell_;
            size_t current_index_;
            std::optional<triangle_data_type> min_data_;
        };

        // -- Iterators for cell-specific triangle data
        inline cell_iterator candidates_begin(grid2_cell cell) { return cell_iterator(*this, cell); }
        inline cell_iterator candidates_end(grid2_cell cell) { return cell_iterator(*this, cell, std::nullopt, true); }

        // -- Iterators for filtered by triangle data
        inline cell_iterator filtered_candidates_begin(grid2_cell cell, triangle_data_type min_data) 
        { 
            return cell_iterator(*this, cell, min_data); 
        }
        inline cell_iterator filtered_candidates_end(grid2_cell cell) 
        { 
            return cell_iterator(*this, cell, std::nullopt, true); 
        }
        
        inline typename grid2<fp>::iterator cells_begin(const point2<fp>& coord) const { return iterator(*this, coord); }
        inline typename grid2<fp>::triangle_iterator cells_begin(const triangle2<fp> & tri) const { return typename grid2<fp>::triangle_iterator(*this, tri, add_search_mode_ == FasterAddSlowerSearch); }
        inline typename grid2<fp>::line_segment_iterator cells_begin(const line_segment2<fp>& coord) { return typename grid2<fp>::line_segment_iterator(*this, coord); }
        inline typename grid2<fp>::line_segment_iterator cells_begin(const line_segment2<fp>& coord, const bounds2<fp>& box) { return typename grid2<fp>::line_segment_iterator(*this, coord, box); }

        template <class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES(vlo_grid2_cell, VLO)
        class merge_iterator
        {
        public:
            using pointer = Triangle*;
            using reference = Triangle&;
            using value_type = Triangle;
            using iterator_category = std::forward_iterator_tag;
            using difference_type = typename std::vector<Triangle>::iterator::difference_type;

            inline merge_iterator(triangle2_stack& stack, const VLO& cells, std::optional<fp> min_data = std::nullopt)
                : stack_(stack), cells_(cells), min_data_(min_data)
            {
                assert(stack_.operation_flags_.sorted && "Triangles must be sorted before retrieval");
                merge_sequences();
                indices_it_ = merged_triangles_.begin();
            }

            inline reference operator*() 
            { 
                return *indices_it_; 
            }
            inline pointer operator->() 
            { 
                return &*indices_it_; 
            }
            inline merge_iterator& operator++()
            {
                if (indices_it_ != merged_triangles_.end())
                {
                    ++indices_it_;
                }
                return *this;
            }
            inline bool operator!=(const merge_iterator& other) const
            {
                // If both iterators are at the end
                if (indices_it_ == merged_triangles_.end() && other.indices_it_ == other.merged_triangles_.end())
                {
                    return false;
                }

                // If one iterator is at the end but the other isn't
                if ((indices_it_ == merged_triangles_.end() && other.indices_it_ != other.merged_triangles_.end()) ||
                    (indices_it_ != merged_triangles_.end() && other.indices_it_ == other.merged_triangles_.end()))
                {
                    return true;
                }

                // Default comparison
                return indices_it_ != other.indices_it_;
            }

        private:
            inline void merge_sequences()
            {
                std::vector<Triangle> temp_merged_triangles;

                for (const auto& cell : cells_)
                {
                    auto cell_iter = stack_.candidates_begin(cell);
                    auto cell_end = stack_.candidates_end(cell);

                    // Copying and filtering candidates based on min_data_
                    std::copy_if(cell_iter, cell_end, std::back_inserter(temp_merged_triangles), [this](const auto& candidate) {
                        return !min_data_ || candidate.get_data() < min_data_.value();
                        });
                }

                // Sorting by the first element of each pair to prepare for the merge
                std::sort(temp_merged_triangles.begin(), temp_merged_triangles.end(),
                    [](const auto& a, const auto& b) { return a.get_data() > b.get_data(); });

                // Eliminating duplicates based on the data element of each triangle
                temp_merged_triangles.erase(std::unique(temp_merged_triangles.begin(), temp_merged_triangles.end(),
                    [](const auto& a, const auto& b) { return a.get_data() == b.get_data(); }),
                    temp_merged_triangles.end());

                // Assigning the temporary vector to the merged_triangles_
                merged_triangles_ = std::move(temp_merged_triangles);
            }

        private:
            triangle2_stack& stack_;
            VLO cells_;
            std::vector<Triangle> merged_triangles_;
            typename std::vector<Triangle>::iterator indices_it_;
            std::optional<fp> min_data_;
        };

        class triangle2_stack_iterator
        {
        public:
            inline triangle2_stack_iterator(triangle2_stack& stack, size_t index) : stack_(stack), index_(index)
            {
                iter_ = index >= stack_.sorted_indices_.size() ? stack_.triangles_.end() : stack_.triangles_.begin() + stack_.sorted_indices_[index];
            }

            inline bool operator==(triangle2_stack_iterator const& other) const
            {
                return &stack_ == &(other.stack_) && iter_ == other.iter_ && index_ == other.index_;
            }
            inline bool operator!=(triangle2_stack_iterator const& other) const
            {
                return !(*this == other);
            }
            Triangle& operator*() const
            {
                return *iter_;
            }
            Triangle& operator->() const
            {
                return &(*iter_);
            }
            inline triangle2_stack_iterator& operator++()
            {
                ++index_;
                iter_ = index_ >= stack_.sorted_indices_.size() ? stack_.triangles_.end() : stack_.triangles_.begin() + stack_.sorted_indices_[index_];
                return *this;
            }

        private:
            triangle2_stack& stack_;
            typename std::vector<Triangle>::iterator iter_;
            size_t index_;
        };

        inline triangle2_stack_iterator begin()
        { 
            return triangle2_stack_iterator(*this, 0);
        }
        inline triangle2_stack_iterator end()
        { 
            return triangle2_stack_iterator(*this, sorted_indices_.size());
        }

        template <class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES(vlo_grid2_cell, VLO)
            inline merge_iterator<VLO> candidates_begin(const VLO& cells) { return merge_iterator<VLO>(*this, cells); }

        template <class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES(vlo_grid2_cell, VLO)
            inline merge_iterator<VLO> candidates_end(const VLO& cells) { return merge_iterator<VLO>(*this, VLO()); }

        template <class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES(vlo_grid2_cell, VLO)
            inline merge_iterator<VLO> filtered_candidates_begin(const VLO& cells, triangle_data_type min_data) 
        { 
            return merge_iterator<VLO>(*this, cells, min_data); 
        }

        template <class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES(vlo_grid2_cell, VLO)
            inline merge_iterator<VLO> filtered_candidates_end(const VLO& cells) { return merge_iterator<VLO>(*this, VLO()); }

    private:

        class grid2_container
        {
        public:
            grid2_container(int min_x = 0, int min_y = 0, int max_x = 0, int max_y = 0)
            {
                const auto width = max_x - min_x;
                if (width != 0)
                {
                    data_.at_for_writing(min_x);
                    data_.at_for_writing(max_x);

                    const auto height = max_y - min_y;
                    if (height != 0)
                    {
                        for (int x = min_x; x < max_x; ++x)
                        {
                            auto& row = data_.at_for_writing(x);
                            row.at_for_writing(min_y);
                            row.at_for_writing(max_y);
                        }
                    }
                }
            }
            
            inline cell_data<fp>& at(grid2_cell cell) { return at(sixit::guidelines::narrow_cast<int>(cell.x), sixit::guidelines::narrow_cast<int>(cell.y)); }
            
            inline cell_data<fp>& at(int x, int y) 
            {
                assert(x >= data_.min_idx() && x < data_.max_idx() && "Index out of range");
                auto &row = data_.at_for_writing(x);
                assert(y >= row.min_idx() && y < row.max_idx() && "Index out of range");
                return row.at_for_writing(y); 
            }

            inline const cell_data<fp>& at(grid2_cell cell) const { return at(sixit::guidelines::narrow_cast<int>(cell.x), sixit::guidelines::narrow_cast<int>(cell.y)); }
            
            inline const cell_data<fp>& at(int x, int y) const { return data_[x][y]; }
            
            inline cell_data<fp>& at_with_verification(grid2_cell cell) { return at_with_verification(sixit::guidelines::narrow_cast<int>(cell.x), sixit::guidelines::narrow_cast<int>(cell.y)); }

            cell_data<fp>& at_with_verification(int x, int y)
            {
                return data_.at_for_writing(x).at_for_writing(y);
            }

            template<class TFunc>
            void foreach_cell(TFunc func)
            {
                for (int x = data_.min_idx(); x < data_.max_idx(); ++x)
                {
                    auto& row = data_[x];
                    for (int y = row.min_idx(); y < row.max_idx(); ++y)
                    {
                        //auto& cell = row[y];
                        func({ x, y });
                    }
                }
            }

        private:
            sixit::containers::index_preserving_deque< sixit::containers::index_preserving_deque< cell_data<fp>>> data_;
        };
        
        void calculate_cell_square(const grid2_cell& cell)
        {
            cell_data<fp>& data = grid_data_.at(cell);

            if (data.triangles_touching_indices.empty())
            {
                data.square = 0.0f;
            }
            else
            {
                shape2<fp> cell_shape({ polygon2<fp>(triangles_[data.triangles_touching_indices[0]].get_triangle2()) });

                for (size_t i = 1; i < data.triangles_touching_indices.size(); ++i)
                    cell_shape._or(shape2<fp>({ polygon2<fp>(triangles_[data.triangles_touching_indices[i]].get_triangle2()) }));

                const fp step_x = grid2<fp>::get_step_x();
                const fp step_y = grid2<fp>::get_step_y();
                const fp x = fp(float(cell.x)) * step_x;
                const fp y = fp(float(cell.y)) * step_y;
                const polygon2<fp> cell_polygon(
                    { 
                        point2<fp>(
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y)), 
                        point2<fp>(
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y + step_y)), 
                        point2<fp>(
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x + step_x), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y + step_y)), 
                        point2<fp>(
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x + step_x), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y)) });

                cell_shape._and(shape2<fp>({ cell_polygon }));
                data.square = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::square_meter>(cell_shape.square());
            }
        }

        struct operation_flags
        {
            unsigned int sorted : 1 = 0;
            unsigned int square_calculated : 1 = 0;
            unsigned int bounds_calculated : 1 = 0;
        };

        std::vector<Triangle> triangles_;
        std::vector<size_t> sorted_indices_;
        grid2_container grid_data_;
        std::set<size_t> removed_tri_indices_;  //to be reused
        FastAddMode add_search_mode_ = SlowerAddFasterSearch;
        GridFormat grid_storage_mode_ = Resizable;
        mutable bounds2<fp> bounds_;
        mutable operation_flags operation_flags_;
    };

    template <typename fp>
    struct z_order_triangle2
    {
    public:
        inline z_order_triangle2(triangle2<fp> tr, fp z_order = 0.f) : triangle(tr), z_order(z_order) {}
        inline triangle2<fp>& get_triangle2()
        {
            return triangle;
        }
        inline const triangle2<fp>& get_triangle2() const
        {
            return triangle;
        }
        inline fp get_data() const
        {
            return z_order;
        }
        inline bool operator==(const z_order_triangle2<fp>& rhs)
        {
            return z_order == rhs.z_order && triangle == rhs.triangle;
        }
    
    private:
        triangle2<fp> triangle;
        fp z_order;

    };
    template <typename fp>
    bool inline operator<(const z_order_triangle2<fp>& lhs, const z_order_triangle2<fp>& rhs)
    {
        return lhs.get_data() < rhs.get_data();
    }

}; // namespace geometry

namespace geometry 
{
    // aliases 
    template <class Triangle>
    using triangle2_stackf = triangle2_stack<Triangle, float>;
}

}; // namespace sixit

#endif //sixit_geometry_triangle2_stack_h_included

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