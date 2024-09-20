/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_generic_reference_h_included
#define sixit_geometry_low_level_generic_reference_h_included

#include <vector>
#include <cassert>

#include "sixit/geometry/low_level/forward_declaration.h"

class imp_scene_to_mesh3_converter;

namespace R3DSystem_Designer
{
    class imp_scene_to_mesh3_converter;
}

namespace sixit
{
namespace geometry
{

// template<typename fp, typename TSetPixel> 
// SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel) 
// void supercover_line(const line_segment2<fp> &ls2, TSetPixel& SET_PIXEL);

// template<typename TSetVoxel> 
// SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
// void line_voxelize(const line_segment3& ls3, TSetVoxel& SET_VOXEL);

// template<typename TSetVoxel>
// SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel) 
// void triangle_voxelize(const triangle3& trng, TSetVoxel& SET_VOXEL);

namespace low_level
{
    template <typename T>
    struct reference_container
    {
        template<typename T1> friend class ::sixit::graphics::full_mesh3;
        template<class FullMeshT> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, FullMeshT) friend class sixit::graphics::mesh3_importer;
        template <typename fp> friend struct sixit::geometry::projection3to2noscale;
        template <typename fp> friend struct sixit::geometry::projection2to1noscale;

      protected:
        std::vector<T> container;
        inline reference_container(const std::vector<T>& items)
        {
            container.reserve(items.size());
            std::copy(items.begin(), items.end(), std::back_inserter(container));
        }

        inline reference_container(std::vector<T>&& items): container(std::move(items))
        {}

      public:
        inline reference_container() {}

        inline reference_container(reference_container<T>&& o)
        {
            container = std::move(o.container);
            o.container = std::vector<T>();
        }

        inline reference_container& operator=(reference_container<T>&& o)
        {
            container = std::move(o.container);
            o.container = std::vector<T>();
            return *this;
        }

        inline reference_container(const reference_container<T>& o) = delete;
        inline reference_container& operator=(const reference_container<T>& o) = delete;

        inline int64_t get_size() const
        {
            return ssize(container);
        }

        inline T& operator[](size_t i)
        {
            assert(i < container.size());
            return container[i];
        }

        inline const T& operator[](size_t i) const
        {
            assert(i < container.size());
            return container[i];
        }

        inline void copy_to(reference_container<T>& o) const
        {
            o.container.reserve(get_size());
            o.container.clear();
            std::copy(container.begin(), container.end(), std::back_inserter(o.container));
        }
    };

    template <typename T1, typename T2, int I>
    struct indexed_aux
    {
        friend class ::sixit::graphics::mesh3;
        template <class T> friend class ::sixit::graphics::morphable_mesh3;

      protected:
        // constexpr int Items = I;
        reference_container<T1>* ref_container;
        std::array<size_t, I> index;

        friend imp_scene_to_mesh3_converter;

        inline T1& get_point(size_t idx)
        {
            assert(idx < I);
            return (*ref_container)[index[idx]];
        }
        inline const T1& get_point(size_t idx) const
        {
            assert(idx < I);
            return (*ref_container)[index[idx]];
        }

        inline indexed_aux(reference_container<T1>* ref, size_t i)
        {
            ref_container = ref;
            if constexpr (I == 1)
            {
                index = {i};
            }
            assert(index.size() == 1);
        }
        inline indexed_aux(reference_container<T1>* ref, size_t i1, size_t i2)
        {
            ref_container = ref;
            if constexpr(I == 2)
            {
                index = {i1, i2};
            }
            assert(index.size() == 2);
        }
        inline indexed_aux(reference_container<T1>* ref, size_t i1, size_t i2, size_t i3)
        {
            ref_container = ref;
            if constexpr(I == 3)
            {
                index = {i1, i2, i3};
            }
            assert(index.size() == 3);
        }

      public:
        inline T2 deindex_with(const reference_container<T1>*) const
        {
            if constexpr (I == 1)
            {
                return T2((*ref_container)[index[0]]);
            }
            else if constexpr (I == 2)
            {
                return T2((*ref_container)[index[0]], (*ref_container)[index[1]]);
            }
            else if constexpr (I == 3)
            {
                return T2((*ref_container)[index[0]], (*ref_container)[index[1]], (*ref_container)[index[2]]);
            }
            static_assert(I > 0 && I <=3, "deindex_with is not implemented for such type");
        }

        inline bool operator==(const indexed_aux<T1, T2, I>& o) const
        {
            if constexpr (I == 1)
            {
                return index[0] == o.index[0] && ref_container == o.ref_container;
            }
            else if constexpr (I == 2)
            {
                return index[0] == o.index[0] && index[1] == o.index[1] && ref_container == o.ref_container;
            }
            else if constexpr (I == 3)
            {
                return index[0] == o.index[0] && index[1] == o.index[1] && index[2] == o.index[2] && ref_container == o.ref_container;
            }
            static_assert(I > 0 && I <=3, "operator== is not implemented for such type");
            // return index[0] == o.index[0] && index[1] == o.index[1] && index[2] == o.index[2] && ref_container == o.ref_container;
        }
    };

    template <typename T1, typename T2>
    struct indexed_vector
    {
        friend class ::sixit::graphics::mesh3;
        template <class T> friend class ::sixit::graphics::morphable_mesh3;
        template <typename fp> friend struct ::sixit::geometry::polygon2;

      protected:
        // constexpr int Items = I;
        reference_container<T1>* ref_container;
        std::vector<size_t> index;

        inline T1& get_point(size_t idx)
        {
            assert(idx < index.size());
            return (*ref_container)[index[idx]];
        }
        inline const T1& get_point(size_t idx) const
        {
            assert(idx < index.size());
            return (*ref_container)[index[idx]];
        }

        inline indexed_vector(reference_container<T1>* ref, const std::vector<size_t>& indexes)
        {
            ref_container = ref;
            index = indexes;
        }
        inline std::vector<T1> get_points() const
        {
            std::vector<T1> points;
            points.reserve(index.size());
           // for (size_t idx : index)
            for (size_t idx = 0; idx < index.size(); ++idx)
            {
                points.push_back(get_point(idx));
            }
            return points;
        }

      public:
        inline T2 deindex_with(const reference_container<T1>*) const
        {
            return T2(get_points());
        }

        inline bool operator==(const indexed_vector<T1, T2>& o) const
        {
            if (ref_container != o.ref_container)
                return false;
            if (index.size() != o.size())
                return false;
            bool equal = true;
            for (int i = 0; equal && i < index.size(); ++i)
            {
                equal = index[i] == o.index[i];
            }
            return equal;
        }
    };

    template <typename P, size_t I>
    struct points_holder
    {
        using StorageType = std::array<P, I>;
        StorageType points;
        constexpr size_t size() { return points.size(); }

        inline P& get_point(size_t idx)
        {
            assert(idx < points.size());
            return points[idx];
        }
        inline const P& get_point(size_t idx) const
        {
            assert(idx < points.size());
            return points[idx];
        }
    };

    // TODO: delete after to AofSTRUCT after TR-2362 and related refactoring
    template <typename Array>
    class VectorLikeArrayWrapper
    {
        Array& array_;
        size_t size_ = 0;

      public:
        using value_type = typename Array::value_type;

        explicit VectorLikeArrayWrapper(Array& array) : array_(array)
        {
            if (std::is_const_v<Array>)
            {
                size_ = array_.size();
            }
        }

        [[nodiscard]] size_t size() const
        {
            return size_;
        }
        void reserve(size_t n)
        {
            assert(n == array_.size());
        }
        const value_type& operator[](size_t i) const
        {
            return array_[i];
        }
        void push_back(const value_type& p)
        {
            assert(size_ < array_.size());
            array_[size_++] = p;
        }
    };

    template <typename P>
    struct two_points_holder: protected points_holder<P, 2>
    {
        // these friend decaratons aren't considered by XCode 15 for derived classes, for example,
        // on line_segment3 and triangle3 parameters. So we have to duplicate them to derived class as well
        // template<typename fp, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void(::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp>& ls2, TSetPixel& SET_PIXEL);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_PIXEL);   
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::line_voxelize)(const ::sixit::geometry::line_segment3& ls3, TSetVoxel& SET_VOXEL);

      protected:
        two_points_holder(const P& p1, const P& p2)
        {
            points_holder<P,2>::points[0] = p1;
            points_holder<P,2>::points[1] = p2;
        }
        using points_holder<P, 2>::points;
        inline P& p1()
        {
            return points_holder<P, 2>::get_point(0);
        }
        inline const P& p1() const
        {
            return points_holder<P, 2>::get_point(0);
        }
        inline P& p2()
        {
            return points_holder<P, 2>::get_point(1);
        }
        inline const P& p2() const
        {
            return points_holder<P, 2>::get_point(1);
        }
    };

    template <typename P>
    struct three_points_holder: protected points_holder<P, 3>
    {
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void (::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_VOXEL);

      protected:
        three_points_holder(const P& p1, const P& p2, const P& p3)
        {
            points_holder<P,3>::points[0] = p1;
            points_holder<P,3>::points[1] = p2;
            points_holder<P,3>::points[2] = p3;
        }
        using points_holder<P, 3>::points;
        inline P& p1()
        {
            return points_holder<P, 3>::get_point(0);
        }
        inline const P& p1() const
        {
            return points_holder<P, 3>::get_point(0);
        }
        inline P& p2()
        {
            return points_holder<P, 3>::get_point(1);
        }
        inline const P& p2() const
        {
            return points_holder<P, 3>::get_point(1);
        }
        inline P& p3()
        {
            return points_holder<P, 3>::get_point(2);
        }
        inline const P& p3() const
        {
            return points_holder<P, 3>::get_point(2);
        }
    };

    template <typename P>
    struct four_points_holder : protected points_holder<P, 4>
    {
      protected:
        four_points_holder(const P& p1, const P& p2, const P& p3, const P& p4)
        {
            points_holder<P, 4>::points[0] = p1;
            points_holder<P, 4>::points[1] = p2;
            points_holder<P, 4>::points[2] = p3;
            points_holder<P, 4>::points[3] = p4;
        }
        using points_holder<P, 4>::points;
        inline P& p1()
        {
            return points_holder<P, 4>::get_point(0);
        }
        inline const P& p1() const
        {
            return points_holder<P, 4>::get_point(0);
        }
        inline P& p2()
        {
            return points_holder<P, 4>::get_point(1);
        }
        inline const P& p2() const
        {
            return points_holder<P, 4>::get_point(1);
        }
        inline P& p3()
        {
            return points_holder<P, 4>::get_point(2);
        }
        inline const P& p3() const
        {
            return points_holder<P, 4>::get_point(2);
        }
        inline P& p4()
        {
            return points_holder<P, 4>::get_point(3);
        }
        inline const P& p4() const
        {
            return points_holder<P, 4>::get_point(3);
        }
    };

}; // namespace low_level
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_generic_reference_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Volodymyr Melnychuk

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
