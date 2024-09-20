/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_indexed_h_included
#define sixit_geometry_indexed_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/indexed_polygon2.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/line_common_impl.h"
#include "sixit/geometry/low_level/polygon_common_impl.h"
#include "sixit/geometry/reference_concepts.h"
#include "sixit/core/units.h"

#include "sixit/core/lwa.h"

#include <utility>
#include <numeric>

class imp_scene_to_mesh3_converter;

namespace R3DSystem_Designer
{
    class imp_scene_to_mesh3_converter;
}

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct indexed_point1: low_level::indexed_aux<point1<fp>, point1<fp>, 1>
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::indexed_point1<fp>>;

      private:
        using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;
        
        inline indexed_point1(low_level::reference_container<point1<fp>>* ref, size_t i)
            : low_level::indexed_aux<point1<fp>, point1<fp>, 1>(ref, {i})
        {}

        inline const class_value& vec() const
        {
            return indexed_point1<fp>::get_point(0).vec();
        }
        inline class_value& vec()
        {
            return indexed_point1<fp>::get_point(0).vec();
        }

      public:
        // inline bool operator==(const indexed_point1& o) const
        // {
        //     return index[0] == o.index[0] && ref_container == o.ref_container;
        // }
        inline bool is_same(const indexed_point1& o) const
        {
            return vec() == o.vec();
        }
    };

    template <typename fp>
    struct indexed_point2: low_level::indexed_aux<point2<fp>, point2<fp>, 1>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_point2<fp>>;
        friend struct indexed_polygon2<fp>;
        friend struct indexed_triangle2<fp>;
        template <typename fp1>
        friend struct polygon2_impl;

      private:
        inline indexed_point2(low_level::reference_container<point2<fp>>* ref, size_t i)
            :low_level::indexed_aux<point2<fp>, point2<fp>, 1>(ref, i) {}

        inline const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& vec() const
        {
            return indexed_point2<fp>::get_point(0).vec();
        }
        inline low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& vec()
        {
            return indexed_point2<fp>::get_point(0).vec();
        }

      public:
        // inline bool operator==(const indexed_point2& o) const
        // {
        //     return index[0] == o.index[0] && ref_container == o.ref_container;
        // }
        inline bool is_same(const indexed_point2<fp>& o) const
        {
            return vec() == o.vec();
        }
    };

    template<typename fp, reference_points3<fp> RefP3ConceptT>
    struct indexed_point3: low_level::indexed_aux<point3<fp>, point3<fp>, 1>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_point3<fp, RefP3ConceptT>>;
        friend struct plane3<fp>;
        template<typename fp1, reference_points3<fp1> T> friend struct indexed_line_segment3;
        template<class T> friend class sixit::graphics::morphable_mesh3;
        friend class sixit::graphics::mesh3;

      private:
         inline indexed_point3(low_level::reference_container<point3<fp>>* ref, size_t i)
            :low_level::indexed_aux<point3<fp>, point3<fp>, 1>(ref, i) {}

        inline const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& vec() const
        {
            return indexed_point3<fp, RefP3ConceptT>::get_point(0).vec();
        }
        inline low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& vec()
        {
            return indexed_point3<fp, RefP3ConceptT>::get_point(0).vec();
        }

      public:
        // inline bool operator==(const indexed_point3& o) const
        // {
        //     return index[0] == o.index[0] && ref_container == o.ref_container;
        // }
        inline bool is_same(const indexed_point3<fp, RefP3ConceptT>& o) const
        {
            return vec() == o.vec();
        }
    };

    template<typename fp, reference_points3<fp> RefP3ConceptT>
    struct indexed_line_segment3: low_level::indexed_aux<point3<fp>, line_segment3<fp>, 2>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment3<fp, RefP3ConceptT>>;
        friend struct plane3<fp>;
        template<class T> friend class sixit::graphics::morphable_mesh3;
        template<typename fp1, reference_points3<fp1> T> friend struct indexed_triangle3;

      private:
        inline point3<fp>& p1()
        {
            return indexed_line_segment3<fp, RefP3ConceptT>::get_point(0);
        }
        inline const point3<fp>& p1() const
        {
            return indexed_line_segment3<fp, RefP3ConceptT>::get_point(0);
        }
        inline point3<fp>& p2()
        {
            return indexed_line_segment3<fp, RefP3ConceptT>::get_point(1);
        }
        inline const point3<fp>& p2() const
        {
            return indexed_line_segment3<fp, RefP3ConceptT>::get_point(1);
        }
        //  inline indexed_line_segment3(low_level::reference_container<point3>* ref, size_t i[2])
        //     :low_level::indexed_aux<point3, line_segment3, 2>(ref, {i) {}

        inline indexed_line_segment3(low_level::reference_container<point3<fp>>* ref, size_t i1, size_t i2)
            :low_level::indexed_aux<point3<fp>, line_segment3<fp>, 2>(ref, i1, i2) {}

        inline line_segment3<fp> get() const
        {
            return line_segment3(
                indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), 
                indexed_line_segment3<fp, RefP3ConceptT>::get_point(1));
        }

      public:

        inline bool operator < (const indexed_line_segment3& ls) const
        {
          //  assert(ref_container == ls.ref_container);//needs sorting out
            if (indexed_line_segment3<fp, RefP3ConceptT>::index[0] < ls.index[0])
                return true;
            else if (indexed_line_segment3<fp, RefP3ConceptT>::index[0] == ls.index[0])
                return (indexed_line_segment3<fp, RefP3ConceptT>::index[1] < ls.index[1]);
            return false;
        }
        inline line3<fp> line() const
        {
            return line3(indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), indexed_line_segment3<fp, RefP3ConceptT>::get_point(1));
        }

        inline indexed_line_segment3 opposite() const
        {
            return indexed_line_segment3(indexed_line_segment3<fp, RefP3ConceptT>::ref_container, 
                                            indexed_line_segment3<fp, RefP3ConceptT>::index[1], 
                                            indexed_line_segment3<fp, RefP3ConceptT>::index[0]);
        }

        inline bool sorted_indices() const
        {
            return indexed_line_segment3<fp, RefP3ConceptT>::index[0] < indexed_line_segment3<fp, RefP3ConceptT>::index[1];
        }

        inline direction3<fp> direction() const
        {
            return low_level::ls3_impl<fp>::direction(indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), 
                                                        indexed_line_segment3<fp, RefP3ConceptT>::get_point(1));
        }

        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length() const
        {
            return low_level::ls3_impl<fp>::length(
                                                    indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), 
                                                    indexed_line_segment3<fp, RefP3ConceptT>::get_point(1));
        }
        inline std::pair<point3<fp>, rotation3<fp>> track(fp x) const
        {
            return low_level::ls3_impl<fp>::track(
                            indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), 
                            indexed_line_segment3<fp, RefP3ConceptT>::get_point(1), x);
        }
        inline bounds3<fp> bounds() const
        {
            return low_level::ls3_impl<fp>::bounds(indexed_line_segment3<fp, RefP3ConceptT>::get_point(0), 
                                                    indexed_line_segment3<fp, RefP3ConceptT>::get_point(1));
        }

        // inline bool operator==(const indexed_line_segment3& o) const
        // {
        //     return index[0] == o.index[0] && index[1] == o.index[1] && ref_container == o.ref_container;
        // }
    };

    template <typename fp>
    struct indexed_line_segment2: low_level::indexed_aux<point2<fp>, line_segment2<fp>, 2>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment2<fp>>;
        friend struct plane3<fp>;
        friend struct indexed_triangle2<fp>;
        friend struct indexed_polygon2<fp>;

      private:
        inline point2<fp>& p1()
        {
            return indexed_line_segment2<fp>::get_point(0);
        }
        inline const point2<fp>& p1() const
        {
            return indexed_line_segment2<fp>::get_point(0);
        }
        inline point2<fp>& p2()
        {
            return indexed_line_segment2<fp>::get_point(1);
        }
        inline const point2<fp>& p2() const
        {
            return indexed_line_segment2<fp>::get_point(1);
        }
        //  inline indexed_line_segment2(low_level::reference_container<point2>* ref, size_t i[2])
        //     :low_level::indexed_aux<point2, line_segment2, 2>(ref, i) {}

         inline indexed_line_segment2(low_level::reference_container<point2<fp>>* ref, size_t i1, size_t i2)
            :low_level::indexed_aux<point2<fp>, line_segment2<fp>, 2>(ref, i1, i2) {}

        inline line_segment2<fp> get() const
        {
            return line_segment2<fp>(indexed_line_segment2<fp>::get_point(0), 
                                        indexed_line_segment2<fp>::get_point(1));
        }

      public:
        inline line2<fp> line() const
        {
            return line2<fp>(indexed_line_segment2<fp>::get_point(0), 
                                    indexed_line_segment2<fp>::get_point(1));
        }
        inline direction2<fp> direction() const
        {
            return low_level::ls2_impl<fp>::direction(indexed_line_segment2<fp>::get_point(0), 
                                                            indexed_line_segment2<fp>::get_point(1));
        }
        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length() const
        {
            return low_level::ls2_impl<fp>::length(indexed_line_segment2<fp>::get_point(0), 
                                                        indexed_line_segment2<fp>::get_point(1));
        }
        inline std::pair<point2<fp>, rotation2<fp>> track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> x) const
        {
            return low_level::ls2_impl<fp>::track(indexed_line_segment2<fp>::get_point(0), 
                                                        indexed_line_segment2<fp>::get_point(1), x);
        }
        inline bounds2<fp> bounds() const
        {
            return low_level::ls2_impl<fp>::bounds(indexed_line_segment2<fp>::get_point(0), 
                                                        indexed_line_segment2<fp>::get_point(1));
        }

        // inline bool operator==(const indexed_line_segment2& o) const
        // {
        //     return index[0] == o.index[0] && index[1] == o.index[1] && ref_container == o.ref_container;
        // }
    };

    template <typename fp>
    struct indexed_line_segment1: low_level::indexed_aux<point1<fp>, line_segment1<fp>, 2>
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment1<fp>>;
        // friend struct plane3;
        friend struct line_segment1<fp>;

      private:
        inline point1<fp>& p1()
        {
            return indexed_line_segment1<fp>::get_point(0);
        }
        inline const point1<fp>& p1() const
        {
            return indexed_line_segment1<fp>::get_point(0);
        }
        inline point1<fp>& p2()
        {
            return indexed_line_segment1<fp>::get_point(1);
        }
        inline const point1<fp>& p2() const
        {
            return indexed_line_segment1<fp>::get_point(1);
        }

        inline indexed_line_segment1(low_level::reference_container<point1<fp>>* ref, size_t i1, size_t i2)
            :low_level::indexed_aux<point1<fp>, line_segment1<fp>, 2>(ref, i1, i2) {}
        inline line_segment1<fp> get() const
        {
            return line_segment1(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1));
        }

      public:
        inline line1<fp> line() const
        {
            return line1<fp>(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1));
        }
        inline direction1<fp> direction() const
        {
            return low_level::ls1_impl<fp>::direction(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1));
        }
        inline fp length() const
        {
            return low_level::ls1_impl<fp>::length(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1));
        }
        inline std::pair<point1<fp>, rotation1<fp>> track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> x) const
        {
            return low_level::ls1_impl<fp>::track(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1), x);
        }

        inline bounds1<fp> bounds() const
        {
            return low_level::ls1_impl<fp>::bounds(indexed_line_segment1<fp>::get_point(0), indexed_line_segment1<fp>::get_point(1));
        }

        // inline bool operator==(const indexed_line_segment1& o) const
        // {
        //     return index[0] == o.index[0] && index[1] == o.index[1] && ref_container == o.ref_container;
        // }
    };

    template<typename fp, reference_points3<fp> RefP3ConceptT>
    struct indexed_triangle3: low_level::indexed_aux<point3<fp>, triangle3<fp>, 3>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_triangle3<fp, RefP3ConceptT>>;
        friend struct triangle3<fp>;
        template<class T> friend class sixit::graphics::morphable_mesh3;
        friend class sixit::graphics::mesh3;
        template<class T> friend class sixit::graphics::full_mesh3;
        friend class sixit::graphics::triangulator3;

      private:
        inline indexed_triangle3(low_level::reference_container<point3<fp>>* ref, size_t i1, size_t i2, size_t i3)
            :low_level::indexed_aux<point3<fp>, triangle3<fp>, 3>(ref, i1, i2, i3) {}
        inline triangle3<fp> get() const
        {
            return triangle3(indexed_triangle3<fp, RefP3ConceptT>::get_point(0), 
                                indexed_triangle3<fp, RefP3ConceptT>::get_point(1), 
                                indexed_triangle3<fp, RefP3ConceptT>::get_point(2));
        }
        inline point3<fp>& p1()
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(0);
        }
        inline const point3<fp>& p1() const
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(0);
        }
        inline point3<fp>& p2()
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(1);
        }
        inline const point3<fp>& p2() const
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(1);
        }
        inline point3<fp>& p3()
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(2);
        }
        inline const point3<fp>& p3() const
        {
            return indexed_triangle3<fp, RefP3ConceptT>::get_point(2);
        }

      public:
        inline plane3<fp> plane() const
        {
            return low_level::tri3_impl<fp>::plane(indexed_triangle3<fp, RefP3ConceptT>::get_point(0), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(1), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(2));
        }
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const
        {
            return low_level::tri3_impl<fp>::square(indexed_triangle3<fp, RefP3ConceptT>::get_point(0), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(1), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(2));
        }

        template<typename T3>
        inline bool intersects(const T3& o) const
        {
            return low_level::tri3_impl<fp>::intersects(indexed_triangle3<fp, RefP3ConceptT>::p1(), 
                                                        indexed_triangle3<fp, RefP3ConceptT>::p2(), 
                                                        indexed_triangle3<fp, RefP3ConceptT>::p3(), 
                                                        o.p1(), o.p2(), o.p3());
        }

        inline bounds3<fp> bounds() const
        {
            return low_level::tri3_impl<fp>::bounds(indexed_triangle3<fp, RefP3ConceptT>::get_point(0), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(1), 
                                                indexed_triangle3<fp, RefP3ConceptT>::get_point(2));
        }

        inline plane3<fp> orthogonal_plane(int edge_number) const
        {
            return this->get().orthogonal_plane(edge_number);
        }

        inline size_t n_vertices() const
        {
            return 3;
        }

        inline indexed_triangle3<fp, RefP3ConceptT> vertex(int idx) const
        {
            assert(idx < this->index.size());
            return indexed_point3<fp, RefP3ConceptT>(indexed_triangle3<fp, RefP3ConceptT>::ref_container, 
                                                        indexed_triangle3<fp, RefP3ConceptT>::index[idx]);
        }

        inline size_t n_edges() const
        {
            return 3;
        }

        inline indexed_line_segment3<fp, RefP3ConceptT> edge(int idx) const
        {
            if (idx == 0)
            {
                return indexed_line_segment3<fp, RefP3ConceptT>(indexed_triangle3<fp, RefP3ConceptT>::ref_container, 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[0], 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[1]);
            }
            else if (idx == 1)
            {
                return indexed_line_segment3<fp, RefP3ConceptT>(indexed_triangle3<fp, RefP3ConceptT>::ref_container, 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[1], 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[2]);
            }
            else if (idx == 2)
            {
                return indexed_line_segment3<fp, RefP3ConceptT>(indexed_triangle3<fp, RefP3ConceptT>::ref_container, 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[2], 
                                                                indexed_triangle3<fp, RefP3ConceptT>::index[0]);
            }
            assert(0);
        }

        inline bool has_edge(const indexed_line_segment3<fp, RefP3ConceptT>& edge) const
        {
            if ((this->index[0] != edge.index[0]) && (this->index[1] != edge.index[0]) && (this->index[2] != edge.index[0]))
                return false;
            if ((this->index[0] != edge.index[1]) && (this->index[1] != edge.index[1]) && (this->index[2] != edge.index[1]))
                return false;
            return true;
        }

        inline size_t n_faces() const
        {
            return 1;
        }

        inline indexed_triangle3 face(int idx) const
        {
            return *this;
        }

        inline ray3<fp> bisector(int idx) const
        {
            return low_level::tri3_impl<fp>::bisector(this->get_point(0), this->get_point(1), this->get_point(2), idx);
        }

        inline bool has_common_edge(const indexed_triangle3& o) const
        {
            assert(this->ref_container == o.ref_container);
            auto tri1 = this->get();
            auto tri2 = o.get();
            auto edge10 = tri1.edge(0);
            auto edge11 = tri1.edge(1);
            auto edge12 = tri1.edge(2);
            auto edge20 = tri2.edge(0);
            auto edge21 = tri2.edge(1);
            auto edge22 = tri2.edge(2);

            bool has_common_1 = edge10 == edge20 || edge10 == edge21 || edge10 == edge22;
            bool has_common_2 = has_common_1 || edge11 == edge20 || edge11 == edge21 || edge11 == edge22;
            bool has_common_3 = has_common_1 || has_common_2 || edge12 == edge20 || edge12 == edge21 || edge12 == edge22;
            return has_common_3;
        }

        inline direction3<fp> normal() const
        {
            return low_level::tri3_impl<fp>::normal(this->get_point(0), this->get_point(1), this->get_point(2));
        }

        template<typename T2>
        inline T2 indexed_map_on(low_level::reference_container<point2<fp>>* ref) const
        {
            return T2(ref, this->index[0], this->index[1], this->index[2]);
        }

        // inline bool operator==(const indexed_triangle3& o) const
        // {
        //     return index[0] == o.index[0] && index[1] == o.index[1] && index[2] == o.index[2] && ref_container == o.ref_container;
        // }
    };

    template <typename fp>
    struct indexed_triangle2: low_level::indexed_aux<point2<fp>, triangle2<fp>, 3>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_triangle2<fp>>;
        friend struct triangle2<fp>;
        friend struct indexed_polygon2<fp>;
        friend imp_scene_to_mesh3_converter;
        friend R3DSystem_Designer::imp_scene_to_mesh3_converter;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct indexed_triangle3;

      private:
        inline indexed_triangle2(low_level::reference_container<point2<fp>>* ref, size_t i1, size_t i2, size_t i3)
            :low_level::indexed_aux<point2<fp>, triangle2<fp>, 3>(ref, i1, i2, i3) {}

        inline triangle2<fp> get() const
        {
            return triangle2<fp>(this->get_point(0), this->get_point(1), this->get_point(2));
        }
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const
        {
            return low_level::tri2_impl<fp>::square(this->get_point(0), this->get_point(1), this->get_point(2));
        }

        // inline bool intersects(const triangle2& other) const
        // {
        //     return get().intersects(other);
        // }

        inline bounds2<fp> bounds() const
        {
            return low_level::tri2_impl<fp>::bounds(this->get_point(0), this->get_point(1), this->get_point(2));
        }

        inline int64_t n_vertices() const
        {
            return 3;
        }

        inline indexed_point2<fp> vertex(int64_t idx) const
        {
            assert(idx < ssize(this->index));
            return indexed_point2(this->ref_container, this->index[idx]);
        }

        inline int64_t n_edges() const
        {
            return 3;
        }

        inline indexed_line_segment2<fp> edge(int64_t idx) const
        {
            if (idx == 0)
            {
                return indexed_line_segment2(this->ref_container, this->index[0], this->index[1]);
            }
            else if (idx == 1)
            {
                return indexed_line_segment2(this->ref_container, this->index[1], this->index[2]);
            }
            else if (idx == 2)
            {
                return indexed_line_segment2(this->ref_container, this->index[2], this->index[0]);
            }
        }

        inline int64_t n_faces() const
        {
            return 1;
        }

        inline indexed_triangle2 face(int) const
        {
            return *this;
        }

        int normal() const
        {
            return low_level::tri2_impl<fp>::normal(this->get_point(0), this->get_point(1), this->get_point(2));
        }

        bool is_inside(const point2<fp>& p) const
        {
            return low_level::tri2_impl<fp>::is_inside_basic(this->get_point(0), this->get_point(1), this->get_point(2), p);
        }

        bool is_inside_combinable(const point2<fp>& p) const
        {
            return low_level::tri2_impl<fp>::is_inside_combinable(this->get_point(0), this->get_point(1), this->get_point(2), p);
        }

        inline bool has_common_edge(const indexed_triangle2& o) const
        {
            assert(this->ref_container == o.ref_container);
            auto tri1 = get();
            auto tri2 = o.get();
            auto edge10 = tri1.edge(0);
            auto edge11 = tri1.edge(1);
            auto edge12 = tri1.edge(2);
            auto edge20 = tri2.edge(0);
            auto edge21 = tri2.edge(1);
            auto edge22 = tri2.edge(2);

            bool has_common_1 = edge10 == edge20 || edge10 == edge21 || edge10 == edge22;
            bool has_common_2 = has_common_1 || edge11 == edge20 || edge11 == edge21 || edge11 == edge22;
            bool has_common_3 = has_common_1 || has_common_2 || edge12 == edge20 || edge12 == edge21 || edge12 == edge22;
            return has_common_3;
        }

        // inline bool operator==(const indexed_triangle2& o) const
        // {
        //     return index[0] == o.index[0] && index[1] == o.index[1] && index[2] == o.index[2] && ref_container == o.ref_container;
        // }
    };

    template <typename fp>
    struct standalone_reference_points3: low_level::reference_container<point3<fp>>
    {
        template<class T1> friend class graphics::full_mesh3;
        template<class T> friend class graphics::morphable_mesh3;

      private:
        inline standalone_reference_points3(std::vector<point3<fp>> v) : low_level::reference_container<point3<fp>>(v)
        {}

      public:
        standalone_reference_points3() = default;

        template<typename T>
        inline point3<fp> deindex(const indexed_point3<fp, T>& ip)
        {
            return ip.deindex_with(this);
        }

        template<typename T>
        inline line_segment3<fp> deindex(const indexed_line_segment3<fp, T>& ip)
        {
            return ip.deindex_with(this);
        }

        template<typename T>
        inline triangle3<fp> deindex(const indexed_triangle3<fp, T>& ip)
        {
            return ip.deindex_with(this);
        }

        bounds3<fp> bounds() const
        {
            auto bb = bounds3<fp>( (*this)[0]);
            for (size_t i = 1; i < (*this).get_size(); ++i)
                bb.expand_by((*this)[i]);

            return bb;
        }
    };

    template <typename fp> using _standalone_indexed_point3 = indexed_point3<fp, standalone_reference_points3<fp>> ;
    template <typename fp> using _standaloneindexed_line_segment3 = indexed_line_segment3<fp, standalone_reference_points3<fp>> ;
    template <typename fp> using _standalone_indexed_triangle3 = indexed_triangle3<fp, standalone_reference_points3<fp>> ;

    template <typename fp>
    struct standalone_reference_points2: low_level::reference_container<point2<fp>>
    {
        template<class T1> friend class graphics::full_mesh3;
        template<class T> friend class graphics::morphable_mesh3;
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) friend class sixit::graphics::mesh3_importer;
        friend struct projection3to2noscale<fp>;
      private:
        inline standalone_reference_points2(std::vector<point2<fp>> v) : low_level::reference_container<point2<fp>>(v)
        {}

        public:

        standalone_reference_points2() = default;
        inline point2<fp> deindex(const indexed_point2<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        inline line_segment2<fp> deindex(const indexed_line_segment2<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        inline triangle2<fp> deindex(const indexed_triangle2<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        inline polygon2<fp> deindex(const indexed_polygon2<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        bounds2<fp> bounds() const
        {
            auto bb = bounds2<fp>((*this)[0]);
            for (size_t i = 1; i < (*this).get_size(); ++i)
                bb.expand_by((*this)[i]);

            return bb;
        }
    };

    template <typename fp>
    struct reference_points1: low_level::reference_container<point1<fp>>
    {
      public:
        inline point1<fp> deindex(const indexed_point1<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        inline line_segment1<fp> deindex(const indexed_line_segment1<fp>& ip)
        {
            return ip.deindex_with(this);
        }

        bounds1<fp> bounds() const
        {
            auto bb = bounds1<fp>((*this)[0]);
            for (size_t i = 1; i < (*this).get_size(); ++i)
                bb.expand_by((*this)[i]);

            return bb;
        }

    };
}; // namespace geometry

namespace geometry 
{
    // aliases 
    using indexed_point1f = indexed_point1<float>;
    using indexed_point2f = indexed_point2<float>;
    template <reference_points3<float> RefP3ConceptT>
    using indexed_point3f = indexed_point3<float, RefP3ConceptT>;

    template <reference_points3<float> RefP3ConceptT>
    using indexed_line_segment3f = indexed_line_segment3<float, RefP3ConceptT>;
    using indexed_line_segment2f = indexed_line_segment2<float>;
    using indexed_line_segment1f = indexed_line_segment1<float>;

    template <reference_points3<float> RefP3ConceptT>
    using indexed_triangle3f = indexed_triangle3<float, RefP3ConceptT>;
    using indexed_triangle2f = indexed_triangle2<float>;

    using standalone_reference_points3f = standalone_reference_points3<float>;
    using standalone_reference_points2f = standalone_reference_points2<float>;

    using reference_points1f = reference_points1<float>;

    using standalone_indexed_point3f = _standalone_indexed_point3<float>;
    using standalone_indexed_point3 [[deprecated]] = _standalone_indexed_point3<float>;

    using standaloneindexed_line_segment3f = _standaloneindexed_line_segment3<float>;
    using standaloneindexed_line_segment3 [[deprecated]] = _standaloneindexed_line_segment3<float>;

    using standalone_indexed_triangle3f = _standalone_indexed_triangle3<float>;
    using standalone_indexed_triangle3 [[deprecated]] = _standalone_indexed_triangle3<float>;
}


}; // namespace sixit

#endif //sixit_geometry_indexed_h_included

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