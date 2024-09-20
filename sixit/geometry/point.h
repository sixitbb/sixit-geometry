/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_point_h_included
#define sixit_geometry_point_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/units.h"
#include "sixit/core/units.h"

#include "sixit/core/lwa.h"
#include <vector>

namespace sixit
{

    namespace graphics
    {
        // forward only
        struct camera;
        class intersectable_mesh;
    }

namespace geometry
{
    enum class XYZ;

    namespace low_level
    {
        // forward only
        class from_interpolation_tag_t;
        class interpolation_friend;
    }

    template <typename fp>
    struct point1
    {
        friend struct sixit::lwa::fmt::formatter<point1<fp>>;
        friend struct line_segment1<fp>;
        friend struct triangle3<fp>;
        friend struct low_level::ls1_impl<fp>;
        friend struct bounds1<fp>;
        friend struct direction1<fp>;
        friend struct indexed_point1<fp>;
        friend struct indexed_line_segment1<fp>;
        friend struct rotation1<fp>;
        friend struct affine_transform2to1<fp>;
        friend struct projection2to1<fp>;
        friend struct projection2to1ex<fp>;
        friend struct projection2to1noscale<fp>;
        friend class low_level::interpolation_friend;
      public:
        using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;
      private:
        
        class_value x;

        inline class_value& vec()
        {
            return x;
        }
        inline const class_value& vec() const
        {
            return x;
        }
        inline point1(class_value v): x{v} {};
        inline static point1 max_point()
        {
            return point1(low_level::mathf::infinity);
        }
        inline static point1 min_point()
        {
            return point1(-low_level::mathf::infinity);
        }

        // mb: interpolation
        point1(const low_level::from_interpolation_tag_t&, std::tuple<class_value> t): point1(std::get<0>(t)) {}
        std::tuple<class_value> get_interpolation_data() const { return {x}; }

      public:
        inline point1() : 
                x(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero()) {}
        bool operator==(const point1<fp>& other) const
        {
            return vec() == other.vec();
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"point1">(comparser, obj);
            sixit::rw::read_write<&point1<fp>::x, "x", sixit::rw::F32>(comparser);
            sixit::rw::end_struct(comparser);
        }

    };

    template <typename fp>
    struct point2
    {
        template <typename fp2> friend struct point2;
        friend class voxelizer;
        friend struct triangle3<fp>;
        friend struct sixit::lwa::fmt::formatter<point2<fp>>;
        friend struct bounds2<fp>;
        friend struct barycentric<fp>;
        friend struct ray2<fp>;
        template <typename fp1, XYZ axis> friend struct axis_oriented_ray2;
        friend struct direction2<fp>;
        friend struct translation2<fp>;
        friend struct scale2<fp>;
        friend struct trs2<fp>;
        friend struct line2<fp>;
        friend struct line_segment2<fp>;
        friend struct arc2<fp>;
        friend struct qbezier2<fp>;

        friend class grid2<fp>;
        friend struct cbezier2<fp>;
        friend struct hermit2<fp>;
        friend struct curve2<fp>;
        friend class curv_builder<fp>;
        friend struct curved_polygon2<fp>;

        friend struct low_level::ls2_impl<fp>;
        friend struct low_level::tri2_impl<fp>;
        friend struct low_level::polygon2_impl<fp>;
        friend struct polygon2<fp>;
        friend struct indexed_point2<fp>;
        friend struct indexed_line_segment2<fp>;
        friend struct indexed_polygon2<fp>;
        friend struct indexed_triangle2<fp>;
        friend struct shape2_base<fp>;
        friend struct shape2<fp>;
        template <class Triangle, typename fp1> friend class triangle2_stack;
        friend struct affine_transform2<fp>;
        friend struct affine_transform3to2<fp>;
        friend struct affine_transform2to1<fp>;
        friend struct mirror_transform2<fp>;
        friend class ::sixit::graphics::texcoord_buffer_vec2_uint16;
        friend class ::sixit::graphics::texcoord_buffer_vec2;
        friend struct ::sixit::graphics::trs3_animation;
        template<class T> friend class ::sixit::graphics::full_mesh3;
        template<class T, class T2> SIXIT_LWA_OPTIONAL_REQUIRES2(::sixit::geometry::mesh_buffers, T, ::sixit::graphics::texture2, T2) friend class ::sixit::graphics::base_mesh3;
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) friend class ::sixit::graphics::mesh3_importer;

        template<typename T> friend class default_buffer;
        friend struct projection2<fp>;
        friend struct projection3to2<fp>;
        friend struct projection2to1<fp>;
        friend struct projection3to2ex<fp>;
        friend struct projection2to1ex<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct projection2to1noscale<fp>;
        friend struct projection_perspective2<fp>;
        friend struct inverse_projection_perspective3<fp>;
        friend struct axis_x_projection<fp>;
        friend struct axis_y_projection<fp>;
        friend struct axis_z_projection<fp>;

        template <typename fp1>
        friend point2<fp1> low_level::for_importer_only::make_point2_dont_use_outside_of_importer(fp1 x, fp1 y);
        template <typename fp1>
        friend point2<fp1> low_level::for_renderers_only::make_point2_dont_use_outside_of_renderer(fp1 x, fp1 y);
        template <typename fp1>
        friend std::tuple<fp1, fp1> low_level::for_renderers_only::xy_dont_use_outside_of_renderer(const point2<fp1>& p);
        template <typename fp1>
        friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_renderers_only::xy_dont_use_outside_of_renderer(const line_segment2<fp1>& l);

        // template<typename fp1, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void (::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp1>& ls2, TSetPixel& SET_PIXEL);
        
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void (::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_VOXEL);

        friend class low_level::interpolation_friend;

        friend void ::gltf::double_conversion_eq(const char*, bool);

      private:
        using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;
        
        low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> vector;

        inline point2(class_value x, class_value y) : vector(x, y) {}
        inline low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& vec()
        {
            return vector;
        }
        inline const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& vec() const
        {
            return vector;
        }
        inline point2(const  low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& v): vector(v) {}
        inline static point2 max_point()
        {
            fp fp_inf(low_level::mathf::infinity);
            auto ds_inf = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_inf);

            return point2(ds_inf, ds_inf);
        }
        inline static point2 min_point()
        {
            fp fp_inf(-low_level::mathf::infinity);
            auto ds_inf = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_inf);
            
            return point2(ds_inf, ds_inf);
        }

        // mb: interpolation
        point2(const low_level::from_interpolation_tag_t&, std::tuple<class_value, class_value> t): vector(std::get<0>(t), std::get<1>(t)) {}
        std::tuple<class_value, class_value> get_interpolation_data() const { return {vector.x, vector.y}; }

      public:
        inline point2() : vector(
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero(),
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero()) {}
        bool operator==(const point2& other) const
        {
            return vector == other.vector;
        }
        template<typename fp2>
        bool operator==(const point2<fp2>& other) const
        {
            return vector == other.vector;
        }

        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> distance(const point2& other) const
        {
            low_level::dimensional_vector2 d = vec() - other.vec();
            return d.magnitude();
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            decltype(obj.vector)::read_write(obj.vector, comparser);
        }

        template<typename fp2>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const point2<fp2>& other, size_t n = 1) const
        {
            return vector._for_test_only_approximate_eq(other.vector, n);
        }
    };

    template <typename fp>
    struct point3
    {
        friend class voxelizer;
        friend sixit::graphics::intersectable_mesh;
        friend class grid3<fp>;
        friend class grid3_with_bounds<fp>;
        template<typename VoxelDataT> friend class voxel_grid3;
        friend struct sixit::lwa::fmt::formatter<point3<fp>>;
        friend struct trs3<fp>;
        friend struct ::sixit::graphics::trs3_animation;
        friend struct bounds3<fp>;
        friend struct barycentric<fp>;
        friend struct ray3<fp>;
        friend struct plane3<fp>;
        friend struct direction3<fp>;
        friend struct translation3<fp>;
        friend struct scale3<fp>;
        friend struct line_segment3<fp>;
        friend struct triangle3<fp>;
        friend struct projection3<fp>;
        friend struct projection3to2ex<fp>;
        friend struct projection_perspective3<fp>;
        friend struct inverse_projection_perspective3<fp>;
        friend struct projection_perspective2<fp>;
        friend struct affine_transform3<fp>;
        friend struct affine_transform3to2<fp>;
        friend struct mirror_transform3<fp>;
        friend struct ::sixit::graphics::camera;
        friend struct low_level::ls3_impl<fp>;
        friend struct low_level::tri3_impl<fp>;
        friend struct low_level::plane3_impl<fp>;
        friend struct for_renderers_only::frustum<fp>;
        template<class T, class T2> SIXIT_LWA_OPTIONAL_REQUIRES2(::sixit::geometry::mesh_buffers, T, ::sixit::graphics::texture2, T2) friend class ::sixit::graphics::base_mesh3;
        template<class T> friend class ::sixit::graphics::full_mesh3;
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) friend class sixit::graphics::mesh3_importer;
        template<typename T> friend class ::sixit::graphics::default_buffer;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct indexed_point3;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct indexed_line_segment3;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct indexed_triangle3;
        friend class ::sixit::graphics::gpu_vertex;
        friend struct projection3to2<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct axis_x_projection<fp>;
        friend struct axis_y_projection<fp>;
        friend struct axis_z_projection<fp>;
        friend struct point3_cloud<fp>;
        friend struct indexed_point3_cloud<fp>;
        template<class PCloud> friend plane3<fp> best_fit_plane3(const PCloud& pcloud);
        template<typename VLO> friend plane3<fp> best_fit_plane3_vlo(const VLO& vlo);
        friend class ::sixit::graphics::skeleton_joint;

        template <typename fp1> friend point3<fp1> low_level::for_renderers_only::make_point3_dont_use_outside_of_renderer(fp1 x, fp1 y, fp1 z);
        template <typename fp1> friend point3<fp1> low_level::for_importer_only::make_point3_dont_use_outside_of_importer(fp1 x, fp1 y, fp1 z);
        template <typename fp1> friend point3<fp1> _orthogonal_projection_helper(const point3<fp1>& p, const plane3<fp1>& plane);

        template <typename fp1>
        friend std::tuple<fp1, fp1, fp1> low_level::for_renderers_only::xyz_dont_use_outside_of_renderer(const point3<fp1>& p);
        //friend std::tuple<fp, fp, fp, fp> low_level::for_renderers_only::xyzw_dont_use_outside_of_renderer(const point3<fp>& p);
        //friend std::tuple<fp, fp, fp, fp> low_level::for_renderers_only::xyzw_dont_use_outside_of_renderer(const point3<fp>& p);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_importer_only::xyz_dont_use_outside_of_importer(const point3<fp1>& p);
        template <typename fp1> friend point3<fp1> low_level::for_data_transfers_only::make_point3_dont_use_outside_of_data_transfers(fp1 x, fp1 y, fp1 z);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_data_transfers_only::xyz_dont_use_outside_of_data_transfers(const point3<fp1>& p);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void (::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_VOXEL);
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void line_voxelize(const line_segment3& ls3, TSetVoxel& SET_VOXEL);

        friend class low_level::interpolation_friend;

      private:
        low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> vector;

        inline  low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& vec()
        {
            return vector;
        }
        inline const  low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& vec() const
        {
            return vector;
        }
        inline point3(const low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>& v): vector(v) {}
        inline point3(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> x, 
                        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> y, 
                        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> z): vector(x, y, z,  low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::position_type) {}

        inline static point3 max_point()
        {
            return point3(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity));
        }
        inline static point3 min_point()
        {
            return point3(
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-low_level::mathf::infinity), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-low_level::mathf::infinity), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-low_level::mathf::infinity));
        }

        // mb: interpolation
        point3(const low_level::from_interpolation_tag_t&, std::tuple<sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, 
                        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>> t): vector(std::get<0>(t), std::get<1>(t), std::get<2>(t)) {}
        std::tuple<sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, 
                        sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>> get_interpolation_data() const { return {
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(vector.x()), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(vector.y()), 
                            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(vector.z())}; }

      public:
        inline point3() : vector(
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero(),
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero(),
            sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero(),
            low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::position_type) {}
            
        bool operator==(const point3& other) const
        {
            return vector == other.vector;
        }

        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> distance(const point3& other) const
        {
            low_level::dimensional_vector3 d = vec() - other.vec();
            return d.magnitude();
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            decltype(obj.vector)::read_write(obj.vector, comparser);
        }

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const point3& other, size_t n = 1) const
        {
            return vector._for_test_only_approximate_eq(other.vector, n);
        }
    };

    template <typename fp>
    struct direction1
    {
        friend struct sixit::lwa::fmt::formatter<direction1>;
        friend struct line_segment1<fp>;
        friend struct low_level::ls1_impl<fp>;
        friend struct rotation1<fp>;
        friend class low_level::interpolation_friend;

      private:
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x;

        inline direction1(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x)
//            : x(low_level::mathf::clamp(x, sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-1.0f),
//                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f)))
            : x(low_level::mathf::clamp(x, fp(-1.0f), fp(1.0f)))
        {}

        inline sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>& vec()
        {
            return x;
        }
        inline const sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>& vec() const
        {
            return x;
        }

        // mb: interpolation
        direction1(const low_level::from_interpolation_tag_t&, std::tuple<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>> t): direction1(std::get<0>(t)) {}
        std::tuple<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>> get_interpolation_data() const { return {x}; }

      public:
        inline direction1(const point1<fp>& p1, const point1<fp>& p2) : x(p2.x - p1.x) {}

        inline static direction1 left()
        {
            //return direction1(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-1.0f));
            return direction1(fp(-1.0f));
        }

        inline static direction1 right()
        {
            //return direction1(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f));
            return direction1(fp(1.0f));
        }
    };

    template <typename fp>
    struct direction2
    {
        template <typename fp2> friend struct direction2;
        friend struct sixit::lwa::fmt::formatter<direction2<fp>>;
        friend struct ray2<fp>;
        template <typename fp1, XYZ axis> friend struct axis_oriented_ray2;
        friend struct polygon2<fp>;
        friend struct indexed_polygon2<fp>;
        friend struct rotation2<fp>;
        friend struct trs2<fp>;
        friend struct projection2to1ex<fp>;
        friend struct shape2_base<fp>;

        friend struct line_segment2<fp>;        
        friend struct arc2<fp>;
        template <typename fp1>
        friend struct curve2_primitive;
        friend struct curve2<fp>;
        friend struct curved_polygon2<fp>;
        friend struct qbezier2<fp>;
        friend struct cbezier2<fp>;
        friend class curv_builder<fp>;
        
        friend struct low_level::ls2_impl<fp>;
        friend struct low_level::tri2_impl<fp>;
        friend class low_level::interpolation_friend;

      public:
        using class_value = sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>;

        low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> vector;

        inline low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& vec()
        {
            return vector;
        }
        inline const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& vec() const
        {
            return vector;
        }
        template <sixit::units::physical_dimension dim1>
        inline direction2(const low_level::dimensional_vector2<fp, dim1>& v) : vector(v.normalized()) {}
        inline direction2(class_value x, class_value y): direction2(low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>(x, y)) {}
        inline direction2(const radians<fp>& angle) : vector(
            sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::cos(angle.value_)), 
            sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::sin(angle.value_))) {}
        inline direction2(const degrees<fp>& angle) : direction2(radians(angle)) {}

        // mb: interpolation
        direction2(const low_level::from_interpolation_tag_t&, std::tuple<fp, fp> t): vector(std::get<0>(t), std::get<1>(t)) {}
        std::tuple<fp, fp> get_interpolation_data() const { return {vector.x, vector.y}; }

      public:
        direction2() = default;
        inline direction2(const point2<fp>& p1, const point2<fp>& p2) : vector((p2.vec() - p1.vec()).normalized()) {}

        inline bool operator==(const direction2& other) const
        {
            return vector == other.vector;
        }
        template<typename fp2>
        inline bool operator==(const direction2<fp2>& other) const
        {
            return vector == other.vector;
        }

        inline static direction2 up()
        {
            return direction2(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(1.f));
        }
        inline static direction2 down()
        {
            return direction2(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(-1.f));
        }
        inline static direction2 left()
        {
            return direction2(
                sixit::units::create_dimensionless_scalar<fp>(-1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }
        inline static direction2 right()
        {
            return direction2(
                sixit::units::create_dimensionless_scalar<fp>(1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }

        inline static point2<fp> move_point(const point2<fp>& p, const direction2<fp>& dir, meters<fp> dist)
        {
            return point2<fp>(p.vec() + dir.vec() * dist.value_);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            decltype(obj.vector)::read_write(obj.vector, comparser);
        }

        template<typename fp2>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const direction2<fp2>& other, size_t n = 1) const
        {
            return vector._for_test_only_approximate_eq(other.vector, n);
        }
    };

    template <typename fp>
    struct direction3
    {
        friend class voxelizer;
        friend sixit::graphics::intersectable_mesh;
        friend struct sixit::lwa::fmt::formatter<direction3<fp>>;
        friend struct sixit::lwa::fmt::formatter<tangent<fp>>;
        friend struct tangent<fp>;
        friend struct ray3<fp>;
        friend struct plane3<fp>;
        friend struct rotation3<fp>;
        friend struct line_segment3<fp>;
        friend struct triangle3<fp>;
        friend struct projection3<fp>;
        friend struct projection3to2ex<fp>;
        friend struct mirror_transform3<fp>;
        friend struct ::sixit::graphics::camera;
        friend struct low_level::ls3_impl<fp>;
        friend struct low_level::tri3_impl<fp>;
        friend struct low_level::plane3_impl<fp>;
        template<class T> friend class ::sixit::graphics::full_mesh3;
        template<class T, class T2> SIXIT_LWA_OPTIONAL_REQUIRES2(::sixit::geometry::mesh_buffers, T, ::sixit::graphics::texture2, T2) friend class ::sixit::graphics::base_mesh3;
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) friend class ::sixit::graphics::mesh3_importer;
        template<typename T> friend class ::sixit::graphics::default_buffer;
        friend class ::sixit::graphics::normals_buffer_packed_int32;
        friend class ::sixit::graphics::normals_buffer_vec3_float16;
        friend class ::sixit::graphics::tangents_buffer_packed_int32;
        friend class ::sixit::graphics::tangents_buffer_vec4_float16;
        friend struct projection3to2<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct ::sixit::graphics::trs3_animation;
        template<class PCloud> friend plane3<fp> best_fit_plane3(const PCloud& pcloud);
        template<typename VLO> friend plane3<fp> best_fit_plane3_vlo(const VLO& vlo);
        template <typename fp1> friend direction3<fp1> low_level::for_importer_only::make_direction3_dont_use_outside_of_importer(fp1 x, fp1 y, fp1 z);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_renderers_only::xyz_dont_use_outside_of_renderer(const direction3<fp1>& d);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_renderers_only::xyz_dont_use_outside_of_renderer(const tangent<fp1>& t);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, handedness> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const tangent<fp1>& t);
        template <typename fp1> friend direction3<fp1> low_level::for_data_transfers_only::make_direction3_dont_use_outside_of_data_transfers(fp1 x, fp1 y, fp1 z);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_data_transfers_only::xyz_dont_use_outside_of_data_transfers(const direction3<fp1>& d);
        template <typename fp1> friend point3<fp1> _orthogonal_projection_helper(const point3<fp1>& p, const plane3<fp1>& plane);

        friend class low_level::interpolation_friend;

        friend void ::gltf::double_conversion_eq(const char*, bool);

      private:
      low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> vector;

        inline low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>& vec()
        {
            return vector;
        }
        inline const low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>& vec() const
        {
            return vector;
        }
        inline direction3(const low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>& v): vector(v.normalized()) {}
        inline direction3(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y,
                            sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> z): direction3(low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>(x, y, z, 
                                                                low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::normal_type)) {};

        // mb: interpolation
        direction3(const low_level::from_interpolation_tag_t&, std::tuple<fp, fp, fp> t): vector(std::get<0>(t), std::get<1>(t), std::get<2>(t)) {}
        std::tuple<fp, fp, fp> get_interpolation_data() const { return {vector.x(), vector.y(), vector.z()}; }

      public:
        direction3() : vector(sixit::units::create_dimensionless_scalar<fp>(0.f),
                                sixit::units::create_dimensionless_scalar<fp>(0.f),
                                sixit::units::create_dimensionless_scalar<fp>(0.f)) {}
        inline direction3(const point3<fp>& p1, const point3<fp>& p2) : vector((p2.vec() - p1.vec()).normalized()) {}
        inline direction3(const radians<fp>& yaw, const radians<fp>& pitch) : 
            vector(
                sixit::units::create_dimensionless_scalar<fp>(sixit::geometry::low_level::mathf::cos(yaw.value_) * sixit::geometry::low_level::mathf::cos(pitch.value_)), 
                sixit::units::create_dimensionless_scalar<fp>(sixit::geometry::low_level::mathf::sin(pitch.value_)), 
                sixit::units::create_dimensionless_scalar<fp>(sixit::geometry::low_level::mathf::sin(yaw.value_) * sixit::geometry::low_level::mathf::cos(pitch.value_))) {}
        inline direction3(const degrees<fp>& yaw, const degrees<fp>& pitch) : direction3(radians(yaw), radians(pitch)) {}

        bool operator==(const direction3& other) const
        {
            return vector == other.vector;
        }

        inline static direction3 forward()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(1.f));
        }
        inline static direction3 back()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(-1.f));
        }
        inline static direction3 up()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }
        inline static direction3 down()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(-1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }
        inline static direction3 left()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(-1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }
        inline static direction3 right()
        {
            return direction3(
                sixit::units::create_dimensionless_scalar<fp>(1.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f), 
                sixit::units::create_dimensionless_scalar<fp>(0.f));
        }

        inline static point3<fp> move_point(const point3<fp>& point, const direction3<fp>& direction, meters<fp> distance)
        {
            //return point3(point.vec() + direction.vec() * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(distance.value_));
            return point3<fp>(point.vec() + direction.vec() * distance.value_);
        }

        fp collinearity(direction3<fp> other) const
        {
            return low_level::vector3<fp>::dot( vec(), other.vec() );
        }

        [[nodiscard]] bool _for_test_only_approximate_eq(const direction3<fp>& other, size_t n = 1) const
        {
            return vector._for_test_only_approximate_eq(other.vector, n);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            decltype(obj.vector)::read_write(obj.vector, comparser);
        }
    
        struct averaging
        {
        private:
            low_level::vector3<fp> vector;

        public:
            averaging() = default;
            averaging(const direction3<fp>& direction) : vector(direction.vec()){}
            inline averaging(fp x, fp y, fp z) : vector(x, y, z){};
            void add(const direction3<fp>& direction)
            {
                vector += direction.vec();
            }
            void add(const direction3<fp>& direction, sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> weight)
            {
                vector += direction.vec() * weight;
            }
            operator direction3<fp>() const
            {
                return direction3<fp>(vector);
            }
        };
    };

    enum class handedness : int8_t
    {
        RIGHT = 1,
        LEFT = -1
    };

    template <typename fp>
    struct tangent
    {
        friend struct  sixit::lwa::fmt::formatter<tangent>;
        template <typename fp1>
        friend tangent<fp1> low_level::for_importer_only::make_tangent_dont_use_outside_of_importer(fp1, fp1, fp1, handedness);
        template <typename fp1>
        friend tangent<fp1> low_level::for_renderers_only::make_tangent_dont_use_outside_of_renderer(fp1, fp1, fp1, handedness);
        template <typename fp1>
        friend std::tuple<fp1, fp1, fp1> low_level::for_renderers_only::xyz_dont_use_outside_of_renderer(const tangent<fp1>& t);
        template <typename fp1>
        friend std::tuple<fp1, fp1, fp1, handedness> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const tangent<fp1>& t);
        friend class ::sixit::graphics::tangents_buffer_packed_int32;
        friend class ::sixit::graphics::tangents_buffer_vec4_float16;
        template <class T>
        friend class ::sixit::graphics::full_mesh3;
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) 
        friend class ::sixit::graphics::mesh3_importer;
        friend struct ::sixit::graphics::trs3_animation;

        friend void ::gltf::double_conversion_eq(const char*, bool);

      private:
        direction3<fp> dir3_;
        handedness handedness_;

        tangent(fp x, fp y, fp z, handedness w) : dir3_(
            sixit::units::create_dimensionless_scalar<fp>(x), 
            sixit::units::create_dimensionless_scalar<fp>(y), 
            sixit::units::create_dimensionless_scalar<fp>(z)), handedness_(w)
        {
        }

      public:
        bool operator==(const tangent<fp>& other) const
        {
            return dir3_ == other.dir3_ && handedness_ == other.handedness_;
        }

        tangent() = default;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            typename ComparserT::aliases::template enum_validator<handedness::RIGHT, handedness::LEFT> validator;
            sixit::rw::begin_struct<"tangent">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.dir3_.vec().x());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.dir3_.vec().y());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.dir3_.vec().z());
            sixit::rw::read_write<&tangent::handedness_, "w", sixit::rw::ENUM>(comparser, validator);
            sixit::rw::end_struct(comparser);
        }
    };

};

namespace geometry 
{
    using point1f = point1<float>;
    using point2f = point2<float>;
    using point3f = point3<float>;
    using direction1f = direction1<float>;
    using direction2f = direction2<float>;
    using direction3f = direction3<float>;
    using tangentf = tangent<float>;
}

};

#endif //sixit_geometry_point_h_included

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
