/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_forward_declaration_h_included
#define sixit_geometry_low_level_forward_declaration_h_included

#include <tuple>
#include <complex>

#include "sixit/geometry/reference_concepts.h"
#include "sixit/core/units.h"

namespace sixit
{

namespace units
{
    struct physical_dimension;
} // namespace units


namespace geometry
{
    // this should be hidden

    template <typename fp> class grid2;
    template <typename fp> class grid3;
    template <typename fp> class grid3_with_bounds;
    template<typename VoxelDataT> class voxel_grid3;

    template <typename fp> struct direction1;
    template <typename fp> struct direction2;
    template <typename fp> struct direction3;
    template <typename fp> struct point3;
    template <typename fp> struct point2;
    template <typename fp> struct point1;

    enum class handedness:int8_t;
    template <typename fp> struct tangent;

    template <typename fp> struct barycentric;

    template <typename fp> struct projection3;
    template <typename fp> struct projection2;
    template <typename fp> struct projection3to2;
    template <typename fp> struct projection2to1;
    template <typename fp> struct projection3to2ex;
    template <typename fp> struct projection2to1ex;
    template <typename fp> struct projection3to2noscale;
    template <typename fp> struct projection2to1noscale;
    template <typename fp> struct axis_x_projection;
    template <typename fp> struct axis_y_projection;
    template <typename fp> struct axis_z_projection;
    template <typename fp> struct point3_cloud;
    template <typename fp> struct indexed_point3_cloud;

    template <typename fp> struct standalone_reference_points3;
    template <typename fp> struct standalone_reference_points2;
    template <typename fp> struct reference_points1;
    template <typename fp, reference_points3<fp> RefP3ConceptT> struct indexed_point3;
    template <typename fp> struct indexed_point2;
    template <typename fp> struct indexed_point1;

    template <typename fp> struct line3;
    template <typename fp> struct line2;
    template <typename fp> struct line1;
    template <typename fp> struct bounds3;
    template <typename fp> struct bounds2;
    template <typename fp> struct bounds1;

    template <typename fp> struct line_segment3;
    template <typename fp> struct line_segment2;
    template <typename fp> struct line_segment1;

    template <typename fp, reference_points3<fp> RefP3ConceptT> struct indexed_line_segment3;
    template <typename fp> struct indexed_line_segment2;
    template <typename fp> struct indexed_line_segment1;

    template <typename fp> struct rotation3;
    template <typename fp> struct rotation2;
    template <typename fp> struct rotation1;
    template <typename fp> struct translation3;
    template <typename fp> struct translation2;
    template <typename fp> struct scale3;
    template <typename fp> struct scale2;

    template <typename fp> struct trs3;
    template <typename fp> struct trs2;

    template <typename fp> struct ray3;
    template <typename fp> struct ray2;
    
    template <typename fp> struct arc2;
    template <typename fp> struct qbezier2;
    template <typename fp> struct cbezier2;
    template <typename fp> struct hermit2;
    template <typename fp> struct curve2;
    template <typename fp> struct curved_polygon2;
    template <typename fp> struct polygon2;
    template <typename fp> struct indexed_polygon2;

    template <typename fp> struct triangle3; 
    template <typename fp> struct triangle2;

    template <typename fp, reference_points3<fp> RefP3ConceptT> struct indexed_triangle3;
    template <typename fp> struct indexed_triangle2;
    template <typename fp> struct plane3;

    template <typename fp> struct affine_transform3;
    template <typename fp> struct affine_transform2;
    template <typename fp> struct affine_transform3to2;
    template <typename fp> struct affine_transform2to1;
    template <typename fp> struct mirror_transform3;
    template <typename fp> struct mirror_transform2;
    
    template <class fp> class curv_builder;
    template <class Triangle, typename fp> class triangle2_stack;

    template <typename fp> struct shape2;
    template <typename fp> struct shape2_base;
    template <typename fp> struct indexed_shape2;

    template <typename fp> struct projection_perspective3;
    template <typename fp> struct inverse_projection_perspective3;
    template <typename fp> struct projection_perspective2;
    namespace gpu
    {
        template <typename fp> struct mat4;
    }

    template<typename fp>  class meters;

namespace low_level
{
    template <typename fp> struct quaternion;
    template <typename fp> struct matrix4x4;
    template <typename fp> struct matrix3x3;
    template <typename fp> struct vector3;
    template <typename fp, sixit::units::physical_dimension dim> struct dimensional_vector3;
    template <typename fp> struct vector2;
    template <typename fp, sixit::units::physical_dimension dim> struct dimensional_vector2;
    template <typename fp> struct ls3_impl;
    template <typename fp> struct ls2_impl;
    template <typename fp> struct ls1_impl;
    template <typename fp> struct tri3_impl;
    template <typename fp> struct tri2_impl;
    template <typename fp> struct plane3_impl;
    template <typename fp> struct polygon2_impl;

    namespace for_renderers_only
    {
        template <typename fp> inline static gpu::mat4<fp> matrix_dont_use_outside_of_renderer(const trs3<fp>& trs);
        template <typename fp> inline static matrix4x4<fp> get_camera_projection_matrix4x4_dont_use_outside_of_renderer(const projection_perspective3<fp>& proj,
            const point3<fp>& position, const direction3<fp>& direction, const meters<fp>& focus_distance);

        template <typename fp> inline static bounds3<fp> bounds3_projected_dont_use_outside_of_renderer(const trs3<fp>& trs, const bounds3<fp>& bounds);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const point3<fp>& p);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const direction3<fp>& d);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_renderer(const tangent<fp>& t);
        template <typename fp> inline static std::tuple<fp, fp> xy_dont_use_outside_of_renderer(const point2<fp>& p);
        template <typename fp> inline static point3<fp> make_point3_dont_use_outside_of_renderer(fp x, fp y, fp z);
        template <typename fp> inline static point2<fp> make_point2_dont_use_outside_of_renderer(fp x, fp y);
        template <typename fp> inline static rotation3<fp> make_rotation3_dont_use_outside_of_renderer(fp x, fp y, fp z, fp w);
        template <typename fp> inline static std::tuple<fp, fp, fp, fp> xyzw_dont_use_outside_of_renderer(const rotation3<fp>& r);
        template <typename fp> inline static std::tuple<fp, fp, fp, fp> xy_dont_use_outside_of_renderer(const line_segment2<fp>& p);
        template <typename fp> inline static tangent<fp> make_tangent_dont_use_outside_of_renderer(fp x, fp y, fp z, handedness w);
    }
    namespace for_importer_only
    {
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const point3<fp>& p);
        template <typename fp> inline static point3<fp> make_point3_dont_use_outside_of_importer(fp x, fp y, fp z);
        template <typename fp> inline static point2<fp> make_point2_dont_use_outside_of_importer(fp x, fp y);
        template <typename fp> inline static direction3<fp> make_direction3_dont_use_outside_of_importer(fp x, fp y, fp z);
        template <typename fp> inline static rotation3<fp> make_rotation3_dont_use_outside_of_importer(fp x, fp y, fp z, fp w);
        template <typename fp> inline static tangent<fp> make_tangent_dont_use_outside_of_importer(fp, fp, fp, handedness);
        template <typename fp> inline static std::tuple<fp, fp, fp, handedness> xyzw_dont_use_outside_of_importer(const tangent<fp>& p);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const geometry::translation3<fp>& tr3);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_importer(const geometry::scale3<fp>& sc3);
        template <typename fp> inline static std::tuple<fp, fp, fp, fp> xyzw_dont_use_outside_of_importer(const geometry::rotation3<fp>& rt3);
    }
    namespace for_data_transfers_only
    {
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_data_transfers(const point3<fp>& p);
        template <typename fp> inline static std::tuple<fp, fp, fp> xyz_dont_use_outside_of_data_transfers(const direction3<fp>& d);
        template <typename fp> inline static point3<fp> make_point3_dont_use_outside_of_data_transfers(fp x, fp y, fp z);
        template <typename fp> inline static direction3<fp> make_direction3_dont_use_outside_of_data_transfers(fp x, fp y, fp z);
        template <typename fp> inline static std::tuple<fp, fp, fp, fp> xyzw_dont_use_outside_of_data_transfers(const rotation3<fp>& r);
        template <typename fp> inline static rotation3<fp> make_rotation3_dont_use_outside_of_data_transfers(fp x, fp y, fp z, fp w);
    }
}

} // namespace geometry

namespace graphics
{

class grid;
struct trs3_animation;

template<class T> class full_mesh3;
template <typename MeshBuffersT, typename Texture2>
SIXIT_LWA_OPTIONAL_REQUIRES2(::sixit::geometry::mesh_buffers, MeshBuffersT, ::sixit::graphics::texture2, Texture2)
class base_mesh3;
template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T) class mesh3_importer;
template<class T> class morphable_mesh3;

template <typename T>
class default_buffer;
class gpu_vertex;
class normals_buffer_packed_int32;
class normals_buffer_vec3_float16;
class tangents_buffer_packed_int32;
class tangents_buffer_vec4_float16;
class mesh3;
class skeleton_joint;
class bone_formatter;
template <typename T>
class skinned_mesh3_rigging;
class texcoord_buffer_vec2_uint16;
class texcoord_buffer_vec2;
class triangulator2;
class triangulator3;

namespace low_level
{
namespace for_renderers_only
{

template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_normals_buffer_dont_use_outside_of_renderer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_normals_buffer_dont_use_outside_of_renderer(
    const BufferT&);
template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_tangents_buffer_dont_use_outside_of_renderer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_tangents_buffer_dont_use_outside_of_renderer(
    const BufferT&);
template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_texcoord_buffer_dont_use_outside_of_renderer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_texcoord_buffer_dont_use_outside_of_renderer(
    const BufferT&);

}

namespace for_importer_only
{

template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_normals_buffer_dont_use_outside_of_importer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_normals_buffer_dont_use_outside_of_importer(
    const BufferT&);
template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_tangents_buffer_dont_use_outside_of_importer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_tangents_buffer_dont_use_outside_of_importer(
    const BufferT&);
template <typename BufferT>
inline static typename BufferT::internal_type* get_ptr_from_texcoord_buffer_dont_use_outside_of_importer(BufferT&);
template <typename BufferT>
inline static const typename BufferT::internal_type* get_ptr_from_texcoord_buffer_dont_use_outside_of_importer(
    const BufferT&);

}
}

} // namespace graphics

} // namespace sixit

namespace gltf
{
void double_conversion_eq(const char*, bool);
}

#endif //sixit_geometry_low_level_forward_declaration_h_included

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