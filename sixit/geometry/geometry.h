/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_geometry_h_included
#define sixit_geometry_geometry_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/indexed.h"
#include "sixit/geometry/low_level/matrix.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/low_level/complex_number_rotation.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/curve_primitive.h"
#include "sixit/geometry/curved_polygon2.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/arc.h"
#include "sixit/geometry/curve.h"
#include "sixit/geometry/qbezier.h"
#include "sixit/geometry/cbezier.h"
#include "sixit/geometry/curve_primitive_impl.h"
#include "sixit/geometry/curved_polygon2_impl.h"
#include "sixit/geometry/ellipse.h"
#include "sixit/geometry/plane.h"
#include "sixit/geometry/plane_impl.h"
#include "sixit/geometry/polygon2.h"
#include "sixit/geometry/indexed_polygon2.h"
#include "sixit/geometry/shape2.h"
#include "sixit/geometry/grid2.h"
#include "sixit/geometry/triangle2_stack.h"
#include "sixit/geometry/point3_cloud.h"
#include "sixit/geometry/point3_cloud_impl.h"
#include "sixit/geometry/indexed_point3_cloud.h"
#include "sixit/geometry/indexed_point3_cloud_impl.h"
#include "sixit/geometry/low_level/matrix_impl.h"
#include "sixit/geometry/low_level/vector_impl.h"
#include "sixit/geometry/low_level/quaternion_impl.h"
#include "sixit/geometry/low_level/complex_number_rotation_impl.h"
#include "sixit/geometry/rotation_impl.h"
#include "sixit/geometry/line_segment_impl.h"
#include "sixit/geometry/polygon2_impl.h"
#include "sixit/geometry/shape2_impl.h"
#include "sixit/geometry/barycentric.h"
#include "sixit/geometry/barycentric_impl.h"
#include "sixit/geometry/projection3to2noscale.h"
#include "sixit/geometry/projection3to2noscale_impl.h"
#include "sixit/geometry/projection2to1noscale.h"
#include "sixit/geometry/projection2to1noscale_impl.h"
#include "sixit/geometry/projection_perspective.h"
#include "sixit/geometry/ondemand_reference_point.h"

#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/low_level/line_common_impl.h"
#include "sixit/geometry/indexed_polygon2_impl.h"
#include "sixit/geometry/low_level/formatters.h"

#endif //sixit_geometry_geometry_h_included

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