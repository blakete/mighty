/* ----------------------------------------------------------------------------
 * Polyhedron -> decomp_ros_msgs conversion, vendored from DecompROS2
 * (github.com/kotakondo/DecompROS2 @ a2bc42e,
 *  decomp_rviz_plugins/include/decomp_rviz_plugins/data_ros_utils.hpp).
 *
 * Only the two functions MIGHTY uses are kept. Including the original header
 * required depending on the decomp_rviz_plugins package, whose exported CMake
 * target links rviz, OGRE, Qt5 and VTK, which pulled a desktop GUI stack into
 * the hardware image for these ~40 lines.
 *
 * Original license (BSD 3-Clause), reproduced as its terms require:
 *
 * Copyright (c) 2018, sikang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <decomp_geometry/polyhedron.h>

#include <decomp_ros_msgs/msg/polyhedron.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace DecompROS {

inline decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly) {
  decomp_ros_msgs::msg::Polyhedron msg;
  for (const auto& p : poly.hyperplanes()) {
    geometry_msgs::msg::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = 0;
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = 0;
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  // Close the 2D polygon into a thin slab so it renders as a 3D polyhedron.
  geometry_msgs::msg::Point pt1, n1;
  pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
  n1.x = 0, n1.y = 0, n1.z = 1;
  msg.points.push_back(pt1);
  msg.normals.push_back(n1);

  geometry_msgs::msg::Point pt2, n2;
  pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
  n2.x = 0, n2.y = 0, n2.z = -1;
  msg.points.push_back(pt2);
  msg.normals.push_back(n2);

  return msg;
}

inline decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly) {
  decomp_ros_msgs::msg::Polyhedron msg;
  for (const auto& p : poly.hyperplanes()) {
    geometry_msgs::msg::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}

template <int Dim>
decomp_ros_msgs::msg::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs) {
  decomp_ros_msgs::msg::PolyhedronArray msg;
  for (const auto& v : vs) msg.polyhedrons.push_back(polyhedron_to_ros(v));
  return msg;
}

}  // namespace DecompROS
