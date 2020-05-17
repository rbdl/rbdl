//==============================================================================
/* 
 * RBDL - Rigid Body Dynamics Library: Addon : luamodel structs
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef LUASTRUCTS_H
#define LUASTRUCTS_H

#include <iostream>
#include <sstream>
#include <assert.h>
#include <cstdlib>
#include <string>
#include <vector>

#include <limits>
#include <rbdl/rbdl_math.h>

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_errors.h>

/**
  A struct for a named body-fixed-point. The names used in this
  struct should be updated but remain as seen below for historical
  reasons. In the future perhaps something clearer such as
  BodyFixedPoint should be used with fields of name, body_id,
  body_name, and r.

  @param name the name of the point
  @param body_id the integer id of the body that this point is fixed to
  @param body_name the name of the body that this point is fixed to
  @param point_local the coordinates of this point relative to the body's
          origin in the coordinates of the body.
*/
struct Point {
  Point() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    point_local (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
      )
  { }

  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d point_local;
};

/**
  A struct for a named motion capture marker.

  @param name the name of the marker
  @param body_id the integer id of the body that this point is fixed to
  @param body_name the name of the body that this point is fixed to
  @param point_local the coordinates of this point relative to the body's
          origin in the coordinates of the body.
*/
struct MotionCaptureMarker {
  MotionCaptureMarker() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    point_local (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
      )
  { }

  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d point_local;
};

/**
  A struct for a named body fixed frame.

  @param name the name of the point
  @param body_id the integer id of the body that this local frame is fixed to
  @param body_name the name of the body that this local frame is fixed to
  @param r the translation from the body's origin to the origin of the
         local frame, in the coordinates of the body.
  @param E the rotation matrix that transforms vectors from the
         coordinates of the local frame to the coordinates of the body.
*/
struct LocalFrame {
  LocalFrame() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    r ( std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN()),
    E ( std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN())
  { }
  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d r;
  RigidBodyDynamics::Math::Matrix3d E;
};
/* LUASTRUCTS_H */
#endif
