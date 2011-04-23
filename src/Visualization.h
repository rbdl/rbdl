#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H

namespace RigidBodyDynamics {

namespace Visualization {

/** \brief Primitive types that can be used for the body visualization
 *
 * All values contained in this enum are valid primitive types when
 * specifying the visualization for a body.
 *
 * See also Model.SetBodyVisualizationBox().
 */
enum PrimitiveType {
	PrimitiveTypeNone = 0,
	PrimitiveTypeBox,
	PrimitiveTypeSphere,
	PrimitiveTypeLast
};

/** \brief Base structure for primitives for the visualization
 *
 * This structure is used to specify the geomentric shape that is used in
 * the visualization for a body.
 *
 * See also Model.SetBodyVisualizationBox().
 */
struct Primitive {
	/// \brief Type of the visualization (e.g. PrimitiveBox)
	PrimitiveType type;
	/// \brief Color of the primitive
	Vector3d color;

	/// \brief Minimum coordinates when type is a box
	Vector3d min;
	/// \brief Maximum coordinates when type is a box
	Vector3d max;

	/// \brief Center of the sphere when type is a sphere
	Vector3d center;
	/// \brief Radius of the sphere when type is a sphere
	double radius;
};

/** \brief Creates a visualization primitive of type box
 *
 * \param color   color of the box
 * \param min_coords minimum coordinates of the box in body coordinates
 * \param max_coords maximum coordinates of the box in body coordinates
 */
inline Primitive PrimitiveBox (
		const Vector3d &color,
		const Vector3d &min_coords,
		const Vector3d &max_coords) {
	Primitive result;

	result.type = PrimitiveTypeBox;
	result.color = color;
	result.min = min_coords;
	result.max = max_coords;

	return result;
};

/** \brief Creates a visualization primitive of type sphere
 *
 * \param color   color of the box
 * \param center  center of the sphere
 * \param radius  radius of the sphere
 */
inline Primitive PrimitiveSphere (
		const Vector3d &color,
		const Vector3d &center,
		const float radius) {
	Primitive result;

	result.type = PrimitiveTypeSphere;
	result.color = color;
	result.center = center;
	result.radius = radius;

	return result;
};

}

}

/* _VISUALIZATION_H */
#endif
