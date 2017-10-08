#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	Vector3D dir = pm.position - origin;
	if (dir.norm() <= radius) {
		dir.normalize();
		Vector3D correct = dir * radius + origin - pm.last_position;
		pm.position = pm.last_position  + correct * (1.0 - friction);
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
