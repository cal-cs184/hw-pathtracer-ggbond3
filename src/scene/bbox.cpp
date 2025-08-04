#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>
#include <utility>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  // Ray-box intersection using the slab method
  double tx_min, tx_max, ty_min, ty_max, tz_min, tz_max;

  // X-axis slabs
  if (r.d.x == 0) {
    // Ray is parallel to x-axis
    if (r.o.x < min.x || r.o.x > max.x) return false;
    tx_min = -INF_D;
    tx_max = INF_D;
  } else {
    double inv_dx = 1.0 / r.d.x;
    tx_min = (min.x - r.o.x) * inv_dx;
    tx_max = (max.x - r.o.x) * inv_dx;
    if (tx_min > tx_max) std::swap(tx_min, tx_max);
  }

  // Y-axis slabs
  if (r.d.y == 0) {
    // Ray is parallel to y-axis
    if (r.o.y < min.y || r.o.y > max.y) return false;
    ty_min = -INF_D;
    ty_max = INF_D;
  } else {
    double inv_dy = 1.0 / r.d.y;
    ty_min = (min.y - r.o.y) * inv_dy;
    ty_max = (max.y - r.o.y) * inv_dy;
    if (ty_min > ty_max) std::swap(ty_min, ty_max);
  }

  // Z-axis slabs
  if (r.d.z == 0) {
    // Ray is parallel to z-axis
    if (r.o.z < min.z || r.o.z > max.z) return false;
    tz_min = -INF_D;
    tz_max = INF_D;
  } else {
    double inv_dz = 1.0 / r.d.z;
    tz_min = (min.z - r.o.z) * inv_dz;
    tz_max = (max.z - r.o.z) * inv_dz;
    if (tz_min > tz_max) std::swap(tz_min, tz_max);
  }

  // Find the intersection of all slabs
  double t_enter = std::max({tx_min, ty_min, tz_min});
  double t_exit = std::min({tx_max, ty_max, tz_max});

  // Check if intersection exists
  if (t_enter > t_exit) return false;

  // Check if intersection is within ray's valid range
  if (t_exit < r.min_t || t_enter > r.max_t) return false;

  // Update t0 and t1 with the intersection interval
  t0 = std::max(t_enter, r.min_t);
  t1 = std::min(t_exit, r.max_t);

  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
