#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  // Ray-sphere intersection using quadratic formula
  // Ray: r.o + t * r.d
  // Sphere: |p - o|^2 = r^2
  // Substitute ray equation into sphere equation
  // |r.o + t * r.d - o|^2 = r^2
  // |(r.o - o) + t * r.d|^2 = r^2
  // |A + t * B|^2 = r^2 where A = r.o - o, B = r.d
  // (A + t * B) · (A + t * B) = r^2
  // A·A + 2t(A·B) + t^2(B·B) = r^2
  // t^2(B·B) + 2t(A·B) + (A·A - r^2) = 0
  
  Vector3D A = r.o - o;
  Vector3D B = r.d;
  
  double a = dot(B, B);
  double b = 2 * dot(A, B);
  double c = dot(A, A) - this->r * this->r;
  
  double discriminant = b * b - 4 * a * c;
  
  if (discriminant < 0) {
    return false;
  }
  
  double sqrt_disc = sqrt(discriminant);
  t1 = (-b - sqrt_disc) / (2 * a);
  t2 = (-b + sqrt_disc) / (2 * a);
  
  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  if (!test(r, t1, t2)) {
    return false;
  }
  
  // Find the valid intersection
  double t_intersect = INF_D;
  
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t_intersect = t1;
  } else if (t2 >= r.min_t && t2 <= r.max_t) {
    t_intersect = t2;
  }
  
  return t_intersect != INF_D;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  if (!test(r, t1, t2)) {
    return false;
  }
  
  // Find the valid intersection
  double t_intersect = INF_D;
  
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t_intersect = t1;
  } else if (t2 >= r.min_t && t2 <= r.max_t) {
    t_intersect = t2;
  }
  
  if (t_intersect == INF_D) {
    return false;
  }
  const_cast<Ray&>(r).max_t = t_intersect;
  
  // Fill intersection data
  i->t = t_intersect;
  i->primitive = this;
  i->bsdf = get_bsdf();
  
  // Compute normal at intersection point
  Vector3D hit_point = r.o + t_intersect * r.d;
  i->n = (hit_point - o).unit();
  
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
