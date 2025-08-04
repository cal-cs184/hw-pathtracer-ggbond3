#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "pathtracer/bsdf.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

    // Monte Carlo integration over hemisphere
  if (scene->lights.empty()) return Vector3D();

  for (int k = 0; k < num_samples; ++k) {
    // Sample direction in hemisphere (local)
    Vector3D wi_local = hemisphereSampler->get_sample(); // uniform hemisphere
    double pdf = 1.0 / (2.0 * PI);

    // Convert to world space
    Vector3D wi_world = o2w * wi_local;
    Ray shadowRay(hit_p + EPS_F * wi_world, wi_world);
    shadowRay.max_t = INF_D;
    Intersection light_isect;
    if (!bvh->intersect(shadowRay, &light_isect)) continue; // didn't hit anything
    Vector3D emission = light_isect.bsdf->get_emission();
    if (emission == Vector3D()) continue; // not a light

    Vector3D f_val = isect.bsdf->f(w_out, wi_local);
    double cos_term = abs_cos_theta(wi_local);
    L_out += emission * f_val * cos_term / pdf;
  }
  if (num_samples > 0) L_out /= num_samples;
  return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0);

  // Loop over lights
  for (SceneObjects::SceneLight* light : scene->lights) {
    int ns = light->is_delta_light() ? 1 : ns_area_light;
    Vector3D L_sum(0);
    for (int s = 0; s < ns; ++s) {
      Vector3D wi_world;
      double distToLight, pdf_light;
      Vector3D Li = light->sample_L(hit_p, &wi_world, &distToLight, &pdf_light);
      if (Li == Vector3D() || pdf_light == 0) continue;

      // Shadow ray
      Ray shadowRay(hit_p + EPS_F * wi_world, wi_world, EPS_F, distToLight - EPS_F);
      Intersection sh_isect;
      if (bvh->intersect(shadowRay, &sh_isect)) {
        // If we hit something before the light and it is NOT the light itself, skip
        if (sh_isect.bsdf->get_emission() == Vector3D()) continue;
      }

      // Convert to local coordinates
      Vector3D wi_local = w2o * wi_world;
      if (wi_local.z <= 0) continue; // below surface

      Vector3D f_val = isect.bsdf->f(w_out, wi_local);
      double cos_term = abs_cos_theta(wi_local);
      L_sum += Li * f_val * cos_term / pdf_light;
    }
    if (!light->is_delta_light() && ns > 0) L_sum /= ns;
    L_out += L_sum;
  }
  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                           const Intersection &isect) {
  // Return emitted radiance if the hit object is emissive
  return isect.bsdf ? isect.bsdf->get_emission() : Vector3D();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.


  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  

  // zero-bounce emission
  //L_out = zero_bounce_radiance(r, isect);
  // direct lighting (importance sampling)
  L_out += estimate_direct_lighting_importance(r, isect);
  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  int num_samples = ns_aa;
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  double inv_w = 1.0 / sampleBuffer.w;
  double inv_h = 1.0 / sampleBuffer.h;

  Vector3D total_radiance(0, 0, 0);
  for (int i = 0; i < num_samples; ++i) {
    Vector2D sample = gridSampler->get_sample(); // in [0,1]^2
    double normalized_x = (x + sample.x) * inv_w;
    double normalized_y = (y + sample.y) * inv_h;

    Ray ray = camera->generate_ray(normalized_x, normalized_y);
    Vector3D radiance = est_radiance_global_illumination(ray);
    total_radiance += radiance;
  }

  Vector3D average_radiance = total_radiance / static_cast<double>(num_samples);

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  sampleBuffer.update_pixel(average_radiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
