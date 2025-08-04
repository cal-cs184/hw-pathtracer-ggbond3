#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>
#include <algorithm>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  // Compute the bounding box of all primitives
  BBox bbox;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  // Create a new node with the computed bounding box
  BVHNode *node = new BVHNode(bbox);

  // If we have no more than max_leaf_size primitives, this is a leaf node
  size_t num_primitives = std::distance(start, end);
  if (num_primitives <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  }

  // Otherwise, we need to split the primitives
  // Find the axis with the largest extent
  Vector3D extent = bbox.extent;
  int axis = 0;
  if (extent.y > extent.x) axis = 1;
  if (extent.z > extent[axis]) axis = 2;

  // Compute the split point (midpoint of the axis)
  double split_point = bbox.min[axis] + extent[axis] / 2.0;

  // Partition primitives based on their centroids
  auto mid = std::partition(start, end, [axis, split_point](Primitive* p) {
    return p->get_bbox().centroid()[axis] < split_point;
  });

  // Handle the case where all primitives are on one side
  if (mid == start || mid == end) {
    // If all primitives are on one side, split at the middle primitive
    mid = start + num_primitives / 2;
  }

  // Recursively construct left and right children
  node->l = construct_bvh(start, mid, max_leaf_size);
  node->r = construct_bvh(mid, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  // Check if ray intersects with the node's bounding box
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  // If this is a leaf node, test against all primitives
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;
      }
    }
    return false;
  }

  // Otherwise, recursively test children
  // We can short-circuit if we find an intersection in the left child
  if (has_intersection(ray, node->l)) {
    return true;
  }
  
  return has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  // Check if ray intersects with the node's bounding box
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  // If this is a leaf node, test against all primitives
  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  }

  // Otherwise, recursively test children
  // We need to test both children to find the closest intersection
  bool hit_left = intersect(ray, i, node->l);
  bool hit_right = intersect(ray, i, node->r);
  
  return hit_left || hit_right;
}

} // namespace SceneObjects
} // namespace CGL
