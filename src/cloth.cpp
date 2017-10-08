#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <set>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

bool validSpring(int x, int y, int w, int h) {
  if (x >= 0 && x < w && y >= 0 && y < h) {
    return true;
  }
  return false;
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

  double unit_width = width / num_width_points;
  double unit_height = height / num_height_points;
  Vector3D pos;

  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {
      // horizontal
      if (orientation == 0) {
        pos = Vector3D(unit_width * i, 1.0, unit_height * j);
      }
      else {
        // vertical, -1/1000 and 1/1000
        // double z = (rand() - 0.5) / 500.0;
        double z = (rand() / (double)RAND_MAX) * 0.002 - 0.001;
        pos = Vector3D(unit_width * i, unit_height * j, z);
      }
      // check if pm's (x, y) index is within pinned vector.
      std::vector<int> v = {i, j};
      if (std::find(pinned.begin(), pinned.end(), v) != pinned.end()) {
        PointMass pm = PointMass(pos, true);
        point_masses.push_back(pm);
      }
      else {
        PointMass pm = PointMass(pos, false);
        point_masses.push_back(pm);
      }
    }
  }
  // spring
  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {

      int str1x = i-1, str1y = j, str2x = i, str2y = j-1;
      int she1x = i-1, she1y = j-1, she2x = i+1, she2y = j-1;
      int ben1x = i, ben1y = j-2, ben2x = i+2, ben2y = j;
      int cur_pm = j * num_height_points + i;

      if (validSpring(str1x, str1y, num_width_points, num_height_points)) {
        Spring str1 = Spring(&point_masses[cur_pm], &point_masses[str1y * num_height_points + str1x], STRUCTURAL);
        springs.push_back(str1);
      }
      if (validSpring(str2x, str2y, num_width_points, num_height_points)) {
        Spring str2 = Spring(&point_masses[cur_pm], &point_masses[str2y * num_height_points + str2x], STRUCTURAL);
        springs.push_back(str2);
      }
      if (validSpring(she1x, she1y, num_width_points, num_height_points)) {
        Spring she1 = Spring(&point_masses[cur_pm], &point_masses[she1y * num_height_points + she1x], SHEARING);
        springs.push_back(she1);
      }
      if (validSpring(she2x, she2y, num_width_points, num_height_points)) {
        Spring she2 = Spring(&point_masses[cur_pm], &point_masses[she2y * num_height_points + she2x], SHEARING);
        springs.push_back(she2);
      }
      if (validSpring(ben1x, ben1y, num_width_points, num_height_points)) {
        Spring ben1 = Spring(&point_masses[cur_pm], &point_masses[ben1y * num_height_points + ben1x], BENDING);
        springs.push_back(ben1);
      }
      if (validSpring(ben2x, ben2y, num_width_points, num_height_points)) {
        Spring ben2 = Spring(&point_masses[cur_pm], &point_masses[ben2y * num_height_points + ben2x], BENDING);
        springs.push_back(ben2);
      }
    }
  }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.


  Vector3D total = Vector3D(0,0,0);
  for (Vector3D &v : external_accelerations) {
    total += v;
  }
  Vector3D f = total * mass;
  for (PointMass &pm : point_masses) {
    pm.forces = f;
  }

  for (Spring &s : springs) {
    if (cp->enable_structural_constraints && s.spring_type == STRUCTURAL) {
      Vector3D diff = s.pm_b->position - s.pm_a->position;
      double d = diff.norm();
      diff.normalize();
      Vector3D forward_force = cp->ks * (d - s.rest_length) * diff;
      s.pm_a->forces += forward_force;
      s.pm_b->forces += -forward_force;
    }
    if (cp->enable_shearing_constraints && s.spring_type == SHEARING) {
      Vector3D diff = s.pm_b->position - s.pm_a->position;
      double d = diff.norm();
      diff.normalize();
      Vector3D forward_force = cp->ks * (d - s.rest_length) * diff;
      s.pm_a->forces += forward_force;
      s.pm_b->forces += -forward_force;
    }
    if (cp->enable_bending_constraints && s.spring_type == BENDING) {
      Vector3D diff = s.pm_b->position - s.pm_a->position;
      double d = diff.norm();
      diff.normalize();
      Vector3D forward_force = cp->ks * (d - s.rest_length) * diff;
      s.pm_a->forces += forward_force;
      s.pm_b->forces += -forward_force;
    }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass &pm : point_masses) {
    if (!pm.pinned) {
      // Vector3D old_pos = pm.position;
      Vector3D accel = pm.forces / mass;
      Vector3D new_pos = pm.position + (1.0 - cp->damping / 100.0) * (pm.position - pm.last_position) + accel * pow(delta_t, 2);
      pm.last_position = pm.position;
      pm.position = new_pos;

    }
  }


  // TODO (Part 4): Handle self-collisions.
  // This won't do anything until you complete Part 4.
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  // This won't do anything until you complete Part 3.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

  for (Spring &s : springs) {
    double dist = (s.pm_a->position - s.pm_b->position).norm() - s.rest_length * 1.1;
    if (dist > 0) {
      if (s.pm_a->pinned && !s.pm_b->pinned) {
        // Vector3D dir = s.pm_b - s.pm_a;
        Vector3D dir = s.pm_a->position - s.pm_b->position;
        dir.normalize();
        dir *= dist;
        s.pm_b->position += dir;
      }
      if (!s.pm_a->pinned && s.pm_b->pinned) {
        // Vector3D dir = s.pm_a - s.pm_b;
        Vector3D dir = s.pm_b->position - s.pm_a->position;
        dir.normalize();
        dir *= dist;
        s.pm_a->position += dir;
      }
      if (!s.pm_a->pinned && !s.pm_b->pinned) {
        double half = dist / 2.0;
        // Vector3D a_dir = s.pm_a - s.pm_b;
        Vector3D a_dir = s.pm_b->position - s.pm_a->position;
        Vector3D b_dir = -a_dir;
        a_dir.normalize();
        a_dir *= half;
        b_dir.normalize();
        b_dir *= half;
        s.pm_a->position += a_dir;
        s.pm_b->position += b_dir;
      }
    }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    // cout << "spec code" << endl;
    delete (entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  // std::set<float> s = {};
  // std::vector<PointMass *> *my_vector;
  for (PointMass &pm : point_masses) {
    float hash_value = hash_position(pm.position);
    if (!map.count(hash_value)) {
      // cout << hash_value << endl;
      map[hash_value] = new std::vector<PointMass *>();
    }
    // cout << "add to vector" << endl;
    map[hash_value]->push_back(&pm);
    // cout << "done" << endl;
  }
  // cout << "outside for loop" << endl;

  //   if (!(s.find(hash_value) != s.end())) {
  //     s.insert(hash_value);
  //     cout << "insert to set" << endl;
  //     // my_vector = new std::vector<PointMass *>();
  //     cout << "create new vector" << endl;
  //     // map[hash_value] = my_vector;
  //     map[hash_value] = new std::vector<PointMass *>();
  //     cout << "Assign vector" << endl;
  //     map[hash_value]->push_back(&pm);
  //     cout << "add item to vector" << endl;
  //   }
  //   map[hash_value]->push_back(&pm); 
  // }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  
  Vector3D total = Vector3D(0,0,0);
  double count = 0.0;
  float hash_v = hash_position(pm.position);
  if (map.count(hash_v)) {
    vector<PointMass *> *v = map[hash_v];
    for (PointMass *p : *v) {
      // cout << "start for loop" << endl;
      if (!(p->position == pm.position)) {
        // cout << "different pm" << endl;
        double dist = (p->position - pm.position).norm();
        // cout << "get dist" << endl;
        if (dist < 2.0 * thickness) {
          // cout << "do change" << endl;
          double move = 2.0 * thickness - dist;
          Vector3D dir = pm.position - p->position;
          // cout << "get direction" << endl;
          dir.normalize();
          dir *= move;
          total += dir;
          count += 1.0;
          // cout << "done if statement" << endl;
        }
      }
    }
  }
  // cout << "done for loop" << endl;
  if (count != 0.0) {
    total = total / count / simulation_steps;
    // cout << "get total" << endl;
    pm.position += total;
    // cout << "update position" << endl;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents
  // membership in some uniquely identified 3D box volume.

  // return 0.f;
  double w = 3.0 * width / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, h);
  // return roundf((float)(fmod(pos.x, w))) * roundf((float)(fmod(pos.y, h))) * roundf((float)(fmod(pos.z, t)));
  double a = 0.0, b = 0.0, c = 0.0;
  while ((a+1.0)*w < pos.x) {
    a += 1.0;
  }
  while ((b+1.0)*h < pos.y) {
    b += 1.0;
  }
  while ((c+1.0)*t < pos.z) {
    c += 1.0;
  }

  return (float)(pow((a+1.0), 1) + pow((b+1.0), 2) + pow((c+1.0), 3));

}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm, pm + num_width_points, pm + 1));
      triangles.push_back(new Triangle(pm + 1, pm + num_width_points,
                                       pm + num_width_points + 1));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
