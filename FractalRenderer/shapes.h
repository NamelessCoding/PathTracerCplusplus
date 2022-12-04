#pragma once
#include "vec.h"
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>

#include "info.h"

#include "OBJ_Loader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <bvh/bvh.hpp>
#include <bvh/vector.hpp>
#include <bvh/triangle.hpp>
#include <bvh/ray.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>



using std::shared_ptr;
using std::make_shared;

template<class T>
class shape {
public:
    shape() {}
    inline shape(vec3<T> pos, T s, vec3<T> col, T l1, T r1, T s1) : position(pos), scale(s), color(col), l(l1), r(r1), s(s1) {}
    virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const = 0;

public:
    vec3<T> position;
    T scale;
    vec3<T> color;
    T l;
    T r;
    T s;
};

template<class T>
class sphere : public shape<T> {
      public:
          inline sphere(vec3<T> pos, T s, vec3<T> col, T l1, T r1, T s1) {
              this->position = pos;
              this->scale = s;
              this->color = col;
              this->l = l1;
              this->r = r1;
              this->s = s1;
          }

          virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const override;

          T dist(const vec3<T>& p) const {
              return (this->position - p).length - this->scale;
          }
};

template<class T>
inline info<T> sphere<T>::intersect(const vec3<T>& p, const vec3<T>& d) const
{
    info<T> inf;
    //inf.distance = (this->position - p).length() - this->scale;

    /*vec2 sphIntersect(in vec3 ro, in vec3 rd, in vec3 ce, float ra)
    {
        vec3 oc = ro - ce;
        float b = dot(oc, rd);
        float c = dot(oc, oc) - ra * ra;
        float h = b * b - c;
        if (h < 0.0) return vec2(-1.0); // no intersection
        h = sqrt(h);
        return vec2(-b - h, -b + h);
    }*/
    inf.col = this->color;
    inf.l = this->l;
    inf.r = this->r;
    inf.s = this->s;

    /*float intersect(vec3 p, vec3 d, vec4 sph) {
        vec3 o_minus_c = p - sph.xyz;

        float p2 = dot(d, o_minus_c);
        float q = dot(o_minus_c, o_minus_c) - (sph.w * sph.w);

        float discriminant = (p2 * p2) - q;


        float dRoot = sqrt(discriminant);
        float dist1 = -p2 - dRoot;
        float dist2 = -p2 + dRoot;

        return dist2;

    }*/
    vec3<T> o_minus_c = p - this->position;
    T p2 = d.dot(d, o_minus_c);
    float q = o_minus_c.dot(o_minus_c, o_minus_c) - this->scale * this->scale;
    if (p2 > 0. && q > 0.) {
        inf.hit = false;
        return inf;
    }
    float discriminant = (p2*p2)-q;
    if (discriminant < 0.0) {
        inf.hit = false;
        return inf;
    }
    float dRoot = std::sqrt(discriminant);
    float dist1 = -p2 - dRoot;
    
    //solve for tc]
    //float t2 = tc + t1c;

    inf.hit = true;
    inf.distance = dist1;
    inf.normal = (p + d * inf.distance) - this->position;
    inf.normal.normalize();
    return inf;
}


template<class T>
class box : public shape<T> {
public:
    inline box(vec3<T> pos, vec3<T> s, vec3<T> col, T l1, T r1, T s1) {
        this->position = pos;
        this->size = s;
        this->color = col;
        this->l = l1;
        this->r = r1;
        this->s = s1;

    }

    inline T maximum(T a, T b) {
        return (a>b) ? a:b ;
    }

    virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const override;
    T dist(const vec3<T>& p) const {
        vec3<T> a = (this->position - p).absol() - this->size;
        return std::max(std::max(a.e[0], a.e[1]), a.e[2]);
    }

public:
    vec3<T> size;
};


template<class T>
inline info<T> box<T>::intersect(const vec3<T>& p, const vec3<T>& d) const
{
    info<T> inf;

    /*vec3<T> a = this->position - p;
    a.absolute();
    a -= this->size;
    T max1 = (a.e[0] > a.e[1]) ? a.e[0] : a.e[1] ;
    T max2 = (max1 > a.e[2]) ? max1 : a.e[2];
    inf.distance = max2;
    */
    inf.col = this->color;
    inf.l = this->l;
    inf.r = this->r;
    inf.s = this->s;
    inf.ref = false;
       
    T rayMinTime = 0.0;
    T rayMaxTime = std::numeric_limits<T>::max();

    // find the intersection of the intersection times of each axis to see if / where the
    // ray hits.
    for (int axis = 0; axis < 3; ++axis)
    {
        //calculate the min and max of the box on this axis
        T axisMin = this->position.e[axis] - this->size.e[axis];
        T axisMax = this->position.e[axis] + this->size.e[axis];

        //if the ray is paralel with this axis
        if (abs(d.e[axis]) < 0.0000000001)
        {
            //if the ray isn't in the box, bail out we know there's no intersection
            if (p.e[axis] < axisMin || p.e[axis] > axisMax) {
                inf.hit = false;
                return inf;
            }
        }
        else
        {
            //figure out the intersection times of the ray with the 2 values of this axis
            T axisMinTime = (axisMin - p.e[axis]) / d.e[axis];
            T axisMaxTime = (axisMax - p.e[axis]) / d.e[axis];

            //make sure min < max
            if (axisMinTime > axisMaxTime)
            {
                T temp = axisMinTime;
                axisMinTime = axisMaxTime;
                axisMaxTime = temp;
            }

            //union this time slice with our running total time slice
            if (axisMinTime > rayMinTime)
                rayMinTime = axisMinTime;

            if (axisMaxTime < rayMaxTime)
                rayMaxTime = axisMaxTime;

            //if our time slice shrinks to below zero of a time window, we don't intersect
            if (rayMinTime > rayMaxTime) {
                inf.hit = false;
                return inf;
            }
        }
    }

    bool fromInside = (rayMinTime == 0.0);
    T collisionTime;
    if (fromInside)
        collisionTime = rayMaxTime;
    else
        collisionTime = rayMinTime;

    inf.distance = collisionTime;
    inf.hit = true;
    vec3<T> pos = p + d * collisionTime;
    pos -= d * 0.03;
    inf.normal = vec3<T>(
        dist(vec3<T>(pos.e[0]+0.01, pos.e[1], pos.e[2])) -dist(vec3<T>(pos.e[0] - 0.01, pos.e[1], pos.e[2])),
        dist(vec3<T>(pos.e[0], pos.e[1]+0.01, pos.e[2])) -dist(vec3<T>(pos.e[0] , pos.e[1]-0.01, pos.e[2])),
        dist(vec3<T>(pos.e[0], pos.e[1], pos.e[2]+0.01)) -dist(vec3<T>(pos.e[0], pos.e[1], pos.e[2]-0.01))
        );
    inf.normal.normalize();
    return inf;
}



template<class T>
class De1 : public shape<T> {
public:
    inline De1(vec3<T> pos, T s, vec3<T> col, T l1, T r1, T s1) {
        this->position = pos;
        this->scale = s;
        this->color = col;
        this->l = l1;
        this->r = r1;
        this->s = s1;

    }

    inline T maximum(T a, T b) {
        return (a > b) ? a : b;
    }

    virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const override;


public:
    vec3<T> size;
};

template<class T>
inline info<T> De1<T>::intersect(const vec3<T>& p, const vec3<T>& d) const
{
    /*float DEmine(vec3 p0) {
    vec4 p = vec4(p0, 1.);
    for (int i = 0; i < 7; i++) {
        p *= 4.8;
        p.xyz = mod(p.xyz - 1.5, 3.) - 1.5;
        float m = length(p.xyz);

        p /= dot(p.xyz, p.xyz) + mod(m, 1.);
    }
    return length(p.xyz) / p.w;
}*/

    vec3<T> a = this->position - p;
    vec3<T> div2 = a / (T)12.;
    a = (div2 - a.floor(div2)) * (T)12.;
    a -= (T)6.;
   //a /= 40.;
    T space = 1.;
    for (int i = 0; i < 7; i++) {
        a *= (T)4.2;
        space *= (T)4.2;
        //	return (a / b - floorf(a/b))*b;
        vec3<T> div = a / (T)3;
        a = (div - a.floor(div)) * (T)3.;
        a -= (T)1.5;

        a /= a.dot(a, a);
        space /= a.dot(a, a);
    }
    info<T> inf;
    inf.distance = (a.length()/space);

    inf.col = this->color;
    inf.l = this->l;
    inf.r = this->r;
    inf.s = this->s;
    return inf;
}

template <typename T>
inline T sign(T t) {
    return t < 0.0 ? T(-1) : T(1);
}

template <typename T>
inline T clamp(T t, T a, T b) {
    //return t < 0.0 ? T(-1) : T(1);
    return (t < a) ? a : (t > b) ? b : t;
}

template <typename T>
inline T min(T t, T t2) {
    return (t < t2) ?t : t2;
}


template <class T>
T ud_triangle(const vec3<T>& v1, const vec3<T>& v2, const vec3<T>& v3, const vec3<T>& p) {

    const auto v21 = v2 - v1;
    const auto v32 = v3 - v2;
    const auto v13 = v1 - v3;

    const auto p1 = p - v1;
    const auto p2 = p - v2;
    const auto p3 = p - v3;

    const auto nor = p.cross(v21, v13);

    const auto inside_out =
        sign(p.dot(p.cross(v21, nor), p1)) +
        sign(p.dot(p.cross(v32, nor), p2)) +
        sign(p.dot(p.cross(v13, nor), p3)) < 2.0;

    auto t1 = p.dot(v21, p1) / p.dot_2(v21);
    t1 = (t1 < 0.0) ? 0.0 : (t1 > 1.0) ? 1.: t1;
    auto t2 = p.dot(v32, p2) / p.dot_2(v32);
    t2 = (t2 < 0.0) ? 0.0 : (t2 > 1.0) ? 1. : t2;
    auto t3 = p.dot(v13, p3) / p.dot_2(v13);
    t3 = (t3 < 0.0) ? 0.0 : (t3 > 1.0) ? 1. : t3;


    const auto dist_sq = inside_out ?
        min(min(
            p.dot_2(v21 * t1 - p1),
            p.dot_2(v32 * t2 - p2)),
            p.dot_2(v13 * t3 - p3))
        :
        p.dot(nor, p1) * p.dot(nor, p1) / p.dot_2(nor);

    return sqrt(dist_sq);

}

template<class T>
class triangle_obj : public shape<T> {
public:
    inline triangle_obj(vec3<T> pos, T s, vec3<T> col, T l1, T r1, const char* file, T s1, bool tt, std::string nn,
        std::string nnr, bool rr, bool reff) {
        this->position = pos;
        this->scale = s;
        this->color = col;
        this->l = l1;
        this->r = r1;
        this->s = s1;
        this->textured = tt;
        this->namef = nn;
        this->namefrough = nnr;
        this->userough = rr;
        this->readfile(file);
        this->ref = reff;
    }

    struct vertex {
        T x;
        T y;
        T z;
    };

    struct texcoord {
        T u;
        T v;
    };

    struct face {
        unsigned int v1, v2, v3;
    };

    struct sph {
        vec3<T> pos;
        float size;
        face trig;
    };

    struct bound {
        std::vector<sph> bounds;
        vec3<T> position;
        float size;
    };

    struct mesh {
        std::vector<vertex> vertecies;
        std::vector<vertex> normals;
        std::vector<face> faces;
        std::vector<texcoord> texcoords;
        std::vector<face> texfaces;
        vec3<T> pos;
        T m_l;
    };

    struct AABB {
        vec3<T> position;
        vec3<T> bounds_min;
        vec3<T> bounds_max;
    };

    bool sphere_check(const vec3<T>& pos,const T size, const vec3<T>& p, const vec3<T>& d) const {
        vec3<T> o_minus_c = p - pos;
        T p2 = d.dot(d, o_minus_c);
        float q = o_minus_c.dot(o_minus_c, o_minus_c) - size * size;
        if (p2 > 0. && q > 0.) {
            return false;
        }
        float discriminant = (p2 * p2) - q;
        if (discriminant <= 0.0) {
            return false;
        }
        return true;
    }

    inline T maximum(T a, T b) {
        return (a > b) ? a : b;
    }

    virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const override;
    
    inline T sign(T t) {
        return t < 0.0 ? T(-1) : T(1);
    }

    inline T minimum(T a, T b) const {
        return (a < b) ?a : b;
    }

    inline bool boxboxintersection(const vec3<T>& b1pos, const vec3<T>& b2pos, const T size1, const T size2) {
        if (b1pos.e[0]+size1 > b2pos.e[0]+size2 && b1pos.e[0]-size1 > b2pos.e[0]+size2) {
            return false;
        }
        if (b1pos.e[0]-size1 < b2pos.e[0]-size2 && b1pos.e[0]+size1 < b2pos.e[0] - size2) {
            return false;
        }

        if (b1pos.e[1] + size1 > b2pos.e[1] + size2 && b1pos.e[1] - size1 > b2pos.e[1] + size2) {
            return false;
        }
        if (b1pos.e[1] - size1 < b2pos.e[1] - size2 && b1pos.e[1] + size1 < b2pos.e[1] - size2) {
            return false;
        }

        if (b1pos.e[2] + size1 > b2pos.e[2] + size2 && b1pos.e[2] - size1 > b2pos.e[2] + size2) {
            return false;
        }
        if (b1pos.e[2] - size1 < b2pos.e[2] - size2 && b1pos.e[2] + size1 < b2pos.e[2] - size2) {
            return false;
        }
        

        return true;
    }


    bool load_image(std::vector<unsigned char>& image, const std::string& filename, int& x, int& y)
    {
        int n;
        unsigned char* data = stbi_load(filename.c_str(), &x, &y, &n, 4);
        if (data != nullptr)
        {
            image = std::vector<unsigned char>(data, data + x * y * 4);
        }
        stbi_image_free(data);
        return (data != nullptr);
    }



    void readfile(const char* filename) {
        if (textured) {
            bool success = load_image(hdr, namef, width, height);
            if (userough) {
                bool success2 = load_image(hdrrough, namefrough, widthr, heightr);
            }
        }
        

          objl::Loader Loader;
          bool loadout = Loader.LoadFile(filename);
          this->m_l = 0.;
          for (int i = 0; i < Loader.LoadedMeshes.size(); i++)
          {
              // Copy one of the loaded meshes to be our current mesh
              objl::Mesh curMesh = Loader.LoadedMeshes[i];

              //TODO optimization
              //For each shape find mid point of all triangles and from that generate a mid point from all the mid points,
              //then for each new mid, find the largest distance from all the mid-vertecies and use that to construct a sphere

              mesh m;
              for (int j = 0; j < curMesh.Vertices.size(); j++)
              {
                  vertex v;
                  v.x = -curMesh.Vertices[j].Position.X;
                  v.y = curMesh.Vertices[j].Position.Z;
                  v.z = curMesh.Vertices[j].Position.Y;
                  m.vertecies.push_back(v);
                  if (textured) {
                      texcoord t;

                      t.u = (T)curMesh.Vertices[j].TextureCoordinate.X;
                      t.v = 1.0 - (T)curMesh.Vertices[j].TextureCoordinate.Y;
                      m.texcoords.push_back(t);
                  }
                  vertex n;
                  n.x = curMesh.Vertices[j].Normal.X;
                  n.y = curMesh.Vertices[j].Normal.Z;
                  n.z = curMesh.Vertices[j].Normal.Y;
                  m.normals.push_back(n);
                  //"TC(" << curMesh.Vertices[j].TextureCoordinate.X << ", " << curMesh.Vertices[j].TextureCoordinate.Y << ")\n";
              }

              std::vector<vec3<T>> midpoints;
              for (int j = 0; j < curMesh.Indices.size(); j += 3)
              {
                  face f;
                  f.v1 = curMesh.Indices[j];
                  f.v2 = curMesh.Indices[j+1];
                  f.v3 = curMesh.Indices[j+2];

                  //printf("\n%i  %i  %i\n",f.v1, f.v2, f.v3);

                  m.faces.push_back(f);

                  vertex v1 = m.vertecies[f.v1];
                  vertex v2 = m.vertecies[f.v2];
                  vertex v3 = m.vertecies[f.v3];

                  vec3<T> vv1 = vec3<T>(v1.x, v1.y, v1.z);
                  vec3<T> vv2 = vec3<T>(v2.x, v2.y, v2.z);
                  vec3<T> vv3 = vec3<T>(v3.x, v3.y, v3.z);

                  bvh::Vector3<T> a1 = bvh::Vector3<T>(v1.x, v1.y, v1.z);
                  bvh::Vector3<T> a2 = bvh::Vector3<T>(v2.x, v2.y, v2.z);
                  bvh::Vector3<T> a3 = bvh::Vector3<T>(v3.x, v3.y, v3.z);
                  triangles.emplace_back(
                      a1,a2,a3
                  );

                  vec3<T> mid = (vv1 + vv2 + vv3) / 3.;
                  midpoints.push_back(mid);
              }

              vec3<T> finalmid = vec3<T>(0.,0.,0.);
              for (int i = 0; i < midpoints.size(); i++) {
                  vec3<T> curr_m = midpoints[i];
                  finalmid += curr_m;
              }
              finalmid /= midpoints.size();
              m.pos = finalmid;

              T max_length = 0.;
              for (int i = 0; i < m.faces.size(); i++) {
                  vertex v1 = m.vertecies[m.faces[i].v1];
                  vertex v2 = m.vertecies[m.faces[i].v2];
                  vertex v3 = m.vertecies[m.faces[i].v3];

                  vec3<T> vv1 = vec3<T>(v1.x, v1.y, v1.z);
                  vec3<T> vv2 = vec3<T>(v2.x, v2.y, v2.z);
                  vec3<T> vv3 = vec3<T>(v3.x, v3.y, v3.z);

                  T ma = std::max(std::max( (finalmid - vv1).length(),  (finalmid - vv2).length() ), (finalmid - vv3).length());
                  max_length = std::max(ma, max_length);
              }
              m.m_l = max_length;

              meshes.push_back(m);
          }
          bvh::SweepSahBuilder<bvh::Bvh<T>> builder(bvh);
          auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
          auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
          builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());

          printf("\n%i\n", meshes.size());

    }

public:
    T m_l;
    std::vector<sph> pre_bound;
    std::vector<bound> boundings;
    std::vector<mesh> meshes;
    //crdiff.png
    std::string namef;

    int width, height;
    std::vector<unsigned char> hdr;
    bool textured;
    bool userough;
    bool ref;

    std::string namefrough;

    int widthr, heightr;
    std::vector<unsigned char> hdrrough;
    
    std::vector<bvh::Triangle<T>> triangles;
    bvh::Bvh<T> bvh;

    const size_t RGBA = 4;
};


template<class T>
inline info<T> triangle_obj<T>::intersect(const vec3<T>& p, const vec3<T>& d) const
{
    info<T> inf;
    inf.col = this->color;
    inf.l = this->l;
    inf.r = this->r;
    inf.s = this->s;
    inf.hit = false;
    inf.ref = this->ref;

    T mindist = std::numeric_limits<T>::max();
    /*
    for (int k = 0; k < meshes.size(); k++) {
        mesh m = meshes[k];
        if (!sphere_check(m.pos, m.m_l, p, d)) {
           continue;
        }
        for (int i = 0; i < m.faces.size(); i++) {
            vertex v1 = m.vertecies[m.faces[i].v1];
            vertex v2 = m.vertecies[m.faces[i].v2];
            vertex v3 = m.vertecies[m.faces[i].v3];

            vec3<T> vv0 = vec3<T>(v1.x, v1.y, v1.z);
            vec3<T> vv1 = vec3<T>(v2.x, v2.y, v2.z);
            vec3<T> vv2 = vec3<T>(v3.x, v3.y, v3.z);

            vec3<T> e_1 = vv1 - vv0;
            vec3<T> e_2 = vv2 - vv0;

            vec3<T> q = d.cross(d, e_2);
            float a = e_1.dot(e_1, q);

            if (abs(a) == 0.0f) {
                continue;
            }

            //const TVector3& s = (rayPos - triangle.m_a) / a;
            vec3<T> s = (p - vv0) / a;
            vec3<T> r = s.cross(s, e_1);
            vec3<T> b;
            b.e[0] = s.dot(s,q);
            b.e[1] = r.dot(r, d);
            b.e[2] = 1.0f - b.e[0] - b.e[1];

            // Intersected outside triangle?
            if ((b.e[0] < 0.0f) || (b.e[1] < 0.0f) || (b.e[2] < 0.0f)) {
                continue;
            }
            float t = e_2.dot(e_2, r);
            if (t < 0.0f) {
                continue;
            }

            // make sure normal is facing opposite of ray direction.
            // this is for if we are hitting the object from the inside / back side.
           vec3<T> normal = e_1.cross(e_1,e_2);
           normal.normalize();
            if (normal.dot(normal, d) > 0.0f)normal *= -1.0f;
                
            
            if (mindist > t) {
                inf.hit = true;
                inf.distance = t;
                inf.normal = normal;

               
                if (textured) {
                    texcoord uv1 = m.texcoords[m.faces[i].v1];
                    texcoord uv2 = m.texcoords[m.faces[i].v2];
                    texcoord uv3 = m.texcoords[m.faces[i].v3];

                    T u = b.e[0] * uv1.u + b.e[1] * uv2.u + b.e[2] * uv3.u;
                    T v = b.e[0] * uv1.v + b.e[1] * uv2.v + b.e[2] * uv3.v;
                    //u = 1.0 - u;
                    //v = 1.0 - v;
                    int x = (int)((float)width * u);
                    int y = (int)((float)height * v);
                    size_t index = RGBA * (y * width + x);
                    vec3<T> diff = vec3<T>(
                        static_cast<float>(hdr[index + 0]) / 255.f,
                        static_cast<float>(hdr[index + 1]) / 255.f,
                        static_cast<float>(hdr[index + 2]) / 255.f
                        );
                    inf.col = diff;

                    //std::vector<unsigned char> hdrrough;
                    if (userough) {
                        int x2 = (int)((float)widthr * u);
                        int y2 = (int)((float)heightr * v);
                        size_t index2 = RGBA * (y2 * widthr + x2);
                        inf.r = static_cast<float>(hdrrough[index2 + 0]) / 255.f;
                    }
                }
                mindist = t;
            }
        }
    }
    //printf("%i\n", checked_triangles);
    return inf;
    */
    bvh::Vector3<T> origin = bvh::Vector3<T>(p.e[0], p.e[1], p.e[2]);
    bvh::Vector3<T> direction = bvh::Vector3<T>(d.e[0], d.e[1], d.e[2]);

    bvh::Ray<T> ray(
        origin, // origin
        direction, // direction
        0.0,                    // minimum distance
        100.0                   // maximum distance
    );

    bvh::ClosestPrimitiveIntersector<bvh::Bvh<T>, bvh::Triangle<T>> primitive_intersector(bvh, triangles.data());
    bvh::SingleRayTraverser<bvh::Bvh<T>> traverser(bvh);

    auto hit = traverser.traverse(ray, primitive_intersector);
    if (hit) {
        auto triangle_index = hit->primitive_index;
        auto intersection = hit->intersection;
        /*std::cout << "Hit triangle " << triangle_index << "\n"
                    << "distance: " << intersection.t << "\n"
                    << "u: " << intersection.u << "\n"
                    << "v: " << intersection.v << std::endl;*/
       // return 0;
        inf.hit = true;
        inf.distance = intersection.t;
        bvh::Triangle<T> trig = triangles[triangle_index];
        bvh::Vector3<T> v1 = trig.n;
        inf.normal = vec3<T>(
            v1.values[0], v1.values[1], v1.values[2]
            );
        inf.normal = -inf.normal;
        inf.normal.normalize();
                
    }
    else {
        inf.hit = false;
        inf.distance = 0.;
        //inf.normal = normal;
    }

    
    return inf;
}



template<class T>
class all_shapes : public shape<T> {
public:
    all_shapes() {}
    all_shapes(shared_ptr<shape<T>> object) { add(object); }

    void clear() { shapes.clear(); }
    void add(shared_ptr<shape<T>> object) { shapes.push_back(object); }


    virtual info<T> intersect(const vec3<T>& p, const vec3<T>& d) const override;
    

public:
    std::vector<shared_ptr<shape<T>>> shapes;
};

template<class T>
inline info<T> all_shapes<T>::intersect(const vec3<T>& p, const vec3<T>& d) const
{
    info<T> inf;
    inf.ref = false;
    inf.hit = false;
    //inf.distance = std::numeric_limits<T>::max();
    T mindist = std::numeric_limits<T>::max();
    for (const auto& s : shapes) {
        info<T> dist = s->intersect(p, d);
        if (dist.hit && mindist > dist.distance) {
            mindist = dist.distance;
            inf = dist;
        }
    }
    return inf;
}


/*
        vec3<T> v21 = vv2 - vv1; vec3<T> p1 = p - vv1;
        vec3<T> v32 = vv3 - vv2; vec3<T> p2 = p - vv2;
        vec3<T> v13 = vv1 - vv3; vec3<T> p3 = p - vv3;
        vec3<T> nor = v21.cross(v21, v13);

        T currdist = 0.;

        vec3<T> f1 = v21.cross(v21, nor);
        T ff1 = f1.dot(f1, p1);
        ff1 = (ff1 < (T)0.0) ? (T)-1.0 : (ff1 > (T)0.0) ? (T)1. : (T)0.0;

        vec3<T> f2 = v21.cross(v32, nor);
        T ff2 = f2.dot(f2, p2);
        ff2 = (ff2 < (T)0.0) ? (T)-1.0 : (ff2 > (T)0.0) ? (T)1. : (T)0.0;

        vec3<T> f3 = v21.cross(v13, nor);
        T ff3 = f3.dot(f3, p3);
        ff3 = (ff3 < (T)0.0) ? (T)-1.0 : (ff3 > (T)0.0) ? (T)1. : (T)0.0;

        if (ff1 + ff2 + ff3 < (T)2.) {

            T j1 = v21.dot(v21, p1) / v21.dot(v21, v21);
            j1 = (j1 < (T)0.0) ? (T)0. : (j1 > (T)1.) ? (T)1. : j1;
            vec3<T> g1 = v21 * j1 - p1;
            T gg1 = g1.dot(g1, g1);

            T j2 = v32.dot(v32, p2) / v32.dot(v32, v32);
            j2 = (j2 < (T)0.0) ? (T)0. : (j2 > (T)1.) ? (T)1. : j2;
            vec3<T> g2 = v32 * j2 - p2;
            T gg2 = g2.dot(g2, g2);

            T j3 = v13.dot(v13, p3) / v13.dot(v13, v13);
            j3 = (j3 < (T)0.0) ? (T)0. : (j3 > (T)1.) ? (T)1. : j3;
            vec3<T> g3 = v13 * j3 - p3;
            T gg3 = g3.dot(g3, g3);

            currdist = std::sqrt(std::min(std::min(gg1, gg2), gg3));
        }
        else {
            currdist = std::sqrt(nor.dot(nor, p1) * nor.dot(nor, p1) / nor.dot(nor, nor));
        }
        mindist = std::min(currdist, mindist);
        */

        