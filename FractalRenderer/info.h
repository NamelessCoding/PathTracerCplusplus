#pragma once
#include "vec.h"

template<class T>
class info {
public:
    inline info() {}
    inline info(vec3<T> c, T dist, T l1, T r1, T s1, bool reff) : col(c), distance(dist), l(l1), r(r1), s(s1), ref(reff)  {}

public:
    vec3<T> col;
    T distance;
    T l;
    T r;
    T s;
    bool hit;
    vec3<T> normal;
    bool ref;
    //map.color = map.get_color(r.origin);
    //map.color = map.color / 3.14159;
    //map.l = map.get_light(r.origin);
    //map.r = map.get_rough(r.origin);
    //map.s = map.get_spec(r.origin);


};