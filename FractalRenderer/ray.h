#pragma once
#include "vec.h"

template<class T>
class ray {
public:
    inline ray(vec3<T> pos, vec3<T> dir, T dist) : origin(pos) , direction(dir), distance(dist){}
    
    void move_ray() {
        origin = origin + direction * distance;
    }

public:
    vec3<T> origin;
    vec3<T> direction;
    T distance;
};
