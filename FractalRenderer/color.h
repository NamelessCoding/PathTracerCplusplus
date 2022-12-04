#pragma once
#include "vec.h"

template<class T>
class color {
public:
    inline color(T r, T g, T b) {
        col.e[0] = r; col.e[1] = g; col.e[2] = b;
    }
    inline color(vec3<T> c) : col(c) {}

public:
    vec3<T> col;
};
