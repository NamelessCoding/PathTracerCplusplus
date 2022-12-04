#pragma once
#include <cmath>
#include <iostream>

using std::sqrt;

//VEC3 class

template<class T>
class vec3 {
public:
    inline vec3() : e{ 0,0,0 } {}
    inline vec3(T e0, T e1, T e2) : e{ e0, e1, e2 } {}

    T x() const { return e[0]; }
    T y() const { return e[1]; }
    T z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }

    inline vec3 operator+(const vec3& other) const {
        vec3 r; for(int i = 0; i < 3; i++)r.e[i] = e[i]+other.e[i]; return r;
    }
    inline vec3 operator-(const vec3& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] - other.e[i]; return r;
    }
    inline vec3 operator*(const vec3& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] * other.e[i]; return r;
    }
    inline vec3 operator/(const vec3& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] / other.e[i]; return r;
    }

    inline vec3 operator+(const T & other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] + other; return r;
    }
    inline vec3 operator-(const T& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] - other; return r;
    }
    inline vec3 operator*(const T& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] * other; return r;
    }
    inline vec3 operator/(const T& other) const {
        vec3 r; for (int i = 0; i < 3; i++)r.e[i] = e[i] / other; return r;
    }

    inline const vec3& operator =(const vec3& other) { 
        for (int i = 0; i < 3; i++) e[i] = other.e[i]; return *this; 
    }
    inline const vec3& operator+=(const vec3& other) {
        for (int i = 0; i < 3; i++) e[i] += other.e[i]; return *this;
    }
    inline const vec3& operator-=(const vec3& other) {
        for (int i = 0; i < 3; i++) e[i] -= other.e[i]; return *this;
    }
    inline const vec3& operator*=(const vec3& other) {
        for (int i = 0; i < 3; i++) e[i] *= other.e[i]; return *this;
    }
    inline const vec3& operator/=(const vec3& other) {
        for (int i = 0; i < 3; i++) e[i] /= other.e[i]; return *this;
    }

    inline const vec3& operator =(const T & other) {
        for (int i = 0; i < 3; i++) e[i] = other; return *this;
    }
    inline const vec3& operator+=(const T & other) {
        for (int i = 0; i < 3; i++) e[i] += other; return *this;
    }
    inline const vec3& operator-=(const T & other) {
        for (int i = 0; i < 3; i++) e[i] -= other; return *this;
    }
    inline const vec3& operator*=(const T & other) {
        for (int i = 0; i < 3; i++) e[i] *= other; return *this;
    }
    inline const vec3& operator/=(const T & other) {
        for (int i = 0; i < 3; i++) e[i] /= other; return *this;
    }

    T length() const;
    T dot(const vec3<T>& a, const vec3<T>& b) const;
    void normalize();
    void absolute();
    vec3<T> absol() const;
    vec3<T> floor(vec3<T>& a);
    vec3<T> reflect(vec3<T>& a, vec3<T>& b);
    vec3<T> cross(const vec3<T>& a, const vec3<T>& b) const;
    T dot_2(const vec3<T>& a) const;
    vec3<T> maximum(const vec3<T>& a, const T b) const;
    vec3<T> minimum(const vec3<T>& a, const vec3<T>& b) const;
    vec3<T> maxim(const vec3<T>& a, const vec3<T>& b) const;
    vec3<T> refract(vec3<T>& normal, vec3<T>& indident, T n1, T n2);

public:
    T e[3];
};

template<class T>
inline T vec3<T>::length() const
{
    T len;
    len = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    return len;
}

template <class T>
inline T vec3<T>::dot(const vec3<T>& a, const vec3<T>& b) const {
    T d;
    d = a.e[0] * b.e[0] + a.e[1] * b.e[1] + a.e[2] * b.e[2];
    return d;
}

template <class T>
inline void vec3<T>::normalize() {
    float len = this->length();
    *this /= len;
}

template <class T>
inline void vec3<T>::absolute() {
    this->e[0] = (e[0] < 0.) ? this->e[0] * -1. : this->e[0];
    this->e[1] = (e[1] < 0.) ? this->e[1] * -1. : this->e[1];
    this->e[2] = (e[2] < 0.) ? this->e[2] * -1. : this->e[2];

}

template <class T>
inline vec3<T> vec3<T>::absol() const{
    return vec3<T>(
        (e[0] < 0.) ? this->e[0] * -1. : this->e[0],
        (e[1] < 0.) ? this->e[1] * -1. : this->e[1],
        (e[2] < 0.) ? this->e[2] * -1. : this->e[2]
        );
}

template <class T>
inline vec3<T> vec3<T>::floor(vec3<T>& a) {
    a.e[0] = floorf(a.e[0]);
    a.e[1] = floorf(a.e[1]);
    a.e[2] = floorf(a.e[2]);
    return a;
}

template <class T>
inline vec3<T> vec3<T>::reflect(vec3<T>& a, vec3<T>& b) {
    return a - b * a.dot(a, b) * (T)2.0;
}

template <class T>
inline vec3<T> vec3<T>::cross(const vec3<T>& a,const vec3<T>& b) const {
    return vec3<T>(
        a.e[1] * b.e[2] - b.e[1] * a.e[2],
        a.e[2] * b.e[0] - b.e[2] * a.e[0],
        a.e[0] * b.e[1] - b.e[0] * a.e[1]
        );
}

template <class T>
inline T vec3<T>::dot_2(const vec3<T>& a) const {
    T d;
    d = a.e[0] * a.e[0] + a.e[1] * a.e[1] + a.e[2] * a.e[2];
    return d;
}

template <class T>
inline vec3<T> vec3<T>::maximum(const vec3<T>& a, const T b) const {
    return vec3<T>(
        (a.e[0] > b) ? a.e[0] : b,
        (a.e[1] > b) ? a.e[1] : b,
        (a.e[2] > b) ? a.e[2] : b
        );
}

template <class T>
inline vec3<T> vec3<T>::minimum(const vec3<T>& a, const vec3<T>& b) const {
    return vec3<T>(
        (a.e[0] < b.e[0]) ? a.e[0] : b.e[0],
        (a.e[1] < b.e[1]) ? a.e[1] : b.e[1],
        (a.e[2] < b.e[2]) ? a.e[2] : b.e[2]
        );
}

template <class T>
inline vec3<T> vec3<T>::maxim(const vec3<T>& a, const vec3<T>& b) const {
    return vec3<T>(
        (a.e[0] > b.e[0]) ? a.e[0] : b.e[0],
        (a.e[1] > b.e[1]) ? a.e[1] : b.e[1],
        (a.e[2] > b.e[2]) ? a.e[2] : b.e[2]
        );
}

//vec3<T> refract(vec3<T>& normal, vec3<T>& indident, T n1, T n2);
template <class T>
inline vec3<T> vec3<T>::refract(vec3<T>& normal, vec3<T>& indident, T n1, T n2) {
    //return a - b * a.dot(a, b) * (T)2.0;
    T n = n1 / n2;
    T cosI = -normal.dot(normal, indident);
    T sinT2 = n * n * (1.0 - cosI * cosI);
    T cosT = std::sqrt(1.0 - sinT2);
    return indident * n + normal * (n * cosI - cosT);
   /* T k = eta * eta * (1.0 - normal.dot(normal, indident)
        * dot(normal, indident));
        return indident * eta - normal *
        (eta * normal.dot(normal, indident) + std::sqrt(k));*/
}
