#pragma once
template<class T>
class Vec3 {
public:
    Vec3() : e{ 0,0,0 } {}
    Vec3(T e0, T e1, T e2) : e{ e0, e1, e2 } {}

    T x() const { return e[0]; }
    T y() const { return e[1]; }
    T z() const { return e[2]; }

    Vec3 operator-() const { return Vec3(-e[0], -e[1], -e[2]); }
    double operator[](int i) const { return e[i]; }
    double& operator[](int i) { return e[i]; }

    Vec3& operator+=(const Vec3& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

public:
    T e[3];

};