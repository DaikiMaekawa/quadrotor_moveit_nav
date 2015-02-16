#include <math.h>

namespace dmath{

static const float tol = 0.000000000000001f;

struct Vector3D{
    double x, y, z;

    Vector3D() : x(0), y(0), z(0)
    {

    }

    Vector3D(double x_in, double y_in, double z_in) : 
        x(x_in), y(y_in), z(z_in)
    {

    }

    Vector3D operator-(){
        return Vector3D(-x, -y, -z);
    }

    Vector3D operator+=(Vector3D u){
        x += u.x;
        y += u.y;
        z += u.z;

        return *this;
    }

    Vector3D operator-=(Vector3D u){
        x -= u.x;
        y -= u.y;
        z -= u.z;
        
        return *this;
    }

    Vector3D operator*=(double s){
        x *= s;
        y *= s;
        z *= s;

        return *this;
    }

    Vector3D operator/=(double s){
        x /= s;
        y /= s;
        z /= s;

        return *this;
    }

};

double magnitude(Vector3D vec){
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

Vector3D normalize(const Vector3D &vec_in){
    Vector3D vec_out;
    
    float m = magnitude(vec_in);
    if(tol >= m) m = 1;
    vec_out.x = vec_in.x / m;
    vec_out.y = vec_in.y / m;
    vec_out.z = vec_in.z / m;

    if(fabs(vec_out.x) < tol) vec_out.x = 0.0f;
    if(fabs(vec_out.y) < tol) vec_out.y = 0.0f;
    if(fabs(vec_out.z) < tol) vec_out.z = 0.0f;
    
    return vec_out;
}

}; //namespace dmath

dmath::Vector3D operator*(float s, dmath::Vector3D u){
    return dmath::Vector3D(u.x*s, u.y*s, u.z*s);
}

dmath::Vector3D operator*(dmath::Vector3D u, float s){
    return dmath::Vector3D(u.x*s, u.y*s, u.z*s);
}

float operator*(dmath::Vector3D u, dmath::Vector3D v){
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

dmath::Vector3D operator/(dmath::Vector3D u, float s){
    return dmath::Vector3D(u.x/s, u.y/s, u.z/s);
}

dmath::Vector3D operator-(dmath::Vector3D u, dmath::Vector3D v){
    return dmath::Vector3D(u.x-v.x, u.y-v.y, u.z-v.z);
}

dmath::Vector3D operator+(dmath::Vector3D u, dmath::Vector3D v){
    return dmath::Vector3D(u.x+v.x, u.y+v.y, u.z+v.z);
}

