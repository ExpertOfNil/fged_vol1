#ifndef ENGINE_H
#define ENGINE_H

#include <math.h>

typedef struct Vec3 Vec3;
typedef struct Mat3 Mat3;

struct Vec3 {
    float x;
    float y;
    float z;
};

Vec3 Vec3_Scale(Vec3 vec, float scale);
float Vec3_Mag(Vec3 vec);
Vec3 Vec3_Norm(Vec3 vec);
Vec3 Vec3_Add(Vec3 a, Vec3 b);
Vec3 Vec3_Sub(Vec3 a, Vec3 b);
float Vec3_Dot(Vec3 a, Vec3 b);
Vec3 Vec3_Cross(Vec3 a, Vec3 b);

struct Mat3 {
    Vec3 x_axis;
    Vec3 y_axis;
    Vec3 z_axis;
};

Vec3 Mat3_MulVec(Mat3 mat, Vec3 vec);
Mat3 Mat3_Mul(Mat3 a, Mat3 b);
Mat3 Mat3_Transpose(Mat3 mat);

#ifdef ENGINE_IMPLEMENTATION
Vec3 Vec3_Scale(Vec3 vec, float scale) {
    return Vec3{
        .x = vec.x * scale,
        .y = vec.y * scale,
        .z = vec.z * scale,
    };
}

float Vec3_Mag(Vec3 vec) {
    return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

Vec3 Vec3_Norm(Vec3 vec) {
    float mag = 1.0f / Vec3_Mag(vec);
    return Vec3{
        .x = vec.x * mag,
        .y = vec.y * mag,
        .z = vec.z * mag,
    };
}

Vec3 Vec3_Add(Vec3 a, Vec3 b) {
    return Vec3{
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
}

Vec3 Vec3_Sub(Vec3 a, Vec3 b) {
    return Vec3{
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
}

/* Vector dot product */
float Vec3_Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/* Vector cross product */
Vec3 Vec3_Cross(Vec3 a, Vec3 b) {
    return Vec3 {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
};

Mat3 Mat3_Transpose(Mat3 mat) {
    return Mat3 {
        .x_axis = Vec3 { mat.x_axis.x, mat.y_axis.x, mat.z_axis.x },
        .y_axis = Vec3 { mat.x_axis.y, mat.y_axis.y, mat.z_axis.y },
        .z_axis = Vec3 { mat.x_axis.z, mat.y_axis.z, mat.z_axis.z },
    };
}

/* Matrix-Vector multiplication */
Vec3 Mat3_MulVec(Mat3 mat, Vec3 vec) {
    return Vec3{
        .x = Vec3_Dot(Vec3{mat.x_axis.x, mat.y_axis.x, mat.z_axis.x}, vec),
        .y = Vec3_Dot(Vec3{mat.x_axis.y, mat.y_axis.y, mat.z_axis.y}, vec),
        .z = Vec3_Dot(Vec3{mat.x_axis.z, mat.y_axis.z, mat.z_axis.z}, vec),
    };
}

/* Matrix multiplication */
Mat3 Mat3_Mul(Mat3 a, Mat3 b) {
    Mat3 mat = {0};
    mat.x_axis.x = Vec3_Dot(Vec3{a.x_axis.x, a.y_axis.x, a.z_axis.x}, b.x_axis);
    mat.x_axis.y = Vec3_Dot(Vec3{a.x_axis.y, a.y_axis.y, a.z_axis.y}, b.x_axis);
    mat.x_axis.z = Vec3_Dot(Vec3{a.x_axis.z, a.y_axis.z, a.z_axis.z}, b.x_axis);

    mat.y_axis.x = Vec3_Dot(Vec3{a.x_axis.x, a.y_axis.x, a.z_axis.x}, b.y_axis);
    mat.y_axis.y = Vec3_Dot(Vec3{a.x_axis.y, a.y_axis.y, a.z_axis.y}, b.y_axis);
    mat.y_axis.z = Vec3_Dot(Vec3{a.x_axis.z, a.y_axis.z, a.z_axis.z}, b.y_axis);

    mat.z_axis.x = Vec3_Dot(Vec3{a.x_axis.x, a.y_axis.x, a.z_axis.x}, b.z_axis);
    mat.z_axis.y = Vec3_Dot(Vec3{a.x_axis.y, a.y_axis.y, a.z_axis.y}, b.z_axis);
    mat.z_axis.z = Vec3_Dot(Vec3{a.x_axis.z, a.y_axis.z, a.z_axis.z}, b.z_axis);

    return mat;
}

#endif /* ENGINE_IMPLEMENTATION */
#endif /* ENGINE_H */
