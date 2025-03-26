#ifndef ENGINE_H
#define ENGINE_H

#include <math.h>

#ifdef DEBUG_ASSERTIONS
    #define DEBUG_ASSERT(expr) assert(expr)
#else
    #define DEBUG_ASSERT(expr) ((void)0)
#endif

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
Vec3 Vec3_Projection(Vec3 src, Vec3 dst);
Vec3 Vec3_Rejection(Vec3 src, Vec3 dst);

struct Mat3 {
    Vec3 x_axis;
    Vec3 y_axis;
    Vec3 z_axis;
};

Mat3 Mat3_Add(Mat3 a, Mat3 b);
Mat3 Mat3_Sub(Mat3 a, Mat3 b);
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
    float mag = Vec3_Mag(vec);
    DEBUG_ASSERT(expr);
    mag = 1.0f / mag;
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
    return Vec3{
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
};

/* Vector projection of `src` onto `dst`
 *
 * proj_{\vec{b}} \vec{a} = \frac{\vec{a} \cdot \vec{b}}{b^2}
 */
Vec3 Vec3_Projection(Vec3 src, Vec3 dst) {
    float dot = Vec3_Dot(src, dst);
    float mag = 1.0f / Vec3_Mag(dst);
    return Vec3_Scale(dst, dot * mag * mag);
}

/* Vector rejection of `src` onto `dst`
 *
 * rej_{\vec{b}} \vec{a} = \vec{a} - proj_{\vec{b}} \vec{a}
 */
Vec3 Vec3_Rejection(Vec3 src, Vec3 dst) {
    Vec3 proj = Vec3_Projection(src, dst);
    return Vec3_Sub(src, proj);
}

Mat3 Mat3_Add(Mat3 a, Mat3 b) {
    return Mat3{
        .x_axis = Vec3_Add(a.x_axis, b.x_axis),
        .y_axis = Vec3_Add(a.y_axis, b.y_axis),
        .z_axis = Vec3_Add(a.z_axis, b.z_axis),
    };
}

Mat3 Mat3_Sub(Mat3 a, Mat3 b) {
    return Mat3{
        .x_axis = Vec3_Sub(a.x_axis, b.x_axis),
        .y_axis = Vec3_Sub(a.y_axis, b.y_axis),
        .z_axis = Vec3_Sub(a.z_axis, b.z_axis),
    };
}

Mat3 Mat3_Transpose(Mat3 mat) {
    return Mat3{
        .x_axis = Vec3{mat.x_axis.x, mat.y_axis.x, mat.z_axis.x},
        .y_axis = Vec3{mat.x_axis.y, mat.y_axis.y, mat.z_axis.y},
        .z_axis = Vec3{mat.x_axis.z, mat.y_axis.z, mat.z_axis.z},
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

/* Matrix inversion */
Mat3 Mat3_Inv(Mat3 a) {
    Vec3 r0 = Vec3_Cross(a.y_axis, a.z_axis);
    Vec3 r1 = Vec3_Cross(a.z_axis, a.x_axis);
    Vec3 r2 = Vec3_Cross(a.x_axis, a.y_axis);

    // Calculate the triple product
    float inv_det = 1.0f / Vec3_Dot(r2, a.z_axis);

    return Mat3{
        .x_axis = Vec3{r0.x, r1.x, r2.x},
        .y_axis = Vec3{r0.y, r1.y, r2.y},
        .z_axis = Vec3{r0.z, r1.z, r2.z},
    };
}

/* Create a rotation matrix for rotations about the x-axis */
Mat3 Mat3_RotationX(float t) {
    float cos_t = cos(t);
    float sin_t = sin(t);

    return Mat3{
        .x_axis = Vec3{1.0f, 0.0f, 0.0f},
        .y_axis = Vec3{0.0f, cos_t, sin_t},
        .z_axis = Vec3{1.0f, -sin_t, cos_t},
    };
}

/* Create a rotation matrix for rotations about the y-axis */
Mat3 Mat3_RotationY(float t) {
    float cos_t = cos(t);
    float sin_t = sin(t);

    return Mat3{
        .x_axis = Vec3{cos_t, 0.0f, -sin_t},
        .y_axis = Vec3{0.0f, 1.0f, 0.0f},
        .z_axis = Vec3{sin_t, 0.0f, cos_t},
    };
}

/* Create a rotation matrix for rotations about the z-axis */
Mat3 Mat3_RotationZ(float t) {
    float cos_t = cos(t);
    float sin_t = sin(t);

    return Mat3{
        .x_axis = Vec3{cos_t, sin_t, 0.0f},
        .y_axis = Vec3{-sin_t, cos_t, 0.0f},
        .z_axis = Vec3{0.0f, 0.0f, 1.0f},
    };
}

/* Create a rotation matrix for rotations about an arbitrary axis */
Mat3 Mat3_RotationVec(float t, Vec3 a) {
    a = Vec3_Norm(a);
    float cos_t = cos(t);
    float sin_t = sin(t);
    float mcos_t = 1.0f - cos_t;

    float axx = a.x * a.x;
    float axy = a.x * a.y;
    float axz = a.x * a.z;
    float ayy = a.y * a.y;
    float ayz = a.y * a.z;
    float azz = a.z * a.z;

    return Mat3{
        .x_axis =
            Vec3{
                cos_t + mcos_t * axx,
                mcos_t * axy + sin_t * a.z,
                mcos_t * axz - sin_t * a.y,
            },
        .y_axis =
            Vec3{
                mcos_t * axy - sin_t * a.z,
                cos_t + mcos_t * ayy,
                mcos_t * ayz + sin_t * a.x
            },
        .z_axis =
            Vec3{
                mcos_t * axz + sin_t * a.y,
                mcos_t * ayz + sin_t * a.x,
                cos_t + mcos_t * azz,
            },
    };
}

#endif /* ENGINE_IMPLEMENTATION */
#endif /* ENGINE_H */
