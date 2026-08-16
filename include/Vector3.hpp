#pragma once

#include <cmath>

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] bool isFinite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    Vector3& operator+=(const Vector3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

[[nodiscard]] inline Vector3 operator*(const Vector3& vector, double scalar) noexcept {
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

[[nodiscard]] inline Vector3 operator/(const Vector3& vector, double scalar) noexcept {
    return {vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

[[nodiscard]] inline Vector3 operator-(const Vector3& left, const Vector3& right) noexcept {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

[[nodiscard]] inline Vector3 cross(const Vector3& left, const Vector3& right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}
