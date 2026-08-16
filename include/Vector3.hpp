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
