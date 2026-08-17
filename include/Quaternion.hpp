#pragma once

#include "Vector3.hpp"

#include <cmath>
#include <stdexcept>

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] bool isFinite() const noexcept {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    [[nodiscard]] double squaredNorm() const noexcept {
        return w * w + x * x + y * y + z * z;
    }

    void normalize() {
        const double norm = std::sqrt(squaredNorm());
        if (!std::isfinite(norm) || norm <= 0.0) {
            throw std::invalid_argument("quaternion norm must be finite and positive");
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    [[nodiscard]] Quaternion conjugate() const noexcept { return {w, -x, -y, -z}; }

    [[nodiscard]] static Quaternion fromAxisAngle(Vector3 unitAxis, double angleRadians) {
        if (!unitAxis.isFinite() || !std::isfinite(angleRadians)) {
            throw std::invalid_argument("axis-angle inputs must be finite");
        }
        const double axisNorm = std::sqrt(
            unitAxis.x * unitAxis.x + unitAxis.y * unitAxis.y + unitAxis.z * unitAxis.z);
        if (axisNorm <= 0.0) {
            throw std::invalid_argument("rotation axis must be non-zero");
        }
        const double halfAngle = angleRadians * 0.5;
        const double scale = std::sin(halfAngle) / axisNorm;
        return {std::cos(halfAngle), unitAxis.x * scale, unitAxis.y * scale, unitAxis.z * scale};
    }
};

[[nodiscard]] inline Quaternion operator*(const Quaternion& left, const Quaternion& right) noexcept {
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
    };
}

[[nodiscard]] inline Vector3 rotateBodyToWorld(const Quaternion& attitude, Vector3 bodyVector) {
    Quaternion normalized = attitude;
    normalized.normalize();
    const Quaternion rotated = normalized * Quaternion{0.0, bodyVector.x, bodyVector.y, bodyVector.z} * normalized.conjugate();
    return {rotated.x, rotated.y, rotated.z};
}

[[nodiscard]] inline Vector3 rotateWorldToBody(const Quaternion& attitude, Vector3 worldVector) {
    Quaternion normalized = attitude;
    normalized.normalize();
    const Quaternion rotated = normalized.conjugate() *
        Quaternion{0.0, worldVector.x, worldVector.y, worldVector.z} * normalized;
    return {rotated.x, rotated.y, rotated.z};
}
