#include "geo/camera_model.hpp"

#include <cmath>

#include "geo/geodetic.hpp"  // deg2rad / rad2deg

namespace georay {

CameraModel CameraModel::fromSensor(double focal_length_mm, double sensor_width_mm,
                                    int image_width, int image_height) {
    CameraModel m(Uninitialised{});
    m.image_width_ = image_width > 0 ? image_width : 1;
    m.image_height_ = image_height > 0 ? image_height : 1;
    m.sensor_width_mm_ = sensor_width_mm > 0.0 ? sensor_width_mm : 36.0;

    // Pixels per mm on the sensor, from the horizontal axis. Square pixels
    // means the same scale applies vertically, so fy == fx.
    const double px_per_mm = m.image_width_ / m.sensor_width_mm_;
    m.fx_ = m.fy_ = focal_length_mm * px_per_mm;
    m.cx_ = m.image_width_ * 0.5;
    m.cy_ = m.image_height_ * 0.5;
    return m;
}

void CameraModel::setIntrinsics(double fx, double fy, double cx, double cy) {
    // A zero or negative focal length would make pixelToRay divide by zero
    // and produce NaN rays that then poison every downstream intersection.
    fx_ = fx > 1e-6 ? fx : 1e-6;
    fy_ = fy > 1e-6 ? fy : 1e-6;
    cx_ = cx;
    cy_ = cy;
}

void CameraModel::setImageSize(int width, int height) {
    const double sx = double(width) / double(image_width_);
    const double sy = double(height) / double(image_height_);
    image_width_ = width > 0 ? width : 1;
    image_height_ = height > 0 ? height : 1;
    // Scale the intrinsics with the image: resizing the rendered view must
    // not silently change the camera's field of view.
    fx_ *= sx;
    fy_ *= sy;
    cx_ *= sx;
    cy_ *= sy;
}

void CameraModel::setHorizontalFovDeg(double fov_deg) {
    const double clamped = glm::clamp(fov_deg, 1.0, 170.0);
    const double f = (image_width_ * 0.5) / std::tan(deg2rad(clamped) * 0.5);
    // Keep the existing fy/fx ratio so a non-square-pixel model stays one.
    const double ratio = fy_ / fx_;
    fx_ = f;
    fy_ = f * ratio;
}

double CameraModel::horizontalFovDeg() const {
    return rad2deg(2.0 * std::atan((image_width_ * 0.5) / fx_));
}

double CameraModel::verticalFovDeg() const {
    return rad2deg(2.0 * std::atan((image_height_ * 0.5) / fy_));
}

double CameraModel::focalLengthMm() const {
    return fx_ * sensor_width_mm_ / double(image_width_);
}

glm::dvec2 CameraModel::distort(const glm::dvec2& n) const {
    if (distortion_.isIdentity()) return n;

    const Distortion& d = distortion_;
    const double r2 = n.x * n.x + n.y * n.y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;

    const double x_tan = 2.0 * d.p1 * n.x * n.y + d.p2 * (r2 + 2.0 * n.x * n.x);
    const double y_tan = d.p1 * (r2 + 2.0 * n.y * n.y) + 2.0 * d.p2 * n.x * n.y;

    return {n.x * radial + x_tan, n.y * radial + y_tan};
}

glm::dvec2 CameraModel::undistort(const glm::dvec2& distorted) const {
    if (distortion_.isIdentity()) return distorted;

    // Brown-Conrady has no closed-form inverse: fixed-point iteration,
    // guessing the undistorted point and correcting by the forward model's
    // residual. The cap keeps a pathological calibration from hanging.
    glm::dvec2 guess = distorted;
    for (int i = 0; i < 20; ++i) {
        const glm::dvec2 residual = distort(guess) - distorted;
        guess -= residual;
        if (glm::dot(residual, residual) < 1e-24) break;
    }
    return guess;
}

glm::dvec3 CameraModel::pixelToRay(const glm::dvec2& pixel) const {
    const glm::dvec2 normalised{(pixel.x - cx_) / fx_, (pixel.y - cy_) / fy_};
    const glm::dvec2 ideal = undistort(normalised);
    return glm::normalize(glm::dvec3(ideal.x, ideal.y, 1.0));
}

glm::dvec2 CameraModel::projectToPixel(const glm::dvec3& p, bool* in_front) const {
    if (in_front) *in_front = p.z > 0.0;
    // Guard the divide rather than the sign: a point exactly on the focal
    // plane has no pixel at all, and letting it through yields infinities.
    const double z = std::fabs(p.z) > 1e-12 ? p.z : (p.z >= 0.0 ? 1e-12 : -1e-12);
    const glm::dvec2 ideal{p.x / z, p.y / z};
    const glm::dvec2 d = distort(ideal);
    return {d.x * fx_ + cx_, d.y * fy_ + cy_};
}

bool CameraModel::pixelInImage(const glm::dvec2& pixel) const {
    return pixel.x >= 0.0 && pixel.x <= double(image_width_) &&
           pixel.y >= 0.0 && pixel.y <= double(image_height_);
}

std::array<glm::dvec2, 4> CameraModel::imageCorners() const {
    const double w = double(image_width_), h = double(image_height_);
    return {glm::dvec2{0, 0}, glm::dvec2{w, 0}, glm::dvec2{w, h}, glm::dvec2{0, h}};
}

glm::mat4 CameraModel::glProjectionMatrix(double near_m, double far_m) const {
    // Derived rather than borrowed from glm::perspective: an off-centre
    // principal point is exactly what glm::perspective cannot express, and a
    // centred approximation here would disagree with the geolocation maths
    // about where a pixel points - the one thing this must get right.
    //
    // Eye space is GL's: +x right, +y up, -z forward, so x_eye = x_optical,
    // y_eye = -y_optical, z_eye = -z_optical (the view matrix applies that
    // flip). Clip w is then -z_eye = z_optical.
    const double w = double(image_width_), h = double(image_height_);

    glm::mat4 p(0.0f);
    p[0][0] = float(2.0 * fx_ / w);
    p[1][1] = float(2.0 * fy_ / h);
    p[2][0] = float(1.0 - 2.0 * cx_ / w);
    p[2][1] = float(2.0 * cy_ / h - 1.0);
    p[2][2] = float(-(far_m + near_m) / (far_m - near_m));
    p[2][3] = -1.0f;
    p[3][2] = float(-2.0 * far_m * near_m / (far_m - near_m));
    return p;
}

}  // namespace georay
