#pragma once

// The UAV camera's intrinsic model: how a direction in the camera's optical
// frame becomes a pixel, and back.
//
// Optical frame convention (the computer-vision one, matching OpenCV):
//   +x right across the image, +y down the image, +z along the view axis.
// Pixel coordinates are continuous, with (0,0) at the top-left corner of the
// top-left pixel, so the centre of pixel (i,j) is at (i+0.5, j+0.5).

#include <array>

#include <glm/glm.hpp>

namespace georay {

// Brown-Conrady lens distortion: three radial terms and two tangential ones.
// Same parameter order and meaning as OpenCV's distCoeffs, so coefficients
// from an OpenCV calibration can be dropped straight in.
struct Distortion {
    double k1 = 0.0, k2 = 0.0, k3 = 0.0;  // radial
    double p1 = 0.0, p2 = 0.0;            // tangential

    bool isIdentity() const {
        return k1 == 0.0 && k2 == 0.0 && k3 == 0.0 && p1 == 0.0 && p2 == 0.0;
    }
};

class CameraModel {
public:
    // Build from physical sensor numbers, which is how a UAV payload is
    // actually specified on a datasheet: focal length in mm, sensor width in
    // mm, image size in pixels. Square pixels and a centred principal point
    // are assumed; setIntrinsics() below overrides any of that.
    static CameraModel fromSensor(double focal_length_mm, double sensor_width_mm,
                                  int image_width, int image_height);

    // A plain 24 mm full-frame camera at 720p, so a default-constructed
    // model is usable rather than degenerate.
    CameraModel() { *this = fromSensor(24.0, 36.0, 1280, 720); }

    // --- intrinsics -------------------------------------------------------
    void setIntrinsics(double fx, double fy, double cx, double cy);
    void setImageSize(int width, int height);

    // Rescale the focal length in terms of the horizontal field of view,
    // which is the knob a user actually wants to turn ("zoom out to 90
    // degrees") rather than a pixel focal length.
    void setHorizontalFovDeg(double fov_deg);
    double horizontalFovDeg() const;
    double verticalFovDeg() const;

    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }
    int imageWidth() const { return image_width_; }
    int imageHeight() const { return image_height_; }
    double aspectRatio() const { return double(image_width_) / double(image_height_); }

    // Focal length in mm, assuming the sensor width this model was built
    // with. Reported in the HUD; not used by any of the maths.
    double focalLengthMm() const;
    double sensorWidthMm() const { return sensor_width_mm_; }

    Distortion& distortion() { return distortion_; }
    const Distortion& distortion() const { return distortion_; }

    // --- projection -------------------------------------------------------

    // Unit direction, in the optical frame, of the ray through a pixel.
    // Lens distortion is removed first, so this is the true physical ray.
    glm::dvec3 pixelToRay(const glm::dvec2& pixel) const;

    // Pixel a point in the optical frame lands on, with distortion applied.
    // `in_front` reports whether the point was actually in front of the lens;
    // a point behind it still yields a (meaningless) pixel, because the
    // projection divides by z regardless of sign.
    glm::dvec2 projectToPixel(const glm::dvec3& point_optical, bool* in_front = nullptr) const;

    bool pixelInImage(const glm::dvec2& pixel) const;

    // The four image corners, in the order TL, TR, BR, BL. Used to draw the
    // camera frustum and to trace the ground footprint outline.
    std::array<glm::dvec2, 4> imageCorners() const;

    // OpenGL projection matrix equivalent to this pinhole model, for an eye
    // space where the camera looks down -z (the GL convention). Lets the
    // simulated camera view be rendered with exactly the intrinsics the
    // geolocation maths uses, which is what makes clicking in that view a
    // meaningful test rather than an approximation.
    glm::mat4 glProjectionMatrix(double near_m, double far_m) const;

private:
    // Tag for the one constructor that leaves the in-class member
    // initialisers alone. fromSensor() needs to build a model without going
    // through the default constructor, which is itself defined in terms of
    // fromSensor() - without this, the two call each other forever.
    struct Uninitialised {};
    explicit CameraModel(Uninitialised) {}

    // Normalised-to-distorted, i.e. the forward lens model. Applied when
    // projecting; inverted numerically when un-projecting.
    glm::dvec2 distort(const glm::dvec2& normalised) const;
    glm::dvec2 undistort(const glm::dvec2& normalised) const;

    double fx_ = 1.0, fy_ = 1.0, cx_ = 0.0, cy_ = 0.0;
    int image_width_ = 1, image_height_ = 1;
    double sensor_width_mm_ = 36.0;
    Distortion distortion_;
};

}  // namespace georay
