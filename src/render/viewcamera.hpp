#pragma once

// The free-flying camera for the 3D inspection view - the window you look at
// the simulation through, as opposed to the UAV camera being simulated.
//
// Orbit rather than first-person: the subject is a point on the ground and a
// ray reaching it, and an orbit camera keeps that subject centred while you
// walk around it to check from another angle whether the ray really does pass
// through the hit point. A WASD camera makes that a chore.

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace georay {

class ViewCamera {
public:
    void setAspect(float aspect) { aspect_ = aspect > 0.01f ? aspect : 1.0f; }

    void orbit(float yaw_delta_deg, float pitch_delta_deg) {
        yaw_deg_ += yaw_delta_deg;
        // Stop just short of straight down and of the horizon: at exactly 90
        // degrees the up vector and the view direction are parallel and
        // glm::lookAt returns a matrix full of NaNs, which blanks the window
        // with no other symptom.
        pitch_deg_ = std::clamp(pitch_deg_ + pitch_delta_deg, -89.0f, -2.0f);
    }

    void zoom(float wheel_delta) {
        // Multiplicative, so a wheel click covers the same visual fraction
        // whether you are 50 m or 5 km out.
        distance_ *= std::pow(0.88f, wheel_delta);
        distance_ = std::clamp(distance_, 20.0f, 30000.0f);
    }

    // Pan across the ground plane, in the camera's own screen directions.
    void pan(const glm::vec2& screen_delta) {
        const glm::vec3 forward = glm::normalize(target_ - position());
        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 0, 1)));
        const glm::vec3 ground_forward = glm::normalize(glm::cross(glm::vec3(0, 0, 1), right));
        // Scale with distance so a drag moves the same number of pixels of
        // ground regardless of zoom.
        const float scale = distance_ * 0.0015f;
        target_ += (-screen_delta.x * right + screen_delta.y * ground_forward) * scale;
    }

    void setTarget(const glm::vec3& t) { target_ = t; }
    const glm::vec3& target() const { return target_; }
    void setDistance(float d) { distance_ = std::clamp(d, 20.0f, 30000.0f); }
    float distance() const { return distance_; }

    glm::vec3 position() const {
        const float pitch = glm::radians(pitch_deg_);
        const float yaw = glm::radians(yaw_deg_);
        const float horizontal = distance_ * std::cos(pitch);
        return target_ + glm::vec3(horizontal * std::sin(yaw), -horizontal * std::cos(yaw),
                                   -distance_ * std::sin(pitch));
    }

    glm::mat4 view() const { return glm::lookAt(position(), target_, glm::vec3(0, 0, 1)); }

    glm::mat4 projection() const {
        // The near plane scales with distance: a fixed 0.1 m near plane with
        // a 60 km far plane throws away most of the depth buffer's precision,
        // and the terrain starts z-fighting with the overlay lines drawn on
        // top of it.
        const float near_plane = std::max(0.5f, distance_ * 0.002f);
        return glm::perspective(glm::radians(55.0f), aspect_, near_plane, 60000.0f);
    }

    glm::mat4 viewProjection() const { return projection() * view(); }

private:
    glm::vec3 target_{0.0f};
    float distance_ = 1200.0f;
    float yaw_deg_ = 20.0f;
    float pitch_deg_ = -32.0f;
    float aspect_ = 16.0f / 9.0f;
};

}  // namespace georay
