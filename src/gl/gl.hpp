#pragma once

// Single place GL headers are pulled in from, so the include order (GLEW
// strictly before anything that might drag in gl.h) is settled once.

#include <GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
