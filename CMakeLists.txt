cmake_minimum_required(VERSION 3.16)

# ccache, when present, makes the clean-rebuild verification cycle cheap.
# Must come before project(): that is when CMake bakes the compiler (and any
# launcher) into the generated rules.
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

project(georay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# ---------------------------------------------------------------------------
# Dependencies
#
# Prefer what the system already has (fast configure, no network) and fall
# back to fetching from source, so a fresh checkout builds either way. glm is
# header-only and almost never packaged, so it is fetched unconditionally
# unless a system copy turns up.
# ---------------------------------------------------------------------------
include(FetchContent)

find_package(OpenGL REQUIRED)
# The live tile fetcher runs on its own thread so the render loop never waits
# on a socket.
find_package(Threads REQUIRED)

find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    message(STATUS "georay: SDL2 not found on system, fetching from source")
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON  CACHE BOOL "" FORCE)
    set(SDL_TEST   OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-2.26.5)
    FetchContent_MakeAvailable(SDL2)
    set(GEORAY_SDL2_TARGET SDL2::SDL2-static)
else()
    # The Ubuntu sdl2-config.cmake exports SDL2::SDL2; older ones only set
    # SDL2_LIBRARIES. Normalise so the link line below does not have to care.
    if(TARGET SDL2::SDL2)
        set(GEORAY_SDL2_TARGET SDL2::SDL2)
    else()
        set(GEORAY_SDL2_TARGET ${SDL2_LIBRARIES})
        include_directories(${SDL2_INCLUDE_DIRS})
    endif()
endif()

find_package(GLEW QUIET)
if(NOT GLEW_FOUND)
    message(STATUS "georay: GLEW not found on system, fetching from source")
    FetchContent_Declare(glew_cmake
        GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
        GIT_TAG glew-cmake-2.2.0)
    FetchContent_MakeAvailable(glew_cmake)
    set(GEORAY_GLEW_TARGET libglew_static)
else()
    set(GEORAY_GLEW_TARGET GLEW::GLEW)
endif()

find_path(GEORAY_SYSTEM_GLM glm/glm.hpp)
if(GEORAY_SYSTEM_GLM)
    add_library(georay_glm INTERFACE)
    target_include_directories(georay_glm INTERFACE ${GEORAY_SYSTEM_GLM})
else()
    message(STATUS "georay: glm not found on system, fetching from source")
    FetchContent_Declare(glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 0.9.9.8)
    FetchContent_MakeAvailable(glm)
    add_library(georay_glm INTERFACE)
    target_include_directories(georay_glm INTERFACE ${glm_SOURCE_DIR})
endif()

# ---------------------------------------------------------------------------
# georay_geo: the geolocation maths, with no window or GL anywhere in it.
#
# Kept as its own library on purpose: it is the part a real photogrammetry
# pipeline would reuse, and it is what --selftest exercises. If it ever needs
# GL to compile, something has leaked in the wrong direction.
# ---------------------------------------------------------------------------
add_library(georay_geo STATIC
    src/geo/geodetic.cpp
    src/geo/camera_model.cpp
    src/geo/dem.cpp
    src/geo/geoid.cpp
    src/geo/json.cpp
    src/geo/mercator.cpp
    src/geo/mission.cpp
    src/geo/pose.cpp
    src/geo/terrain.cpp
    src/geo/geolocate.cpp
)
target_include_directories(georay_geo PUBLIC src)
target_link_libraries(georay_geo PUBLIC georay_glm)

add_executable(georay
    src/main.cpp
    src/map/tilesource.cpp
    src/map/tilecache.cpp
    src/map/tilefetcher.cpp
    src/map/maptexture.cpp
    src/app.cpp
    src/app_input.cpp
    src/app_render.cpp
    src/flight.cpp
    src/selftest.cpp
    src/selftest_support.cpp
    src/selftest_geodetic.cpp
    src/selftest_geolocate.cpp
    src/selftest_dem.cpp
    src/selftest_streaming.cpp
    src/selftest_formats.cpp
    src/gl/shader.cpp
    src/gl/mesh.cpp
    src/gl/text.cpp
    src/render/groundchunks.cpp
    src/render/scene.cpp
)
target_link_libraries(georay
    PRIVATE
        georay_geo
        ${GEORAY_SDL2_TARGET}
        ${GEORAY_GLEW_TARGET}
        OpenGL::GL
        Threads::Threads
)

# Shaders are read at runtime relative to this, so a build directory anywhere
# still finds them.
# stb_image, vendored. See third_party/README.md for why it is needed.
target_include_directories(georay PRIVATE third_party)

target_compile_definitions(georay PRIVATE
    GEORAY_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
    GEORAY_SCENE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenes"
    GEORAY_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")

if(WIN32)
    target_link_libraries(georay PRIVATE SDL2main)
    target_compile_definitions(georay PRIVATE SDL_STATIC GLEW_STATIC)
endif()

enable_testing()
add_test(NAME selftest COMMAND georay --selftest)
