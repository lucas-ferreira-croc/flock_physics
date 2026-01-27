cmake_minimum_required(VERSION 3.0)
project(physics_cookbook)

file(GLOB_RECURSE  sources 
    src/math/vectors.cpp
    src/math/vectors.hpp
    src/math/matrices.cpp
    src/math/matrices.hpp
    src/math/geometry/2d_geometry.hpp
    src/math/geometry/2d_geometry.cpp
    src/math/geometry/3d_geometry.hpp
    src/math/geometry/3d_geometry.cpp
    src/structure/quad_tree.hpp
    src/structure/quad_tree.cpp
    src/physics/rigidbody.hpp
    src/physics/rigidbody_volume.hpp
    src/physics/rigidbody_volume.cpp
    src/physics/physics_system.hpp
    src/physics/particle.hpp
    src/physics/particle.cpp
    src/physics/spring.hpp
    src/physics/spring.cpp
    src/physics/cloth.hpp
    src/physics/cloth.cpp
    src/physics/distance_joint.hpp
    src/physics/distance_joint.cpp
    src/scene.hpp
    src/scene.cpp
    src/camera.hpp
    src/camera.cpp
    src/main.cpp
    )

add_executable(cookbook ${sources})
target_compile_options(cookbook PUBLIC /std:c++17 /W4)
target_include_directories(cookbook PUBLIC src/)