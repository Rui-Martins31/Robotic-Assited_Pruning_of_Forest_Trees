cmake_minimum_required(VERSION 3.8)
project(custom_worlds)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Collada resolves textures relative to the .dae file, so textures must also
# exist alongside the mesh files in the installed dae directory.
install(FILES
  meshes/textures/Ground037_1K-JPG_Color.jpg
  meshes/textures/maple_bark.png
  meshes/textures/maple_leaf.png
  DESTINATION share/${PROJECT_NAME}/meshes/dae
)

ament_package()
