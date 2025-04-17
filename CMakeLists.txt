cmake_minimum_required(VERSION 3.8)
project(can_listener_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define o padrão C++ 17 para a biblioteca ICSNEO
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#Precisei colocar para tirar warnings desnecessários
add_compile_options(-Wno-pedantic)
add_compile_options(-Wno-unused-parameter)


# Encontre as dependências do pacote
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Incluir os diretórios de cabeçalhos
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/libicsneo/include
)

# Link para a biblioteca ICSNEO
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/libicsneo/build
)

# Declarar o executável para o nó can_listener_node
add_executable(can_listener_node src/can_listener.cpp)
ament_target_dependencies(can_listener_node rclcpp std_msgs)
target_link_libraries(can_listener_node
  icsneoc
)
install(TARGETS can_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# Declarar o executável para o nó can_listener_callback_node
add_executable(can_listener_callback_node src/can_listener_callback.cpp)
ament_target_dependencies(can_listener_callback_node rclcpp std_msgs)
target_link_libraries(can_listener_callback_node
  icsneoc
)
install(TARGETS can_listener_callback_node
  DESTINATION lib/${PROJECT_NAME}
)

# Declarar o executável para o nó can_listener_node
add_executable(data_reader src/bag_to_file.cpp)
ament_target_dependencies(data_reader rclcpp std_msgs)
install(TARGETS data_reader
  DESTINATION lib/${PROJECT_NAME}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_package()
