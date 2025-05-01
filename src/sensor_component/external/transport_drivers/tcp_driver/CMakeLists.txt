# Copyright 2023 MAP IV, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Co-developed by MAP IV, Inc. and TIER IV, Inc.

cmake_minimum_required(VERSION 3.5)
project(boost_tcp_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/tcp_socket.cpp
        src/tcp_driver.cpp
        src/http_client.cpp
        src/http_client_driver.cpp
        )
ament_target_dependencies(${PROJECT_NAME} Boost)

ament_auto_package()