# cmake_minimum_required(VERSION 3.8)
# project(qpoases_colcon)

# set(CMAKE_BUILD_TYPE Release)

# find_package(ament_cmake REQUIRED)
# include(FetchContent)

# # 定义变量
# set(QPOASES_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "QPOASES install path")
# set(QPOASES_INCLUDE_DIR ${QPOASES_DEVEL_PREFIX}/include)
# set(QPOASES_LIB_DIR ${QPOASES_DEVEL_PREFIX}/lib)
# set(QPOASES_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
# set(QPOASES_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# # 创建目录
# file(MAKE_DIRECTORY ${QPOASES_INCLUDE_DIR})
# file(MAKE_DIRECTORY ${QPOASES_LIB_DIR})
# file(MAKE_DIRECTORY ${QPOASES_DOWNLOAD_DIR})
# file(MAKE_DIRECTORY ${QPOASES_BUILD_DIR})

# set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
# set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# # ----------------------------------------------------
# # 关键点：恢复网络下载
# # ----------------------------------------------------
# FetchContent_Declare(
#   qpoasesDownload
#   GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
#   GIT_TAG        3.2.0
#   UPDATE_COMMAND ""
#   SOURCE_DIR ${QPOASES_DOWNLOAD_DIR}
#   BINARY_DIR ${QPOASES_BUILD_DIR}
#   BUILD_COMMAND $(MAKE)
#   INSTALL_COMMAND "$(MAKE) install"
# )

# FetchContent_MakeAvailable(qpoasesDownload)

# # 复制头文件
# file(COPY ${QPOASES_DOWNLOAD_DIR}/include/qpOASES.hpp DESTINATION ${QPOASES_INCLUDE_DIR})

# file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/*")
# foreach (HEADER_FILE ${HEADERS})
#   file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES)
# endforeach ()

# file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/extras/*")
# foreach (HEADER_FILE ${HEADERS})
#   file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES/extras)
# endforeach ()

# ament_export_include_directories(${QPOASES_INCLUDE_DIR})
# ament_export_libraries(qpOASES)

# install(TARGETS qpOASES
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib)

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(qpoases_colcon)

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
include(FetchContent)

# 定义变量
set(QPOASES_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "QPOASES install path")
set(QPOASES_INCLUDE_DIR ${QPOASES_DEVEL_PREFIX}/include)
set(QPOASES_LIB_DIR ${QPOASES_DEVEL_PREFIX}/lib)

# 🔥🔥🔥 [核心修改 1] 强制指向本地源码路径 🔥🔥🔥
# 原代码: set(QPOASES_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(QPOASES_DOWNLOAD_DIR "/ws/qpOASES-master")

set(QPOASES_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# 创建目录
file(MAKE_DIRECTORY ${QPOASES_INCLUDE_DIR})
file(MAKE_DIRECTORY ${QPOASES_LIB_DIR})
# file(MAKE_DIRECTORY ${QPOASES_DOWNLOAD_DIR}) # 不需要创建了，因为是存在的源码
file(MAKE_DIRECTORY ${QPOASES_BUILD_DIR})

set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# ----------------------------------------------------
# 关键点：改为本地加载，跳过网络下载
# ----------------------------------------------------
FetchContent_Declare(
  qpoasesDownload
  
  # 🔥🔥🔥 [核心修改 2] 去掉网络参数，使用本地路径 🔥🔥🔥
  # GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  # GIT_TAG        3.2.0
  # UPDATE_COMMAND ""
  
  # 这一行告诉 CMake 去哪里找代码 (即上面的 /ws/qpOASES-master)
  SOURCE_DIR ${QPOASES_DOWNLOAD_DIR}
  
  # 保持原有的构建指令不变
  BINARY_DIR ${QPOASES_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install"
)

FetchContent_MakeAvailable(qpoasesDownload)

# 复制头文件
# 因为变量改了，这里会自动去 /ws/qpOASES-master/include 里找，逻辑是通的
file(COPY ${QPOASES_DOWNLOAD_DIR}/include/qpOASES.hpp DESTINATION ${QPOASES_INCLUDE_DIR})

file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/*")
foreach (HEADER_FILE ${HEADERS})
  file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES)
endforeach ()

file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/extras/*")
foreach (HEADER_FILE ${HEADERS})
  file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES/extras)
endforeach ()

ament_export_include_directories(${QPOASES_INCLUDE_DIR})
ament_export_libraries(qpOASES)

install(TARGETS qpOASES
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

ament_package()