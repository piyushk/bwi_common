cmake_minimum_required(VERSION 2.8.3)
project(bwi_rqt_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  bwi_msgs
  class_loader
  cv_bridge
  image_transport
  rqt_gui
  rqt_gui_cpp
  sensor_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(bwi_rqt_image_view_SRCS
  src/bwi_rqt_plugins/image_view.cpp
  src/bwi_rqt_plugins/ratio_layouted_frame.cpp
)

set(bwi_rqt_image_view_HDRS
  include/bwi_rqt_plugins/image_view.h
  include/bwi_rqt_plugins/ratio_layouted_frame.h
)

set(bwi_rqt_image_view_UIS
  src/bwi_rqt_plugins/image_view.ui
)

set(bwi_rqt_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  CATKIN_DEPENDS
    bwi_msgs
    class_loader
    cv_bridge
    image_transport
    rqt_gui
    rqt_gui_cpp
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})
catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(bwi_rqt_image_view_MOCS ${bwi_rqt_image_view_HDRS})
else()
  qt4_wrap_cpp(bwi_rqt_image_view_MOCS ${bwi_rqt_image_view_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(bwi_rqt_image_view_UIS_H ${bwi_rqt_image_view_UIS})
else()
  qt4_wrap_ui(bwi_rqt_image_view_UIS_H ${bwi_rqt_image_view_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${bwi_rqt_plugins_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${bwi_rqt_image_view_SRCS} ${bwi_rqt_image_view_MOCS} ${bwi_rqt_image_view_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES cpp_plugins.xml python_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
