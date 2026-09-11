# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# -------------------------- IMPORTANT NOTE --------------------------#
# To compile the release project, set SDKCONFIG to sdkconfig.release
# To compile the debug project, set SDKCONFIG to sdkconfig.debug
set(SDKCONFIG sdkconfig.release)

list(APPEND EXTRA_COMPONENT_DIRS "src/main" "src/controller" "src/plant" "utils")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TCC-pid_lmi_controller_and_plant)
