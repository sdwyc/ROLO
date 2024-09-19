cmake_minimum_required(VERSION 3.0.2)

project(pmc-download NONE)

include(ExternalProject)
ExternalProject_Add(pmc
        GIT_REPOSITORY    https://github.com/jingnanshi/pmc.git
        SOURCE_DIR        "/home/yuaqi/3D_slam/rolo/build/pmc-src"
        BINARY_DIR        "/home/yuaqi/3D_slam/rolo/build/pmc-build"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
        TEST_COMMAND      ""
        )
