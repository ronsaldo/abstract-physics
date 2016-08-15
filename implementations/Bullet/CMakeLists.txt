option(BUILD_SHARED_LIBS "" ON)
option(BUILD_UNIT_TESTS "" OFF)
option(BUILD_CPU_DEMOS "" OFF)
option(BUILD_BULLET3 "" OFF)
option(BUILD_BULLET2_DEMOS "" OFF)
option(BUILD_EXTRAS "" OFF)
option(USE_GLUT "" OFF)
option(USE_GRAPHICAL_BENCHMARK "" OFF)
add_subdirectory(bullet3)

set(APHY_Bullet_SOURCES
    icd.c
    object.hpp
)

add_definitions(-DAPHY_BUILD)

add_library(APhyBullet SHARED ${APHY_Bullet_SOURCES})
target_link_libraries(APhyBullet)
