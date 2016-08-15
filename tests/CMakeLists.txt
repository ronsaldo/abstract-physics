set(APhyTests_SOURCES
    Engine.cpp
    Main.cpp
    World.cpp
)

add_executable(APhyTests ${APhyTests_SOURCES})
target_link_libraries(APhyTests ${APHY_MAIN_LIB} ${UNITTESTMM_LIBRARIES})
