#include <APHY/aphy.hpp>

#include <UnitTest++.h>

SUITE(Engine)
{
    TEST(CreateEngine)
    {
        aphy_size numEngines;
        aphy_engine_ref engine;
        aphyGetEngines(1, (aphy_engine**)&engine, &numEngines);
        CHECK(numEngines > 0);
    }

}
