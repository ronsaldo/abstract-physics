#ifndef APHY_ENGINE_HPP_
#define APHY_ENGINE_HPP_

#include "object.hpp"

/**
* APhy bullet engine
*/
struct _aphy_engine : public Object<_aphy_engine>
{
public:
    _aphy_engine();

    void lostReferences();

    aphy_cstring getName (  );
    aphy_int getVersion (  );
};

#endif //
