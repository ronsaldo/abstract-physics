#ifndef _APHY_BULLET_OBJECT_HPP_
#define _APHY_BULLET_OBJECT_HPP_

#include <atomic>
#include <assert.h>
#include "common.hpp"

extern "C" aphy_icd_dispatch aphy_bullet_icd_dispatch;

/**
 * AGPU OpenGL object.
 */
template<typename ST>
class Object
{
public:
    Object()
        : dispatch(&aphy_bullet_icd_dispatch), refcount(1)
    {
    }

    ~Object()
    {
    }

    aphy_error retain()
    {
        auto old = refcount.fetch_add(1, std::memory_order_relaxed);
        assert(old > 0 && "the object has to still exist");
        if(old == 0)
            return APHY_INVALID_OPERATION;
        return APHY_OK;
    }

    aphy_error release()
    {
        auto old = refcount.fetch_sub(1,  std::memory_order_relaxed);
        assert(old > 0 && "the object has to still exist");
        if(old == 0)
            return APHY_INVALID_OPERATION;
        else if (old == 1)
        {
            static_cast<ST*> (this)->lostReferences();
            delete static_cast<ST*> (this);
        }
        return APHY_OK;

    }

private:
    aphy_icd_dispatch *dispatch;
    std::atomic_int refcount;
};

#endif //_APHY_BULLET_OBJECT_HPP_
