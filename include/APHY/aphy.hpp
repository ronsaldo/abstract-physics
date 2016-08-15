
#ifndef APHY_HPP_
#define APHY_HPP_

#include <stdexcept>
#include "APHY/aphy.h"

/**
 * Abstract Physics exception.
 */
class aphy_exception : public std::runtime_error
{
public:
    explicit aphy_exception(aphy_error error)
        : std::runtime_error("AGPU Error"), errorCode(error)
    {
    }

    aphy_error getErrorCode() const
    {
        return errorCode;
    }

private:
    aphy_error errorCode;
};

/**
 * Abstract GPU reference smart pointer.
 */
template<typename T>
class aphy_ref
{
public:
    aphy_ref()
        : pointer(0)
    {
    }

    aphy_ref(const aphy_ref<T> &other)
    {
        if(other.pointer)
            other.pointer->addReference();
        pointer = other.pointer;
    }

    aphy_ref(T* pointer)
        : pointer(pointer)
    {
    }

    ~aphy_ref()
    {
        if (pointer)
            pointer->release();
    }

    aphy_ref<T> &operator=(T *newPointer)
    {
        if (pointer)
            pointer->release();
        pointer = newPointer;
        return *this;
    }

    aphy_ref<T> &operator=(const aphy_ref<T> &other)
    {
        if(pointer != other.pointer)
        {
            if(other.pointer)
                other.pointer->addReference();
            if(pointer)
                pointer->release();
            pointer = other.pointer;
        }
        return *this;
    }

    operator bool() const
    {
        return pointer;
    }

    bool operator!() const
    {
        return !pointer;
    }

    T* get() const
    {
        return pointer;
    }

    T *operator->() const
    {
        return pointer;
    }

private:
    T *pointer;
};

/**
 * Helper function to convert an error code into an exception.
 */
inline void APhyThrowIfFailed(aphy_error error)
{
    if(error < 0)
        throw aphy_exception(error);
}

// Interface wrapper for aphy_engine.
struct _aphy_engine
{
private:
	_aphy_engine() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddEngineReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseEngine( this ));
	}

	inline aphy_cstring getName (  )
	{
		return aphyGetEngineName( this );
	}

	inline aphy_int getVersion (  )
	{
		return aphyGetEngineVersion( this );
	}

};

typedef aphy_ref<aphy_engine> aphy_engine_ref;


#endif /* APHY_HPP_ */
