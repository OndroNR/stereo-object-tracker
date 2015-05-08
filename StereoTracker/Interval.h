// Source: http://noobtuts.com/cpp/interval
#pragma once

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

class Interval
{
private:
    unsigned int initial_;
public:
    // Ctor
    inline Interval() : initial_(GetTickCount())
    {
    }
    // Dtor
    virtual ~Interval()
    {
    }
    inline unsigned int value() const 
    {
        return GetTickCount() - initial_;
    }
	#ifndef _WIN32
    unsigned GetTickCount() const
    {
        struct timeval tv;
        if (gettimeofday(&tv, NULL) != 0)
            return 0;
        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
    }
	#endif
};
