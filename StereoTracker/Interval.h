// Source: http://noobtuts.com/cpp/interval
#include <sys/time.h>

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
    unsigned GetTickCount() const
    {
        struct timeval tv;
        if (gettimeofday(&tv, NULL) != 0)
            return 0;
        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
    }
};
