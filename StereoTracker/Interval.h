// Source: http://noobtuts.com/cpp/interval
#include <windows.h>

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
};
