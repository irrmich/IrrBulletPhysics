#ifndef CHRONO-D_H_INCLUDED
#define CHRONO-D_H_INCLUDED
#include <iostream>
#include "LinearMath/btQuickprof.h"

class chrono
{
public:
    chrono()
    {
        m_clk = new btClock();
    }

    ~chrono()
    {
        delete m_clk;
    }

    void reset()
    {
        m_clk->reset();
    }

    unsigned long int getMillisec()
    {
        return m_clk->getTimeMicroseconds();
    }

    btClock* getClock()
    {
        return m_clk;
    }

    void show(const char* word)
    {
        std::cout << word << m_clk->getTimeMicroseconds() << std::endl;
    }
private:
    btClock* m_clk;
};

#endif // CHRONO-D_H_INCLUDED
