#ifndef CTIMER_H
#define CTIMER_H

#include <chrono>

#define TIMEOUT_INTERVAL 40000

class CTimer
{
public:
    CTimer(int timeOut = TIMEOUT_INTERVAL);
    ~CTimer();

    void reset(int timeOut = TIMEOUT_INTERVAL);
    bool paused();

    int pause();
    int start();
    int getTime();
    bool timeOut();

    int64_t getRealTime();
private:
    int64_t startTime;
    int64_t pauseTime;
    bool running;
    int timeoutInterval;
};

#endif
