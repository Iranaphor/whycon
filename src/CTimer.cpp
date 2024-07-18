#include "CTimer.h"

CTimer::CTimer(int timeout)
{
    reset();
    timeoutInterval = timeout;
    pause();
}

CTimer::~CTimer()
{}

void CTimer::reset(int timeout)
{
    timeoutInterval = timeout;
    startTime = getRealTime();
    pauseTime = startTime;
}

int64_t CTimer::getRealTime()
{
    auto currentTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(currentTime.time_since_epoch());
    return duration.count();
}

int CTimer::getTime()
{
    int result;
    if (running)
    {
        result = getRealTime() - startTime;
    }
    else
    {
        result = pauseTime - startTime;
    }
    return result;
}

bool CTimer::timeOut()
{
    return getTime() > timeoutInterval;
}

bool CTimer::paused()
{
    return (running == false);
}

int CTimer::pause()
{
    running = false;
    pauseTime = getRealTime();
    return pauseTime;
}

int CTimer::start()
{
    startTime += (getRealTime() - pauseTime);
    running = true;
    return getTime();
}
