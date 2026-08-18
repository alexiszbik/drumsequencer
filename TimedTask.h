#ifndef TIMEDTASK_H
#define TIMEDTASK_H

class TimedTask {
private:
    unsigned long interval;
    unsigned long lastRun;
    void (*callback)();

public:
    TimedTask(unsigned long ms, void (*cb)()) {
        interval = ms;
        callback = cb;
        lastRun = 0;
    }

    bool update(unsigned long& currentTime) {
        if (currentTime - lastRun >= interval) {
            lastRun = currentTime;
            if (callback) callback();

            return true;
        }

        return false;
    }

    void setInterval(unsigned long ms) { interval = ms; }
    void setCallback(void (*cb)()) { callback = cb; }
};

#endif //TIMEDTASK_H