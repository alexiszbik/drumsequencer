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

    void update() {
        unsigned long now = millis();
        if (now - lastRun >= interval) {
            lastRun = now;
            if (callback) callback();
        }
    }

    void setInterval(unsigned long ms) { interval = ms; }
    void setCallback(void (*cb)()) { callback = cb; }
};

#endif //TIMEDTASK_H