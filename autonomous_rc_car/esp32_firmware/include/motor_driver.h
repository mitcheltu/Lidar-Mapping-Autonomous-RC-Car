#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

class MotorDriver {
public:
    void setup();
    void setSpeed(int left, int right);
    void stop();
};

#endif
