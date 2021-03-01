#include <pid.h>

namespace neko {
    PID::PID()
        : time(micros), kp(0), ki(0), kd(0), previousError(0), sumOfErrors(0), currentTime(0), previousTime(0) {}

    PID::PID(double kp, double ki, double kd)
        : time(micros), kp(kp), ki(ki), kd(kd), previousError(0), currentTime(0), previousTime(0) {}

    PID::PID(std::function<unsigned long()> time, double kp, double ki, double kd)
        : time(time), kp(kp), ki(ki), kd(kd), previousError(0), currentTime(0), previousTime(0) {}

    double PID::getImpact(double error) {
        currentTime = time();
        double out = getP(error) + getI(error) + getD(error);
        previousError = error;
        previousTime = currentTime;
        return out;
    }

    void PID::setTimeFun(std::function<unsigned long()> time) {
        this->time = time;
    }

    std::function<unsigned long()> PID::getTimeFun() {
        return time;
    }

    unsigned long PID::timeDifference() {
        return currentTime - previousTime;
    }

    void PID::setCoef(double kp, double ki, double kd) {
        //setting params
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void PID::setKp(double kp) {
        this->kp = kp;
    }

    void PID::setKi(double ki) {
        this->ki = ki;
    }

    void PID::setKd(double kd) {
        this->kd = kd;
    }

    double PID::getKp() {
        return kp;
    }

    double PID::getKi() {
        return ki;
    }

    double PID::getKd() {
        return kd;
    }

    double PID::getP(double error) {
        //if Kp = 0 we don't need to count it, it's automatically 0
        if (kp == 0) {
            return 0;
        }
        return kp * error;
    }

    double PID::getI(double error) {
        //if Ki = 0 we don't need to count it
        if (ki == 0) {
            return 0;
        }
        sumOfErrors += error;
        return ki * timeDifference() * sumOfErrors;
    }

    double PID::getD(double error) {
        //if Kd = 0 we don't need to count it
        if (kd == 0) {
            return 0;
        }
        return kd * (error - previousError) / timeDifference();
    }
} // namespace pidreg