#include <pid.h>

namespace pidreg {
    PID::PID()
        : kp(0), ki(0), kd(0), previousError(0), currentTime(0), previousTime(0), sumOfError(0) {}

    PID::PID(const double &kp, const double &ki, const double &kd)
        : kp(kp), ki(ki), kd(kd), previousError(0), currentTime(0), previousTime(0), sumOfError(0) {}

    
    double PID::getImpact(const double &error) {
        currentTime = millis();
        double out = getP(error) + getI(error) + getD(error);
        previousError = error;
        previousTime = millis();
        return out;
    }

    unsigned long PID::timeDifference() {
        return currentTime - previousTime;
    }

    void PID::setCoef(const double &kp, const double &ki, const double &kd) {
        //setting params
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void PID::setKp(const double &kp) {
        this->kp = kp;
    }

    void PID::setKi(const double &ki) {
        this->ki = ki;
    }

    void PID::setKd(const double &kd) {
        this->kd = kd;
    }

    double PID::getKp() {
        return kp;
    }

    double PID::getKp() {
        return kp;
    }

    double PID::getP(const double &error) {
        //if Kp = 0 we don't need to count it, it's automatically 0
        if (kp == 0) {
            return 0;
        }
        return kp * error;
    }

    double PID::getI(const double &error) {
        //if Ki = 0 we don't need to count it
        if (ki == 0) {
            return 0;
        }
        sumOfError += error;
        return ki * timeDifference() * sumOfError;
    }

    double PID::getD(const double &error) {
        //if Kd = 0 we don't need to count it
        if (kd == 0) {
            return 0;
        }
        return kd * (error - previousError) / timeDifference();
    }
} // namespace pidreg