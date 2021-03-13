#include <Arduino.h>
#include <functional>

#ifndef FFT_PID_H
#define FFT_PID_H
//lib version is 0.7

namespace neko {
    //TODO написать более подробное описание
    /**
     * @brief PID-regulator. 
     * If you need PI, ID etc regulators, then set an appropriate coefficient zero.
     * 
     */
    class PID {
    private:
        std::function<unsigned long()> time;
        double kp;
        double ki;
        double kd;
        double previousError;
        double sumOfErrors;
        unsigned long currentTime;
        unsigned long previousTime;

        /**
         * @brief Returns time difference between now and previous call of impact function
         * 
         * @return unsigned long 
         */
        unsigned long timeDifference();

        double getP(double error);
        double getI(double error);
        double getD(double error);

    public:
        /**
         * @brief Construct a new PID object with coefficients equal to zero and micros as time function
         * 
         */
        PID();

        /**
         * @brief Construct a new PID object with params and micros as time function
         * 
         * @param kp proportional coefficient
         * @param ki integral coefficient
         * @param kd derivative coefficient
         */
        PID(double kp, double ki = 0, double kd = 0);
        
        /**
         * @brief Construct a new PID object
         * 
         * @param time function for getting time, usually micros or millis
         * @param kp proportional coefficient
         * @param ki integral coefficient
         * @param kd derivative coefficient 
         */
        PID(std::function<unsigned long()> time, double kp = 0, double ki = 0, double kd = 0);

        /**
         * @brief Get the Impact value to system. Main function of regulator.
         * 
         * @param error magnitude deviation
         * @return double - impact value to system
         */
        double getImpact(double error);

        /**
         * @brief Set coefficients of the PID-regulator
         * 
         * @param kp proportional coefficient
         * @param ki integral coefficient
         * @param kd derivative coefficient
         */
        void setCoef(double kp, double ki, double kd);

        /**
         * @brief Set the Kp coefficient
         * 
         * @param kp proportional coefficient
         */
        void setKp(double kp);
        /**
         * @brief Set the Ki coefficient
         * 
         * @param ki integral coefficient
         */
        void setKi(double ki);
        /**
         * @brief Set the Kp coefficient
         * 
         * @param kd derivative coefficient
         */
        void setKd(double kd);
        
        /**
         * @brief Get the Kp coefficient
         * 
         * @return double 
         */
        double getKp();
        /**
         * @brief Get the Ki coefficient
         * 
         * @return double 
         */
        double getKi();
        /**
         * @brief Get the Kd coefficient
         * 
         * @return double 
         */
        double getKd();
        
        /**
         * @brief Construct a copy of regulator
         * 
         * @param pid being copied regulator
         */
        PID(const PID &pid) = default;
        /**
         * @brief Copy a regulator
         * 
         * @param pid the being copied regulator
         * @return PID& reference to this object
         */
        PID &operator=(const PID &pid) = default;
        /**
         * @brief Destroy the PID object
         * 
         */
        ~PID() = default;
    };
} // namespace pidreg

#endif