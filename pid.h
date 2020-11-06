#include <Arduino.h>

#ifndef FFT_PID_H
#define FFT_PID_H
//lib version is 0.3

namespace pidreg {
    //TODO написать более подробное описание
    /**
     * @brief PID-regulator. 
     * If you need PI, ID etc regulators, then set an appropriate coefficient zero.
     * 
     */
    class PID {
    private:
        double kp;
        double ki;
        double kd;
        double previousError;
        double sumOfError;
        unsigned long currentTime;
        unsigned long previousTime;

        /**
         * @brief Returns time difference between now and previous call of impact function
         * 
         * @return unsigned long 
         */
        unsigned long timeDifference();

        double getP(const double &error);
        double getI(const double &error);
        double getD(const double &error);

    public:
        /**
         * @brief Construct a new PID object with coefficients equal to zero
         * 
         */
        PID();

        /**
         * @brief Construct a new PID object with params
         * 
         * @param kp proportional coefficient
         * @param ki integral coefficient
         * @param kd derivative coefficient
         */
        PID(const double &kp, const double &ki = 0, const double &kd = 0);

        /**
         * @brief Get the Impact value to system. Main function of regulator.
         * 
         * @param error magnitude deviation
         * @return double - impact value to system
         */
        double getImpact(const double &error);

        /**
         * @brief Set coefficients of the PID-regulator
         * 
         * @param kp proportional coefficient
         * @param ki integral coefficient
         * @param kd derivative coefficient
         */
        void setCoef(const double &kp, const double &ki, const double &kd);

        /**
         * @brief Set the Kp coefficient
         * 
         * @param kp proportional coefficient
         */
        void setKp(const double &kp);
        /**
         * @brief Set the Ki coefficient
         * 
         * @param ki integral coefficient
         */
        void setKi(const double &ki);
        /**
         * @brief Set the Kp coefficient
         * 
         * @param kd derivative coefficient
         */
        void setKd(const double &kd);
        
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