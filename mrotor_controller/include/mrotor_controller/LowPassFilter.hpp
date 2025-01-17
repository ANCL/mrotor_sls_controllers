#include "Eigen/Dense"
#include "ros/ros.h"
#include "iostream"
#define PI           3.14159265358979323846  /* pi */


template <typename Type>
class SecondOrderFilter {
    public:
        SecondOrderFilter(double cutoff_freq, double Q, bool verbose):verbose_(verbose) {
            if(verbose_) std::cout << "[2ndFilter] Construction Started" << std::endl;
            omega_ = 2 * PI * cutoff_freq;
            xi_ = 1/(2*Q);
            if(verbose_) std::cout << "[2ndFilter] Construction Complete" << std::endl;
        };
        ~SecondOrderFilter() {
            if(verbose_) std::cout << "[2ndFilter] Destructed" << std::endl;
        };

        Type updateFilter(Type x_k, double sampling_time) {
            const double omega = omega_;
            const double xi = xi_;
            const double T = sampling_time;
            const double k1 = omega*omega*T*T + 4*omega*xi*T + 4;
            const double k2 = 2*omega*omega*T*T - 8;
            const double k3 = omega*omega*T*T - 4*omega*xi*T + 4;
            const double k4 = omega*omega*T*T;
            const double k5 = 2*k4;
            const double k6 = k4;
            a_[0] = 1;
            a_[1] = k2 / k1;
            a_[2] = k3 / k1;
            b_[0] = k4 / k1;
            b_[1] = k5 / k1;
            b_[2] = k6 / k1;

            std::cout << "[filterUpdate] T=" << T << " omega=" << omega << " xi=" << xi << " b=" << b_[0] << "," << b_[1] << "," << b_[2] << " a=" << a_[0] << "," << a_[1] << "," << a_[2] << std::endl;

            Type y_k;
            y_k = b_[0] * x_k + b_[1] * x_k_1_ + b_[2]*x_k_2_ - a_[1] * y_k_1_ - a_[2] * y_k_2_;

            x_k_2_ = x_k_1_;
            x_k_1_ = x_k;

            y_k_2_ = y_k_1_;
            y_k_1_ = y_k;

            return y_k;
        }
    private:
        double b_[3];   // coefficients of numerator
        double a_[3];   // coefficients of denominator
        Type x_k_1_, x_k_2_;   // x(k-1), x(k-2)
        Type y_k_1_, y_k_2_;   // y(k-1), y(k-2)    
        double omega_;
        double xi_;
        double verbose_;
    protected:
};