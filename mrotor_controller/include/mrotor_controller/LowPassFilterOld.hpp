#include "Eigen/Dense"

// template <typename Eigen::Vector3d>
class FirstOrderFilter {
/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/

  public:
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, Eigen::Vector3d initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    Eigen::Vector3d updateFilter(Eigen::Vector3d inputState, double samplingTime) {
      /*
      This method will apply a first order filter on the inputState.
      */
      Eigen::Vector3d outputState;

      // Calcuate the outputState if accelerating.
      double alphaUp = exp(- samplingTime / timeConstantUp_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

      previousState_ = outputState;
      return outputState;

    }
    ~FirstOrderFilter() {}

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    Eigen::Vector3d previousState_;
};


class LowPassFilter {
  public:

};

class LowPassFilter2nd : public LowPassFilter {
  
};