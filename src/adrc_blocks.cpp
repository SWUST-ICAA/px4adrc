#include "px4adrc/adrc_blocks.hpp"

#include <cmath>

namespace px4adrc
{

double fal(double error, double alpha, double delta)
{
  const double abs_error = std::abs(error);
  if (abs_error <= delta) {
    return error / std::pow(delta, 1.0 - alpha);
  }

  return std::pow(abs_error, alpha) * (error >= 0.0 ? 1.0 : -1.0);
}

void update_tracking_differentiator(double reference, TrackingDifferentiator & td)
{
  const double error = td.x1 - reference;
  td.x1 += td.h * td.x2;
  td.x2 += td.h * (-2.0 * td.r * td.x2 - td.r * td.r * error);
}

void update_eso(double measurement, double control_input, ExtendedStateObserver & eso)
{
  const double error = eso.z1 - measurement;
  eso.z1 += eso.h * (eso.z2 - eso.beta1 * error);
  eso.z2 += eso.h * (eso.z3 - eso.beta2 * fal(error, 0.5, 0.01) + eso.b0 * control_input);
  eso.z3 += eso.h * (-eso.beta3 * fal(error, 0.25, 0.01));
}

double compute_nlsef(double e1, double e2, const NlsefGains & gains)
{
  return gains.k1 * fal(e1, gains.alpha1, gains.delta) +
         gains.k2 * fal(e2, gains.alpha2, gains.delta);
}

}  // namespace px4adrc
