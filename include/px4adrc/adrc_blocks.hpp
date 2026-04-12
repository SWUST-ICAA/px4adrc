#pragma once

namespace px4adrc
{

struct TrackingDifferentiator
{
  double x1{0.0};
  double x2{0.0};
  double r{1.0};
  double h{0.01};
};

struct ExtendedStateObserver
{
  double z1{0.0};
  double z2{0.0};
  double z3{0.0};
  double beta1{1.0};
  double beta2{1.0};
  double beta3{1.0};
  double b0{1.0};
  double h{0.01};
};

struct TrackingDifferentiatorGains
{
  double r{1.0};
};

struct EsoGains
{
  double beta1{1.0};
  double beta2{1.0};
  double beta3{1.0};
  double b0{1.0};
};

struct NlsefGains
{
  double k1{1.0};
  double k2{1.0};
  double alpha1{0.5};
  double alpha2{0.25};
  double delta{0.01};
};

double fal(double error, double alpha, double delta);
void update_tracking_differentiator(double reference, TrackingDifferentiator & td);
void update_eso(double measurement, double control_input, ExtendedStateObserver & eso);
double compute_nlsef(double e1, double e2, const NlsefGains & gains);

}  // namespace px4adrc
