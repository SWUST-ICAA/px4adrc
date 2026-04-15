#include <gtest/gtest.h>

#include "px4adrc/adrc_blocks.hpp"

TEST(AdrcBlocks, TrackingDifferentiatorConvergesTowardReference) {
  px4adrc::TrackingDifferentiator td{};
  td.r = 20.0;
  td.h = 0.01;

  for (int i = 0; i < 200; ++i) {
    px4adrc::update_tracking_differentiator(1.0, td);
  }

  EXPECT_NEAR(td.x1, 1.0, 5e-2);
}

TEST(AdrcBlocks, EsoTracksConstantSignal) {
  px4adrc::ExtendedStateObserver eso{};
  eso.beta1 = 80.0;
  eso.beta2 = 400.0;
  eso.beta3 = 500.0;
  eso.b0 = 1.0;
  eso.h = 0.01;

  for (int i = 0; i < 400; ++i) {
    px4adrc::update_eso(0.5, 0.0, eso);
  }

  EXPECT_NEAR(eso.z1, 0.5, 5e-2);
}

TEST(AdrcBlocks, NlsefProducesZeroForZeroError) {
  px4adrc::NlsefGains gains{};
  gains.k1 = 2.0;
  gains.k2 = 1.0;
  gains.alpha1 = 0.5;
  gains.alpha2 = 0.25;
  gains.delta = 0.01;

  EXPECT_NEAR(px4adrc::compute_nlsef(0.0, 0.0, gains), 0.0, 1e-12);
}
