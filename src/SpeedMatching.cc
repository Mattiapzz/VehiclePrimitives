#include "include/SpeedMatching.hh"

using namespace vprim;


// Validates that the velocity never drops below zero during the maneuver
bool SpeedMatching::check_velocity() const {
  if (T_ <= 0.0) return false;
  
  const int samples = 50;
  const double dt = T_ / samples;
  const double epsilon = 1e-4;

  for (int i = 1; i < samples; ++i) {
    double t = i * dt;
    // Only check strictly inside the interval
    if (t > epsilon && t < (T_ - epsilon)) {
      if (poly_v_.eval(t) < 0.0) {
        return false; 
      }
    }
  }
  return true;
}


bool SpeedMatching::build(
  const LongitudinalState& start, 
  const LongitudinalState& goal)  
{
  start_state_ = start;
  is_valid_ = true;

  double ds = goal.s - start.s;
  double jmax2 = jmax_ * jmax_;

  // 1. Setup the 6th-degree polynomial to find the minimum time T
  Polynomial T_poly = {
    ( -3600.0 * ds * ds ) / jmax2,
    ( 2880.0 * ds * (start.v + goal.v) ) / jmax2,
    ( (360.0 * start.a - 360.0 * goal.a) * ds - 576.0 * start.v * start.v - 1008.0 * start.v * goal.v - 576.0 * goal.v * goal.v ) / jmax2,
    ( (-144.0 * start.v - 96.0 * goal.v) * start.a + 96.0 * (start.v + 1.5 * goal.v) * goal.a ) / jmax2,
    ( -9.0 * start.a * start.a + 6.0 * start.a * goal.a - 9.0 * goal.a * goal.a ) / jmax2,
    ( 0.0 ) / jmax2,
    wT_
  };

  // Initial guess for the solver based on average velocity
  double v_m = (start.v + goal.v) / 2.0;
  double T0 = ds / std::max(std::abs(v_m), 1.0);

  // 2. Solve for optimal time using the built-in Polynomial solver
  auto root_opt = T_poly.closest_root(T0);

  if (!root_opt.has_value() || root_opt.value() <= 1e-4 || root_opt.value() > 100.0) {
      is_valid_ = false;
      return false;
  }
  T_ = root_opt.value();

  // 3. Compute Spatial Trajectory Coefficients (Quintic)
  double T2 = T_ * T_;
  double T3 = T2 * T_;
  double T4 = T3 * T_;
  double T5 = T4 * T_;

  double c0 = start.s;
  double c1 = start.v;
  double c2 = start.a / 2.0;
  double c3 = ((-3.0 * start.a + goal.a) * T2 + (-12.0 * start.v - 8.0 * goal.v) * T_ + 20.0 * ds) / (2.0 * T3);
  double c4 = ((3.0 * start.a - 2.0 * goal.a) * T2 + (16.0 * start.v + 14.0 * goal.v) * T_ - 30.0 * ds) / (2.0 * T4);
  double c5 = ((-start.a + goal.a) * T2 + (-6.0 * start.v - 6.0 * goal.v) * T_ + 12.0 * ds) / (2.0 * T5);

  // 4. Cascade Generate Kinematic Profiles
  poly_s_ = {c0, c1, c2, c3, c4, c5};
  poly_v_ = poly_s_.derivative();
  poly_a_ = poly_v_.derivative();
  poly_j_ = poly_a_.derivative();

  // 5. Validation Check
  if (!check_velocity()) {
    is_valid_ = false;
    return false;
  }

  // 6. Compute Final Analytical Cost
  cost_ = (wT_ * T_) 
        + ( ((9.0 * start.a * start.a - 6.0 * start.a * goal.a + 9.0 * goal.a * goal.a) / T_) ) / jmax2
        + ( (((72.0 * start.v + 48.0 * goal.v) * start.a) - 48.0 * (start.v + 1.5 * goal.v) * goal.a) / T2) / jmax2
        + ( ((-120.0 * start.a * ds + 120.0 * goal.a * ds + 192.0 * start.v * start.v + 336.0 * start.v * goal.v + 192.0 * goal.v * goal.v) / T3) ) / jmax2
        - ( (720.0 * ds * (start.v + goal.v) / T4) ) / jmax2
        + ( (720.0 * ds * ds / T5) ) / jmax2;

  return is_valid_;
}


LongitudinalState 
SpeedMatching::eval(double t) const {
  t = std::clamp(t,0.0,T_);

  LongitudinalState state;
  state.s = poly_s_.eval(t);
  state.v = poly_v_.eval(t);
  state.a = poly_a_.eval(t);
  state.j = poly_j_.eval(t);

  return state;
}