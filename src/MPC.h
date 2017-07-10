#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double ref_cte, double ref_epsi, double ref_vel, double cte_w, double epsi_w, double vel_w, double delta_w, double acc_w, double delta_change_w, double acc_change_w);

};

#endif /* MPC_H */
