//
// Created by rgrandia on 18.02.21.
//

#pragma once

#include <memory>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * This class implements the interface between Linear Quadratic optimal control problems defined in OCS2 and the HPIPM solver.
 * If the problem dimensions change, resize needs to be called to re-initialize HPIPM.
 */
class HpipmInterface {
 public:
  struct OcpSize {
    /// !Need to adapt isSizeEqual in implementation if this struct changes!
    int N;                  // Number of stages
    std::vector<int> nu;    // Number of inputs
    std::vector<int> nx;    // Number of states
    std::vector<int> nbu;   // Number of input box inequality constraints
    std::vector<int> nbx;   // Number of state box inequality constraints
    std::vector<int> ng;    // Number of general inequality constraints
    std::vector<int> nsbu;  // Number of slack variables for input box inequalities
    std::vector<int> nsbx;  // Number of slack variables for state box inequalities
    std::vector<int> nsg;   // Number of slack variables for general inequalities
    OcpSize(int N_, int nx_, int nu_)
        : N(N_),
          nx(N_ + 1, nx_),
          nu(N_ + 1, nu_),
          nbu(N_ + 1, 0),
          nbx(N_ + 1, 0),
          ng(N_ + 1, 0),
          nsbu(N_ + 1, 0),
          nsbx(N_ + 1, 0),
          nsg(N_ + 1, 0) {
      nu.back() = 0;
    }
    OcpSize() : OcpSize(0, 0, 0) {};
  };

  struct Settings {
    /// !Need to adapt isSettingsEqual in implementation if this struct changes!
    int iter_max = 30;
    scalar_t alpha_min = 1e-8;
    scalar_t mu0 = 1e4;
    scalar_t tol_stat = 1e-5;
    scalar_t tol_eq = 1e-5;
    scalar_t tol_ineq = 1e-5;
    scalar_t tol_comp = 1e-5;
    scalar_t reg_prim = 1e-12;
    int warm_start = 0;
    int pred_corr = 1;
    int ric_alg = 0;
    Settings() {}
  };

  HpipmInterface() : HpipmInterface(OcpSize{0, 0, 0}) {};
  HpipmInterface(OcpSize ocpSize, const Settings& settings = Settings());
  ~HpipmInterface();

  /**
   * Resize with new settings
   */
  void resize(OcpSize ocpSize, const Settings& settings);

  /**
   * Resize with old settings
   */
  void resize(OcpSize ocpSize);

  /**
   * @return  HPIPM returned with flag:
   *    0 = QP solved;
   *    1 = Maximum number of iterations reached;
   *    2 = Minimum step length reached;
   *    3 = NaN in computations;
   *    4 = Unknown return flag;
   */
  int solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
             std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
             std::vector<vector_t>& stateTrajectory, std::vector<vector_t>& inputTrajectory, bool verbose = false);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ocs2