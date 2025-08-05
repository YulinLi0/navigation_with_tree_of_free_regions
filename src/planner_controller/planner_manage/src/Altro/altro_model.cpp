// Copyright [2021] Optimus Ride Inc.

#include "Altro/altro_model.h"

#include <cmath>

#include "altro/utils/utils.hpp"

namespace altro {
namespace models {

void Quadrupedal::Evaluate(const VectorXdRef& x, const VectorXdRef& u, const float t,
                               Eigen::Ref<VectorXd> xdot) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(x);
// std::cout << "MODEL EVALUATE" << std::endl;
  //single rigid body model
    xdot(0) = u(0);
    xdot(1) = u(1);
    xdot(2) = u(2);
    xdot(3) = u(3); 
    xdot(4) = u(4);
    xdot(5) = u(5);

}

void Quadrupedal::Jacobian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                        Eigen::Ref<MatrixXd> jac) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(x);
// std::cout << "MODEL JAC" << std::endl;
    jac.setZero();
    jac(0, 6) = 1;
    jac(1, 7) = 1;
    jac(2, 8) = 1;
    jac(3, 9) = 1;
    jac(4, 10) = 1;
    jac(5, 11) = 1;

}

void Quadrupedal::Hessian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                       const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
    ALTRO_UNUSED(t);
    ALTRO_UNUSED(x);
    ALTRO_UNUSED(b);
    ALTRO_UNUSED(u);
}

}  // namespace examples
}  // namespace altro