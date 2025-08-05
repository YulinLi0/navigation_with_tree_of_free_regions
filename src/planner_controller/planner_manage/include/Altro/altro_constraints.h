// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <limits>
#include <vector>
#include <string> 


#include "altro/constraints/constraint.hpp"
#include "altro/eigentypes.hpp"
#include "altro/utils/utils.hpp"
#include "./SDPsolver/SDPsolver.h"

using namespace std;
namespace altro {
namespace constraints {

class GoalConstraint : public constraints::Constraint<constraints::Equality> {
 public:
  explicit GoalConstraint(const VectorXd& xf) : xf_(xf) {}

  static constraints::ConstraintPtr<constraints::Equality> Create(const VectorXd& xf) {
    return std::make_shared<GoalConstraint>(xf);
  }

  std::string GetLabel() const override { return "Goal Constraint"; }
  int StateDimension() const override { return xf_.size(); }
  int OutputDimension() const override { return xf_.size(); }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
    ALTRO_UNUSED(u);
    c = x - xf_;
  }
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);
    jac.setIdentity();
  }

 private:
  VectorXd xf_;
};

class ControlBound : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
  explicit ControlBound(const int m)
      : m_(m),
        lower_bound_(m, -std::numeric_limits<double>::infinity()),
        upper_bound_(m, +std::numeric_limits<double>::infinity()) {}

  ControlBound(const std::vector<double>& lb, const std::vector<double>& ub)
      : m_(lb.size()), lower_bound_(lb), upper_bound_(ub) {
    ALTRO_ASSERT(lb.size() == ub.size(), "Upper and lower bounds must have the same length.");
    ALTRO_ASSERT(lb.size() > 0, "Cannot pass in empty bounds.");
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  void SetUpperBound(const std::vector<double>& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = ub;
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetUpperBound(std::vector<double>&& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = std::move(ub);
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetLowerBound(const std::vector<double>& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = lb;
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  void SetLowerBound(std::vector<double>&& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = std::move(lb);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  std::string GetLabel() const override { return "Control Bound";}

  int ControlDimension() const override { return m_; }

  int OutputDimension() const override {
    return index_lower_bound_.size() + index_upper_bound_.size();
  }

  void Evaluate(const VectorXdRef& /*x*/, const VectorXdRef& u,
                Eigen::Ref<VectorXd> c) override {
    ALTRO_ASSERT(u.size() == m_, "Inconsistent control dimension when evaluating control bound.");

    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      c(i) = lower_bound_.at(j) - u(j);
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      c(i + offset) = u(j) - upper_bound_.at(j);
    }
  }

  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    (void) u; // surpress erroneous unused variable error
    ALTRO_ASSERT(u.size() == m_, "Inconsistent control dimension when evaluating control bound.");
    jac.setZero();

    int n = x.size();  // state dimension
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      jac(i, n + j) = -1;
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      jac(i + offset, n + j) = 1;
    }
  }

 private:
  void ValidateBounds() {
    for (int i = 0; i < m_; ++i) {
      ALTRO_ASSERT(lower_bound_[i] <= upper_bound_[i],
                   "Lower bound isn't less than the upper bound.");
    }
  }
  static void GetFiniteIndices(const std::vector<double>& bound, std::vector<size_t>* index) {
    index->clear();
    for (size_t i = 0; i < bound.size(); ++i) {
      if (std::abs(bound[i]) < std::numeric_limits<double>::max()) {
        index->emplace_back(i);
      }
    }
  }
  int m_;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
  std::vector<size_t> index_lower_bound_;
  std::vector<size_t> index_upper_bound_;
};

class SDPConstraint : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
explicit SDPConstraint(const string &envname, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, const Eigen::MatrixXd &F, const Eigen::VectorXd &g, const Eigen::VectorXd &c):
  envname_(envname), A_(A), b_(b), F_(F), g_(g), c_(c)
  {
    std::cout << "SDPConstraint created start" << std::endl;
    // std::cout << "SDPConstraint created" << std::endl;
  }

   //name is "SDP constraint" + envname
   std::string GetLabel() const override { return "SDP Constraint";}

  int ControlDimension() const override { return 0; }

  int OutputDimension() const override { return 1; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
// std::cout << "SDPConstraint Evaluate start" << std::endl;
    // ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);
    // ALTRO_UNUSED(c);
// std::cout << "SDPConstraint Evaluate start" << std::endl;
    Eigen::VectorXd q;
// std::cout << "SDPConstraint Evaluate start1" << std::endl;
// std::cout << "x: " << x << std::endl;
    q = x;
// std::cout <<envname_<< "q: " << q << std::endl;
//print the shape of A_, b_, F_, g_, q, c_
// std::cout << "A_: " << A_.rows() << "x" << A_.cols() << std::endl;
// std::cout << "b_: " << b_.rows() << "x" << b_.cols() << std::endl;
// std::cout << "F_: " << F_.rows() << "x" << F_.cols() << std::endl;
// std::cout << "g_: " << g_.rows() << "x" << g_.cols() << std::endl;
// std::cout << "q: " << q.rows() << "x" << q.cols() << std::endl;
// std::cout << "c_: " << c_.rows() << "x" << c_.cols() << std::endl;
    sdp.solveSDP(envname_, A_, b_, F_, g_, q, c_);
    Eigen::VectorXd alpha_temp;
    alpha_temp.resize(1);
    //sdp.getAlpha() is double, put it in alpha_temp
    alpha_temp(0) = sdp.getAlpha()-1;
// std::cout << "SDPConstraint Evaluate TEST1" << std::endl;
    c = alpha_temp;
    // std::cout<<"constraint value: "<<c<<std::endl;

// std::cout << "SDPConstraint Evaluate end" << std::endl;
//     c = x - xf_;
  }
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);
    // std::cout << "jac: " << jac.rows() << "x" << jac.cols() << std::endl;
    // ALTRO_UNUSED(jac);
// std::cout << "SDPConstraint Jacobian start" << std::endl;
    //print the
    // if((sdp.getAlpha()-1)<=0)
    // {
    //   jac.setZero();
    // }
    // else
    // {
      Eigen::VectorXd jac_x= sdp.getGradient();
      //print jac_x
      // std::cout << "jac_x: " << jac_x << std::endl;
      Eigen::VectorXd jac_u;
      jac_u.resize(1);
      // // jac_x << 0;
      // jac_u(0) = 0;
      jac_x(3)  =0;
      jac_x(4)  =0;
      jac << jac_x.transpose(),jac_u;
    // }

    // jac = jac_x;
    // //print jac
    // std::cout << "jac: " << jac << std::endl;
// std::cout << "SDPConstraint Jacobian end" << std::endl;
    // jac.setIdentity();
  }
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::MatrixXd F_;
  Eigen::VectorXd g_;
  Eigen::VectorXd c_;
 private:
  Eigen::VectorXd xf_;
  SDPsolver sdp;
  string envname_;

  // Eigen::VectorXd q_;

};


}  // namespace examples
}  // namespace altro