#include "Altro/altro_problem.h"

namespace altro {
namespace problems {

QuadrupedalProblem::QuadrupedalProblem() {
}

problem::Problem QuadrupedalProblem::MakeProblem(const bool add_constraints) {
    problem::Problem prob(N);
    // tf = 3;
    float h = GetTimeStep();
    xref.resize(N);
    // lb = {-q_bnd};
    // ub = {+q_bnd};
    lb_v.resize(6);
    ub_v.resize(6);
    lb_v = {-v_bnd,-v_bnd,-v_bnd,-v_bnd,-v_bnd,-vyaw_bnd};
    ub_v = {+v_bnd,+v_bnd,+v_bnd,+v_bnd,+v_bnd,+vyaw_bnd};
    lb_vyaw = {-vyaw_bnd};
    ub_vyaw = {+vyaw_bnd};
    Q.diagonal().setConstant(5);
    Q(0,0) = 10;
    Q(1,1) = 10;
    Q(2,2) = 40;
    Q(3,3) = 0;
    Q(4,4) = 0;
    R.diagonal().setConstant(0.1);
    Qf.diagonal().setConstant(100);
    Qf(3,3) = 0;
    Qf(4,4) = 0;
    A.resize(1);
    b.resize(1);
    F.resize(2);
    g.resize(2);
    c.resize(2);
    A[0] = Eigen::MatrixXd::Zero(6, 3);
    b[0] = Eigen::VectorXd::Zero(6);
    // robot link shapes
    A[0]  << 1, 0  , 0,
            -1, 0  , 0,
             0, 1  , 0,
             0, -1 , 0,
             0, 0  , 1,
             0, 0  , -1;
    b[0] << 0.38, 0.42, 0.22, 0.22, 0.10, 0.10;
    // A[0] = Eigen::MatrixXd::Zero(8, 3);
    // b[0] = Eigen::VectorXd::Zero(8);
    // A[0]    << -0.70710678,  0.70710678, -0.,
    //            -0.5547002,  -0.83205029,  0.,
    //            0.,          0.,         -1.,
    //            -0.,         -0.,          1.,
    //            -0.,          1.,         -0.,
    //            0.4472136,  -0.89442719,  0.,
    //            0.51449576,  0.85749293, -0.,
    //            0.89442719, -0.4472136,   0.;
    // b[0]    << 0.21213203, 0.16641006, 0.1,   0.1,   0.15,  0.17888544, 0.12862394, 0.2236068;
    // A[0] = Eigen::MatrixXd::Zero(7, 3);
    // b[0] = Eigen::VectorXd::Zero(7);
    // A[0]    << -0.70710678, -0.70710678,  0.,
    //            -0.58123819,  0.81373347,  0.,
    //            -0.24253563, -0.9701425,   0.,
    //            -0.,         -0.,         -1.,
    //            0.,          0.,          1.,
    //            0.8,        -0.6,         0.,
    //            0.85749293,  0.51449576, -0.;
    // b[0]    << 0.24748737, 0.20343337, 0.1940285,  0.1,    0.1,    0.12, 0.12862394;
    // A[0] = Eigen::MatrixXd::Zero(5, 3);
    // b[0] = Eigen::VectorXd::Zero(5);
    // A[0]    << -0.806405,   -0.59136366, -0.,
    //            0.,         -0.,         -1.,
    //            0.,          0.,          1.,
    //            -0.,          1.,         -0., 
    //             0.806405,   -0.59136366,  0.;        
    // b[0]    << 0.08870455, 0.1 ,       0.1 ,       0.15 ,      0.08870455;


    for (int k = 0; k < N; ++k) {
        qcost = std::make_shared<costs::QuadraticCost>(costs::QuadraticCost::LQRCost(Q, R, xref[k], uref));
        prob.SetCostFunction(qcost, k);
    }

    qterm = std::make_shared<costs::QuadraticCost>(costs::QuadraticCost::LQRCost(Qf, R * 0, xf, uref, true));
    prob.SetCostFunction(qterm, N);

    //Dynamics
    for (int k = 0; k < N; ++k) {
        prob.SetDynamics(std::make_shared<ModelType>(model), k);
    }

    // Constraints
    for (int k = 0; k < N; ++k) {
        prob.SetConstraint(std::make_shared<constraints::ControlBound>(lb_v, ub_v), k);
    }

    int freeregionID = 0;
    for (int k = 1; k <= N; ++k) {
        prob.SetConstraint(std::make_shared<constraints::SDPConstraint>("envname", A[0], b[0], F[0], g[0], c[0]), k);
    }

    prob.SetConstraint(std::make_shared<constraints::GoalConstraint>(xf), N);

    // Initial State
    prob.SetInitialState(x0);


    return prob;
}

}
}