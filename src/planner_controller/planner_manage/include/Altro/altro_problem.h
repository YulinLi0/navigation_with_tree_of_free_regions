#pragma once

#include <fmt/format.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "altro/eigentypes.hpp"
#include "altro/problem/problem.hpp"

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/augmented_lagrangian/al_problem.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "altro/problem/discretized_model.hpp"
#include "Altro/altro_constraints.h"
#include "Altro/altro_cost.h"
#include "Altro/altro_model.h"
#include "Altro/altro_problem.h"
// #include "Altro_control/SDP_constraints.hpp"
#include <decomp_ros_utils/data_ros_utils.h>

namespace altro {
namespace problems {

    class QuadrupedalProblem {
        public:
            static constexpr int NStates = 6;
            static constexpr int NControls = 6;

            QuadrupedalProblem();
            using ModelType = altro::problem::DiscretizedModel<altro::models::Quadrupedal>;
	        using CostFunType = altro::costs::QuadraticCost;

            //Problem Data
            static constexpr int HEAP = Eigen::Dynamic;
            const int n = NStates;
            const int m = NControls;
            ModelType model = ModelType(altro::models::Quadrupedal());
            int N = 10;

            //build a 6x6 identity matrix
            Eigen::MatrixXd Q = Eigen::VectorXd::Constant(NStates, 1e-2).asDiagonal();
            Eigen::MatrixXd R = Eigen::VectorXd::Constant(NControls, 1e-2).asDiagonal();
            Eigen::MatrixXd Qf = Eigen::VectorXd::Constant(NStates, 100).asDiagonal();
            Eigen::VectorXd xf = Eigen::VectorXd::Zero(NStates);
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(NStates);
            Eigen::VectorXd u0 = Eigen::VectorXd::Constant(NControls, 0.0);
            Eigen::VectorXd uref = Eigen::VectorXd::Zero(NControls);
            std::vector<Eigen::VectorXd> xref;
            // std::shared_ptr<examples::QuadraticCost> qcostt;
            std::shared_ptr<costs::QuadraticCost> qcost;
            std::shared_ptr<costs::QuadraticCost> qterm;
            
            // double q_bnd = 0.05; 
            // std::vector<double> lb;
            // std::vector<double> ub;

            double v_bnd = 0.6; 
            std::vector<double> lb_v;
            std::vector<double> ub_v;

            double vyaw_bnd = 0.8; 
            std::vector<double> lb_vyaw;
            std::vector<double> ub_vyaw;

            altro::problem::Problem MakeProblem(const bool add_constraints = true);

            template <int n_size = NStates, int m_size = NControls>
            altro::Trajectory<n_size, m_size> InitialTrajectory();

            template <int n_size = NStates, int m_size = NControls>
            altro::ilqr::iLQR<n_size, m_size> MakeSolver();

            template <int n_size = NStates, int m_size = NControls>
            altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver();

            float GetTimeStep() const { return tf / N; }

            void setRobotshape(const std::vector<LinearConstraint3D>& LinearConstraints, const std::vector<Vec3f>& freeregionCenters)
            {
                //get the length of the LinearConstraints
                int n = LinearConstraints.size();
// std::cout << "n: " << n << std::endl;
                for (int k = 0; k < n; ++k) {
                    F[k] = LinearConstraints[k].A_;
                    g[k] = LinearConstraints[k].b_; 
                    c[k] = freeregionCenters[k];
                }
// std::cout << "Set Down" << std::endl;
            }
            float tf;
            private:

            std::vector<Eigen::MatrixXd> A;
            std::vector<Eigen::VectorXd> b;
            std::vector<Eigen::VectorXd> q;
            std::vector<Eigen::MatrixXd> F;
            std::vector<Eigen::VectorXd> g;
            std::vector<Eigen::VectorXd> c;
    };
    template <int n_size, int m_size>
    altro::Trajectory<n_size, m_size> QuadrupedalProblem::InitialTrajectory() {
        altro::Trajectory<n_size, m_size> Z(n, m, N);
        for (int k = 0; k < N; ++k) {
            Z.Control(k) = u0;
        }
        float h = GetTimeStep(); 
        Z.SetUniformStep(h);
        return Z;
    }

    template <int n_size, int m_size>
    altro::ilqr::iLQR<n_size, m_size> QuadrupedalProblem::MakeSolver()
    {
        altro::problem::Problem prob = MakeProblem();
        prob = altro::augmented_lagrangian::BuildAugLagProblem<n_size, m_size>(prob);

        altro::ilqr::iLQR<n_size, m_size> solver(prob);

        std::shared_ptr<altro::Trajectory<n_size, m_size>> traj_ptr =
            std::make_shared<altro::Trajectory<n_size, m_size>>(InitialTrajectory<n_size, m_size>());

        solver.SetTrajectory(traj_ptr);
        solver.Rollout();
        return solver;
    }

    template <int n_size, int m_size>
    altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
    QuadrupedalProblem::MakeALSolver() {
        altro::problem::Problem prob = MakeProblem(true);
        altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
        solver_al.SetTrajectory(
            std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
        solver_al.GetiLQRSolver().Rollout();
        return solver_al;
    }
}
}