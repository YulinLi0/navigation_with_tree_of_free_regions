#include "coptcpp_pch.h"
#include "./SDPsolver/SDPsolver.h"
#include <chrono>
using namespace std;

int SDPsolver::buildSDP(const string &envname, Eigen::MatrixXd &A, Eigen::VectorXd &b)
{

    return 0;
}

// envname: name of the environment
// A: matrix A of robot in robot's frame
// b: vector b of robot in robot's frame
// F: matrix F of free_region in world's frame
// g: vector g of free_region in world's frame
// q: vector q of robot's pose in world's frame
// c: vector c of free_region's center in world's frame
// return: vector of gradient of the cofficient subject to the q and alpha
Eigen::VectorXd SDPsolver::solveSDP(const string &envname, Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::MatrixXd &F, Eigen::VectorXd &g, Eigen::VectorXd &q, Eigen::VectorXd &c)
{
    // Envr env;
    // model.eset(new Model(env.CreateModel(envname.c_str())));
    
    model.SetIntParam(COPT_INTPARAM_LOGGING, 0);
    model.SetIntParam(COPT_INTPARAM_LOGTOCONSOLE, 0);
    //get the size of the matrix g
    int n_robot_faces = b.size();
    int n_free_faces = g.size();
    
    //q is x y z roll pitch yaw
    //frome the q, to get the rotation matrix and the translation vector from world frame to robot frame
    //the rotation matrix is R
    //the translation vector is p
    Eigen::Vector3d w_p_free;
    w_p_free << c[0], c[1], c[2];
    Eigen::Vector3d w_p_robot;
    w_p_robot << q[0], q[1], q[2];
    Eigen::Matrix3d w_R_free;
    w_R_free = Eigen::AngleAxisd(c[5], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(c[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(c[3], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d w_R_robot;
    w_R_robot = Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d free_R_robot;
    free_R_robot = w_R_free.transpose() * w_R_robot;
    Eigen::Vector3d free_p_robot;
    free_p_robot = w_R_free.transpose() * (w_p_robot - w_p_free);

    //transform the F and g from world frame to free region's frame
    // freeregion_R_w = w_R_freeregion.transpose
    // freeregion_p_w = -transpose(w_R_freeregion) * w_p_freeregion
    // As = A * freeregion_R_w.transpose
    // bs = b - A*(-transpose(freeregion_R_w) * freeregion_p_w)
    Eigen::MatrixXd F_free = F * w_R_free;
    Eigen::VectorXd g_free = g - F * (-w_R_free * (-w_R_free.transpose() * w_p_free));

    //show F_free and g_free
// std::cout << "F_free: " << F_free << std::endl;
// std::cout << "g_free: " << g_free.transpose() << std::endl;



    Var alpha = model.AddVar(0.0, INFINITY, 0.0, COPT_CONTINUOUS, "alpha");
    VarArray vars;
    // vars is sigmas in paper
    // sigma[i][j] flatten into sigma[i*j], 
    // i is the number of free faces, 
    // j is the number of sigma in each robot face
    for (int i = 0; i < n_free_faces; i++)
    {
        for (int j = 0; j < 1 + n_robot_faces; j++)
        {
            string var_name = "X" + to_string(i) + "_" + to_string(j);
            vars.PushBack(model.AddVar(0.0, INFINITY, 0.0, COPT_CONTINUOUS, var_name.c_str()));
        }
    }
     
    model.SetObjective(1.0*alpha, COPT_MINIMIZE);

    //add constraints
    // the cofficient of free region in robot's frame should be 
    // equal to the cofficient of robot relaxed by QMOD
    //constraint[i][j] flatten into constraint[i*j],
    // i is the number of free faces,
    // j is the number of constraints in each free face
    
    // -F*R 
    // Eigen::MatrixXd F_body =  * free_R_robot;
    //alpha*g-F*p
    // Eigen::Vector3d g_body = alpha * g - F_body * free_p_robot;
    Eigen::MatrixXd F_body = -F_free * free_R_robot;

    for (int i = 0; i < n_free_faces; i++)
    {
        // j = 0
        Expr lhs;
        Expr rhs;
        lhs = alpha * g_free[i] - F_free.row(i) * free_p_robot;
        rhs = vars[i*(n_robot_faces+1)];
        for (int k = 0; k < n_robot_faces; k++)
        {
            rhs += b[k] * vars[i*(n_robot_faces+1)+k+1];
        }
        // string constraint_name_pos = "constraint_" + to_string(i) + "_" + to_string(0) + "pos";
        string constraint_name = "constraint_" + to_string(i) + "_" + to_string(0);
        model.AddConstr(lhs == rhs, constraint_name.c_str());
        // model.AddConstr(lhs - rhs >=0.0, constraint_name_nag.c_str());

        // j = 1, 2, 3
        for (int j = 0; j < 3; j++) // never mind the magic number 3
        {
            Expr lhs;   //cofficient of free region in robot's frame
            Expr rhs;   //cofficient of robot relaxed by QMOD
            lhs = F_body(i,j);
            rhs = 0.0;
            for (int k = 0; k < n_robot_faces; k++)
            {
                rhs += -A(k,j) * vars[i*(n_robot_faces+1)+k+1];
            }
            // string constraint_name_pos = "constraint_" + to_string(i) + "_" + to_string(j+1)+"pos";
            string constraint_name = "constraint_" + to_string(i) + "_" + to_string(j+1);
            model.AddConstr(lhs==rhs, constraint_name.c_str());
            // model.AddConstr(lhs-rhs>=0.0, constraint_name_nag.c_str());
        } 
    }

    //record the time here
    // auto start = std::chrono::high_resolution_clock::now();
    // for (int i=0;i<10;i++)
    model.Solve();
    //end of record the time
    // auto finish = std::chrono::high_resolution_clock::now();
    //print time in ms
    // std::chrono::duration<double> elapsed = finish - start;
    //change the cnt into ms
    // std::cout << "Time: " << elapsed.count() << " s" << std::endl;

    ConstrArray all_constrs = model.GetConstrs();
// std::cout << "all_constrs.Size(): " << all_constrs.Size() << std::endl;
    Eigen::VectorXd dual_value(all_constrs.Size());
    for (int i = 0; i < all_constrs.Size(); i++)
    {
        //get the dual value of the constraint
        // std::cout << "1111" << std::endl;
        dual_value[i] = all_constrs[i].Get(COPT_DBLINFO_DUAL);
        // std::cout << all_constrs[i].GetName() <<"dual_value: " << dual_value[i] << std::endl;
    }

    //calcuate the gradient of the cofficient  subject to the q
    gradient.setZero();
    for (int i = 0; i < n_free_faces; i++)
    {
        //Ax*dual_0*r11 + Ay*dual_0*r12 + Az*dual_0*r13
        gradient(0) += (F_free(i,0)) * dual_value[i*4+0] * w_R_free(0, 0) + (F_free(i,1)) * dual_value[i*4+0] * w_R_free(0, 1) + (F_free(i,2)) * dual_value[i*4+0]* w_R_free(0, 2);
        //Ax*dual_0*r21 + Ay*dual_0*r22 + Az*dual_0*r23
        gradient(1) += (F_free(i,0)) * dual_value[i*4+0] * w_R_free(1, 0) + (F_free(i,1)) * dual_value[i*4+0] * w_R_free(1, 1) + (F_free(i,2)) * dual_value[i*4+0]* w_R_free(1, 2);
        //Ax*dual_0*r31 + Ay*dual_0*r32 + Az*dual_0*r33 
        gradient(2) += (F_free(i,0)) * dual_value[i*4+0] * w_R_free(2, 0) + (F_free(i,1)) * dual_value[i*4+0] * w_R_free(2, 1) + (F_free(i,2)) * dual_value[i*4+0]* w_R_free(2, 2);
        //Ax*dual_2*(r11*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - r21*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r31*cos(pitch)*cos(roll)) + 
        //Ay*dual_2*(r12*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - r22*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r32*cos(pitch)*cos(roll)) + 
        //Az*dual_2*(r13*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - r23*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r33*cos(pitch)*cos(roll)) - 
        //Ax*dual_3*(r21*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - r11*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + r31*cos(pitch)*sin(roll)) - 
        //Ay*dual_3*(r22*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - r12*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + r32*cos(pitch)*sin(roll)) - 
        //Az*dual_3*(r23*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - r13*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + r33*cos(pitch)*sin(roll))
        gradient(3) += (F_free(i,0)) * dual_value[i*4+2] * (w_R_free(0, 0) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3])) - w_R_free(1, 0) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 0) * cos(q[3])*cos(q[4])) + 
                       (F_free(i,1)) * dual_value[i*4+2] * (w_R_free(0, 1) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3])) - w_R_free(1, 1) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 1) * cos(q[3])*cos(q[4])) + 
                       (F_free(i,2)) * dual_value[i*4+2] * (w_R_free(0, 2) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3])) - w_R_free(1, 2) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 2) * cos(q[3])*cos(q[4])) - 
                       (F_free(i,0)) * dual_value[i*4+3] * (w_R_free(1, 0) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) - w_R_free(0, 0) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 0) * cos(q[3])*sin(q[4])) - 
                       (F_free(i,1)) * dual_value[i*4+3] * (w_R_free(1, 1) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) - w_R_free(0, 1) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 1) * cos(q[3])*sin(q[4])) -
                       (F_free(i,2)) * dual_value[i*4+3] * (w_R_free(1, 2) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) - w_R_free(0, 2) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5])) + w_R_free(2, 2) * cos(q[3])*sin(q[4]));
        //Ax*dual_3*(r11*cos(pitch)*cos(roll)*cos(yaw) - r31*cos(roll)*sin(pitch) + r21*cos(pitch)*cos(roll)*sin(yaw)) - 
        //Ay*dual_1*(r32*cos(pitch) + r12*cos(yaw)*sin(pitch) + r22*sin(pitch)*sin(yaw)) - 
        //Az*dual_1*(r33*cos(pitch) + r13*cos(yaw)*sin(pitch) + r23*sin(pitch)*sin(yaw)) - 
        //Ax*dual_1*(r31*cos(pitch) + r11*cos(yaw)*sin(pitch) + r21*sin(pitch)*sin(yaw)) + 
        //Ay*dual_3*(r12*cos(pitch)*cos(roll)*cos(yaw) - r32*cos(roll)*sin(pitch) + r22*cos(pitch)*cos(roll)*sin(yaw)) + 
        //Az*dual_3*(r13*cos(pitch)*cos(roll)*cos(yaw) - r33*cos(roll)*sin(pitch) + r23*cos(pitch)*cos(roll)*sin(yaw)) + 
        //Ax*dual_2*(r11*cos(pitch)*cos(yaw)*sin(roll) - r31*sin(pitch)*sin(roll) + r21*cos(pitch)*sin(roll)*sin(yaw)) + 
        //Ay*dual_2*(r12*cos(pitch)*cos(yaw)*sin(roll) - r32*sin(pitch)*sin(roll) + r22*cos(pitch)*sin(roll)*sin(yaw)) + 
        //Az*dual_2*(r13*cos(pitch)*cos(yaw)*sin(roll) - r33*sin(pitch)*sin(roll) + r23*cos(pitch)*sin(roll)*sin(yaw))
        gradient(4) += (F_free(i,0)) * dual_value[i*4+3] * (w_R_free(0, 0) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 0) * cos(q[3])*sin(q[4]) + w_R_free(1, 0) * cos(q[4])*cos(q[3])*sin(q[5])) - 
                       (F_free(i,1)) * dual_value[i*4+1] * (w_R_free(2, 1) * cos(q[4]) + w_R_free(0, 1) * cos(q[5])*sin(q[4]) + w_R_free(1, 1) * sin(q[4])*sin(q[5])) - 
                       (F_free(i,2)) * dual_value[i*4+1] * (w_R_free(2, 2) * cos(q[4]) + w_R_free(0, 2) * cos(q[5])*sin(q[4]) + w_R_free(1, 2) * sin(q[4])*sin(q[5])) - 
                       (F_free(i,0)) * dual_value[i*4+1] * (w_R_free(2, 0) * cos(q[4]) + w_R_free(0, 0) * cos(q[5])*sin(q[4]) + w_R_free(1, 0) * sin(q[4])*sin(q[5])) + 
                       (F_free(i,1)) * dual_value[i*4+3] * (w_R_free(0, 1) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 1) * cos(q[3])*sin(q[4]) + w_R_free(1, 1) * cos(q[4])*cos(q[3])*sin(q[5])) + 
                       (F_free(i,2)) * dual_value[i*4+3] * (w_R_free(0, 2) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 2) * cos(q[3])*sin(q[4]) + w_R_free(1, 2) * cos(q[4])*cos(q[3])*sin(q[5])) + 
                       (F_free(i,0)) * dual_value[i*4+2] * (w_R_free(0, 0) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 0) * cos(q[3])*sin(q[4]) + w_R_free(1, 0) * cos(q[4])*cos(q[3])*sin(q[5])) +
                       (F_free(i,1)) * dual_value[i*4+2] * (w_R_free(0, 1) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 1) * cos(q[3])*sin(q[4]) + w_R_free(1, 1) * cos(q[4])*cos(q[3])*sin(q[5])) +
                       (F_free(i,2)) * dual_value[i*4+2] * (w_R_free(0, 2) * cos(q[4])*cos(q[3])*cos(q[5]) - w_R_free(2, 2) * cos(q[3])*sin(q[4]) + w_R_free(1, 2) * cos(q[4])*cos(q[3])*sin(q[5]));
        //Ax*dual_3*(r11*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r21*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))) - 
        //Ax*dual_2*(r11*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + r21*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))) - 
        //Ay*dual_2*(r12*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + r22*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))) + 
        //Ay*dual_3*(r12*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r22*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))) - 
        //Az*dual_2*(r13*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + r23*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))) + 
        //Az*dual_3*(r13*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + r23*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))) + 
        //Ax*dual_1*(r21*cos(pitch)*cos(yaw) - r11*cos(pitch)*sin(yaw)) + 
        //Ay*dual_1*(r22*cos(pitch)*cos(yaw) - r12*cos(pitch)*sin(yaw)) + 
        //Az*dual_1*(r23*cos(pitch)*cos(yaw) - r13*cos(pitch)*sin(yaw))
        gradient(5) += (F_free(i,0)) * dual_value[i*4+3] * (w_R_free(0, 0) * (cos(q[5])*sin(q[3]) - cos(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 0) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3]))) - 
                       (F_free(i,0)) * dual_value[i*4+2] * (w_R_free(0, 0) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 0) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5]))) - 
                       (F_free(i,1)) * dual_value[i*4+2] * (w_R_free(0, 1) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 1) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5]))) - 
                       (F_free(i,1)) * dual_value[i*4+3] * (w_R_free(0, 1) * (cos(q[5])*sin(q[3]) - cos(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 1) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3]))) - 
                       (F_free(i,2)) * dual_value[i*4+2] * (w_R_free(0, 2) * (cos(q[4])*cos(q[5]) + sin(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 2) * (cos(q[5])*sin(q[4]) - cos(q[4])*sin(q[3])*sin(q[5]))) - 
                       (F_free(i,2)) * dual_value[i*4+3] * (w_R_free(0, 2) * (cos(q[5])*sin(q[3]) - cos(q[3])*sin(q[4])*sin(q[5])) + w_R_free(1, 2) * (sin(q[5])*sin(q[4]) + cos(q[5])*cos(q[4])*sin(q[3]))) - 
                       (F_free(i,0)) * dual_value[i*4+1] * (w_R_free(1, 0) * cos(q[4])*cos(q[3]) - w_R_free(0, 0) * cos(q[3]*sin(q[5]))) -
                       (F_free(i,1)) * dual_value[i*4+1] * (w_R_free(1, 1) * cos(q[4])*cos(q[3]) - w_R_free(0, 1) * cos(q[3]*sin(q[5]))) -
                       (F_free(i,2)) * dual_value[i*4+1] * (w_R_free(1, 2) * cos(q[4])*cos(q[3]) - w_R_free(0, 2) * cos(q[3]*sin(q[5])));
        // std::cout << "gradient: " << gradient.transpose() << std::endl;
    }

    
    Eigen::VectorXd vars_value(n_free_faces*(n_robot_faces+1));
    for (size_t i = 0; i < n_free_faces*(n_robot_faces+1); i++)
    {
      vars_value<< vars[i].Get(COPT_DBLINFO_VALUE);
    }
    
    //show vars_value
    // std::cout << "vars_value: " << vars_value.transpose() << std::endl;
    Eigen::VectorXd result(7);
    alpha_value = alpha.Get(COPT_DBLINFO_VALUE);
    result << gradient, alpha_value;
    // std::cout << "result: " << result.transpose() << std::endl;
    return result;
}