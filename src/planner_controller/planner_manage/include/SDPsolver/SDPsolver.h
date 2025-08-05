#pragma once

#include "coptcpp_pch.h"
#include "Eigen/Dense"

 
using namespace std;
//build a class called SDPsolver
class SDPsolver
{
    //public functions
public:
    //constructor initialize the env and model
    SDPsolver():model(env.CreateModel("sss")){gradient.resize(6);};
    //destructor
    ~SDPsolver() = default;
    // 
    int buildSDP(const string &envname,Eigen::MatrixXd &A, Eigen::VectorXd &b);
    Eigen::VectorXd solveSDP(const string &envname, Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::MatrixXd &F, Eigen::VectorXd &g, Eigen::VectorXd &q, Eigen::VectorXd &c);
    //return the gradient
    Eigen::VectorXd getGradient(){return gradient;}
    //return the alpha value
    double getAlpha(){
        // model_ptr->ResetAll();
        // env.Close();
        // model.ResetAll();
        model.Clear();
        return alpha_value;}
private:
    Envr env;
    Model model;
    //private variables
    // unique_ptr<Model> model_ptr;
    Eigen::VectorXd gradient;
    double alpha_value;;
    //end of class
};