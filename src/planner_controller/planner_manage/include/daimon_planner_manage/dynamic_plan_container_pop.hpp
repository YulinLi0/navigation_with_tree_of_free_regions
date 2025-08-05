#ifndef _DYNAMIC_PLAN_CONTAINER_H_
#define _DYNAMIC_PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>
#include "daimon_traj_planner/bspline/uniform_bspline.h"
#include "daimon_traj_planner/polynomial/polynomial_traj.h"

const double PI = 3.14159265358979323846;

using std::vector;

class GlobalData
{
private:
    double start_time_        = 0.0;
    double dur_time_          = 0.0;
    double last_process_time_ = 0.0;            // use to calculate local traj

    void discretePath()
    /*
     * @brief: regenerate landmarks from global path
     */
    {
        global_path_.clear();
        double delta_t = 0.01;

        // init id
        int id = 0;

        // check from start to end
        for (double t = tm_start_ + delta_t; t < tm_end_; t += delta_t)
        {
            // add to global path
            Eigen::Vector3d pos_t = getPosition(t);
            global_path_.push_back(pos_t);
        }

        // add end_pos to global path
        global_path_.push_back(end_pos_);
    }


public:
    UniformBspline pos_traj_, vel_traj_, acc_traj_;
    double tm_start_, tm_end_;
    double end_roll_, end_pitch_, end_yaw_;
    Eigen::Vector3d start_pos_, end_pos_;

    std::vector<Eigen::Vector3d> global_path_;

    GlobalData(){};
    ~GlobalData(){};

    void setGlobalPath(const std::vector<Eigen::Vector3d> &point_sets)
    {
        global_path_ = point_sets;
    }

    Eigen::Vector3d getStartPos()
    {
        return global_path_[0];
    }
    void getClosetIndex(Eigen::Vector3d &pos, int &index){
        double min_dist = 1e10, dist;
        for (int i = 0; i < global_path_.size(); i++)
        {
            dist = (pos(0) - global_path_[i](0)) * (pos(0) - global_path_[i](0)) + (pos(1) - global_path_[i](1)) * (pos(1) - global_path_[i](1)) + (pos(2) - global_path_[i](2)) * (pos(2) - global_path_[i](2));
            if (dist < min_dist)
            {
                min_dist = dist;
                index = i;
            }
        }
    }

    Eigen::Vector3d getPosition(double t)
    {
        return pos_traj_.evaluateDeBoor(t);
    }

void getGoalRPY(double &roll, double &pitch, double &yaw){
    roll = end_roll_;
    pitch = end_pitch_;
    yaw = end_yaw_;
}

    void setGoalRPY(double roll, double pitch, double yaw){
        end_roll_ = roll;
        end_pitch_ = pitch;
        end_yaw_ = yaw;
    }

    Eigen::Vector3d getVelocity(double t)
    {
        return vel_traj_.evaluateDeBoor(t);
    }

    Eigen::Vector3d getAcceleration(double t)
    {
        return acc_traj_.evaluateDeBoor(t);
    }

    // double calculateCurveRadius(Eigen::Vector2d vel, Eigen::Vector2d acc)
    // {
    //     double rou = std::pow(vel.norm(), 3) / (vel(0) * acc(1) - vel(1) * acc(0));
    //     return std::abs(rou);
    // }

    // double calculateRotationVelocity(Eigen::Vector2d vel, Eigen::Vector2d acc)
    // {
    //     double a_norm = (vel(0) * acc(1) - vel(1) * acc(0)) / vel.norm();
    //     double w = a_norm / vel.norm();
    //     return w;
    // }

    void resetData(const UniformBspline &pos_traj)
    {
        pos_traj_ = pos_traj;
        vel_traj_ = pos_traj_.getDerivative();
        acc_traj_ = vel_traj_.getDerivative();

        // assign timespan to tm_start_ & tm_end_
        pos_traj_.getTimeSpan(tm_start_, tm_end_);
        dur_time_ = tm_end_ - tm_start_;

        // get start & end points
        last_process_time_ = start_time_ = tm_start_;
        start_pos_ = getPosition(tm_start_);
        end_pos_ = getPosition(tm_end_);

        // reset landmarks according to pos_traj
        discretePath();
    }

    double getStartTime()
    {
        return start_time_;
    }

    double getDurTime()
    {
        return dur_time_;
    }

    double getEndTime()
    {
        return start_time_ + dur_time_;
    }

    void setLastProcessTime(const double &time)
    {
        last_process_time_ = time;
    }

    double getLastProcessTime()
    {
        return last_process_time_;
    }

    std::vector<Eigen::Vector3d> getGlobalPath()
    {
        return global_path_;
    }

    std::vector<Eigen::Vector3d> getControlPoints()
    {
        Eigen::MatrixXd control_pts_matrix = pos_traj_.get_control_points(); // (dim,cols)

        std::vector<Eigen::Vector3d> ctrl_pts;
        ctrl_pts.clear();
        for (int i = 0; i < control_pts_matrix.cols(); i++)
        {
            ctrl_pts.push_back(control_pts_matrix.col(i));
        }

        return ctrl_pts;
    }
};

struct LocalTrajData
{
    /* info of generated traj */
    double start_time_;
    Eigen::Vector3d start_pos_;
    vector<Eigen::Vector3d> local_path_;
    vector<Eigen::Vector3d> local_vel_;
    vector<Eigen::Vector3d> rpy_list_;
    vector<double> yaw_;
    vector<double> t_list_;


    LocalTrajData()
    {
        reset();
    }

    bool isEmpty()
    {
        return local_path_.empty();
    }

    void reset()
    {
        start_time_ = 0.0;
        local_path_.clear();
        local_vel_.clear();
        yaw_.clear();
        t_list_.clear();
    }

    // get the index which is the first element that has distance d from current pos 
    void getDistanceIndex(Eigen::Vector3d &pos, int &index, double distance, int &start_index){
        int i = local_path_.size() - 1;
        double d = 0.0;
        while (i > start_index && d > distance){
            d = (pos(0) - local_path_[i](0)) * (pos(0) - local_path_[i](0)) + (pos(1) - local_path_[i](1)) * (pos(1) - local_path_[i](1)) + (pos(2) - local_path_[i](2)) * (pos(2) - local_path_[i](2));
            index = i;
            i++;
        }
    }

    void getNearestPointIndex(Eigen::Vector3d &pos, int &index)
    {
        double min_dist = 1e10, dist;

        for (int i = 0; i < local_path_.size(); i++)
        {
            dist = (pos(0) - local_path_[i](0)) * (pos(0) - local_path_[i](0)) + (pos(1) - local_path_[i](1)) * (pos(1) - local_path_[i](1)) + (pos(2) - local_path_[i](2)) * (pos(2) - local_path_[i](2));
            if (dist < min_dist)
            {
                min_dist = dist;
                index = i;
            }
        }
    }
    void resetData(const vector<Eigen::Vector3d> &local_path, const vector<Eigen::Vector3d> &local_vel,
                   const vector<Eigen::Vector3d> &rpy_list, const vector<double> &t_list,
                   double start_time)
    {
        reset();
        local_path_ = local_path;
        local_vel_ = local_vel;
        rpy_list_ = rpy_list;
        t_list_ = t_list;
        start_time_ = start_time;
    }

    void resetDataPOP(const vector<Eigen::Vector3d> &local_path, const vector<Eigen::Vector3d> &rpy_list, 
                    Eigen::Vector3d &start_pos)
    {
        reset();
        local_path_ = local_path;
        rpy_list_ = rpy_list;
        start_pos_ = start_pos;

    }

    void getRefTraj(vector<Eigen::Vector3d> &traj_list)
    {
        traj_list.clear();
        for (int i = 0; i < local_path_.size(); i++)
        {
            Eigen::Vector3d pt;
            // pt << local_path_[i](0), local_path_[i](1), yaw_[i];
            pt << local_path_[i](0), local_path_[i](1), local_path_[i](2);
            traj_list.push_back(pt);
        }
    }

    double nearestPointTime(const Eigen::Vector3d &pos, int &ind)
    {
        double min_dist = 1e10, dist;

        for (int i = 0; i < local_path_.size(); i++)
        {
            dist = (pos(0) - local_path_[i](0)) * (pos(0) - local_path_[i](0)) + (pos(1) - local_path_[i](1)) * (pos(1) - local_path_[i](1)) + (pos(2) - local_path_[i](2)) * (pos(2) - local_path_[i](2));
            if (dist < min_dist)
            {
                min_dist = dist;
                ind = i;
            }
        }
        return t_list_[ind];
    }
};

struct PlanParameters
{
    /* planning algorithm parameters */

    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist_;                 // distance between adjacient B-spline control points
    double global_pt_dist_;               // distance between adjacient global path points
    // double time_resolution_;                // for select smaple from timed_astar path

    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    bool use_distinctive_trajs_;
    double local_time_horizon_;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
};

#endif