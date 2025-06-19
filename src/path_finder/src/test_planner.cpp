/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "path_finder/rrt_sharp.h"
#include "path_finder/rrt_star.h"
#include "path_finder/rrt.h"
#include "path_finder/brrt.h"
#include "path_finder/brrt_star.h"
#include "path_finder/brrt_sample_gravity.h"
#include "path_finder/testcase.h"
#include "visualization/visualization.hpp"
#include "path_finder/sampler.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#define NUMBER_TEST_TIMES 1000
class TesterPathFinder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Timer execution_timer_;
    ros::ServiceClient rcv_glb_obs_client_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;
    shared_ptr<path_plan::RRTSharp> rrt_sharp_ptr_;
    shared_ptr<path_plan::RRTStar> rrt_star_ptr_;
    shared_ptr<path_plan::RRT> rrt_ptr_;
    shared_ptr<path_plan::BRRT> brrt_ptr_;
    shared_ptr<path_plan::BRRTStar> brrt_star_ptr_;
    shared_ptr<path_plan::BRRT_Optimize> brrt_optimize_ptr_;

    Eigen::Vector3d start_, goal_;

    bool run_rrt_, run_rrt_star_, run_rrt_sharp_;
    bool run_brrt_, run_brrt_star_, run_brrt_optimize_;
    // Implement for testing path planning algorithms
    BiasSampler sampler_;
    BRRTExperimentMultiAlgo *manager;

public:
    TesterPathFinder(const ros::NodeHandle &nh) : nh_(nh)
    {
        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(nh_);
        manager = new BRRTExperimentMultiAlgo(
            "/home/x/sampling-based-path-finding/brrt_input.json",
            "/home/x/sampling-based-path-finding/brrt_output");
        vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        rrt_sharp_ptr_ = std::make_shared<path_plan::RRTSharp>(nh_, env_ptr_);
        rrt_sharp_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_sharp_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_sharp_final_wpts");

        rrt_star_ptr_ = std::make_shared<path_plan::RRTStar>(nh_, env_ptr_);
        rrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_star_final_wpts");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("rrt_star_paths");

        rrt_ptr_ = std::make_shared<path_plan::RRT>(nh_, env_ptr_);
        rrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_final_wpts");

        brrt_ptr_ = std::make_shared<path_plan::BRRT>(nh_, env_ptr_);
        brrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_final_wpts");

        brrt_star_ptr_ = std::make_shared<path_plan::BRRTStar>(nh_, env_ptr_);
        brrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_star_final_wpts");

        brrt_optimize_ptr_ = std::make_shared<path_plan::BRRT_Optimize>(nh_, env_ptr_);
        brrt_optimize_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_optimize_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_optimize_final_wpts");

        goal_sub_ = nh_.subscribe("/goal", 1, &TesterPathFinder::goalCallback, this);
        execution_timer_ = nh_.createTimer(ros::Duration(1), &TesterPathFinder::executionCallback, this);
        rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

        start_.setZero();

        nh_.param("run_rrt", run_rrt_, true);
        nh_.param("run_rrt_star", run_rrt_star_, true);
        nh_.param("run_rrt_sharp", run_rrt_sharp_, true);
        nh_.param("run_brrt", run_brrt_, false);
        nh_.param("run_brrt_star", run_brrt_star_, false);
        nh_.param("run_brrt_optimize", run_brrt_optimize_, false);
    }
    ~TesterPathFinder()
    {
        delete manager;
    };

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = goal_msg->pose.position.z;
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);

        // BiasSampler sampler;
        // sampler.setSamplingRange(env_ptr_->getOrigin(), env_ptr_->getMapSize());
        // vector<Eigen::Vector3d> preserved_samples;
        // for (int i = 0; i < 5000; ++i)
        // {
        //     Eigen::Vector3d rand_sample;
        //     sampler.uniformSamplingOnce(rand_sample);
        //     preserved_samples.push_back(rand_sample);
        // }
        // rrt_ptr_->setPreserveSamples(preserved_samples);
        // rrt_star_ptr_->setPreserveSamples(preserved_samples);
        // rrt_sharp_ptr_->setPreserveSamples(preserved_samples);

        if (run_rrt_)
        {
            bool rrt_res = rrt_ptr_->plan(start_, goal_);
            if (rrt_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_final_wpts");
                vector<std::pair<double, double>> slns = rrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT] final path len: " << slns.back().first);
            }
        }

        if (run_rrt_star_)
        {
            bool rrt_star_res = rrt_star_ptr_->plan(start_, goal_);
            if (rrt_star_res)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_star_ptr_->getAllPaths();
                vis_ptr_->visualize_path_list(routes, "rrt_star_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = rrt_star_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_star_final_wpts");
                vector<std::pair<double, double>> slns = rrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT*] final path len: " << slns.back().first);
            }
        }

        if (run_rrt_sharp_)
        {
            bool rrt_sharp_res = rrt_sharp_ptr_->plan(start_, goal_);
            if (rrt_sharp_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_sharp_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_sharp_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_sharp_final_wpts");
                vector<std::pair<double, double>> slns = rrt_sharp_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT#] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_)
        {
            bool brrt_res = brrt_ptr_->plan(start_, goal_);
            if (brrt_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_final_wpts");
                vector<std::pair<double, double>> slns = brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_star_)
        {
            bool brrt_star_res = brrt_star_ptr_->plan(start_, goal_);
            if (brrt_star_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_star_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_star_final_wpts");
                vector<std::pair<double, double>> slns = brrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT*] final path len: " << slns.back().first);
            }
        }
        if (run_brrt_optimize_)
        {
            bool brrt_optimize_res = brrt_optimize_ptr_->plan(start_, goal_);
            if (brrt_optimize_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_optimize_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_optimize_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_optimize_final_wpts");
                vector<std::pair<double, double>> slns = brrt_optimize_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRTOpitmize*] final path len: " << slns.back().first);
            }
        }

        start_ = goal_;
    }
    void print_vector3d(std::string name, Eigen::Vector3d &p)
    {
        std::cout << name << " x: " << p[0] << " y: " << p[1] << " z: " << p[2] << std::endl;
    }
    Eigen::Vector3d get_sample_valid()
    {
        Eigen::Vector3d x_rand;
        sampler_.setSamplingRange(env_ptr_->getOrigin(), env_ptr_->getMapSize());
        sampler_.samplingOnce(x_rand);
        // samplingOnce(x_rand);
        while (!env_ptr_->isStateValid(x_rand))
        {
            sampler_.samplingOnce(x_rand);
        }
        return x_rand;
    }
    void executionCallback(const ros::TimerEvent &event)
    {

        if (!env_ptr_->mapValid())
        {
            ROS_INFO("no map rcved yet.");
            self_msgs_and_srvs::GlbObsRcv srv;
            if (!rcv_glb_obs_client_.call(srv))
                ROS_WARN("Failed to call service /pub_glb_obs");
        }
        else
        {
            execution_timer_.stop();
            ROS_WARN("Timer tick");
            experiment_test();
        }
    };
    void experiment_test()
    {
        for (int i = 0; i < NUMBER_TEST_TIMES; ++i)
        {


            Eigen::Vector3d goal = get_sample_valid();
            print_vector3d("Start Vector", start_);
            print_vector3d("Goal Vector", goal);
            // if (manager->current_run_index() > NUMBER_TEST_TIMES) {
            //
            //     return;
            // }
            const auto &input = manager->get_input();
            ROS_INFO("Running Test %d - Run #%d", input.trial, manager->current_run_index());

            std::map<std::string, AlgoResult> algo_outputs;

            // Simulate BRRT
            bool brrt_res = brrt_ptr_->plan(start_, goal_);
            if (brrt_res)
            {
                vector<std::pair<double, double>> slns = brrt_ptr_->getSolutions();
                int num_nodes = brrt_ptr_->get_valid_tree_node_nums();
                int num_iterations = brrt_ptr_->get_number_of_iteration();
                algo_outputs["BRRT"] = {true, slns.back().second, slns.back().first, num_nodes, num_iterations, start_, goal_};
            }
            else
            {
                algo_outputs["BRRT"] = {false, 0.0, 0.0, 0, 0, start_, goal_};
            }
            // Simulate BRRT Optimize
            brrt_optimize_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma);
            bool brrt_optimize_res = brrt_optimize_ptr_->plan(start_, goal_);
            if (brrt_optimize_res)
            {
                vector<std::pair<double, double>> slns = brrt_optimize_ptr_->getSolutions();
                int num_nodes = brrt_optimize_ptr_->get_valid_tree_node_nums();
                int num_iterations = brrt_optimize_ptr_->get_number_of_iteration();
                algo_outputs["BRRT_Optimize"] = {true, slns.back().second, slns.back().first, num_nodes, num_iterations, start_, goal_};
            }
            else
            {
                algo_outputs["BRRT_Optimize"] = {false, 0.0, 0.0, 0, 0, start_, goal_};
            }
            start_ = goal;

            manager->store_output_for_run(algo_outputs);
        }
        manager->save_json();
        ROS_INFO("Completed all runs.");
        ros::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_path_finder_node");
    ros::NodeHandle nh("~");

    TesterPathFinder tester(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}