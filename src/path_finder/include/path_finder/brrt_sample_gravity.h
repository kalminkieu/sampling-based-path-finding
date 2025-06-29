/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com), Longji Yin (ljyin6038@163.com )
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
#ifndef BRRT_OPTIMIZE_H
#define BRRT_OPTIMIZE_H

#include "occ_grid/occ_map.h"
#include "visualization/visualization.hpp"
#include "sampler.h"
#include "node.h"
#include "kdtree.h"

#include <ros/ros.h>
#include <utility>
#include <queue>
#include <algorithm>
namespace path_plan
{
  class BRRT_Optimize
  {
  public:
    BRRT_Optimize() {};
    BRRT_Optimize(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapPtr) : nh_(nh), map_ptr_(mapPtr)
    {
      nh_.param("BRRT/steer_length", steer_length_, 0.0);
      nh_.param("BRRT/search_time", search_time_, 0.0);
      nh_.param("BRRT/max_tree_node_nums", max_tree_node_nums_, 0);

      nh_.param("BRRT_Optimize/p1", brrt_optimize_p1_, 0.8);
      nh_.param("BRRT_Optimize/u_p", brrt_optimize_u_p, 2.0);
      nh_.param("BRRT_Optimize/step", brrt_optimize_step_, 0.1);

      nh_.param("BRRT_Optimize/alpha", brrt_optimize_alpha_, 0.5);
      nh_.param("BRRT_Optimize/beta", brrt_optimize_beta_, 0.3);
      nh_.param("BRRT_Optimize/gamma", brrt_optimize_gamma_, 0.5);
      nh_.param("BRRT_Optimize/max_iteration", max_iteration_, 0);
      nh_.param("BRRT_Optimize/enable2d", brrt_enable_2d, true);

      ROS_WARN_STREAM("[BRRT_Optimize] param: steer_length: " << steer_length_);
      ROS_WARN_STREAM("[BRRT_Optimize] param: search_time: " << search_time_);
      ROS_WARN_STREAM("[BRRT_Optimize] param: max_tree_node_nums: " << max_tree_node_nums_);

      sampler_.setSamplingRange(mapPtr->getOrigin(), mapPtr->getMapSize());

      valid_tree_node_nums_ = 0;
      nodes_pool_.resize(max_tree_node_nums_);
      for (int i = 0; i < max_tree_node_nums_; ++i)
      {
        nodes_pool_[i] = new TreeNode;
      }
    }
    ~BRRT_Optimize() {};

    bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      reset();
      if (!map_ptr_->isStateValid(s))
      {
        ROS_ERROR("[BRRT_Optimize]: Start pos collide or out of bound");
        return false;
      }
      if (!map_ptr_->isStateValid(g))
      {
        ROS_ERROR("[BRRT_Optimize]: Goal pos collide or out of bound");
        return false;
      }
      /* construct start and goal nodes */
      start_node_ = nodes_pool_[1];
      start_node_->x = s;
      start_node_->cost_from_start = 0.0;
      goal_node_ = nodes_pool_[0];
      goal_node_->x = g;
      goal_node_->cost_from_start = 0.0; // important
      valid_tree_node_nums_ = 2;         // put start and goal in tree

      vis_ptr_->visualize_a_ball(s, 0.3, "start", visualization::Color::pink);
      vis_ptr_->visualize_a_ball(g, 0.3, "goal", visualization::Color::steelblue);

      ROS_INFO("[BRRT_Optimize]: BRRT starts planning a path");
      return brrt_optimize(s, g);
    }

    vector<Eigen::Vector3d> getPath()
    {
      return final_path_;
    }

    vector<vector<Eigen::Vector3d>> getAllPaths()
    {
      return path_list_;
    }

    vector<std::pair<double, double>> getSolutions()
    {
      return solution_cost_time_pair_list_;
    }
    void set_heuristic_param(double p1, double u_p, double alpha, double beta, double gamma)
    {
      brrt_optimize_p1_ = p1;
      brrt_optimize_u_p = u_p;
      brrt_optimize_alpha_ = alpha;
      brrt_optimize_beta_ = beta;
      brrt_optimize_gamma_ = gamma;
    }
    void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
    {
      vis_ptr_ = visPtr;
    };
    int get_number_of_iteration(){
      return number_of_iterations_;
    }
    int get_valid_tree_node_nums()
    {
      return valid_tree_node_nums_;
    }
  private:
    // nodehandle params
    ros::NodeHandle nh_;

    BiasSampler sampler_;
    double brrt_optimize_p1_;
    double brrt_optimize_u_p;
    double brrt_optimize_step_;
    double brrt_optimize_alpha_;
    double brrt_optimize_beta_;
    double brrt_optimize_gamma_;
    int max_iteration_;
    double steer_length_;
    double search_time_;
    int max_tree_node_nums_;
    int number_of_iterations_;
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;
    bool brrt_enable_2d;

    double cost_best_;
    std::vector<TreeNode *> nodes_pool_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector3d> final_path_;
    vector<vector<Eigen::Vector3d>> path_list_;
    vector<std::pair<double, double>> solution_cost_time_pair_list_;

    // environment
    env::OccMap::Ptr map_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    void reset()
    {
      final_path_.clear();
      path_list_.clear();
      cost_best_ = DBL_MAX;

      solution_cost_time_pair_list_.clear();
      for (int i = 0; i < valid_tree_node_nums_; i++)
      {
        nodes_pool_[i]->parent = nullptr;
        nodes_pool_[i]->children.clear();
      }
      valid_tree_node_nums_ = 0;
    }

    double calDist(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      return (p1 - p2).norm();
    }

    RRTNode3DPtr addTreeNode(RRTNode3DPtr &parent, const Eigen::Vector3d &state,
                             const double &cost_from_start, const double &cost_from_parent)
    {
      RRTNode3DPtr new_node_ptr = nodes_pool_[valid_tree_node_nums_];
      valid_tree_node_nums_++;
      new_node_ptr->parent = parent;
      parent->children.push_back(new_node_ptr);
      new_node_ptr->x = state;
      new_node_ptr->cost_from_start = cost_from_start;
      new_node_ptr->cost_from_parent = cost_from_parent;
      return new_node_ptr;
    }

    void changeNodeParent(RRTNode3DPtr &node, RRTNode3DPtr &parent, const double &cost_from_parent)
    {
      if (node->parent)
        node->parent->children.remove(node); // DON'T FORGET THIS, remove it form its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descedants, change the cost_from_start and tau_from_start;
      RRTNode3DPtr descendant(node);
      std::queue<RRTNode3DPtr> Q;
      Q.push(descendant);
      while (!Q.empty())
      {
        descendant = Q.front();
        Q.pop();
        for (const auto &leafptr : descendant->children)
        {
          leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
          Q.push(leafptr);
        }
      }
    }

    void fillPath(const RRTNode3DPtr &node_A, const RRTNode3DPtr &node_B, vector<Eigen::Vector3d> &path)
    {
      path.clear();
      RRTNode3DPtr node_ptr = node_A;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(start_node_->x);
      std::reverse(std::begin(path), std::end(path));

      node_ptr = node_B;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(goal_node_->x);
    }

    Eigen::Vector3d steer(const Eigen::Vector3d &nearest_node_p, const Eigen::Vector3d &rand_node_p, double len)
    {
      Eigen::Vector3d diff_vec = rand_node_p - nearest_node_p;
      double dist = diff_vec.norm();
      if (diff_vec.norm() <= len)
        return rand_node_p;
      else
        return nearest_node_p + diff_vec * len / dist;
    }

    bool greedySteer(const Eigen::Vector3d &x_near, const Eigen::Vector3d &x_target, vector<Eigen::Vector3d> &x_connects, const double len)
    {
      double vec_length = (x_target - x_near).norm();
      Eigen::Vector3d vec_unit = (x_target - x_near) / vec_length;
      x_connects.clear();

      if (vec_length < len)
        return map_ptr_->isSegmentValid(x_near, x_target);

      Eigen::Vector3d x_new, x_pre = x_near;
      double steered_dist = 0;

      while (steered_dist + len < vec_length)
      {
        x_new = x_pre + len * vec_unit;
        if ((!map_ptr_->isStateValid(x_new)) || (!map_ptr_->isSegmentValid(x_new, x_pre)))
          return false;

        x_pre = x_new;
        x_connects.push_back(x_new);
        steered_dist += len;
      }
      return map_ptr_->isSegmentValid(x_target, x_pre);
    }
    double computeH(const Eigen::Vector3d &si, const Eigen::Vector3d &gi)
    {
      Eigen::Vector3d si_gi, si_G, gi_S;
      double si_gi_dist, si_G_dist, gi_S_dist, h;
      si_gi = si - gi;
      si_G = si - goal_node_->x;
      gi_S = gi - start_node_->x;
      si_gi_dist = si_gi.norm();
      si_G_dist = si_G.norm();
      gi_S_dist = gi_S.norm();
      h = brrt_optimize_alpha_ * si_gi_dist + brrt_optimize_beta_ * si_G_dist + brrt_optimize_gamma_ * gi_S_dist;
      return h;
    }
    std::pair<RRTNode3DPtr, RRTNode3DPtr> search_nearest_heuristic(kdtree *treeA, kdtree *treeB)
    {
      RRTNode3DPtr selected_SI, selected_GI;
      struct kdres *nodesB, *nodesA;
      double min_houristic = DBL_MAX;
      RRTNode3DPtr nodeSi, nodeGi;
      nodesA = kd_nearest_range3(treeA, 0, 0, 0, DBL_MAX);
      selected_GI = goal_node_;
      selected_SI = start_node_;
      for (int i = 0; i < kd_res_size(nodesA); ++i)
      {
        nodeSi = (RRTNode3DPtr)kd_res_item_data(nodesA);
        nodesB = kd_nearest_range3(treeB, 0, 0, 0, DBL_MAX);
        for (int j = 0; j < kd_res_size(nodesB); ++j)
        {
          nodeGi = (RRTNode3DPtr)kd_res_item_data(nodesB);
          double h = computeH(nodeSi->x, nodeGi->x);
          if (h < min_houristic)
          {
            min_houristic = h;
            selected_SI = nodeSi;
            selected_GI = nodeGi;
          }
          kd_res_next(nodesB);
        }
        kd_res_next(nodesA);
        kd_res_free(nodesB);
      }
      kd_res_free(nodesA);
      return std::make_pair(selected_SI, selected_GI);
    }
    Eigen::Vector3d get_sample_valid()
    {
      Eigen::Vector3d x_rand;
      sampler_.samplingOnce(x_rand);
      // samplingOnce(x_rand);
      while (!map_ptr_->isStateValid(x_rand))
      {
        sampler_.samplingOnce(x_rand);
      }
      return x_rand;
    }

    bool intersectRaySphere(const Eigen::Vector3d &A, const Eigen::Vector3d &D, const Eigen::Vector3d &B, double radius, Eigen::Vector3d &intersection, float escape = 0.002)
    {
      Eigen::Vector3d m = A - B;
      double a = D.dot(D), b = 2.0 * D.dot(m), c = m.dot(m) - radius * radius;
      double discriminant = b * b - 4 * a * c;
      if (discriminant < 0)
        return false;

      double sqrt_disc = std::sqrt(discriminant), t1 = (-b - sqrt_disc) / (2 * a), t2 = (-b + sqrt_disc) / (2 * a);
      double t = (std::abs(t1) > escape) ? t1 : ((std::abs(t2) > escape) ? t2 : std::numeric_limits<double>::max());
      if (t == std::numeric_limits<double>::max())
        return false;
      intersection = A + t * D;
      return true;
    }

    Eigen::Vector3d computeT(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &X)
    {
      Eigen::Vector3d AX = X - A;
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d D = AX + AB;
      double radius = AB.norm();
#ifdef DEBUG
      if (vis_ptr_)
      {
        vis_ptr_->visualize_a_ball(B, radius, "/brrt_optimize/guide", visualization::Color::yellow,0.3);
      }
#endif
      Eigen::Vector3d intersection;
      if (intersectRaySphere(A, D, B, radius, intersection))
      {
        return intersection;
      }
      else
      {
        return A + D;
      }
    }
#ifdef DEBUG
    void print_vector3d(std::string name, Eigen::Vector3d &p)
    {
      std::cout << name << " x: " << p[0] << " y: " << p[1] << " z: " << p[2] << std::endl;
    }
#endif
    double computePbias(
        double Pinit,
        double h_start_goal,
        const Eigen::Vector3d &sguide,
        const Eigen::Vector3d &tguide)
    {

      if (h_start_goal == 0.0)
      {
        // Avoid division by zero
        return Pinit;
      }

      double h_sguide_tguide = computeH(sguide, tguide);
      double ratio = brrt_optimize_u_p*(h_start_goal - h_sguide_tguide) / h_start_goal;
      double Pbias = Pinit * std::exp(-ratio);
      return Pbias;
    }
    bool brrt_optimize(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      std::cout <<"[BRRT_Optimize] Start planning with parameter: p1: " << brrt_optimize_p1_ <<  " u_p: " <<  brrt_optimize_u_p  <<  " alpha: " <<  brrt_optimize_alpha_ <<  " beta: " <<  brrt_optimize_beta_  <<  " gamma: " <<  brrt_optimize_gamma_   << std::endl;
      ros::Time rrt_start_time = ros::Time::now();
      bool tree_connected = false;
      bool path_reverse = false;

      double h_start_goal = computeH(start_node_->x, goal_node_->x);

      /* kd tree init */
      kdtree *kdtree_1 = kd_create(3);
      kdtree *kdtree_2 = kd_create(3);
      // Add start and goal nodes to kd trees
      kd_insert3(kdtree_1, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
      kd_insert3(kdtree_2, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);
      // RRTNode3DPtr selected_SI = start_node_, selected_GI = goal_node_;
      // double min_houristic = h_start_goal; 
      kdtree *treeA = kdtree_1;
      kdtree *treeB = kdtree_2;

      std::random_device rd;                                // Seed
      std::mt19937 gen(rd());                               // Mersenne Twister engine
      std::uniform_real_distribution<double> dis(0.0, 1.0); // Uniform distribution [0,1)

      /* main loop */
      number_of_iterations_ = 0;
      std::pair<RRTNode3DPtr, RRTNode3DPtr> heuristic_pair;
#ifdef DEBUG
      std::cout << "[BRRT_Optimize] Start sampling..." << std::endl;
#endif

      for (number_of_iterations_ = 0; number_of_iterations_ < max_iteration_; ++number_of_iterations_)
      {
        /* random sampling */
        Eigen::Vector3d x_rand = get_sample_valid();
        Eigen::Vector3d x_new;
        double random01 = dis(gen);
        struct kdres *p_nearestA = nullptr, *p_nearestB = nullptr;

        RRTNode3DPtr nearest_nodeA, nearest_nodeB;
        heuristic_pair = search_nearest_heuristic(treeA, treeB);
        double pbias = computePbias(
            brrt_optimize_p1_,
            h_start_goal,
            heuristic_pair.first->x,
            heuristic_pair.second->x);

        if (random01 < pbias)
        {

          Eigen::Vector3d x_tmp = computeT(heuristic_pair.first->x, heuristic_pair.second->x, x_rand);
          nearest_nodeA = heuristic_pair.first;

          // x_new = map_ptr_->getFreeNodeInLine(nearest_nodeA->x, x_tmp, brrt_optimize_step_);
          x_new = steer(nearest_nodeA->x, x_tmp, steer_length_);
          nearest_nodeB = heuristic_pair.second;
#ifdef DEBUG
          vis_ptr_->visualize_a_ball(x_tmp, 0.5, "/brrt_optimize/x_tmp", visualization::Color::red);
          std::cout << "[BRRT_Optimize] Use heuristic steer " << pbias << std::endl;
#endif
        }
        else
        {
// x_new = map_ptr_->getFreeNodeInLine(nearest_nodeA->x, x_rand, brrt_optimize_step_);
#ifdef DEBUG
          std::cout << "[BRRT_Optimize] Use normal steer " << pbias << std::endl;
#endif
          p_nearestA = kd_nearest3(treeA, x_rand[0], x_rand[1], x_rand[2]);

          if (p_nearestA == nullptr)
          {
#ifdef DEBUG
            ROS_ERROR("nearest query error");
#endif
            continue;
          }
          nearest_nodeA = (RRTNode3DPtr)kd_res_item_data(p_nearestA);
          kd_res_free(p_nearestA);
          x_new = steer(nearest_nodeA->x, x_rand, steer_length_);

          p_nearestB = kd_nearest3(treeB, x_new[0], x_new[1], x_new[2]);
          if (p_nearestB == nullptr)
          {
#ifdef DEBUG
            ROS_ERROR("nearest query error");
#endif
            continue;
          }
          nearest_nodeB = (RRTNode3DPtr)kd_res_item_data(p_nearestB);
          kd_res_free(p_nearestB);
        }

        if ((!map_ptr_->isStateValid(x_new)) || (!map_ptr_->isSegmentValid(nearest_nodeA->x, x_new)))
        {
          /* Steer Trapped */
#ifdef DEBUG
          print_vector3d("[BRRT_Optimize] x_new", x_new);
          print_vector3d("[BRRT_Optimize] nearest_nodeA", nearest_nodeA->x);
          print_vector3d("[BRRT_Optimize] nearest_nodeB", nearest_nodeB->x);
          std::cout << "[BRRT_Optimize] Steer Trapped, swap trees" << map_ptr_->isSegmentValid(nearest_nodeA->x, x_new) << "  " << map_ptr_->isStateValid(x_new) << std::endl;
#endif
          std::swap(treeA, treeB);
          path_reverse = !path_reverse;
          continue;
        }

        // Extend Node A by steer
        double dist_from_A = nearest_nodeA->cost_from_start + steer_length_;
        RRTNode3DPtr new_nodeA(nullptr);
        new_nodeA = addTreeNode(nearest_nodeA, x_new, dist_from_A, steer_length_);
        kd_insert3(treeA, x_new[0], x_new[1], x_new[2], new_nodeA);

        /* request x_new's nearest node in treeB */
        /* Greedy steer & check connection */
        vector<Eigen::Vector3d> x_connects;
        bool isConnected = greedySteer(nearest_nodeB->x, x_new, x_connects, steer_length_);

        /* Add the steered nodes to treeB */
        RRTNode3DPtr new_nodeB = nearest_nodeB;
        if (!x_connects.empty())
        {
          if (valid_tree_node_nums_ + (int)x_connects.size() >= max_tree_node_nums_)
          {
            valid_tree_node_nums_ = max_tree_node_nums_; // max_node_num reached
            break;
          }

          for (auto x_connect : x_connects)
          {
            new_nodeB = addTreeNode(new_nodeB, x_connect, new_nodeB->cost_from_start + steer_length_, steer_length_);
            kd_insert3(treeB, x_connect[0], x_connect[1], x_connect[2], new_nodeB);
          }
        }

        /* If connected, trace the connected path */
        if (isConnected)
        {

          tree_connected = true;
          double path_cost = new_nodeA->cost_from_start + new_nodeB->cost_from_start + calDist(new_nodeB->x, new_nodeA->x);
          if (path_cost < cost_best_)
          {
            vector<Eigen::Vector3d> curr_best_path;
            if (path_reverse)
              fillPath(new_nodeB, new_nodeA, curr_best_path);
            else
              fillPath(new_nodeA, new_nodeB, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(path_cost, (ros::Time::now() - rrt_start_time).toSec());
            cost_best_ = path_cost;
          }
#ifdef DEBUG
          std::cout << "[BRRT_Optimize]**********Find path after " << number_of_iterations_ << " iterations" << std::endl;
#endif
          break;
        }

#ifdef DEBUG
        std::cout << "[BRRT_Optimize] Iteration " << number_of_iterations_ << " completed, tree size: " << valid_tree_node_nums_ << std::endl;
        visualizeWholeTree();

        vis_ptr_->visualize_a_ball(x_rand, 0.5, "/brrt_optimize/x_rand", visualization::Color::blue);
        vis_ptr_->visualize_a_ball(x_new, 0.5, "/brrt_optimize/x_new", visualization::Color::green);
        vis_ptr_->visualize_a_ball(nearest_nodeA->x, 0.5, "/brrt_optimize/nearest_nodeA", visualization::Color::black);
        vis_ptr_->visualize_a_ball(nearest_nodeB->x, 0.5, "/brrt_optimize/nearest_nodeB", visualization::Color::white);
        usleep(500000); // Sleep for 0.1 seconds to visualize the tree growth
#endif

        /* Swap treeA&B */
        std::swap(treeA, treeB);
        path_reverse = !path_reverse;
      } // End of sampling iteration
#ifdef DEBUG
      visualizeWholeTree();
#endif
      if (tree_connected)
      {
        final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
     
#ifdef DEBUG   
        ROS_INFO_STREAM("[BRRT_Optimize]: find_path_use_time: " << solution_cost_time_pair_list_.front().second << ", length: " << solution_cost_time_pair_list_.front().first);
#endif
        // vis_ptr_->visualize_a_text(Eigen::Vector3d(0, 0, 0), "find_path_use_time","find_path_use_time: " + std::to_string(solution_cost_time_pair_list_.front().second), visualization::Color::black);
        // vis_ptr_->visualize_a_text(Eigen::Vector3d(0, 0, 0.5), "length","length: " + std::to_string(solution_cost_time_pair_list_.front().first), visualization::Color::black);

        // visualizeWholeTree();
        final_path_ = path_list_.back();
      }
 #ifdef DEBUG   
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        // visualizeWholeTree();
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      }
#endif
      return tree_connected;
    }

    void visualizeWholeTree()
    {
      // Sample and visualize the resultant tree
      vector<Eigen::Vector3d> vertice;
      vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
      vertice.clear();
      edges.clear();
      sampleWholeTree(start_node_, vertice, edges);
      sampleWholeTree(goal_node_, vertice, edges);
      std::vector<visualization::BALL> tree_nodes;
      tree_nodes.reserve(vertice.size());
      visualization::BALL node_p;
      node_p.radius = 0.12;
      for (size_t i = 0; i < vertice.size(); ++i)
      {
        node_p.center = vertice[i];
        tree_nodes.push_back(node_p);
      }
      vis_ptr_->visualize_balls(tree_nodes, "tree_vertice", visualization::Color::blue, 1.0);
      vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::red, 0.06);
    }

    void sampleWholeTree(const RRTNode3DPtr &root, vector<Eigen::Vector3d> &vertice, vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges)
    {
      if (root == nullptr)
        return;

      // whatever dfs or bfs
      RRTNode3DPtr node = root;
      std::queue<RRTNode3DPtr> Q;
      Q.push(node);
      while (!Q.empty())
      {
        node = Q.front();
        Q.pop();
        for (const auto &leafptr : node->children)
        {
          vertice.push_back(leafptr->x);
          edges.emplace_back(std::make_pair(node->x, leafptr->x));
          Q.push(leafptr);
        }
      }
    }

  public:
    void samplingOnce(Eigen::Vector3d &sample)
    {
      static int i = 0;
      sample = preserved_samples_[i];
      i++;
      i = i % preserved_samples_.size();
    }

    void setPreserveSamples(const vector<Eigen::Vector3d> &samples)
    {
      preserved_samples_ = samples;
    }
    vector<Eigen::Vector3d> preserved_samples_;
  };

} // namespace path_plan
#endif