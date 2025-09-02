#ifndef BRRT_OPTIMIZE_H
#define BRRT_OPTIMIZE_H

#include "occ_grid/occ_map.h"
#include "visualization/visualization.hpp"
#include "sampler.h"
#include "node.h"
#include "kdtree.h"
#include "path_utils.h"
#include <ros/ros.h>
#include <utility>
#include <queue>
#include <algorithm>
#include <random>

namespace path_plan
{
  class BRRT_Optimize
  {
  public:
    BRRT_Optimize() {};
    BRRT_Optimize(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapPtr) : nh_(nh), map_ptr_(mapPtr)
    {
      resolution_m_per_px_ = mapPtr->getResolution();
      nh_.param("BRRT/steer_length", steer_length_, 0.0);
      nh_.param("BRRT/search_time", search_time_, 0.0);
      nh_.param("BRRT/max_tree_node_nums", max_tree_node_nums_, 0);
      nh_.param("BRRT_Optimize/p", brrt_optimize_p_, 0.5);
      nh_.param("BRRT_Optimize/step", brrt_optimize_step_, 0.1);
      nh_.param("BRRT_Optimize/alpha", brrt_optimize_alpha_, 0.6);
      nh_.param("BRRT_Optimize/beta", brrt_optimize_beta_, 0.2);
      nh_.param("BRRT_Optimize/gamma", brrt_optimize_gamma_, 0.2);
      nh_.param("BRRT_Optimize/max_iteration", max_iteration_, 0);
      std::cout << "[BRRT_Optimize] param: p: " << brrt_optimize_p_ << " step: " << brrt_optimize_step_ << std::endl;
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
      start_node_ = nodes_pool_[1];
      start_node_->x = s;
      start_node_->cost_from_start = 0.0;
      goal_node_ = nodes_pool_[0];
      goal_node_->x = g;
      goal_node_->cost_from_start = 0.0;
      valid_tree_node_nums_ = 2;
      vis_ptr_->visualize_a_ball(s, 0.3, "start", visualization::Color::pink);
      vis_ptr_->visualize_a_ball(g, 0.3, "goal", visualization::Color::steelblue);
      // [VISUALIZATION] visualize two nodes
      ROS_INFO("[BRRT_Optimize]: BRRT starts planning a path");
      return brrt_optimize(s, g);
    }
    vector<Eigen::Vector3d> getPath() { return final_path_; }
    vector<vector<Eigen::Vector3d>> getAllPaths() { return path_list_; }
    vector<std::pair<double, double>> getSolutions() { return solution_cost_time_pair_list_; }
    void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr) { vis_ptr_ = visPtr; };
  private:
    ros::NodeHandle nh_;
    BiasSampler sampler_;
    double brrt_optimize_p_;
    double brrt_optimize_step_;
    double brrt_optimize_alpha_;
    double brrt_optimize_beta_;
    double brrt_optimize_gamma_;
    int max_iteration_;
    double steer_length_;
    double search_time_;
    int max_tree_node_nums_;
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;
    double resolution_m_per_px_;
    double cost_best_;
    std::vector<TreeNode *> nodes_pool_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector3d> final_path_;
    vector<vector<Eigen::Vector3d>> path_list_;
    vector<std::pair<double, double>> solution_cost_time_pair_list_;
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
    double calDist(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
      return (p1 - p2).norm() / resolution_m_per_px_;
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
        node->parent->children.remove(node);
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);
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
    Eigen::Vector3d getFreeNodeInLine(const Eigen::Vector3d &start, const Eigen::Vector3d &target, double step, const Eigen::Vector3d &guide)
    {
      Eigen::Vector3d direction = (target - start).normalized();
      Eigen::Vector3d guide_direction = (guide - start).normalized();
      direction = (1.0 - brrt_optimize_alpha_) * direction + brrt_optimize_alpha_ * guide_direction;
      direction.normalize();
      return start + step * direction;
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
    void findGuidePair(kdtree* treeS, kdtree* treeT, RRTNode3DPtr& s_guide, RRTNode3DPtr& t_guide)
    {
      Eigen::Vector3d S_T, S_Goal, T_Start;
      double ST_dist, SG_dist, TS_dist;
      double h;
      double min_heuristic = DBL_MAX;
      struct kdres* nodesS = kd_nearest_range3(treeS, 0, 0, 0, DBL_MAX);
      if (!nodesS) return;
      kd_res_rewind(nodesS);
      while (!kd_res_end(nodesS))
      {
        RRTNode3DPtr nodeS = (RRTNode3DPtr)kd_res_item_data(nodesS);
        struct kdres* nodesT = kd_nearest_range3(treeT, 0, 0, 0, DBL_MAX);
        if (!nodesT)
        {
          kd_res_next(nodesS);
          continue;
        }
        kd_res_rewind(nodesT);
        while (!kd_res_end(nodesT))
        {
          RRTNode3DPtr nodeT = (RRTNode3DPtr)kd_res_item_data(nodesT);
          S_T = nodeS->x - nodeT->x;
          S_Goal = nodeS->x - goal_node_->x;
          T_Start = nodeT->x - start_node_->x;
          ST_dist = S_T.norm();
          SG_dist = S_Goal.norm();
          TS_dist = T_Start.norm();
          h = brrt_optimize_alpha_ * ST_dist + brrt_optimize_beta_ * SG_dist + brrt_optimize_gamma_ * TS_dist;
          if (h < min_heuristic)
          {
            min_heuristic = h;
            s_guide = nodeS;
            t_guide = nodeT;
          }
          kd_res_next(nodesT);
        }
        kd_res_free(nodesT);
        kd_res_next(nodesS);
      }
      kd_res_free(nodesS);
    }
    bool brrt_optimize(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      ros::Time rrt_start_time = ros::Time::now();
      bool tree_connected = false;
      bool path_reverse = false;
      kdtree *kdtree_1 = kd_create(3);
      kdtree *kdtree_2 = kd_create(3);
      kd_insert3(kdtree_1, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
      kd_insert3(kdtree_2, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);
      kdtree *treeS = kdtree_1;
      kdtree *treeT = kdtree_2;
      kdtree *currentTree;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<double> dis(0.0, 1.0);
      RRTNode3DPtr s_guide = start_node_;
      RRTNode3DPtr t_guide = goal_node_;
      ROS_INFO_STREAM("[BRRT_Optimize]: Start optimizing path with p: " << brrt_optimize_p_);
      ROS_INFO_STREAM("[BRRT_Optimize]: Max iteration: " << max_iteration_);
      ROS_INFO_STREAM("[BRRT_Optimize]: Steer length: " << steer_length_);
      ROS_INFO_STREAM("[BRRT_Optimize]: alpha: " << brrt_optimize_alpha_
                      << ", beta: " << brrt_optimize_beta_
                      << ", gamma: " << brrt_optimize_gamma_);
      Eigen::Vector3d q_rand;
      int node1 = 0;
      int node2 = 0;
      for (int idx = 0; idx < max_iteration_; ++idx)
      {
        //Sampling a random point in the map
        sampler_.samplingOnce(q_rand, true);
        while (!map_ptr_->isStateValid(q_rand))
        {
          sampler_.samplingOnce(q_rand, true);
        }
        //get a random probabilistic value to decide whether to use guide pair or not
        if (dis(gen) < brrt_optimize_p_)
        {
          // Use guide pair to steer
          findGuidePair(treeS, treeT, s_guide, t_guide);
          struct kdres *p_nearestS = kd_nearest3(treeS, q_rand[0], q_rand[1], q_rand[2]);
          if (p_nearestS == nullptr)
          {
            continue;
          }
          RRTNode3DPtr nearest_nodeS = (RRTNode3DPtr)kd_res_item_data(p_nearestS);
          kd_res_free(p_nearestS);
          Eigen::Vector3d q_new = getFreeNodeInLine(nearest_nodeS->x, q_rand, brrt_optimize_step_, s_guide->x);
          ROS_INFO_STREAM("[BRRT_Optimize]: new node: " << q_new);
          if (map_ptr_->isStateValid(q_new) && map_ptr_->isSegmentValid(nearest_nodeS->x, q_new))
          {
            double step_len = (q_new - nearest_nodeS->x).norm() / resolution_m_per_px_;
            double dist_from_S = nearest_nodeS->cost_from_start + step_len;
            RRTNode3DPtr new_nodeS = addTreeNode(nearest_nodeS, q_new, dist_from_S, step_len);
            kd_insert3(treeS, q_new[0], q_new[1], q_new[2], new_nodeS);
            struct kdres *p_nearestT = kd_nearest3(treeT, q_new[0], q_new[1], q_new[2]);
            if (p_nearestT != nullptr)
            {
              RRTNode3DPtr nearest_nodeT = (RRTNode3DPtr)kd_res_item_data(p_nearestT);
              kd_res_free(p_nearestT);
              vector<Eigen::Vector3d> x_connects;
              bool isConnected = greedySteer(nearest_nodeT->x, q_new, x_connects, steer_length_);
              if (!x_connects.empty())
              {
                RRTNode3DPtr new_nodeT = nearest_nodeT;
                for (auto &x_connect : x_connects) {
                  double step_len_T = (x_connect - new_nodeT->x).norm();
                  double cost_T = new_nodeT->cost_from_start + step_len_T;
                  new_nodeT = addTreeNode(new_nodeT, x_connect, cost_T, step_len_T);
                  kd_insert3(treeT, x_connect[0], x_connect[1], x_connect[2], new_nodeT);
                }
              }
              if (isConnected)
              {
                // If the two trees are connected, calculate the path cost and store the path
                tree_connected = true;
                double path_cost = new_nodeS->cost_from_start + nearest_nodeT->cost_from_start + calDist(nearest_nodeT->x, new_nodeS->x);
                if (path_cost < cost_best_)
                {
                  vector<Eigen::Vector3d> curr_best_path;
                  if (path_reverse)
                    fillPath(nearest_nodeT, new_nodeS, curr_best_path);
                  else
                    fillPath(new_nodeS, nearest_nodeT, curr_best_path);
                  path_list_.emplace_back(curr_best_path);
                  solution_cost_time_pair_list_.emplace_back(path_cost, (ros::Time::now() - rrt_start_time).toSec());
                  cost_best_ = path_cost;
                }
                std::cout << "[BRRT_Optimized]**********find path after " << idx << " iterations" << std::endl;
                break;
              }
            }
          }
        }
        else
        {
          // Do not use guide pair, just sample a random point
          struct kdres *p_nearestS = kd_nearest3(treeS, q_rand[0], q_rand[1], q_rand[2]);
          if (p_nearestS == nullptr)
          {
            continue;
          }
          RRTNode3DPtr nearest_nodeS = (RRTNode3DPtr)kd_res_item_data(p_nearestS);
          kd_res_free(p_nearestS);
          // Steer towards the random point
          // and check if the new point is valid
          // [SPECIAL] use the guide node to steer
          // [SPECIAL] use the last updated guide node to steer
          // [SPECIAL] if not do not have the last updated guide node, use the nearest node to steer
          if (s_guide == nullptr)
          {
            s_guide = nearest_nodeS;
          }
          Eigen::Vector3d q_new = getFreeNodeInLine(nearest_nodeS->x, q_rand, brrt_optimize_step_, s_guide->x);
          if (map_ptr_->isStateValid(q_new) && map_ptr_->isSegmentValid(nearest_nodeS->x, q_new))
          {
            double step_len = (q_new - nearest_nodeS->x).norm() / resolution_m_per_px_;
            double dist_from_S = nearest_nodeS->cost_from_start + step_len;
            RRTNode3DPtr new_nodeS = addTreeNode(nearest_nodeS, q_new, dist_from_S, step_len);
            kd_insert3(treeS, q_new[0], q_new[1], q_new[2], new_nodeS);
            struct kdres *p_nearestT = kd_nearest3(treeT, q_new[0], q_new[1], q_new[2]);
            if (p_nearestT != nullptr)
            {
              RRTNode3DPtr nearest_nodeT = (RRTNode3DPtr)kd_res_item_data(p_nearestT);
              kd_res_free(p_nearestT);
              vector<Eigen::Vector3d> x_connects;
              bool isConnected = greedySteer(nearest_nodeT->x, q_new, x_connects, steer_length_);
              if (!x_connects.empty())
              {
                RRTNode3DPtr new_nodeT = nearest_nodeT;
                for (auto &x_connect : x_connects) {
                  double step_len_T = (x_connect - new_nodeT->x).norm();
                  double cost_T = new_nodeT->cost_from_start + step_len_T;
                  new_nodeT = addTreeNode(new_nodeT, x_connect, cost_T, step_len_T);
                  kd_insert3(treeT, x_connect[0], x_connect[1], x_connect[2], new_nodeT);
                }
              }
              if (isConnected)
              {
                tree_connected = true;
                double path_cost = new_nodeS->cost_from_start + nearest_nodeT->cost_from_start + calDist(nearest_nodeT->x, new_nodeS->x);
                if (path_cost < cost_best_)
                {
                  vector<Eigen::Vector3d> curr_best_path;
                  if (path_reverse)
                    fillPath(nearest_nodeT, new_nodeS, curr_best_path);
                  else
                    fillPath(new_nodeS, nearest_nodeT, curr_best_path);
                  path_list_.emplace_back(curr_best_path);
                  solution_cost_time_pair_list_.emplace_back(path_cost, (ros::Time::now() - rrt_start_time).toSec());
                  cost_best_ = path_cost;
                }
                std::cout << "[BRRT_Optimized]**********find path after " << idx << " iterations" << std::endl;
                break;
              }
            }
          }
        }
        currentTree = treeS;
        // ROS_INFO_STREAM("current tree S: " << treeS << " nodes");
        if (currentTree == kdtree_1) node1+=1;
        else node2+=1;
        std::swap(treeS, treeT);
        // ROS_INFO_STREAM("current tree S: " << treeS << " nodes");
        path_reverse = !path_reverse;
        visualizeWholeTree();
        // [VISUALIZATION] visualize the two trees after connected
      }
      ROS_INFO_STREAM("Total seps in tree1: " << node1);
      ROS_INFO_STREAM("Total seps in tree2: " << node2);
      if (tree_connected)
      {
        final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
        final_path_ = path_list_.back();
        double L = computePathLength(final_path_);
        ROS_INFO_STREAM("[BRRT_Optimize]: Completed after " << final_path_use_time_ << " seconds");
        ROS_INFO_STREAM("[BRRT_Optimize]: find_path_use_time: " << final_path_use_time_ << ", length: " << L);
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      }
      kd_free(kdtree_1);
      kd_free(kdtree_2);
      return tree_connected;
    }
    void visualizeWholeTree()
    {
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
      // []VISUALIZATION] visualize tree nodes
      vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::red, 0.06);
      // [VISUALIZATION] visualize tree edges
    }
    void sampleWholeTree(const RRTNode3DPtr &root, vector<Eigen::Vector3d> &vertice, vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges)
    {
      if (root == nullptr)
        return;
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
}

#endif