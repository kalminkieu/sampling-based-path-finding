
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
      nh_.param("BRRT_Optimize/p2", brrt_optimize_p2_, 0.1);
      nh_.param("BRRT_Optimize/p3", brrt_optimize_p3_, 0.1);
      nh_.param("BRRT_Optimize/u_p", brrt_optimize_u_p, 2.0);
      nh_.param("BRRT_Optimize/step", brrt_optimize_step_, 0.1);
      nh_.param("BRRT_Optimize/alpha", brrt_optimize_alpha_, 0.5);
      nh_.param("BRRT_Optimize/beta", brrt_optimize_beta_, 0.3);
      nh_.param("BRRT_Optimize/gamma", brrt_optimize_gamma_, 0.5);
      nh_.param("BRRT_Optimize/max_iteration", max_iteration_, 0);
      nh_.param("BRRT_Optimize/enable2d", enable_2d_, false); // From Version 3, default false for 3D compatibility

      std::cout << "[BRRT_Optimize] param: p1: " << brrt_optimize_p1_ << " p2: " << brrt_optimize_p2_ << " p3: " << brrt_optimize_p3_ << " step: " << brrt_optimize_step_ << std::endl;
      std::cout << "[BRRT_Optimize] param: alpha: " << brrt_optimize_alpha_ << " beta: " << brrt_optimize_beta_ << " gamma: " << brrt_optimize_gamma_ << std::endl;
      ROS_WARN_STREAM("[BRRT_Optimize] param: steer_length: " << steer_length_);
      ROS_WARN_STREAM("[BRRT_Optimize] param: search_time: " << search_time_);
      ROS_WARN_STREAM("[BRRT_Optimize] param: max_tree_node_nums: " << max_tree_node_nums_);
      ROS_WARN_STREAM("[BRRT_Optimize] param: enable_2d: " << enable_2d_);

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

    void set_heuristic_param(double p1, double u_p, double alpha, double beta, double gamma)
    {
      brrt_optimize_p1_ = p1;
      brrt_optimize_u_p = u_p;
      brrt_optimize_alpha_ = alpha;
      brrt_optimize_beta_ = beta;
      brrt_optimize_gamma_ = gamma;
    }
        int get_valid_tree_node_nums()
    {
      return valid_tree_node_nums_;
    }
    int get_number_of_iteration()
    {
      return number_of_iterations_;
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

    void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
    {
      vis_ptr_ = visPtr;
    };

  private:
    // nodehandle params
    ros::NodeHandle nh_;

    BiasSampler sampler_;
    double brrt_optimize_p1_;
    double brrt_optimize_p2_;
    double brrt_optimize_p3_;
    double brrt_optimize_u_p;
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
    bool enable_2d_; 
    int number_of_iterations_;

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

    void generateMap(double percent_obs)
    {
    }


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
        node->parent->children.remove(node); // DON'T FORGET THIS, remove it from its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descendants, change the cost_from_start
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

    bool brrt_optimize(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      ros::Time rrt_start_time = ros::Time::now();
      bool tree_connected = false;
      bool path_reverse = false;

      /* kd tree init */
      kdtree *kdtree_1 = kd_create(3);
      kdtree *kdtree_2 = kd_create(3);
      // Add start and goal nodes to kd trees
      kd_insert3(kdtree_1, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
      kd_insert3(kdtree_2, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);

      kdtree *treeA = kdtree_1;
      kdtree *treeB = kdtree_2;

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<double> dis(0.0, 1.0);
      Eigen::Vector3d si_gi, si_G, gi_S;
      double si_gi_dist, si_G_dist, gi_S_dist, h;

      /* main loop */
      for (int idx = 0; idx < max_iteration_; ++idx)
      {
        /* random sampling */
        usleep(100000); // Consistent with Version 1
        double random01 = dis(gen);
        double min_heuristic = DBL_MAX;
        RRTNode3DPtr s_guide = nullptr, t_guide = nullptr;

        // Heuristic node selection (from Version 1)
        struct kdres *nodesA = kd_nearest_range3(treeA, 0, 0, 0, DBL_MAX);
        for (int i = 0; i < kd_res_size(nodesA); ++i)
        {
          RRTNode3DPtr nodeSi = (RRTNode3DPtr)kd_res_item_data(nodesA);
          struct kdres *nodesB = kd_nearest_range3(treeB, 0, 0, 0, DBL_MAX);
          for (int j = 0; j < kd_res_size(nodesB); ++j)
          {
            RRTNode3DPtr nodeGi = (RRTNode3DPtr)kd_res_item_data(nodesB);
            si_gi = nodeSi->x - nodeGi->x;
            si_G = nodeSi->x - goal_node_->x;
            gi_S = nodeGi->x - start_node_->x;
            si_gi_dist = si_gi.norm();
            si_G_dist = si_G.norm();
            gi_S_dist = gi_S.norm();
            h = brrt_optimize_alpha_ * si_gi_dist + brrt_optimize_beta_ * si_G_dist + brrt_optimize_gamma_ * gi_S_dist;
            if (h < min_heuristic)
            {
              min_heuristic = h;
              s_guide = nodeSi;
              t_guide = nodeGi;
            }
            kd_res_next(nodesB);
          }
          kd_res_next(nodesA);
          kd_res_free(nodesB);
        }
        kd_res_free(nodesA);

        if (s_guide == nullptr || t_guide == nullptr)
        {
          ROS_ERROR("No suitable guide pair found");
          continue;
        }

        // Sampling (merged from Version 1 and Version 3)
        Eigen::Vector3d x_rand;
        if (random01 < brrt_optimize_p1_)
        {
          x_rand = t_guide->x; // Goal-directed (Version 1 and 3)
        }
        else if (random01 < brrt_optimize_p1_ + brrt_optimize_p2_)
        {
          Eigen::Vector3d center = (s_guide->x + t_guide->x) / 2.0;
          double radius = enable_2d_ ? 3 * brrt_optimize_step_ : (s_guide->x - t_guide->x).norm() / 2.0;
          double u = dis(gen) * 2.0 - 1.0;
          double theta = dis(gen) * 2.0 * M_PI;
          double r = cbrt(dis(gen)) * radius;
          if (enable_2d_)
          {
            // 2D sampling (from Version 3)
            x_rand[0] = center[0] + r * cos(theta);
            x_rand[1] = center[1] + r * sin(theta);
            x_rand[2] = center[2];
          }
          else
          {
            // 3D sampling (from Version 1)
            double phi = acos(u);
            x_rand[0] = center[0] + r * sin(phi) * cos(theta);
            x_rand[1] = center[1] + r * sin(phi) * sin(theta);
            x_rand[2] = center[2] + r * cos(phi);
          }
          if (!map_ptr_->isStateValid(x_rand))
          {
            x_rand = t_guide->x; // Fallback to t_guide (Version 3)
          }
        }
        else
        {
          sampler_.samplingOnce(x_rand);
          while (!map_ptr_->isStateValid(x_rand))
          {
            sampler_.samplingOnce(x_rand);
          }
        }

        // Steering (from Version 1)
        struct kdres *p_nearestA = kd_nearest3(treeA, x_rand[0], x_rand[1], x_rand[2]);
        if (p_nearestA == nullptr)
        {
          ROS_ERROR("nearest query error");
          continue;
        }
        RRTNode3DPtr nearest_nodeA = (RRTNode3DPtr)kd_res_item_data(p_nearestA);
        Eigen::Vector3d x_new = map_ptr_->getFreeNodeInLine(nearest_nodeA->x, x_rand, brrt_optimize_step_);
        kd_res_free(p_nearestA);

        if (vis_ptr_)
        {
          vis_ptr_->visualize_a_ball(x_rand, 0.5, "sample_node", visualization::Color::black);
          vis_ptr_->visualize_a_ball(x_new, 0.5, "q_nearest", visualization::Color::yellow);
        }

        if (!map_ptr_->isStateValid(x_new) || !map_ptr_->isSegmentValid(nearest_nodeA->x, x_new))
        {
          std::swap(treeA, treeB);
          path_reverse = !path_reverse;
          continue;
        }

        // Add x_new to treeA
        double dist_from_A = nearest_nodeA->cost_from_start + steer_length_;
        RRTNode3DPtr new_nodeA = addTreeNode(nearest_nodeA, x_new, dist_from_A, steer_length_);
        kd_insert3(treeA, x_new[0], x_new[1], x_new[2], new_nodeA);

        // Heuristic-based treeB node selection (from Version 3)
        struct kdres *p_nearestB = kd_nearest_range3(treeB, x_new[0], x_new[1], x_new[2], 5.0);
        RRTNode3DPtr nearest_nodeB = nullptr;
        double min_heuristic_B = DBL_MAX;
        while (!kd_res_end(p_nearestB))
        {
          RRTNode3DPtr nodeTj = (RRTNode3DPtr)kd_res_item_data(p_nearestB);
          Eigen::Vector3d tj = nodeTj->x;
          double d_si_tj = (x_new - tj).norm();
          double d_tj_s0 = (tj - start_node_->x).norm();
          double d_si_t0 = (x_new - goal_node_->x).norm();
          double heuristic = brrt_optimize_alpha_ * d_si_tj + brrt_optimize_beta_ * d_tj_s0 + brrt_optimize_gamma_ * d_si_t0;
          if (heuristic < min_heuristic_B)
          {
            min_heuristic_B = heuristic;
            nearest_nodeB = nodeTj;
          }
          kd_res_next(p_nearestB);
        }
        kd_res_free(p_nearestB);

        if (nearest_nodeB == nullptr)
        {
          ROS_ERROR("No suitable tj found");
          continue;
        }

        // Greedy steer & check connection
        vector<Eigen::Vector3d> x_connects;
        bool isConnected = greedySteer(nearest_nodeB->x, x_new, x_connects, steer_length_);

        // Add steered nodes to treeB
        RRTNode3DPtr new_nodeB = nearest_nodeB;
        if (!x_connects.empty())
        {
          if (valid_tree_node_nums_ + (int)x_connects.size() >= max_tree_node_nums_)
          {
            valid_tree_node_nums_ = max_tree_node_nums_;
            break;
          }
          for (auto x_connect : x_connects)
          {
            new_nodeB = addTreeNode(new_nodeB, x_connect, new_nodeB->cost_from_start + steer_length_, steer_length_);
            kd_insert3(treeB, x_connect[0], x_connect[1], x_connect[2], new_nodeB);
          }
        }

        // If connected, trace the path
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
          std::cout << "[BRRT_Optimize]**********Find path after " << idx << " iterations" << std::endl;
          break;
        }

        std::swap(treeA, treeB);
        path_reverse = !path_reverse;
        visualizeWholeTree();
      } // End of sampling iteration

      visualizeWholeTree();
      if (tree_connected)
      {
        final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
        ROS_INFO_STREAM("[BRRT_Optimize]: find_path_use_time: " << solution_cost_time_pair_list_.front().second << ", length: " << solution_cost_time_pair_list_.front().first);
        final_path_ = path_list_.back();
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[BRRT_Optimize]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      }
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
      vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::red, 0.06);
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
} // namespace path_plan
#endif
