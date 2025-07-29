#ifndef PATH_UTILS_H
#define PATH_UTILS_H

#include <vector>
#include <Eigen/Core>

inline double computePathLength(const std::vector<Eigen::Vector3d>& path) {
  double L = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    L += (path[i] - path[i-1]).norm();
  }
  return L;
}

#endif  // PATH_UTILS_H
