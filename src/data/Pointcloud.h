#ifndef SRC_DATA_POINTCLOUD_H_
#define SRC_DATA_POINTCLOUD_H_

#include <eigen3/Eigen/Dense>
#include "geometry.h"

/** \brief a laserscan with possibly remission.
 *
 *  \author behley
 */

class Laserscan {
 public:
  void clear() {
    points.clear();
    colors.clear();
  }
  uint32_t size() const { return points.size(); }
  bool hasColors() const { return (points.size() > 0) && (points.size() == colors.size()); }

  Eigen::Matrix4f pose;
  std::vector<Point3f> points;
  std::vector<uint32_t> colors;
};

#endif /* SRC_DATA_POINTCLOUD_H_ */
