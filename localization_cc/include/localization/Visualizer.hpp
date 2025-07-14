#pragma once

#include <absl/container/flat_hash_map.h>
#include <absl/strings/string_view.h>
#include <ouster/point_viz.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace navigator {

namespace localization {

class Visualizer {
 public:
  ~Visualizer();
  void update(absl::string_view object_name,
              pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  void remove(absl::string_view object);
  void run();

 private:
  std::thread render_thread_;
  std::condition_variable viz_ready_cv_;
  std::mutex render_mutex_;
  std::shared_ptr<ouster::viz::PointViz> viz_;
  absl::flat_hash_map<std::string, std::shared_ptr<ouster::viz::Cloud>> clouds_;
};
}  // namespace localization

}  // namespace navigator