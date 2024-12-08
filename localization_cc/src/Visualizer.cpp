#include "localization/Visualizer.hpp"

#include <ouster/point_viz.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <condition_variable>
#include <memory>
#include <mutex>

namespace navigator {

namespace localization {

Visualizer::~Visualizer() {
  if (render_thread_.joinable()) render_thread_.join();
}

void Visualizer::update(absl::string_view object_name,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  std::lock_guard<std::mutex> lock(render_mutex_);

  auto cloud_viz = std::make_shared<ouster::viz::Cloud>(cloud->size());

  std::vector<float> points(3 * cloud->size());
  for (size_t i = 0; i < cloud->size(); i++) {
    const pcl::PointXYZ &point = cloud->points[i];
    points[i] = point.x;
    points[i + cloud->size()] = point.y;
    points[i + 2 * cloud->size()] = point.z;
  }
  cloud_viz->set_xyz(points.data());

  std::vector<float> colors(cloud->size(), 0.5);
  cloud_viz->set_key(colors.data());

  clouds_[std::string(object_name)] = cloud_viz;

  viz_->add(cloud_viz);
  viz_->update();
}

void Visualizer::remove(absl::string_view object_name) {
  std::lock_guard<std::mutex> lock(render_mutex_);

  auto cloud = clouds_.find(std::string(object_name));
  if (cloud != clouds_.end()) {
    viz_->remove(cloud->second);
    clouds_.erase(cloud);
    viz_->update();
  }
}

void Visualizer::run() {
  render_mutex_.lock();
  render_thread_ = std::thread([this] {
    viz_ = std::make_shared<ouster::viz::PointViz>("Global Map");
    ouster::viz::add_default_controls(*viz_);
    // Drop the mutex before running the visualizer.
    render_mutex_.unlock();
    viz_->run();
  });
}

}  // namespace localization

}  // namespace navigator