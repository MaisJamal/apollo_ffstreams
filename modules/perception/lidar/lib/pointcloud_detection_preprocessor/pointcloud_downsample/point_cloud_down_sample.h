/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/point.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudDownSample : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;

 public:
  PointCloudDownSample();
  virtual ~PointCloudDownSample() = default;

  bool Init(const PluginConfig& plugin_config) override;
  bool Process(DataFrame* data_frame) override;
  bool Process(DataFrame* data_frame, float * point_array, int * num_points_result) override;
  bool IsEnabled() override { return enable_; }
  std::string Name() override {return name_;}

 protected:
  bool enable_ = false;

 private:
  bool PointCloudDownSample::DownSample(LidarFrame* lidar_frame, float * points_array);
  // to store lidar_frame
  LidarFrame* lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>> original_world_cloud_;

  base::PointFCloudPtr cur_cloud_ptr_;

  //time statistics
  double downsample_time_ = 0.0;
  std::string name_;


  bool enable_downsample_pointcloud_;
  bool enable_downsample_beams_;

};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
