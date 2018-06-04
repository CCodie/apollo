/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <set>
#include "Eigen/Dense"
#include "Eigen/Core"

#include "modules/drivers/proto/ultrasonic_radar.pb.h"
#include "modules/perception/proto/perception_ultrasonic.pb.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/ultrasonic/interface/base_ultrasonic_obstacle_detector.h"
#include "modules/perception/obstacle/ultrasonic/detector/ultrasonic_obstacle_detector.h"
#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/common/geometry_util.h"


namespace apollo {
namespace perception {

class UltrasonicObstacleSubnode : public Subnode {
 public:
  UltrasonicObstacleSubnode() : _seq_num(0) {}
  ~UltrasonicObstacleSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  void OnUltrasonic(const apollo::drivers::Ultrasonic& message);
  bool InitInternal() override;
  void RegistAllAlgorithm();
  bool init_algorithm_plugin();
  bool set_ultrasonic_type(const std::string& type);
  bool PublishDataAndEvent(double timestamp,
                           const SharedDataPtr<SensorObjects>& data);

 private:
  uint32_t _seq_num;

  SensorType _ultrasonic_type;

  std::shared_ptr<BaseUltrasonicObstacleDetector> _ultrasonic_detector;
  // share data
  UltrasonicObjectData* processing_data_ = nullptr;
  std::string device_id_;

  DISALLOW_COPY_AND_ASSIGN(UltrasonicObstacleSubnode);
};

REGISTER_SUBNODE(UltrasonicObstacleSubnode);
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_
