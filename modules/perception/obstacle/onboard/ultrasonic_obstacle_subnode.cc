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

#include <map>

#include "modules/perception/obstacle/onboard/ultrasonic_obstacle_subnode.h"
#include "modules/perception/obstacle/ultrasonic/common/ultrasonic_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"

#include "modules/perception/lib/base/timer.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/lib/base/string_util.h"
#include "modules/common/log.h"
#include "gflags/gflags.h"

namespace apollo {
namespace perception {

using Eigen::Matrix4d;
using Eigen::Translation3d;
using Eigen::Quaterniond;
using common::Header;
using apollo::common::adapter::AdapterManager;

bool UltrasonicObstacleSubnode::InitInternal() {
    if (!set_ultrasonic_type(FLAGS_ultrasonic_type)) {
        return false;
    }

    RegistAllAlgorithm();

    // init algorithm plugin
    if (!init_algorithm_plugin()) {
        AERROR << "Failed to init algorithm plugin.";
        return false;
    }

    // parse reserve fileds
    std::map<std::string, std::string> reserve_field_map;
    if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
      AERROR << "Failed to parse reserve filed: " << reserve_;
      return false;
    }

    if (reserve_field_map.find("device_id") == reserve_field_map.end()) {
      AERROR << "Failed to find field device_id, reserve: " << reserve_;
      return false;
    }
    device_id_ = reserve_field_map["device_id"];

    CHECK(AdapterManager::GetUltrasonic())
        << "Failed to get Ultrasonic adapter";
    AdapterManager::AddUltrasonicCallback(
                    &UltrasonicObstacleSubnode::OnUltrasonic, this);

    AINFO << "Succeed to finish ultrasonic detector initialization!";

    return true;
}

void UltrasonicObstacleSubnode::OnUltrasonic(
                            const apollo::drivers::Ultrasonic& in_message) {
    AINFO << "OnUltrasonic.";
    _seq_num++;
    // detector process ultrasonic datas
    PERF_BLOCK_START();
    std::shared_ptr<SensorObjects> sensor_objects(new SensorObjects);
    const double timestamp = in_message.header().timestamp_sec()
                             + FLAGS_timestamp_offset;
    sensor_objects->timestamp = timestamp;
    sensor_objects->sensor_type = ULTRASONIC_12;
    sensor_objects->sensor_id = device_id_;
    sensor_objects->seq_num = _seq_num;

    if (!_ultrasonic_detector->detect(in_message, sensor_objects)) {
        AERROR << "Failed to run ultrasonic detect.";
        sensor_objects->error_code = apollo::common::PERCEPTION_ERROR_PROCESS;
        return;
    }
    if (!PublishDataAndEvent(timestamp, sensor_objects)) {
        AERROR << "Failed to publish data.";
        sensor_objects->error_code = apollo::common::PERCEPTION_ERROR_PROCESS;
        return;
    }
    AINFO << "ultrasonic object size: " << sensor_objects->objects.size();
    PERF_BLOCK_END("ultrasonic_detect");
}

bool UltrasonicObstacleSubnode::set_ultrasonic_type(const std::string& type) {
    if (type == "ULTRASONIC_12") {
        _ultrasonic_type = ULTRASONIC_12;
    } else {
        AERROR << "Failed to set ultrasonic type from "
               << "UltrasonicObstacleSubnode: unknown ultrasonic type, exit.";
        return false;
    }
    return true;
}

void UltrasonicObstacleSubnode::RegistAllAlgorithm() {
  RegisterFactoryUltrasonicObstacleDetector();
}

bool UltrasonicObstacleSubnode::init_algorithm_plugin() {
    AINFO << "onboard ultrasonic_detector: "
          << FLAGS_obs_onboard_ultrasonic_detector;

    _ultrasonic_detector.reset(
        BaseUltrasonicObstacleDetectorRegisterer::GetInstanceByName(
                                    FLAGS_obs_onboard_ultrasonic_detector));
    if (_ultrasonic_detector == nullptr) {
        AERROR << "Failed to get instance: "
               << FLAGS_obs_onboard_ultrasonic_detector;
        return false;
    }

    if (!_ultrasonic_detector->init(FLAGS_extrin_path,
                                    FLAGS_ultrasonic_id_list_path)) {
        AERROR << "Failed to init ultrasonic_detector: "
               << _ultrasonic_detector->name();
        return false;
    }

    AINFO << "Init algorithm plugin successfully, _ultrasonic_detector: "
            << _ultrasonic_detector->name();
    /// init share data
    CHECK(shared_data_manager_ != nullptr);
    // init preprocess_data
    const std::string processing_data_name("UltrasonicObjectData");
    processing_data_ = dynamic_cast<UltrasonicObjectData*>(
        shared_data_manager_->GetSharedData(processing_data_name));
    if (processing_data_ == nullptr) {
      AERROR << "Failed to get shared data instance "
             << processing_data_name;
      return false;
    }
    AINFO << "Init shared data successfully, data: "
          << processing_data_->name();
    return true;
}

bool UltrasonicObstacleSubnode::PublishDataAndEvent(
    double timestamp, const SharedDataPtr<SensorObjects>& data) {
  // set shared data
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return false;
  }
  processing_data_->Add(key, data);
  // pub events
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
  return true;
}


}  // namespace perception
}  // namespace apollo
