#include "global_segment_map/label_fusion.h"

namespace voxblox {
namespace utils {

void InstanceLabelFusion::increaseLabelInstanceCount(
    const Label& label, const SemanticLabel& instance_label) {
  auto label_it = label_instance_count_.find(label);
  if (label_it != label_instance_count_.end()) {
    auto instance_it = label_it->second.find(instance_label);
    if (instance_it != label_it->second.end()) {
      ++instance_it->second;
    } else {
      label_it->second.emplace(instance_label, 1);
    }
  } else {
    SLMap instance_count;
    instance_count.emplace(instance_label, 1);
    label_instance_count_.emplace(label, instance_count);
  }
}

void InstanceLabelFusion::decreaseLabelInstanceCount(
    const Label& label, const InstanceLabel& instance_label) {
  auto label_it = label_instance_count_.find(label);
  if (label_it != label_instance_count_.end()) {
    auto instance_it = label_it->second.find(instance_label);
    if (instance_it != label_it->second.end()) {
      --instance_it->second;
    } else {
      // label_it->second.emplace(instance_label, 1);
    }
  } else {
    // SLMap instance_count;
    // instance_count.emplace(instance_label, 1);
    // label_instance_count_.emplace(label, instance_count);
  }
}

void InstanceLabelFusion::increaseLabelFramesCount(const Label& label) {
  auto label_count_it = label_frames_count_.find(label);
  if (label_count_it != label_frames_count_.end()) {
    ++label_count_it->second;
  } else {
    label_frames_count_.insert(std::make_pair(label, 1));
  }
}

InstanceLabel InstanceLabelFusion::getLabelInstance(
    const Label& label,
    const std::set<InstanceLabel>& assigned_instances) const {
  SemanticLabel instance_label = 0u;
  int max_count = 0;
  auto label_it = label_instance_count_.find(label);
  if (label_it != label_instance_count_.end()) {
    for (auto const& instance_count : label_it->second) {
      if (instance_count.second > max_count && instance_count.first != 0u &&
          assigned_instances.find(instance_count.first) ==
              assigned_instances.end()) {
        int frames_count = 0;
        auto label_count_it = label_frames_count_.find(label);
        if (label_count_it != label_frames_count_.end()) {
          frames_count = label_count_it->second;
        }
        if (instance_count.second >
            0.0 * (float)(frames_count - instance_count.second)) {
          instance_label = instance_count.first;
          max_count = instance_count.second;
        }
      }
    }
  } else {
    // LOG(ERROR) << "No semantic class for label?";
  }
  // TODO(margaritaG): handle this remeshing!!
  // auto prev_instance_it = label_instance_map_.find(label);
  //   if (prev_instance_it != label_instance_map_.end()) {
  //     if (prev_instance_it->second != instance_label) {
  //       *remesh_ptr_ = true;
  //     }
  //   }
  //   label_instance_map_[label] = instance_label;
  //   return instance_label;

  return instance_label;
}

void SemanticLabelFusion::increaseLabelClassCount(
    const Label& label, const SemanticLabel& semantic_label) {
  auto label_it = label_class_count_.find(label);
  if (label_it != label_class_count_.end()) {
    auto class_it = label_it->second.find(semantic_label);
    if (class_it != label_it->second.end()) {
      ++class_it->second;
    } else {
      label_it->second.emplace(semantic_label, 1);
    }
  } else {
    SLMap class_points_count;
    class_points_count.emplace(semantic_label, 1);
    label_class_count_.emplace(label, class_points_count);
  }
}

SemanticLabel SemanticLabelFusion::getSemanticLabel(const Label& label) const {
  SemanticLabel semantic_label = 0;
  int max_count = 0;
  auto label_it = label_class_count_.find(label);
  if (label_it != label_class_count_.end()) {
    for (auto const& class_count : label_it->second) {
      if (class_count.second > max_count && class_count.first != 0u) {
        semantic_label = class_count.first;
        max_count = class_count.second;
      }
    }
  }
  return semantic_label;
}

}  // namespace utils
}  // namespace voxblox