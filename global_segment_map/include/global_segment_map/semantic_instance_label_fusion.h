#ifndef GLOBAL_SEGMENT_MAP_SEMANTIC_LABEL_FUSION_H_
#define GLOBAL_SEGMENT_MAP_SEMANTIC_LABEL_FUSION_H_

#include <map>

#include "global_segment_map/common.h"

namespace voxblox {

class SemanticInstanceLabelFusion {
 public:

  bool getNextInstanceMerge(const SemanticLabel& semantic_label, InstanceLabel& new_label, InstanceLabel& old_label, std::set<InstanceLabel>& checked_labels);

  void mergeInstanceLabel(const SemanticLabel& semantic_label, const InstanceLabel& new_label, const InstanceLabel& old_label);

  void mergeInstanceLabels();

  void computeInstanceMergeCandidate(const Label& label, const InstanceLabel& instance_label, const SemanticLabel& semantic_label, const int count = 1);

  void increaseLabelInstanceCount(const Label& label,
                                  const InstanceLabel& instance_label);

  void decreaseLabelInstanceCount(const Label& label,
                                  const InstanceLabel& instance_label);

  void increaseLabelFramesCount(const Label& label, const int count = 1);

  void increaseClassInstanceLabelCount(const Label& label, const InstanceLabel& instance_label, 
                                        const SemanticLabel& semantic_label, const int count = 1);

  void increaseLabelClassInstanceCount(const Label& label, const InstanceLabel& instance_label, 
                                        const SemanticLabel& semantic_label, const int count = 1);

  void increaseClassInstanceCount(const InstanceLabel& instance_label, const SemanticLabel& semantic_label, const int count = 1);

  void increaseLabelPendingInstanceCount(const Label& label, const InstanceLabel& instance_label);

  SemanticLabel getPendingInstanceLabelClass(const Label& label, const InstanceLabel& instance_label) const;

  void checkPendingInstanceLabel(const Label& label, const SemanticLabel& semantic_label, bool enable_instance_label_merging);

//   InstanceLabel getInstanceLabel(
//       const Label& label, const std::set<InstanceLabel>& assigned_instances =
//                               std::set<InstanceLabel>()) const;

//   InstanceLabel getInstanceLabel(
//       const Label& label, const float count_threshold_factor,
//       const std::set<InstanceLabel>& assigned_instances =
//           std::set<InstanceLabel>()) const;
  InstanceLabel getInstanceLabel(const Label& label, const float count_threshold_factor = 0.0f,
                                 const std::set<InstanceLabel>& assigned_instances = std::set<InstanceLabel>()) const;

  InstanceLabel getInstanceLabel(const Label& label, const SemanticLabel& semantic_label,
                                 const std::set<InstanceLabel>& assigned_instances = std::set<InstanceLabel>()) const;

  InstanceLabel getInstanceLabel(const Label& label, const SemanticLabel& semantic_label, const float count_threshold_factor = 0.0f,
                                 const std::set<InstanceLabel>& assigned_instances = std::set<InstanceLabel>()) const;
                    
  std::pair<SemanticLabel, InstanceLabel> getSemanticInstanceLabel(const Label& label, const float count_threshold_factor) const;

  void increaseLabelClassCount(const Label& label, const SemanticLabel& semantic_label, const int count = 1);

  SemanticLabel getSemanticLabel(const Label& label) const;

 protected:
  std::map<Label, int> label_frames_count_;
  std::map<Label, std::map<InstanceLabel, int>> label_instance_count_;
  std::map<Label, std::map<SemanticLabel, std::map<InstanceLabel, int>>> label_class_instance_count_;
  std::map<Label, std::map<InstanceLabel, int>> label_pending_instance_count_;

  // Per frame voxel count of semantic label.
  LSLMap label_class_count_;

  // Used for instance label fusion
  std::map<SemanticLabel, std::map<InstanceLabel, std::map<Label, int>>> class_instance_label_count_;
  std::map<SemanticLabel, std::map<InstanceLabel, int>> class_instance_count_;
  std::map<SemanticLabel, std::map<InstanceLabel, std::map<InstanceLabel, int>>> instance_merge_candidate_;

 
};

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_SEMANTIC_LABEL_FUSION_H_
