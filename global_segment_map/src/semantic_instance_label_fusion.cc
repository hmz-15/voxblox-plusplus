#include "global_segment_map/semantic_instance_label_fusion.h"

namespace voxblox {

bool SemanticInstanceLabelFusion::getNextInstanceMerge(const SemanticLabel& semantic_label, InstanceLabel& new_label, 
    InstanceLabel& old_label, std::set<InstanceLabel>& checked_labels)
{
    auto sem_it = instance_merge_candidate_.find(semantic_label);
    if (sem_it != instance_merge_candidate_.end())
    {
        // Organize all instance labels in ascent order, first merge the newest instance
        std::vector<InstanceLabel> ordered_labels;
        for (auto inst_merge_it = sem_it->second.begin(); inst_merge_it != sem_it->second.end(); inst_merge_it++)
        {
            if (checked_labels.find(inst_merge_it->first) != checked_labels.end())
                continue;
            ordered_labels.push_back(inst_merge_it->first);
        }
        std::sort(ordered_labels.begin(), ordered_labels.end()); 

        // Loop over all instance labels
        while (ordered_labels.size() > 0)
        {
            InstanceLabel max_label = ordered_labels.back();
            checked_labels.emplace(max_label);
            std::map<InstanceLabel, int> candidate_map = sem_it->second.find(max_label)->second;

            std::vector<std::pair<int, InstanceLabel>> ordered_candidate_labels;
            for (auto inst_it = candidate_map.begin(); inst_it != candidate_map.end(); inst_it++)
            {
                ordered_candidate_labels.push_back(std::make_pair(inst_it->second, inst_it->first));
            }     

            auto cmp = [](const auto& a, const auto& b)->bool {return a.first < b.first;};
            std::sort(ordered_candidate_labels.begin(), ordered_candidate_labels.end(), cmp);  
            
            // Loop over all instance label candidates
            while (ordered_candidate_labels.size() > 0)
            {
                InstanceLabel max_candidate_label = ordered_candidate_labels.back().second;
                int max_count = ordered_candidate_labels.back().first;

                if (semantic_label > 80u && semantic_label != 122u && semantic_label != 121u)
                // if (semantic_label > 80u)
                {
                    // If stuff, merge all instances with label segments in common
                    new_label = max_label;
                    old_label = max_candidate_label;
                    return true;
                }
                else
                {
                    // If thing, merge instances if IOU over a threshood
                    auto sem_it_i = class_instance_count_.find(semantic_label)->second.find(max_label);
                    auto sem_it_j = class_instance_count_.find(semantic_label)->second.find(max_candidate_label);
                    if ((sem_it_i != class_instance_count_.find(semantic_label)->second.end()) && (sem_it_j != class_instance_count_.find(semantic_label)->second.end()))
                    {
                        if ((max_count > 0.5 * (sem_it_i->second + sem_it_j->second)) && (max_count > 10))
                        {
                            new_label = max_label;
                            old_label = max_candidate_label;
                            return true;
                        }
                    }   
                }
                ordered_candidate_labels.erase(ordered_candidate_labels.end()-1);
            }
            ordered_labels.erase(ordered_labels.end()-1);
        }
    }
    return false;
}

void SemanticInstanceLabelFusion::mergeInstanceLabel(const SemanticLabel& semantic_label, const InstanceLabel& new_label, const InstanceLabel& old_label)
{
    // Update label_class_instance_count_
    for (auto label_it = label_class_instance_count_.begin(); label_it != label_class_instance_count_.end(); label_it++)
    {
        auto sem_it = label_it->second.find(semantic_label);
        if (sem_it != label_it->second.end())
        {
            auto inst_it_new = sem_it->second.find(new_label);
            if (inst_it_new != sem_it->second.end())
            {
                // Actually need to add to old labels
                auto inst_it_old = sem_it->second.find(old_label);
                if (inst_it_old != sem_it->second.end())
                    inst_it_old->second += inst_it_new->second;
                else
                    sem_it->second.insert(std::make_pair(old_label, inst_it_new->second));
                sem_it->second.erase(inst_it_new);
            }
        }
    }

    // Update class_instance_label_count_
    auto sem_itt = class_instance_label_count_.find(semantic_label);
    if (sem_itt != class_instance_label_count_.end())
    {
        auto inst_it = sem_itt->second.find(new_label);
        if (inst_it != sem_itt->second.end())
            sem_itt->second.erase(inst_it);
    }

    // Update pending instance labels
    for (auto pending_label_it = label_pending_instance_count_.begin(); pending_label_it != label_pending_instance_count_.end(); pending_label_it++)
    {
        auto pending_inst_it_new = pending_label_it->second.find(new_label);
        if (pending_inst_it_new != pending_label_it->second.end())
        {
            // auto pending_inst_it_old = pending_label_it->second.find(old_label);
            // if (pending_inst_it_old != pending_label_it->second.end())
            //     pending_inst_it_old->second += pending_inst_it_new->second;
            // else
            //     pending_label_it->second.insert(std::make_pair(old_label, pending_inst_it_new->second));
            pending_label_it->second.erase(pending_inst_it_new);
        }
    }

    // Update instance_merge_candidate_ by erasing the new instance label
    auto sem_it = instance_merge_candidate_.find(semantic_label);
    if (sem_it != instance_merge_candidate_.end())
    {   
        for (auto inst_it = sem_it->second.begin(); inst_it != sem_it->second.end(); )
        {
            if (inst_it->first == new_label)
            {  
                inst_it = sem_it->second.erase(inst_it);
                continue;
            }
            else
            {
                for (auto inst_count_it = inst_it->second.begin(); inst_count_it != inst_it->second.end(); )
                {
                    if (inst_count_it->first == new_label)
                    {
                        inst_count_it = inst_it->second.erase(inst_count_it);
                        continue;
                    }
                    else
                        inst_count_it++;
                }
                inst_it++;
            } 
        }
    }
}

void SemanticInstanceLabelFusion::mergeInstanceLabels()
{
    // Merge instance labels that own labels in common
    for (auto sem_it = instance_merge_candidate_.begin(); sem_it != instance_merge_candidate_.end(); sem_it++)
    {
        SemanticLabel semantic_label = sem_it->first;
        std::set<InstanceLabel> checked_labels;
        InstanceLabel new_label;
        InstanceLabel old_label;
        while (getNextInstanceMerge(semantic_label, new_label, old_label, checked_labels))
            mergeInstanceLabel(semantic_label, new_label, old_label);        
    }

}

void SemanticInstanceLabelFusion::computeInstanceMergeCandidate(const Label& label, const InstanceLabel& instance_label, 
    const SemanticLabel& semantic_label, const int count)
{
    auto sem_it = instance_merge_candidate_.find(semantic_label);
    if (sem_it == instance_merge_candidate_.end())
    {
        // Add new class to instance_merge_candidate
        std::map<InstanceLabel, int> inst_count_map = {};
        std::map<InstanceLabel, std::map<InstanceLabel, int>> inst_map = {std::make_pair(instance_label, inst_count_map)};
        instance_merge_candidate_.insert(std::make_pair(semantic_label, inst_map));
        sem_it = instance_merge_candidate_.find(semantic_label);
    }
    auto inst_it = sem_it->second.find(instance_label);
    if (inst_it == sem_it->second.end())
    {
        // Add new instance
        std::map<InstanceLabel, int> inst_count_map = {};
        sem_it->second.insert(std::make_pair(instance_label, inst_count_map));
        inst_it = sem_it->second.find(instance_label);
    }

    // The instance already has merging candidate, then loop over all instances
    // If another instance has common labels, update the count in instance_merge_candidate_
    std::map<InstanceLabel, std::map<Label, int>> instance_label_map = class_instance_label_count_.find(semantic_label)->second;
    for (auto inst_label_it = instance_label_map.begin(); inst_label_it != instance_label_map.end(); inst_label_it++)
    {
        InstanceLabel current_instance_label = inst_label_it->first;
        // Only consider pair-wise merging candidates
        if (current_instance_label == instance_label)
            continue;

        if (current_instance_label < instance_label)
        {
            // For the instances to merge into
            if (semantic_label < 80u || semantic_label == 122u || semantic_label == 121u)
            // if (semantic_label < 80u)
            {
                // For thing, check if there exists common labels; for stuff, merge all instance labels with same class
                auto label_it = inst_label_it->second.find(label);
                if (label_it == inst_label_it->second.end())
                    continue;
            }
            auto instance_merge_it = inst_it->second.find(current_instance_label);
            if (instance_merge_it != inst_it->second.end())
                instance_merge_it->second += count;
            else
                inst_it->second.insert(std::make_pair(current_instance_label, count));
        }
        else
        {
            // For the instances that merges to the observed instance 
            if (semantic_label < 80u || semantic_label == 122u || semantic_label == 121u)
            // if (semantic_label < 80u)
            {
                // For thing, check if there exists common labels; for stuff, merge all instance labels with same class
                auto label_it = inst_label_it->second.find(label);
                if (label_it == inst_label_it->second.end())
                    continue;
            }
            auto inst_itt = sem_it->second.find(current_instance_label);
            if (inst_itt != sem_it->second.end())
            {
                auto instance_merge_it = inst_itt->second.find(instance_label);
                if (instance_merge_it != inst_itt->second.end())
                    instance_merge_it->second += count;
                else
                    inst_itt->second.insert(std::make_pair(instance_label, count));
            }
            else
            {
                std::map<InstanceLabel, int> inst_count_map = {std::make_pair(instance_label, count)};
                sem_it->second.insert(std::make_pair(current_instance_label, inst_count_map));
            }
        }     
    }
}

void SemanticInstanceLabelFusion::increaseClassInstanceLabelCount(const Label& label, const InstanceLabel& instance_label, 
    const SemanticLabel& semantic_label, const int count)
{
    auto sem_it = class_instance_label_count_.find(semantic_label);
    if (sem_it != class_instance_label_count_.end())
    {
        auto inst_it = sem_it->second.find(instance_label);
        if (inst_it != sem_it->second.end())
        {
            // Also increase class-instance count
            auto label_it = inst_it->second.find(label);
            if (label_it != inst_it->second.end())
                label_it->second += count;
            else
                inst_it->second.insert(std::make_pair(label, count));
        }
        else
        {
            std::map<Label, int> label_map = {std::make_pair(label, count)}; 
            std::pair<InstanceLabel, std::map<Label, int>> inst_pair = std::make_pair(instance_label, label_map);
            sem_it->second.insert(inst_pair);
        }
    }
    else
    {
        std::map<Label, int> label_map = {std::make_pair(label, count)}; 
        std::map<InstanceLabel, std::map<Label, int>> inst_map = {std::make_pair(instance_label, label_map)};
        std::pair<SemanticLabel, std::map<InstanceLabel, std::map<Label, int>>> sem_pair = {std::make_pair(semantic_label, inst_map)};
        class_instance_label_count_.insert(sem_pair);
    } 
}

void SemanticInstanceLabelFusion::increaseLabelClassInstanceCount(const Label& label, const InstanceLabel& instance_label, 
    const SemanticLabel& semantic_label, const int count)
{
    auto label_it = label_class_instance_count_.find(label);
    if (label_it != label_class_instance_count_.end())
    {
        auto sem_it = label_it->second.find(semantic_label);
        if (sem_it != label_it->second.end())
        {
            auto inst_it = sem_it->second.find(instance_label);
            if (inst_it != sem_it->second.end())
                inst_it->second += count;
            else
                sem_it->second.insert(std::make_pair(instance_label, count));
        }
        else
        {
            std::map<InstanceLabel, int> inst_map = {std::make_pair(instance_label, count)}; 
            std::pair<SemanticLabel, std::map<InstanceLabel, int>> sem_pair = std::make_pair(semantic_label, inst_map);
            label_it->second.insert(sem_pair);
        }
    }
    else
    {
        std::map<InstanceLabel, int> inst_map = {std::make_pair(instance_label, count)}; 
        std::map<SemanticLabel, std::map<InstanceLabel, int>> sem_map = {std::make_pair(semantic_label, inst_map)};
        std::pair<Label, std::map<SemanticLabel, std::map<InstanceLabel, int>>> label_pair = std::make_pair(label, sem_map);
        label_class_instance_count_.insert(label_pair);
    } 
}

void SemanticInstanceLabelFusion::increaseClassInstanceCount(const InstanceLabel& instance_label, const SemanticLabel& semantic_label, const int count)
{
    auto sem_it = class_instance_count_.find(semantic_label);
    if (sem_it != class_instance_count_.end())
    {
        auto inst_it = sem_it->second.find(instance_label);
        if (inst_it != sem_it->second.end())
            inst_it->second += count;
        else
            sem_it->second.insert(std::make_pair(instance_label, count));
    }
    else
    {
        std::map<InstanceLabel, int> inst_map = {std::make_pair(instance_label, count)};
        class_instance_count_.insert(std::make_pair(semantic_label, inst_map));
    }
}

void SemanticInstanceLabelFusion::increaseLabelInstanceCount(
    const Label& label, const InstanceLabel& instance_label) {
  auto label_it = label_instance_count_.find(label);
  if (label_it != label_instance_count_.end()) {
    auto instance_it = label_it->second.find(instance_label);
    if (instance_it != label_it->second.end()) {
      ++instance_it->second;
    } else {
      label_it->second.emplace(instance_label, 1);
    }
  } else {
    std::map<InstanceLabel, int> instance_count;
    instance_count.emplace(instance_label, 1);
    label_instance_count_.emplace(label, instance_count);
  }
}

void SemanticInstanceLabelFusion::decreaseLabelInstanceCount(
    const Label& label, const InstanceLabel& instance_label) {
  auto label_it = label_instance_count_.find(label);
  if (label_it != label_instance_count_.end()) {
    auto instance_it = label_it->second.find(instance_label);
    if (instance_it != label_it->second.end()) {
      --instance_it->second;
    } else {
      LOG(FATAL) << "Decreasing a non existing label-instance count.";
    }
  } else {
    LOG(FATAL) << "Decreasing a non existing label-instance count.";
  }
}

void SemanticInstanceLabelFusion::increaseLabelFramesCount(const Label& label, const int count) {
  auto label_count_it = label_frames_count_.find(label);
  if (label_count_it != label_frames_count_.end()) {
    label_count_it->second += count;
  } else {
    label_frames_count_.insert(std::make_pair(label, count));
  }
}

void SemanticInstanceLabelFusion::increaseLabelPendingInstanceCount(const Label& label, const InstanceLabel& instance_label)
{
    auto label_it = label_pending_instance_count_.find(label);
    if (label_it != label_pending_instance_count_.end())
    {
        auto instance_it = label_it->second.find(instance_label);
        if (instance_it != label_it->second.end())
            instance_it->second++;
        else
            label_it->second.insert(std::make_pair(instance_label, 1));
    }
    else
    {
        std::map<InstanceLabel, int> pending_inst_count_map = {std::make_pair(instance_label, 1)};
        label_pending_instance_count_.insert(std::make_pair(label, pending_inst_count_map)); 
    }
}

SemanticLabel SemanticInstanceLabelFusion::getPendingInstanceLabelClass(const Label& label, const InstanceLabel& instance_label) const
{
    SemanticLabel semantic_label = 80u;
    auto label_it = label_class_instance_count_.find(label);
    if (label_it != label_class_instance_count_.end())
    {
        for (auto const& class_instance_count: label_it->second)
        {
            auto inst_it = class_instance_count.second.find(instance_label);
            if ((inst_it != class_instance_count.second.end()) && (inst_it->second > 2))
                semantic_label = class_instance_count.first;
        }
    }
    return semantic_label;
}

void SemanticInstanceLabelFusion::checkPendingInstanceLabel(const Label& label, const SemanticLabel& semantic_label, bool enable_instance_label_merging)
{
    auto label_it = label_pending_instance_count_.find(label);
    if (label_it != label_pending_instance_count_.end())
    {
        auto label_itt = label_class_instance_count_.find(label);
        if (label_itt != label_class_instance_count_.end())
        {
            auto sem_it = label_itt->second.find(semantic_label);
            if (sem_it != label_itt->second.end())
            {
                for (auto pending_inst_it = label_it->second.begin(); pending_inst_it != label_it->second.end();)
                {  
                    auto inst_it = sem_it->second.find(pending_inst_it->first);
                    if ((inst_it != sem_it->second.end()) && (inst_it->second > 2))
                    {
                        increaseLabelClassInstanceCount(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
                        increaseLabelClassCount(label, semantic_label, pending_inst_it->second);
                        increaseLabelFramesCount(label, pending_inst_it->second);
                        if (enable_instance_label_merging)
                        {
                            increaseClassInstanceLabelCount(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
                            increaseClassInstanceCount(pending_inst_it->first, semantic_label, pending_inst_it->second);  
                            computeInstanceMergeCandidate(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
                        }
                        pending_inst_it = label_it->second.erase(pending_inst_it);
                    }
                    else
                        pending_inst_it++;
                }
            }
        }
        // auto sem_it = class_instance_label_count_.find(semantic_label);
        // if (sem_it != class_instance_label_count_.end())
        // {
        //     for (auto pending_inst_it = label_it->second.begin(); pending_inst_it != label_it->second.end();)
        //     {  
        //         if (sem_it->second.find(pending_inst_it->first) != sem_it->second.end())
        //         {
        //             increaseLabelClassInstanceCount(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
        //             increaseLabelClassCount(label, semantic_label, pending_inst_it->second);
        //             increaseLabelFramesCount(label, pending_inst_it->second);
        //             // increaseLabelClassInstanceCount(label, pending_inst_it->first, semantic_label, 1);
        //             // increaseLabelClassCount(label, semantic_label, 1);
        //             if (enable_instance_label_merging)
        //             {
        //                 // increaseClassInstanceLabelCount(label, pending_inst_it->first, semantic_label, 1);
        //                 // increaseClassInstanceCount(pending_inst_it->first, semantic_label, 1);  
        //                 // computeInstanceMergeCandidate(label, pending_inst_it->first, semantic_label, 1);
        //                 increaseClassInstanceLabelCount(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
        //                 increaseClassInstanceCount(pending_inst_it->first, semantic_label, pending_inst_it->second);  
        //                 computeInstanceMergeCandidate(label, pending_inst_it->first, semantic_label, pending_inst_it->second);
        //             }
        //             pending_inst_it = label_it->second.erase(pending_inst_it);
        //         }
        //         else
        //             pending_inst_it++;
        //     }
        // }
    }
}

// InstanceLabel SemanticInstanceLabelFusion::getInstanceLabel(const Label& label, const float count_threshold_factor,
//                                  const std::set<InstanceLabel>& assigned_instances) const
// {
//     InstanceLabel instance_label = 0u;
//     SemanticLabel semantic_label = getSemanticLabel(label);
//     instance_label = getInstanceLabel(label, semantic_label, count_threshold_factor, assigned_instances);
//     return instance_label;
// }

// InstanceLabel SemanticInstanceLabelFusion::getInstanceLabel(const Label& label, const SemanticLabel& semantic_label,
//                                  const std::set<InstanceLabel>& assigned_instances) const
// {
//     InstanceLabel instance_label = 0u;
//     float count_threshold_factor = 0.0f;
//     instance_label = getInstanceLabel(label, semantic_label, count_threshold_factor, assigned_instances);
//     return instance_label;
// }

// InstanceLabel SemanticInstanceLabelFusion::getInstanceLabel(
//     const Label& label, const SemanticLabel& semantic_label, const float count_threshold_factor,
//     const std::set<InstanceLabel>& assigned_instances) const
// {
//     InstanceLabel instance_label = 0u;
//     int max_count = 0;
//     auto label_it = label_class_instance_count_.find(label);
//     if (label_it != label_class_instance_count_.end())
//     {
//         auto sem_it = label_it->second.find(semantic_label);
//         if (sem_it != label_it->second.end())
//         {
//             for (auto const& instance_count : sem_it->second)
//             {
//                 if (instance_count.second > max_count && instance_count.first != 0u &&
//                     assigned_instances.find(instance_count.first) == assigned_instances.end())
//                 {
//                     int frames_count = 0;
//                     auto label_count_it = label_frames_count_.find(label);
//                     if (label_count_it != label_frames_count_.end())
//                         frames_count = label_count_it->second;
//                     if (instance_count.second > count_threshold_factor * (float)(frames_count - instance_count.second))
//                     {
//                         instance_label = instance_count.first;
//                         max_count = instance_count.second;
//                     } 
//                 }
//             }
//         }                
//     }
//     return instance_label;
// }

// std::pair<SemanticLabel, InstanceLabel> SemanticInstanceLabelFusion::getSemanticInstanceLabel(
//     const Label& label, const float count_threshold_factor) const 
// {
//     InstanceLabel instance_label = 0u;
//     SemanticLabel semantic_label = getSemanticLabel(label);
//     if (semantic_label != 80u)
//         instance_label = getInstanceLabel(label, semantic_label, count_threshold_factor);
//     return std::make_pair(semantic_label, instance_label);
// }

InstanceLabel SemanticInstanceLabelFusion::getInstanceLabel(
    const Label& label,
    const std::set<InstanceLabel>& assigned_instances) const {
  return getInstanceLabel(label, 0.0f, assigned_instances);
}

InstanceLabel SemanticInstanceLabelFusion::getInstanceLabel(
    const Label& label, const float count_threshold_factor,
    const std::set<InstanceLabel>& assigned_instances) const {
  InstanceLabel instance_label = 0u;
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
            count_threshold_factor *
                (float)(frames_count - instance_count.second)) {
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

void SemanticInstanceLabelFusion::increaseLabelClassCount(
    const Label& label, const SemanticLabel& semantic_label, const int count) {
  auto label_it = label_class_count_.find(label);
  if (label_it != label_class_count_.end()) {
    auto class_it = label_it->second.find(semantic_label);
    if (class_it != label_it->second.end()) {
      class_it->second += count;
    } else {
      label_it->second.emplace(semantic_label, count);
    }
  } else {
    SLMap class_points_count;
    class_points_count.emplace(semantic_label, count);
    label_class_count_.emplace(label, class_points_count);
  }
}

// SemanticLabel SemanticInstanceLabelFusion::getSemanticLabel(
//     const Label& label) const {
//   SemanticLabel semantic_label = 80u;

//   int max_count = 0;
//   auto label_it = label_class_count_.find(label);
//   if (label_it != label_class_count_.end()) {
//     for (auto const& class_count : label_it->second) {
//       if (class_count.second > max_count &&
//           class_count.first != 80u) {
//         semantic_label = class_count.first;
//         max_count = class_count.second;
//       }
//     }
//   }
//   return semantic_label;
// }
SemanticLabel SemanticInstanceLabelFusion::getSemanticLabel(
    const Label& label) const {
  SemanticLabel semantic_label = 80u;

  if (getInstanceLabel(label) == BackgroundLabel) {
    return semantic_label;
  }
  int max_count = 0;
  auto label_it = label_class_count_.find(label);
  if (label_it != label_class_count_.end()) {
    for (auto const& class_count : label_it->second) {
      if (class_count.second > max_count &&
          class_count.first != BackgroundLabel) {
        semantic_label = class_count.first;
        max_count = class_count.second;
      }
    }
  }
  return semantic_label;
}



}  // namespace voxblox
