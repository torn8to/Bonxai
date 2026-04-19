#ifndef BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_

#include <memory>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <vector>

#include "bonxai_ros/msg/bonxai_voxel_map.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

namespace bonxai_ros {
class BonxaiDisplay : public rviz_common::MessageFilterDisplay<bonxai_ros::msg::BonxaiVoxelMap> {
  Q_OBJECT
 public:
  BonxaiDisplay();
  ~BonxaiDisplay();

 private Q_SLOTS:
  void updateStyle();

 protected:
  void onInitialize() override;
  void processMessage(bonxai_ros::msg::BonxaiVoxelMap::ConstSharedPtr msg) override;

  std::vector<std::unique_ptr<rviz_rendering::Shape>> cubes_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::BoolProperty* use_z_color_property_;
  rviz_common::properties::FloatProperty* min_z_property_;
  rviz_common::properties::FloatProperty* max_z_property_;
};
}  // namespace bonxai_ros

#endif  // BONXAI_RVIZ_PLUGIN__BONXAI_DISPLAY_HPP_