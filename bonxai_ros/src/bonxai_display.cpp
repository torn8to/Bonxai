#include "bonxai_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

namespace bonxai_ros {

BonxaiDisplay::BonxaiDisplay() {
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(0, 255, 0), "Color of the voxels.",
      this, SLOT(updateStyle()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateStyle()));

  use_z_color_property_ = new rviz_common::properties::BoolProperty(
      "Color Scaling by Z", true, "Scale color based on Z axis instead of using fixed color.",
      this, SLOT(updateStyle()));

  min_z_property_ = new rviz_common::properties::FloatProperty(
      "Min Z", 0.0, "Minimum Z value for color scaling.",
      this, SLOT(updateStyle()));

  max_z_property_ = new rviz_common::properties::FloatProperty(
      "Max Z", 3.0, "Maximum Z value for color scaling.",
      this, SLOT(updateStyle()));
}

BonxaiDisplay::~BonxaiDisplay() {
  cubes_.clear();
}

void BonxaiDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void BonxaiDisplay::processMessage(bonxai_ros::msg::BonxaiVoxelMap::ConstSharedPtr message) {
  cubes_.clear();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(message->header, position, orientation)) {
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  double res = message->voxel_resolution;
  uint32_t num_pts = message->num_points;

  cubes_.reserve(num_pts);

  cubes_.reserve(num_pts);

  for (uint32_t i = 0; i < num_pts; ++i) {
    if (i * 3 + 2 >= message->data.size()) {
      break; // Message data is malformed
    }

    auto cube = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Cube,
      context_->getSceneManager(),
      scene_node_
    );

    int32_t vx = message->data[i * 3 + 0];
    int32_t vy = message->data[i * 3 + 1];
    int32_t vz = message->data[i * 3 + 2];

    Ogre::Vector3 pos(vx * res, vy * res, vz * res);
    cube->setPosition(pos);
    cube->setScale(Ogre::Vector3(res, res, res));

    cubes_.push_back(std::move(cube));
  }

  updateStyle();
}

void BonxaiDisplay::updateStyle() {
  Ogre::ColourValue fixed_color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();
  fixed_color.a = alpha;

  bool use_z = use_z_color_property_->getBool();
  float min_z = min_z_property_->getFloat();
  float max_z = max_z_property_->getFloat();

  if (max_z == min_z) {
    max_z = min_z + 0.001f;
  }

  for (auto& cube : cubes_) {
    if (use_z) {
      float z = cube->getPosition().z;
      float norm = (z - min_z) / (max_z - min_z);
      norm = std::max(0.0f, std::min(1.0f, norm));
      
      float hue = 0.66f * (1.0f - norm);
      Ogre::ColourValue color;
      color.setHSB(hue, 1.0f, 1.0f);
      color.a = alpha;
      cube->setColor(color.r, color.g, color.b, color.a);
    } else {
      cube->setColor(fixed_color.r, fixed_color.g, fixed_color.b, fixed_color.a);
    }
  }
}

}  // namespace bonxai_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bonxai_ros::BonxaiDisplay, rviz_common::Display)
