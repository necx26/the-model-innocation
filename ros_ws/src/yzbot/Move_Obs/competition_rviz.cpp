/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include <vector>

#include <QDateTime>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <cstdlib>
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/resource_config.hpp"
#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"

int main(int argc, char ** argv)
{
  // remove ROS arguments before passing to QApplication
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

  QApplication qapp(non_ros_argc, non_ros_args_c_strings.data());
  if (const char * media = std::getenv("YZBOT_RVIZ_MEDIA")) {
    rviz_rendering::set_resource_directory(media);
  }

  // TODO(wjwwood): use node's logger here in stead
  auto logger = rclcpp::get_logger("rviz2");
  // install logging handlers to route logging through ROS's logging system
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    }
  );

  rviz_common::VisualizerApp vapp(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  vapp.setApp(&qapp);
  if (vapp.init(argc, argv)) {
    QTimer health;
    QObject::connect(&health, &QTimer::timeout, [&qapp]() {
      if (!std::getenv("YZBOT_LOG")) {return;}
      QJsonArray displays;
      bool healthy=true;
      for (auto * widget : qapp.topLevelWidgets()) {
        auto * frame=qobject_cast<rviz_common::VisualizationFrame *>(widget);
        if (!frame || !frame->getManager()) {continue;}
        auto * group=frame->getManager()->getRootDisplayGroup();
        for (int i=0;i<group->numDisplays();++i) {
          auto * display=group->getDisplayAt(i);
          if (!display->isEnabled()) {continue;}
          QJsonObject item;item["name"]=display->getName();
          rviz_common::properties::StatusProperty * status=nullptr;
          for (int j=0;j<display->numChildren();++j) {
            auto * property=display->childAt(j);
            if (property->getName().startsWith("Status")) {
              status=dynamic_cast<rviz_common::properties::StatusProperty *>(property);break;
            }
          }
          int level=status ? static_cast<int>(status->getLevel()) : 0;
          item["level"]=level;item["status"]=status ? status->getValue().toString() : "OK";
          QJsonArray details;
          if (status) {
            for (int j=0;j<status->numChildren();++j) {
              auto * child=status->childAt(j);QJsonObject entry;
              entry["name"]=child->getName();entry["value"]=child->getValue().toString();details.append(entry);
            }
          }
          item["details"]=details;displays.append(item);healthy=healthy && level==0;
        }
      }
      QJsonObject report;report["healthy"]=healthy && !displays.isEmpty();report["displays"]=displays;
      report["timestamp"]=QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
      QFile output(QString::fromLocal8Bit(std::getenv("YZBOT_LOG"))+"/rviz-health.json");
      if (output.open(QIODevice::WriteOnly|QIODevice::Truncate)) {output.write(QJsonDocument(report).toJson());}
    });
    health.start(2000);
    return qapp.exec();
  } else {
    return 1;
  }
}
