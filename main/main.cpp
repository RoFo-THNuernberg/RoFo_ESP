#include "esp_err.h"
#include "nvs_flash.h"

#include "Wifi.h"
#include "SensorPose.h"
#include "Marvelmind.h"
#include "NodeHandle.h"
#include "Publisher.h"
#include "RosMsgs.h"
#include "StateMachine.h"

using namespace MARVELMIND;
using namespace WIFI;

#define TAG "Main"

void callback_test(ros_msgs::RosMsg const& p)
{ 
  ros_msgs::Pose2D& pose = (ros_msgs::Pose2D&)p;
  ESP_LOGI(TAG,  "Callback: X: %f, Y: %f, Theta: %f\n", pose.x, pose.y,  pose.theta);
}


extern "C" void app_main(void)
{   
  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  Wifi wifi{};

  ESP_ERROR_CHECK(wifi.init());
  ESP_ERROR_CHECK(wifi.begin());

  ros::NodeHandle nh("robot_1");

  auto pub = nh.advertise<ros_msgs::Pose2D>("pose2D");

  nh.subscribe<ros_msgs::Pose2D>("set_pose2D", callback_test);  

  StateMachine state_machine;
  nh.subscribe<ros_msgs::Point2D>("goal_point", std::bind(&StateMachine::set_goal_point, &state_machine, std::placeholders::_1));
  
  SensorPose *pose_sensor = new Marvelmind();
  ESP_ERROR_CHECK(pose_sensor->init());

  while(1) 
  { 
      ros_msgs::Pose2D x = pose_sensor->get_Pose();
      printf("X: %f, Y: %f, Theta: %f\n", x.x, x.y,  x.theta);

      ros_msgs::Pose2D pose(x.x, x.y, x.theta);

      pub.publish(pose);

      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
