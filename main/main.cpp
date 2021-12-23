#include "esp_err.h"
#include "nvs_flash.h"

#include "Wifi.h"
#include "SensorPose.h"
#include "Marvelmind.h"
#include "SensorPoseSim.h"
#include "NodeHandle.h"
#include "Publisher.h"
#include "RosMsgs.h"
#include "StateMachine.h"
#include "ControllerMaster.h"
#include "OutputVelocity.h"
#include "OutputVelocitySim.h"

using namespace MARVELMIND;
using namespace WIFI;

#define TAG "Main"
#define USE_SIM


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

  #ifdef USE_SIM
    ros::NodeHandle& nh = ros::NodeHandle::init("turtle1");
    SensorPose& pose_sensor = SensorPoseSim::init(nh);
    OutputVelocity& output_velocity = OutputVelocitySim::init(nh);
  #elif
    ros::NodeHandle& nh = ros::NodeHandle::init("robot_1");
    SensorPose& pose_sensor = Marvelmind::init(nh);
    //OutputVelocity& output_velocity = OutputVelocitySim::init(nh);
  #endif

  auto pub = nh.advertise<ros_msgs::Pose2D>("pose2D");

  StateMachine state_machine;
  nh.subscribe<ros_msgs::Point2D>("goal_point", std::bind(&StateMachine::set_goal_point, &state_machine, std::placeholders::_1));
  nh.subscribe<ros_msgs::Twist2D>("vel", std::bind(&StateMachine::set_velocity, &state_machine, std::placeholders::_1));

  while(1) 
  { 
    ros_msgs::Pose2D x = pose_sensor.get_Pose();
    printf("X: %f, Y: %f, Theta: %f\n", x.x, x.y,  x.theta);

    ros_msgs::Pose2D pose(x.x, x.y, x.theta);

    pub.publish(pose);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
