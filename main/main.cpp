#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "Wifi.h"
#include "SensorPose.h"
#include "Marvelmind.h"
#include "SensorPoseSim.h"
#include "NodeHandle.h"
#include "Publisher.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "StateMachine.h"
#include "OutputVelocity.h"
#include "OutputVelocitySim.h"
#include "OutputVelocityImpl.h"
#include "KalmanFilter.h"
#include "MotorController.h"
#include "ControllerMaster.h"

#define TAG "Main"
//#define USE_SIM
#define KALMAN


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

  MotorController& motor_controller = MotorController::init();

  #ifdef USE_SIM
    ros::NodeHandle& node_handle = ros::NodeHandle::init("turtle1");
    SensorPose& pose_sensor = SensorPoseSim::init(node_handle);
    OutputVelocity& output_velocity = OutputVelocitySim::init(node_handle);
  #else
    ros::NodeHandle& node_handle = ros::NodeHandle::init("robot_1");
    OutputVelocity& output_velocity = OutputVelocityImpl::init(motor_controller);

    #ifdef KALMAN
      Marvelmind& marvelmind_sensor = Marvelmind::init();
      SensorPose& pose_sensor = KalmanFilter::init({&marvelmind_sensor}, output_velocity);
    #else
      SensorPose& pose_sensor = Marvelmind::init();
    #endif
  #endif

  ControllerMaster& controller_master = ControllerMaster::init(output_velocity, pose_sensor);

  auto pub = node_handle.advertise<ros_msgs::Pose2D>("pose2D");

  StateMachine& state_machine = StateMachine::init(controller_master, output_velocity);
  node_handle.subscribe<ros_msgs::Point2D>("goal_point", std::bind(&StateMachine::set_goal_point, &state_machine, std::placeholders::_1));
  node_handle.subscribe<ros_msgs::Twist2D>("vel", std::bind(&StateMachine::set_velocity, &state_machine, std::placeholders::_1));

  while(1) 
  { 
    ros_msgs_lw::Pose2D pose;

    if(pose_sensor.getPose(pose))
      printf("X: %f, Y: %f, Theta: %f\n", pose.x, pose.y,  pose.theta);

    ros_msgs::Pose2D pose_msg(pose);

    pub.publish(pose_msg);

    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}
