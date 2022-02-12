#include "esp_err.h"
#include "nvs_flash.h"

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
#include "LedStrip.h"

#define TAG "Main"

#define DATA_LOGGING
#include "DataLogger.h"

//#define USE_SIM
#define KALMAN
//#define STEP_RESPONSE

#define ROS_SOCKET_PORT CONFIG_ROS_SOCKET_PORT
#define SERVER_IP_ADDR CONFIG_SERVER_IP_ADDR


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

  Socket& ros_socket = *new Socket(ROS_SOCKET_PORT, SERVER_IP_ADDR);

  MotorController& motor_controller = MotorController::init();

  #ifdef STEP_RESPONSE
    motor_controller.disablePIcontrol();
  #endif

  #ifdef USE_SIM
    ros::NodeHandle& node_handle = ros::NodeHandle::init("turtle1", ros_socket);
    SensorPose& pose_sensor = SensorPoseSim::init(node_handle);
    OutputVelocity& output_velocity = OutputVelocitySim::init(node_handle);
  #else
    ros::NodeHandle& node_handle = ros::NodeHandle::init("robot_1", ros_socket);
    OutputVelocity& output_velocity = OutputVelocityImpl::init(motor_controller);

    #ifdef KALMAN
      Marvelmind& marvelmind_sensor = Marvelmind::init();
      SensorPose& pose_sensor = KalmanFilter::init({&marvelmind_sensor}, output_velocity);
    #else
      SensorPose& pose_sensor = Marvelmind::init();
    #endif
  #endif


  #ifdef DATA_LOGGING
    ros::Publisher<ros_msgs::String> data_log_publisher = node_handle.advertise<ros_msgs::String>("data_log");
    DataLogger& data_logger = DataLogger::init(data_log_publisher);
    node_handle.subscribe<ros_msgs::String>("start_log", std::bind(&DataLogger::startLogCallback, &data_logger, std::placeholders::_1));
  #endif

  ControllerMaster& controller_master = ControllerMaster::init(output_velocity, pose_sensor);

  auto pose_feedback = node_handle.advertise<ros_msgs::Pose2D>("pose2D");
  auto string_pub = node_handle.advertise<ros_msgs::String>("string");

  StateMachine& state_machine = StateMachine::init(controller_master, output_velocity);
  node_handle.subscribe<ros_msgs::Point2D>("goal_point", std::bind(&StateMachine::set_goal_point, &state_machine, std::placeholders::_1));
  node_handle.subscribe<ros_msgs::Twist2D>("vel", std::bind(&StateMachine::set_velocity, &state_machine, std::placeholders::_1));

  LedStrip& led_strip = LedStrip::init();
  node_handle.subscribe<ros_msgs::String>("led", std::bind(&LedStrip::animation_callback, &led_strip, std::placeholders::_1));

  while(1) 
  { 
    ros_msgs_lw::Pose2D pose;

    if(pose_sensor.getPose(pose))
      ESP_LOGI(TAG, "X: %f, Y: %f, Theta: %f\n", pose.x, pose.y,  pose.theta);

    ros_msgs::Pose2D pose_msg(pose);

    pose_feedback.publish(pose_msg);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}