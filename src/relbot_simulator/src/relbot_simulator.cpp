#include "relbot_simulator.hpp"
#include "Plant.h"

// Constructor + Setting up Topics

RELbotSimulator::RELbotSimulator(double time_step) : Node("RELbot_simulator")
{
  RCLCPP_INFO(get_logger(), "Init");

  create_topics();
  RCLCPP_INFO(get_logger(), "Created Topics");

  if (time_step == 0.0)
  {
    time_step = 0.001;
    RCLCPP_INFO(get_logger(), "Set timestep to 0.001");
  }

  dynamics_timer_ = this->create_wall_timer(std::chrono::duration<double>(time_step), std::bind(&RELbotSimulator::dynamics_timer_callback, this));

  image_stream_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / image_stream_FPS), std::bind(&RELbotSimulator::image_stream_timer_callback, this));

  RCLCPP_INFO(get_logger(), "Created Timer");

  input_vector[0] = 0; /* steer_left */
  input_vector[1] = 0; /* steer_right */

  output_vector[0] = 0.0; /* pos_feedback_left */
  output_vector[1] = 0.0; /* pos_feedback_right */
  output_vector[2] = 0.0; /* Position_x {m} */
  output_vector[3] = 0.0; /* Position_y {m} */
  output_vector[4] = 0.0; /* Theta z (radians) */

  _20Sim_rt_sim.Initialize(input_vector, output_vector, 0.0);
  RCLCPP_INFO(get_logger(), "Reset dyn_sim to 0");

  clock_start = get_clock()->now();
}

void RELbotSimulator::create_topics()
{
  declare_parameter<bool>("use_twist_cmd", RELbotSimulator::DEFAULT_USE_TWIST_CMD);
  declare_parameter<double>("image_stream_FPS", RELbotSimulator::DEFAULT_IMAGE_STREAM_FPS);

  RCLCPP_INFO(this->get_logger(), "Creating topics...");
  RCLCPP_INFO(this->get_logger(), "Creating Publishers");

  // outputs
  moving_camera_output_topic_ = this->create_publisher<sensor_msgs::msg::Image>("output/moving_camera", 1);
  camera_position_topic_ = this->create_publisher<geometry_msgs::msg::PointStamped>("output/camera_position", 1);
  robot_pose_topic = this->create_publisher<geometry_msgs::msg::PoseStamped>("output/robot_pose", 1);

  // get param, can be set by ros2 launch command
  useTwistCmd_ = get_parameter("use_twist_cmd").as_bool();
  image_stream_FPS = get_parameter("image_stream_FPS").as_double();

  RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");
  RCLCPP_INFO(this->get_logger(), "Subscribing to %s", RELbotSimulator::WEBCAM_IMAGE.c_str());

  webcam_input_topic_ = this->create_subscription<sensor_msgs::msg::Image>(
      RELbotSimulator::WEBCAM_IMAGE, 10, std::bind(&RELbotSimulator::webcam_topic_callback, this, _1));

  if (RELbotSimulator::useTwistCmd_)
  {
    RCLCPP_INFO(get_logger(), "Using Twist Command mode");

    RELbotSimulator::twistSubscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        RELbotSimulator::TWIST_TOPIC, rclcpp::SensorDataQoS().reliable(),
        std::bind(&RELbotSimulator::twistCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", RELbotSimulator::TWIST_TOPIC.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Using Individual Motors Command mode");

    // The motor topics are in a namespace, construct the full topic name first
    const std::string right_motor_setpoint_vel_topic =
        RELbotSimulator::RIGHT_MOTOR_NAMESPACE + RELbotSimulator::SETPOINT_VEL_TOPIC;
    RELbotSimulator::rightMotorSetpointVelSubscriber_ = create_subscription<example_interfaces::msg::Float64>(
        right_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotSimulator::rightMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", right_motor_setpoint_vel_topic.c_str());

    const std::string left_motor_setpoint_vel_topic =
        RELbotSimulator::LEFT_MOTOR_NAMESPACE + RELbotSimulator::SETPOINT_VEL_TOPIC;
    RELbotSimulator::leftMotorSetpointVelSubscriber_ = create_subscription<example_interfaces::msg::Float64>(
        left_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotSimulator::leftMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", left_motor_setpoint_vel_topic.c_str());
  }
}

// Input from Subscriptions

void RELbotSimulator::webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img)
{
  input_image = msg_cam_img;
}

void RELbotSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // Extract the base link velocity components from the twist
  double linearVelSetpoint = twist->linear.x;
  double angularVelSetpoint = twist->angular.z;

  double left_wheel_vel;
  double right_wheel_vel;

  // Convert the base link velocity setpoints to individual motor setpoints
  // Don't forget: everything is right hand defined, so w_L CCW = FW
    /* Hi students, code author here. Twists and reference frames are confusing and need rigid definitions. Even ChatGPT struggled with comprehending this 😃. 
    Also, just using separate wheel velocities is totally okay and the originally intended way to control this sim 😉 */
  left_wheel_vel = -((linearVelSetpoint - ((angularVelSetpoint * wheelBaseWidth_) / 2)) / wheelRadius_);
  right_wheel_vel = ((linearVelSetpoint + ((angularVelSetpoint * wheelBaseWidth_) / 2)) / wheelRadius_);


  input_vector[0] = left_wheel_vel / RADS_TO_DUTY_CYCLE;
  input_vector[1] = right_wheel_vel / RADS_TO_DUTY_CYCLE;

}

void RELbotSimulator::rightMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
  input_vector[1] = (setpointVel->data) / RADS_TO_DUTY_CYCLE;
  // velRightMotorSetpoint_ = setpointVel->data;
}

void RELbotSimulator::leftMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
  input_vector[0] = (setpointVel->data) / RADS_TO_DUTY_CYCLE;
  // velLeftMotorSetpoint_ = setpointVel->data;
}

// Timer Callback Dynamics. Calulates kinematics and dynamics

void RELbotSimulator::dynamics_timer_callback()
{
  // Do dynamics integration step
  _20Sim_rt_sim.Calculate(input_vector, output_vector);

  // output robot pose
  geometry_msgs::msg::PoseStamped robot_pose;

  robot_pose.header.stamp = get_clock()->now();
  robot_pose.pose.position.set__x(output_vector[2]);
  robot_pose.pose.position.set__y(output_vector[3]);
  robot_pose.pose.orientation.set__z(output_vector[4]);

  rclcpp::Duration duration = get_clock()->now() - clock_start;

  RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 500, "Sim state!   [time, x,y, θz] = [%f, %f, %f, %f]", duration.seconds(), output_vector[2], output_vector[3], output_vector[4]);

  // Output the actual position
  robot_pose_topic->publish(robot_pose);
}

// Timer Callback Image + CV specific functions

void RELbotSimulator::image_stream_timer_callback()
{
  // RCLCPP_INFO(this->get_logger(), "Received webcam frame");

  /*
  Assumption:
  At init, Right-Hand (RH) reference frame or RELbot and world are equal. Relbot moving will shift X (forward/backward), Y (left/right). Theta describes angle w.r.t. world.
  GOAL:
  Forward     - Image shrinks
  Backwards   - Image 'grows', can over-extend to be img + black padding
  Left/Right  - Nothing (assume img is @infinity, so L/R does nothing)
  Theta       - Pan L/R over IMG, can also extend outside borders

  Requirement: Output IMG can't change in actual size, due to allocation issues, so re-scaling might be needed

  Difficulty:
  - Img uses ARRAY coordinates, we use RH-Coord Frame
  */
  if (input_image == nullptr)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "No Input image received");
    return;
  }

  double x = output_vector[2];     // Positive = Forward
  double theta = output_vector[4]; // Positive = counter-clockwise rotation

  const double height = input_image->height;
  const double width = input_image->width;

  const double img_width_rad = 120 * (M_PI / 180);           // Define 'radial width'  of full cam img)
  const double pixels_per_rad = (width / 2) / img_width_rad; // convert to cam img we get

  int center_pixel_x = (int)(input_image->width / 2) - (theta * pixels_per_rad); // transform into what our center_x should be

  int center_pixel_y = (int)(input_image->height / 2); // Mostly static

  int output_image_dim = (int)(height / 2 - (x * height / 10)); // 0 starts at height/2, so somehwat zoomed.

  output_image_ = RELbotSimulator::CreateCVSubimage(input_image, center_pixel_x, center_pixel_y, output_image_dim);

  // transform img back to sensor_msg
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = get_clock()->now();
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = output_image_;

  sensor_output_img = out_msg.toImageMsg();

  moving_camera_output_topic_->publish(*sensor_output_img.get());

  // make camera_pos topic
  geometry_msgs::msg::PointStamped camera_position;

  // Set header timestamp, useful for ordering messages if they are somehow jumbled
  camera_position.header.stamp = get_clock()->now();
  camera_position.point.set__x(center_pixel_x);
  camera_position.point.set__y(center_pixel_y);
  camera_position_topic_->publish(camera_position);
}

cv::Mat RELbotSimulator::CreateCVSubimage(const sensor_msgs::msg::Image::SharedPtr msg_cam_img, const int center_pixel_x, const int center_pixel_y, int output_image_dim)
{
  cv::Mat resized_frame;
  cv::Mat cv_frame = cv_bridge::toCvCopy(msg_cam_img, "bgr8" /* or other encoding */)->image;

  cv::Size size = cv_frame.size();

  const int window_dim = (int)(msg_cam_img->height / 2); // Dirty calculation of window size we want to show in

  int leftmost_pixel_x = center_pixel_x - window_dim / 2;
  int topmost_pixel_y = center_pixel_y - window_dim / 2;

  // Make edges 'stick, such that we dont get out of bounds
  output_image_dim = std::clamp(output_image_dim, 1, size.height);
  leftmost_pixel_x = std::clamp(leftmost_pixel_x, 0, size.width - output_image_dim);
  topmost_pixel_y = std::clamp(topmost_pixel_y, 0, size.height - output_image_dim);

  // Define the rectangle of interest (pixel location + output width (aka, zoom))
  cv::Rect rect = cv::Rect(leftmost_pixel_x, topmost_pixel_y, output_image_dim, output_image_dim);
  // turn that into valid sub-image
  cv::Mat subImg = cv_frame(rect);
  // resize into default size (so the window stays the same size)
  cv::resize(subImg, resized_frame, cv::Size(window_dim, window_dim), cv::INTER_LINEAR);
  // timer to show we are still receivng an image, ticks every second

  RCLCPP_INFO_ONCE(get_logger(), "Image is has been received and processed!");

  cv::imshow("relbot_simulator sub-image", resized_frame);
  cv::waitKey(1);

  return resized_frame;
}

int main(int argc, char *argv[])
{
  printf("RELbot Simulator Node\n-------------------\n");
  rclcpp::init(argc, argv);
  auto RELbot_simulator = std::make_shared<RELbotSimulator>(0.001);
  rclcpp::spin(RELbot_simulator);
  printf("Done Spinning\n-------------------\n");
  rclcpp::shutdown();
  return 0;
}
