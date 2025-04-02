#include "RosXenoBridge.hpp"

/**
 * @brief Constructor for the RosXenoBridge class.
 * 
 * Initializes publishers, subscribers, and a timer for the ROS2 node.
 * Sets up communication with the Xenomai real-time system.
 */
RosXenoBridge::RosXenoBridge() : Node("RosXenoBridge")
{
  // Initialize cross buffers & polling
  initialize();

  // Initialize subscribers, publishers, parameters etc. here...
  RCLCPP_INFO(this->get_logger(), "Creating publisher for topic 'Xenomai_state'");
  control_publisher_ = this->create_publisher<std_msgs::msg::Int32>("Xenomai_state", 1000);
  RCLCPP_INFO(this->get_logger(), "Creating publisher for topic 'Xeno2Ros'");
  data_publisher_ = this->create_publisher<xrf2_msgs::msg::Xeno2Ros>("Xeno2Ros", 1000);

  RCLCPP_INFO(this->get_logger(), "Subscribing to topic 'Command'");
  control_subscription_ = this->create_subscription<std_msgs::msg::Int32>("Command", 1, std::bind(&RosXenoBridge::controlCallback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Subscribing to topic 'Ros2Xeno'");
  data_subscription_ = this->create_subscription<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 1, std::bind(&RosXenoBridge::dataCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Creating a timer");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RosXenoBridge::timerCallback, this));
}

/**
 * @brief Initializes the communication channels with the Xenomai system.
 * 
 * Opens the necessary file descriptors for data and control communication
 * and ensures they are non-blocking. Additionally, clears any stale data in
 * the buffers.
 */
void RosXenoBridge::initialize()
{
  // Get ROS-Xeno file descriptor
  data_fd = open("/dev/evl/xbuf/Ros-Xeno", O_RDWR);
  if (data_fd < 0)
    error(1, errno, "open('/dev/evl/xbuf/Ros-Xeno') : Check xbuf permissions, see xbuf_udev_rule/readme.txt");

  // Set file descriptor to non-block
  int flags = fcntl(data_fd, F_GETFL, 0);
  int ret = fcntl(data_fd, F_SETFL, flags| O_NONBLOCK);
  if (ret == -1)
    error(1, errno, "fcntl(data_fd, F_SETFL, flags| O_NONBLOCK)");

  // Get control line file descriptor
  control_fd = open("/dev/evl/xbuf/Control", O_RDWR);
  if (control_fd < 0)
    error(1, errno, "open('/dev/evl/xbuf/Control') : Check xbuf permissions, see xbuf_udev_rule/readme.txt");

  // Set file descriptor to non-block
  flags = fcntl(control_fd, F_GETFL, 0);
  ret = fcntl(control_fd, F_SETFL, flags| O_NONBLOCK);
  if (ret == -1)
    error(1, errno, "fcntl(control_fd, F_SETFL, flags| O_NONBLOCK)");

  #if ROS_BRIDGE_DEBUG
  printf("ros fd: %d, xeno fd : %d \n", control_fd, data_fd);
  #endif

  // Empty out current data in the Ros-Xeno buffer
  xrf2_msgs::msg::Xeno2Ros trow_away_data;
  int trash_msg_count = 0;
  while (true)
  {
    int read_size = read(data_fd, &trow_away_data, sizeof(trow_away_data));
    if (read_size < 0) break;
    trash_msg_count++;
  }

  #if ROS_BRIDGE_DEBUG
    RCLCPP_INFO(this->get_logger(), "The amount of messages thrown away equals %d \n", trash_msg_count);
  #endif
  return;
}

/**
 * @brief Handles incoming Ros2Xeno messages.
 * 
 * Sends received messages to the data buffer for communication with Xenomai.
 * 
 * @param msg The received Ros2Xeno message.
 */
void RosXenoBridge::dataCallback(const xrf2_msgs::msg::Ros2Xeno::SharedPtr msg)
{
  // Send received data to the Data buffer to Xenomai
  xrf2_msgs::msg::Ros2Xeno msg_copy = *msg;
  write(data_fd, &msg_copy, sizeof(msg_copy));

  #if ROS_BRIDGE_DEBUG
    RCLCPP_INFO(this->get_logger(), "Received data will be sent to Xenomai");
  #endif
}

/**
 * @brief Handles incoming control messages.
 * 
 * Sends received control messages to the control buffer for communication with Xenomai.
 * 
 * @param msg The received Int32 control message.
 */
void RosXenoBridge::controlCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  // Send received data to the Control buffer to Xenomai
  std_msgs::msg::Int32 msg_copy = *msg;
  write(control_fd, &msg_copy, sizeof(msg_copy));

  #if ROS_BRIDGE_DEBUG
    RCLCPP_INFO(this->get_logger(), "Received command will be sent to Xenomai");
  #endif
}

/**
 * @brief Timer callback for handling periodic tasks.
 * 
 * Checks the data and control buffers for messages from Xenomai and
 * publishes them to their respective ROS2 topics.
 */
void RosXenoBridge::timerCallback()
{
  xrf2_msgs::msg::Xeno2Ros data_msg;
  std_msgs::msg::Int32 control_msg;
  int ret;

  // Check if data is available on the data buffer 
  int msg_count = 0;
  while (true)
  {
    ret = read(data_fd, &data_msg, sizeof(data_msg));
    if (ret > 0) msg_count++;
    else break;
  }
  
  // If data is available in the data xbuffer publish to the Xeno2Ros topic
  if (msg_count > 0)
  {
    #if ROS_BRIDGE_DEBUG
      RCLCPP_INFO(this->get_logger(), "Received msg from Xenomai : %d", msg_count);
    #endif
    data_publisher_->publish(data_msg);
    if (msg_count > 1)
       RCLCPP_INFO(this->get_logger(), "Missed msg from Xenomai : %d", msg_count-1);

  }
  
  // Check if data is available on the control buffer
  msg_count = 0;
  while (true)
  { 
    ret = read(control_fd, &control_msg, sizeof(control_msg));
    if (ret > 0) msg_count++;
    else break;
  }

  // If data is available in the control xbuffer publish to the XenomaiState topic
  if (msg_count > 0)
  {
    #if ROS_BRIDGE_DEBUG
      RCLCPP_INFO(this->get_logger(), "Received Xenomai state msg from Xenomai : %d", msg_count);
    #endif
    control_publisher_->publish(control_msg);
    if (msg_count > 1)
       RCLCPP_INFO(this->get_logger(), "Missed states from Xenomai : %d", msg_count-1);

  }
}

// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
/**
 * @brief Main function to start the RosXenoBridge node.
 * 
 * Initializes ROS 2, spins the node, and handles all ROS 2-related events.
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosXenoBridge>());
  rclcpp::shutdown();
  return 0;
}
