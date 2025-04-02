#include "steering.hpp"

SteerRelbot::SteerRelbot() : Node("steer_relbot")
{
    RCLCPP_INFO(this->get_logger(), "Init");

    // group work code start ####################################################################################

    // Declare the trajectory type with a description
    rcl_interfaces::msg::ParameterDescriptor desc;

    // Set the description and constraints (for visibility, not enforcement)
    desc.description = "Valid trajectories: straight, circle, turn, square, still";
    desc.additional_constraints = "Must be one of: straight, circle, turn, square, still";

    // Allow dynamic reconfiguration
    desc.read_only = false;

    // Sets the default value for the parameter
    std::string initialTrajectory = "still";

    // Declare the parameter with its default and descriptor
    this->declare_parameter<std::string>("trajectoryType", initialTrajectory, desc);

    // Initialize internal variables
    trajectoryTypeLocal = initialTrajectory;
    lastTrajectoryTypeLocal = initialTrajectory;

    // Add a parameter callback to enforce valid values
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : params)
            {
                if (param.get_name() == "trajectoryType")
                {
                    auto value = param.as_string();
                    if (value != "straight" && value != "circle" &&
                        value != "turn" && value != "square" && value != "still")
                    {
                        result.successful = false;
                        result.reason = "Invalid trajectory type: " + value +
                                        ". Must be one of: straight, circle, turn, square, still.";
                        return result;
                    }
                }
            }

            return result;
        });

    // Log the parameter value
    RCLCPP_INFO(this->get_logger(), "Trajectory parameter set to: %s", trajectoryTypeLocal.c_str());

    // log starting time
    startTime = this->get_clock()->now().seconds();

    // group work code end #####################################################################################

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
}

void SteerRelbot::create_topics()
{
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
}

void SteerRelbot::calculate_velocity()
{
    // get the trajectory type from the parameter
    this->get_parameter("trajectoryType", trajectoryTypeLocal);

    if (trajectoryTypeLocal != lastTrajectoryTypeLocal)
    {
        // resets the time once the a trajectory type is change
        startTime = this->get_clock()->now().seconds();

        // updates the last trajectory type
        lastTrajectoryTypeLocal = trajectoryTypeLocal;

        // Log the parameter value
        RCLCPP_INFO(this->get_logger(), "Trajectory parameter set to: %s", trajectoryTypeLocal.c_str());
    }

    // get the relative time value in seconds from the start of this function
    double t = (this->get_clock()->now().seconds() - startTime);

    if (trajectoryTypeLocal == "still")
    {
        // move in a straight line
        left_velocity = 0;
        right_velocity = 0;
    }
    else if (trajectoryTypeLocal == "straight")
    {
        // move in a straight line
        left_velocity = -10;
        right_velocity = 10;
    }
    else if (trajectoryTypeLocal == "circle")
    {
        // move in a circle
        left_velocity = -9;
        right_velocity = 11;
    }
    else if (trajectoryTypeLocal == "turn")
    {
        // checks the relative time t and sets the velocity accordingly
        // move straight for three seconds
        if (t < 3.0)
        {
            left_velocity = -10;
            right_velocity = 10;
        }
        // turn for one second
        else if (t < 4.0)
        {
            left_velocity = -6.95;
            right_velocity = 1;
        }
        // stand still
        else
        {
            trajectoryTypeLocal = "still";
            this->set_parameter(rclcpp::Parameter("trajectoryType", "still"));
        }
    }
    else if (trajectoryTypeLocal == "square")
    {
        // checks the relative time t and sets the velocity accordingly
        // move straight for 4 seconds
        if (t < 4)
        {
            left_velocity = -10;
            right_velocity = 10;
        }
        // turn left for 1 second
        else if (t < 5)
        {
            left_velocity = -6.95;
            right_velocity = 1;
        }
        // reset the counter to repeat this motion
        else
        {
            startTime = this->get_clock()->now().seconds();
        }
    }
}

void SteerRelbot::timer_callback()
{
    // calculate velocity
    calculate_velocity();

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_topic_->publish(left_wheel);
    right_wheel_topic_->publish(right_wheel);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}