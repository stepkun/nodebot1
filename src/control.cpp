#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * Main control node for nodebot1.
 * Propagates cmd_vel command for nodebot1 to nodebot1's chassis.
 */
class Control : public rclcpp::Node
{
  public:
    Control() : Node("control")
    {
        this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nodebot1/cmd_vel", 
            10, 
            std::bind(
                &Control::listener_callback,
                this, _1
            )
        );

        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/nodebot1/chassis/cmd_vel",
            10
        );
    }

  private:
    void listener_callback(const geometry_msgs::msg::Twist & msg) const
    {
        //RCLCPP_DEBUG(this->get_logger(), "/nodebot1/cmd_vel: '%s'", msg);

        // re-publish received cmd_vel message
        publisher_->publish(msg);

    }

    // data fields
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}