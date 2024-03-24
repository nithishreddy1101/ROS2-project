#include <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class Simple_publisher :public rclcpp::Node
{
public:
    Simple_publisher() : Node("Simple_Publisher"), counter_(0)
    {
        create_publisher<std_msgs::msg::String>("Chatter",10);
        timer = create_wall_timer(ls,std::bind(&Simple_publisher::timerCallback,this));

        RCLCPP_INFO(get_logger(),"Publishing at 1Hz");
    }

    void timerCallback()
    {
        auto message=std_msgs::msg::String();
        message.data ="Helloo ROS 2 -couunter"+std::to_string(counter_++);
    }
private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};




