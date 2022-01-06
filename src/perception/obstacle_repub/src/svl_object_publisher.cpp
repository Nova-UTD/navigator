#include "rclcpp/rclcpp.hpp"

#include "lgsvl_msgs/msg/detection3_d_array.hpp"

#include <chrono>
#include <memory>
using namespace std::chrono_literals;


class SVLObjectTestPublisher : public rclcpp::Node  
{   

    public:
        SVLObjectTestPublisher() : Node("svl_object_test_publisher")
        {
            publisher_ = this->create_publisher<lgsvl_msgs::msg::Detection3DArray>("svl_test_objects", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&SVLObjectTestPublisher::timer_callback, this));
        }

    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<lgsvl_msgs::msg::Detection3DArray>::SharedPtr publisher_;
        void timer_callback()
        {
            auto message = lgsvl_msgs::msg::Detection3DArray();                               
            RCLCPP_INFO(this->get_logger(), "Publishing svl object");   
            publisher_->publish(message);
        }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SVLObjectTestPublisher>());
  rclcpp::shutdown();
  return 0;
}