#include <cstdio>
#include <chrono>
extern "C" {
  #include <tcs.h>
}

#include "rclcpp/rclcpp.hpp"
#include "tcs_interface/msg/color_rgbc.hpp"

using namespace std::chrono_literals;

extern "C" {
    // Declare the C functions here
    void tcsGetStandaloneRgbc(uint16_t *rgbc);
}


class TcsPublisher: public rclcpp::Node
{
  public:
    TcsPublisher()
    : Node("tcs_publisher")
    {
      publisher_ = this->create_publisher<tcs_interface::msg::ColorRGBC>("tcs", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TcsPublisher::timer_callback, this));
    }

  private:
    // Read and publish RGBC data
    void timer_callback()
    {
      // Read RGBC data
      uint16_t rgbc_wide[] = {0x0000, 0x0000, 0x0000, 0x0000};
      tcsGetStandaloneRgbc(rgbc_wide);

      // Create message
      auto message = tcs_interface::msg::ColorRGBC();
      message.r = rgbc_wide[0] >> 8;
      message.g = rgbc_wide[1] >> 8;
      message.b = rgbc_wide[2] >> 8;
      message.c = rgbc_wide[3] >> 8;

      // Publish data
      RCLCPP_INFO(this->get_logger(), "Publishing RGBC data: (%d, %d, %d, %d)", 
        message.r, message.g, message.b, message.c);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tcs_interface::msg::ColorRGBC>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcsPublisher>());
  rclcpp::shutdown();

  return 0;
}
