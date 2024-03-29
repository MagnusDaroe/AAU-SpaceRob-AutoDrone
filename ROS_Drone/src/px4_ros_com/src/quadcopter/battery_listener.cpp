#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>


class BatteryListener : public rclcpp::Node
{
public:
    explicit BatteryListener() : Node("battery_listener")
    {
        rmw_qos_profile_t qos_profile=rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status", qos,
        [this](const px4_msgs::msg::BatteryStatus::UniquePtr msg) {
            std::cout << "\nRECEIVED BATTERY STATUS DATA" << std::endl;
            std::cout << "==============================" << std::endl;
            std::cout << "ts: " << msg->timestamp << std::endl;
            std::cout << "voltage: " << msg->voltage_v << std::endl;
        });
    }
    
private:
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting battery listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryListener>());

    rclcpp::shutdown();
    return 0;
}