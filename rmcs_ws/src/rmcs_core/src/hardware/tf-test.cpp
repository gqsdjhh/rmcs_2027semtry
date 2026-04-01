#include <cmath>
#include <fast_tf/impl/cast.hpp>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class TfTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TfTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , yaw_rate_(declare_parameter("yaw_rate", 0.5))
        , pitch_amplitude_(declare_parameter("pitch_amplitude", 0.25))
        , pitch_frequency_(declare_parameter("pitch_frequency", 0.20))
        , camera_x_(declare_parameter("camera_x", 0.06603))
        , camera_y_(declare_parameter("camera_y", 0.0))
        , camera_z_(declare_parameter("camera_z", 0.082)) {
        register_output("/tf", tf_);
        register_output("/tf_test/sim_time", sim_time_, 0.0);
        register_output("/tf_test/yaw_angle", yaw_angle_, 0.0);
        register_output("/tf_test/pitch_angle", pitch_angle_, 0.0);
    }

    TfTest(const TfTest&) = delete;
    TfTest& operator=(const TfTest&) = delete;
    TfTest(TfTest&&) = delete;
    TfTest& operator=(TfTest&&) = delete;

    ~TfTest() override = default;

    void update() override {
        constexpr double kDt = 0.001;
        constexpr double kTwoPi = 2.0 * std::numbers::pi;

        *sim_time_ += kDt;
        *yaw_angle_ += yaw_rate_ * kDt;
        *yaw_angle_ = std::remainder(*yaw_angle_, kTwoPi);
        *pitch_angle_ = pitch_amplitude_ * std::sin(kTwoPi * pitch_frequency_ * (*sim_time_));

        tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(*yaw_angle_);
        // tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(*pitch_angle_);
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{camera_x_, camera_y_, camera_z_});

        const double roll = 0.05 * std::sin(0.6 * (*sim_time_));
        const Eigen::Quaterniond imu_orientation =
            Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()}
            * Eigen::AngleAxisd{*pitch_angle_, Eigen::Vector3d::UnitY()}
            * Eigen::AngleAxisd{*yaw_angle_, Eigen::Vector3d::UnitZ()};
        // tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(imu_orientation);
    }

private:
    const double yaw_rate_;
    const double pitch_amplitude_;
    const double pitch_frequency_;
    const double camera_x_;
    const double camera_y_;
    const double camera_z_;

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<double> sim_time_;
    OutputInterface<double> yaw_angle_;
    OutputInterface<double> pitch_angle_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TfTest, rmcs_executor::Component)
