#include <kistler_driver/kistler_driver.hpp>

namespace kistler_driver
{

KistlerDriver::KistlerDriver(const rclcpp::NodeOptions & node_options)
: Node("kistler_driver", node_options)
{
  using std::placeholders::_1;

  // Subscriber
  sub_can_ = this->create_subscription<Frame>("input/from_can_bus", rclcpp::QoS(1000), std::bind(&KistlerDriver::onCan, this, _1));

  // Publisher
  pub_e0_status_ = this->create_publisher<E0Status>("output/e0_status", 10);
  pub_e1_status_ = this->create_publisher<E1Status>("output/e1_status", 10);
  pub_e2_status_ = this->create_publisher<E2Status>("output/e2_status", 10);
  pub_e3_status_ = this->create_publisher<E3Status>("output/e3_status", 10);
  pub_e4_status_ = this->create_publisher<E4Status>("output/e4_status", 10);
  pub_e5_status_ = this->create_publisher<E5Status>("output/e5_status", 10);
  pub_e6_status_ = this->create_publisher<E6Status>("output/e6_status", 10);
  pub_e7_status_ = this->create_publisher<E7Status>("output/e7_status", 10);
  pub_e8_status_ = this->create_publisher<E8Status>("output/e8_status", 10);
  pub_e9_status_ = this->create_publisher<E9Status>("output/e9_status", 10);
  pub_ea_status_ = this->create_publisher<EAStatus>("output/ea_status", 10);
  pub_eb_status_ = this->create_publisher<EBStatus>("output/eb_status", 10);
}

void KistlerDriver::onCan(Frame::ConstSharedPtr msg)
{
  if (msg->id == 0x07E0)
  {
    auto e0_status_msg = E0Status();
    e0_status_msg.stamp = msg->header.stamp;
    e0_status_msg.velocity_x = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.036;
    e0_status_msg.velocity_y = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.036;
    e0_status_msg.velocity = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.036;
    e0_status_msg.angle = (static_cast<int8_t>((msg->data.at(7) << 8) + msg->data.at(6))) * 0.01;

    pub_e0_status_->publish(e0_status_msg);
  } else if (msg->id == 0x07E1)
  {
    auto e1_status_msg = E1Status();
    e1_status_msg.stamp = msg->header.stamp;
    e1_status_msg.distance = (static_cast<int32_t>((msg->data.at(3) << 8 + msg->data.at(2) << 8 + msg->data.at(1) << 8 + msg->data.at(0)))) * 0.001;

    pub_e1_status_->publish(e1_status_msg);
  } else if (msg->id == 0x07E2)
  {
    auto e2_status_msg = E2Status();
    e2_status_msg.stamp = msg->header.stamp;
    e2_status_msg.pitch = (static_cast<int8_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.01;
    e2_status_msg.roll = (static_cast<int8_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.01;
    e2_status_msg.timestamp = (static_cast<uint16_t>((msg->data.at(5) << 8) + msg->data.at(4)));
    e2_status_msg.radius = (static_cast<float_t>((msg->data.at(7) << 8) + msg->data.at(6))) * 0.01;

    pub_e2_status_->publish(e2_status_msg);
  } else if (msg->id == 0x07E3)
  {
    auto e3_status_msg = E3Status();
    e3_status_msg.stamp = msg->header.stamp;
    e3_status_msg.acc_x_hor = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.01;
    e3_status_msg.acc_y_hor = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.01;
    e3_status_msg.acc_z_hor = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.01;
    e3_status_msg.acc_c_body = (static_cast<int16_t>((msg->data.at(7) << 8) + msg->data.at(6))) * 0.01;

    pub_e3_status_->publish(e3_status_msg);
  } else if (msg->id == 0x07E4)
  {
    auto e4_status_msg = E4Status();
    e4_status_msg.stamp = msg->header.stamp;
    e4_status_msg.ang_vel_x_hor = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.02;
    e4_status_msg.ang_vel_y_hor = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.02;
    e4_status_msg.ang_vel_z_hor = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.02;

    pub_e4_status_->publish(e4_status_msg);
  } else if (msg->id == 0x07E5)
  {
    auto e5_status_msg = E5Status();
    e5_status_msg.stamp = msg->header.stamp;
    e5_status_msg.vel_x_cor = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.036;
    e5_status_msg.vel_y_cor = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.036;
    e5_status_msg.vel_cor = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.036;
    e5_status_msg.angle_cor = (static_cast<int8_t>((msg->data.at(7) << 8) + msg->data.at(6))) * 0.01;

    pub_e5_status_->publish(e5_status_msg);
  } else if (msg->id == 0x07E6)
  {
    auto e6_status_msg = E6Status();
    e6_status_msg.stamp = msg->header.stamp;
    e6_status_msg.acc_x_body = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.01;
    e6_status_msg.acc_y_body = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.01;
    e6_status_msg.acc_z_body = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.01;

    pub_e6_status_->publish(e6_status_msg);
  } else if (msg->id == 0x07E7)
  {
    auto e7_status_msg = E7Status();
    e7_status_msg.stamp = msg->header.stamp;
    e7_status_msg.ang_vel_x_body = (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.02;
    e7_status_msg.ang_vel_y_body = (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.02;
    e7_status_msg.ang_vel_z_body = (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.02;

    pub_e7_status_->publish(e7_status_msg);
  } else if (msg->id == 0x07E8)
  {
    auto e8_status_msg = E8Status();
    e8_status_msg.stamp = msg->header.stamp;
    e8_status_msg.latitude = (static_cast<int8_t>((msg->data.at(3) << 8 + msg->data.at(2) << 8 + msg->data.at(1) << 8 + msg->data.at(0)))) * 1e7;
    e8_status_msg.longitude = (static_cast<int16_t>((msg->data.at(7) << 8 + msg->data.at(6) << 8 + msg->data.at(5) << 8 + msg->data.at(4)))) * 1e7;

    pub_e8_status_->publish(e8_status_msg);
  } else if (msg->id == 0x07E9)
  {
    auto e9_status_msg = E9Status();
    e9_status_msg.stamp = msg->header.stamp;
    e9_status_msg.time_of_week = (static_cast<uint32_t>((msg->data.at(3) << 8 + msg->data.at(2) << 8 + msg->data.at(1) << 8 + msg->data.at(0))));
    e9_status_msg.track = (static_cast<uint16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.01;
    e9_status_msg.height = (static_cast<int16_t>((msg->data.at(7) << 8) + msg->data.at(6)));

    pub_e9_status_->publish(e9_status_msg);
  } else if (msg->id == 0x07EA)
  {
    auto ea_status_msg = EAStatus();
    ea_status_msg.stamp = msg->header.stamp;
    ea_status_msg.input_analog_1 = (static_cast<int8_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.001;
    ea_status_msg.input_analog_2 = (static_cast<int8_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.001;
    ea_status_msg.input_digital = (static_cast<int16_t>((msg->data.at(7) << 8 + msg->data.at(6) << 8 + msg->data.at(5) << 8 + msg->data.at(4))));

    pub_ea_status_->publish(ea_status_msg);
  } else if (msg->id == 0x07EB)
  {
    auto eb_status_msg = EBStatus();
    eb_status_msg.stamp = msg->header.stamp;
    eb_status_msg.sensor_id = (static_cast<uint16_t>((msg->data.at(1) << 8) + msg->data.at(0)));
    eb_status_msg.temperature = static_cast<int8_t>(msg->data.at(2));
    eb_status_msg.lamp_current = static_cast<float_t>(msg->data.at(3)) * 0.01;
    eb_status_msg.filter_setting = static_cast<uint8_t>(msg->data.at(4));
    eb_status_msg.num_of_sat = static_cast<uint8_t>(msg->data.at(5));
    eb_status_msg.stst.data = static_cast<uint8_t>(msg->data.at(6)) & 0x1;
    eb_status_msg.filter_off_on.data = static_cast<uint8_t>(msg->data.at(6) >> 1) & 0x1;
    eb_status_msg.lamp_current_control.data = static_cast<uint8_t>(msg->data.at(6) >> 2) & 0x1;
    eb_status_msg.temperature_ok.data = static_cast<uint8_t>(msg->data.at(6) >> 3) & 0x1;
    eb_status_msg.head_status.data = static_cast<uint8_t>(msg->data.at(6) >> 4) & 0x1;
    eb_status_msg.angle_switched_off.data = static_cast<uint8_t>(msg->data.at(6) >> 5) & 0x1;
    eb_status_msg.direction = static_cast<uint8_t>(msg->data.at(6) >> 6) & 0x1;
    eb_status_msg.satfixed.data = static_cast<uint8_t>(msg->data.at(6) >> 7) & 0x1;
    eb_status_msg.ang_vel_correction.data = static_cast<uint8_t>(msg->data.at(7)) & 0x1;
    eb_status_msg.direction_motion.data = static_cast<uint8_t>(msg->data.at(7) >> 1) & 0x1;
    eb_status_msg.direction_mounting.data = static_cast<uint8_t>(msg->data.at(7) >> 2) & 0x1;
    eb_status_msg.direction_head_is_valid.data = static_cast<uint8_t>(msg->data.at(7) >> 3) & 0x1;
    eb_status_msg.direction_head.data = static_cast<uint8_t>(msg->data.at(7) >> 4) & 0x1;

    pub_eb_status_->publish(eb_status_msg);
  } else
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000, "Receive undefined CAN ID: %d", msg->id);
  }
}

} // namespace kistler_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kistler_driver::KistlerDriver)