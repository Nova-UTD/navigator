/*
 * Package:   can_translation
 * Filename:  src/FloatReporterNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "voltron_msgs/msg/can_frame.hpp"
#include "can_translation/FloatReporterNode.hpp"

using namespace navigator::can_translation;

FloatReporterNode::FloatReporterNode() : rclcpp::Node("float_reporter") {
  this->declare_parameter("input_min");
  this->params.input_min = this->get_parameter("input_min").as_int();
  this->declare_parameter("input_max");
  this->params.input_max = this->get_parameter("input_max").as_int();
  this->declare_parameter("output_min");
  this->params.output_min = this->get_parameter("output_min").as_double();
  this->declare_parameter("output_max");
  this->params.output_max = this->get_parameter("output_max").as_double();
  this->declare_parameter("message_id");
  this->params.message_id = this->get_parameter("message_id").as_int();
  this->declare_parameter("field_start_bit");
  this->params.field_start_bit = this->get_parameter("field_start_bit").as_int();
  this->declare_parameter("field_length_bits");
  this->params.field_length_bits = this->get_parameter("field_length_bits").as_int();
  this->init();
}

FloatReporterNode::FloatReporterNode(float_reporter_params params)
  : rclcpp::Node("steering_reporter") {
  this->params = params;
  this->init();
}

FloatReporterNode::~FloatReporterNode() {}

// Called every time a can frame arrives on the bus
void FloatReporterNode::process_frame(const voltron_msgs::msg::CanFrame::SharedPtr incoming_frame) {
  if(incoming_frame->identifier != this->params.message_id) return; // Skip frames not meant for us

  can_data_t data_field = incoming_frame->data << this->params.field_start_bit; // Isolate the field we're interested in
  data_field = data_field >> (sizeof(can_data_t) - this->params.field_length_bits);

  double data_scaled = (double) (data_field - this->params.input_min); // Scaled from 0 to 1
  data_scaled = data_scaled / (this->params.input_max - this->params.input_min);

  data_scaled = data_scaled + this->params.output_min; // Then rescale between output_min and output_max
  data_scaled = data_scaled * (this->params.output_max - this->params.output_min);

  auto message = std_msgs::msg::Float32(); // Send the message
  message.data = data_scaled;
  this->result_publisher->publish(message);
}

void FloatReporterNode::init() {
  this->result_publisher = this->create_publisher<std_msgs::msg::Float32>
    ("result_topic", 8);
  this->can_subscription = this->create_subscription<voltron_msgs::msg::CanFrame>
    ("incoming_can_frames", 8,
     std::bind(& FloatReporterNode::process_frame, this, std::placeholders::_1));
}
