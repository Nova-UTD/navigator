/*
 * Package:   bridge_manager
 * Filename:  BridgeManagerNode.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 * 
 * This is the core Bridge Manager Node that handles all topic subscription handling and logging to the local server
*/

// Header files
#include "bridge_manager/BridgeManagerNode.hpp"
#include "bridge_manager/HTTPRequest.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"

// Standard C++ libraries
#include <string>
#include <chrono>
#include <vector>

// namespaces
using namespace std::chrono;
using namespace Nova;

// Vector of strings to a single string
std::string BridgeManagerNode::construct(std::vector< std::array<std::string, 2> > list) {
  std::string res;
  for (std::array<std::string, 2> i : list) {
      res = res + i[0] + "=" + i[1] + "&";
  }
  return res;
}

// Initial node constructor
BridgeManagerNode::BridgeManagerNode() 
  : rclcpp::Node("bridge_manager") { // Create node named bridge_manager
  this->initialize(); // Call node initialization
  
  // Testing command:
  // ros2 topic pub -r 1 /incoming_bridge_messages std_msgs/msg/String "{data: 'Hello'}
  // Replace -r 1 with --once for single execution/topic publishing. Replace '1' for a different pub frequency (default 1hz)
  // Data must be in yaml format
}

// Default deconstructor
BridgeManagerNode::~BridgeManagerNode() {
  // tbd
}

// Log event from any topic subscriptions
void BridgeManagerNode::log_event(const std_msgs::msg::String::SharedPtr msg) {
  try { // try-catch for error handling
    http::Request request{"http://localhost:3000/"}; // Create http request from http header to local server handler

    milliseconds ms = duration_cast< milliseconds >( // get request time in milliseconds using std::chrono object
      system_clock::now().time_since_epoch()
    );

    std::vector< std::array<std::string, 2> > list{ // Create vector/list of string table (key/descriptor and value)
      {"group","localization"},
      {"name","ndt_nodes"},
      {"priority","1"},
      {"message",msg->data.c_str()},
      {"time",std::to_string(ms.count())}
    };

    std::string keys = construct(list); // Turn vector list into string

    const auto response = request.send("POST", keys, { // Send request with string as POST request to handler and get response
      "Content-Type: application/x-www-form-urlencoded" // POST request type descriptor
    });

    std::string res = std::string{response.body.begin(), response.body.end()} + '\n'; // Response to log
    RCLCPP_INFO(this->get_logger(), res); // Log message via rclcpp
  }
    catch (const std::exception& e) { // Error handling, output error if error is spotted
      std::cerr << "Request failed, error: " << e.what() << '\n'; // Output error message
    }
}

void BridgeManagerNode::initialize() {
  RCLCPP_INFO(this->get_logger(), "Started bridging manager"); // Initialize node
  
  /*this->bridge_publisher = this->create_publisher<std::string>
    ("outgoing_bridge_messaging", 8);*/
  
  // Create bridge subscription and assign it ros string type, set it to log to log_event
  this->bridge_subscription = this->create_subscription<std_msgs::msg::String>
    ("incoming_bridge_messages", 8,
      std::bind(& BridgeManagerNode::log_event, this, std::placeholders::_1));

  //this->log_event("Beginning Logging");
}