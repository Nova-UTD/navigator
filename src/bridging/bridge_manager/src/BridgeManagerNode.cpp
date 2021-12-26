/*
 * Package:   bridge_manager
 * Filename:  BridgeManagerNode.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "bridge_manager/BridgeManagerNode.hpp"
#include "bridge_manager/HTTPRequest.hpp"

#include <string>
//#include <iostream>
#include <chrono>
#include <vector>

using namespace std::chrono;
using namespace Nova::BridgeManager;

std::string construct(std::vector< std::array<std::string, 2> > list) {
    std::string res;
    for (std::array<std::string, 2> i : list) {
        res = res + i[0] + "=" + i[1] + "&";
    }
    return res;
}

BridgeManagerNode::BridgeManagerNode() : Node("bridge_manager") {
  std::cout << "Started bridging manager" << std::endl;
  this->log_event("Beginning Logging");
}

BridgeManagerNode::~BridgeManagerNode() {

}

void BridgeManagerNode::log_event(std::string message) {
  try {
    http::Request request{"http://localhost:3000/"};

    milliseconds ms = duration_cast< milliseconds >(
      system_clock::now().time_since_epoch()
    );

    std::vector< std::array<std::string, 2> > list{
      {"group","localization"},
      {"name","ndt_nodes"},
      {"priority","1"},
      {"message",message},
      {"time",std::to_string(ms.count())}
    };

    std::string keys = construct(list);

    const auto response = request.send("POST", keys, {
      "Content-Type: application/x-www-form-urlencoded"
    });
    std::cout << std::string{response.body.begin(), response.body.end()} << '\n'; // Debugging
  }
    catch (const std::exception& e) {
      std::cerr << "Request failed, error: " << e.what() << '\n';
    }
}
