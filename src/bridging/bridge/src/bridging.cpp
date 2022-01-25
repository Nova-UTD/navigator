/*
 * Package:   bridge
 * Filename:  briding.hpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Creation of messages to web interface for Nova

#include <iostream>
#include <chrono>
#include <string>
#include <vector>

#include "bridge/HTTPRequest.hpp"
#include "bridge/bridging.hpp"

using namespace std::chrono;

std::string navigator::bridge::construct(std::vector< std::array<std::string, 2> > list) {
    std::string res;
    for (std::array<std::string, 2> i : list) {
        res = res + i[0] + "=" + i[1] + "&";
    }
    return res;
}

void navigator::bridge::log(std::string message)  {
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
        std::cout << std::string{response.body.begin(), response.body.end()} << '\n'; // print the result
    }
    catch (const std::exception& e) {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
}