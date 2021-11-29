// Copyright 2018 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2018 the Autoware Foundation
/// \file
/// \brief source file for hungarian algorithm for optimal linear assignment

//lint -e537 cpplint complains otherwise NOLINT

#include <iostream>
#include <chrono>
#include <string>
#include <vector>

#include "assigner/HTTPRequest.hpp"

using namespace std::chrono;

std::string construct(std::vector< std::array<std::string, 2> > list) {
    std::string res;
    for (std::array<std::string, 2> i : list) {
        res = res + i[0] + "=" + i[1] + "&";
    }
    return res;
}

void message_creator::log(std::string message)  {
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

    return 0;
}