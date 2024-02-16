
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <myactuator_rmd/driver.hpp>

#include <cstring>
#include <iostream>
#include <map>
#include <vector>

myactuator_rmd::Driver rmd1("can0",1);

typedef std::vector<uint8_t> bytes;

namespace myactuator
{
class MyActuatorCAN
{
public:
  MyActuatorCAN();
  ~MyActuatorCAN();

  int init(const std::vector<std::vector<int64_t>> & can_numbers);


private:
 
};
}  // namespace myactuator
