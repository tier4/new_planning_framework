// Copyright 2024 TIER IV, Inc.
//
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

#ifndef AUTOWARE__TRAJECTORY__CONCATENATOR__STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY__CONCATENATOR__STRUCTS_HPP_

namespace autoware::trajectory_selector::trajectory_concatenator
{
struct ConcatenatorParam
{
  explicit ConcatenatorParam(const double duration_time) : duration_time{duration_time} {}

  double duration_time;
};
}  // namespace autoware::trajectory_selector::trajectory_concatenator

#endif  // AUTOWARE__TRAJECTORY__CONCATENATOR__STRUCTS_HPP_