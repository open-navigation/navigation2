// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CORE__NAVIGATOR_EXCEPTIONS_HPP_
#define NAV2_CORE__NAVIGATOR_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace nav2_core
{

class NavigatorException : public std::runtime_error
{
public:
  explicit NavigatorException(const std::string & description)
  : std::runtime_error(description) {}
};

class NavigatorPoseNotAvailable : public NavigatorException
{
public:
  explicit NavigatorPoseNotAvailable(const std::string & description)
  : NavigatorException(description) {}
};

class NavigatorInvalidPath : public NavigatorException
{
public:
  explicit NavigatorInvalidPath(const std::string & description)
  : NavigatorException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__NAVIGATOR_EXCEPTIONS_HPP_
