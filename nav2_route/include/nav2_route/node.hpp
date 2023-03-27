// Copyright (c) 2023 Joshua Wallace
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

#ifndef NAV2_ROUTE__NODE_HPP_
#define NAV2_ROUTE__NODE_HPP_

#include <vector>
namespace nav2_route
{

class Node
{
public:
  typedef Node * NodePtr;
  typedef std::vector<NodePtr> NodeVector;

  struct Coordinates
  {
    Coordinates(const float & x_in, const float & y_in)
    : x(x_in), y(y_in)
    {}

    float x, y;
  };
  typedef std::vector<Coordinates> CoordinateVector;

  explicit Node(const unsigned int index);

  ~Node();

  inline bool & wasVisited()
  {
    return visited_;
  }

  inline void visit()
  {
    visited_ = true;
    queued_ = false;
  }

  inline bool & isQueued()
  {
    return queued_;
  }

  inline void queue()
  {
    queued_ = true;
  }

  inline unsigned int & getIndex()
  {
    return index_;
  }


  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & width)
  {
    return x + y * width;
  }

  static inline Coordinates getCoords(
    const unsigned int & index)
  {
    // Unsigned and sign ints... bad. Get andrew to take a look
    const unsigned int & width = neighbors_grid_offsets[3];
    return {static_cast<float>(index % width), static_cast<float>(index / width)};
  }


  static void initMotionModel(int x_size);

  bool isNodeValid();

  void getNeighbors(NodeVector & neighbors);

  bool backtracePath(CoordinateVector & path);


  NodePtr parent;
  static std::vector<int> neighbors_grid_offsets;

private:
  unsigned int index_;
  bool visited_;
  bool queued_;
};

}  // namespace nav2_route
#endif  // NAV2_ROUTE__NODE_HPP_
