/*
 * Package:   mapping
 * Filename:  OctreeMappingNode.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "mapping/OctreeMappingNode.hpp"

using namespace navigator::perception;
using namespace octomap;

void print_query_info(point3d query, OcTreeNode *node)
{
  if (node != NULL)
  {
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  }
  else
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

OctreeMappingNode::OctreeMappingNode() : Node("octree_mapping_node")
{
  RCLCPP_INFO(this->get_logger(), "Hello, world!");

  OcTree tree(0.1);

  for (int x = -20; x < 20; x++)
  {
    for (int y = -20; y < 20; y++)
    {
      for (int z = -20; z < 20; z++)
      {
        point3d endpoint((float)x * 0.05f, (float)y * 0.05f, (float)z * 0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x = -30; x < 30; x++)
  {
    for (int y = -30; y < 30; y++)
    {
      for (int z = -30; z < 30; z++)
      {
        point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f, (float)z * 0.02f - 1.0f);
        tree.updateNode(endpoint, false); // integrate 'free' measurement
      }
    }
  }

  std::cout << std::endl;
  std::cout << "performing some queries:" << std::endl;

  point3d query(0., 0., 0.);
  OcTreeNode *result = tree.search(query);
  print_query_info(query, result);

  query = point3d(-1., -1., -1.);
  result = tree.search(query);
  print_query_info(query, result);

  query = point3d(1., 1., 1.);
  result = tree.search(query);
  print_query_info(query, result);

  std::cout << std::endl;
  tree.writeBinary("simple_tree.bt");
  std::cout << "wrote example file simple_tree.bt" << std::endl
            << std::endl;
  std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
  std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl
            << std::endl;
}

OctreeMappingNode::~OctreeMappingNode()
{
}
