#include "pdfs/likelihood_field_pdf.hpp"
#include "tf2/utils.h"
#include "nav2_localization/map_utils.hpp"

namespace nav2_localization
{

LikelihoodFieldPdf::LikelihoodFieldPdf(const int &max_number_of_beams,
                                       const double &max_likelihood_distace,
                                       const double &sigma_hit,
                                       const double &z_max,
                                       const double &z_min,
                                       const double &z_rand,
                                       const nav2_util::LifecycleNode::SharedPtr &node,
                                       const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
                                       const geometry_msgs::msg::TransformStamped &laser_pose) :
                                       max_number_of_beams_(max_number_of_beams),
                                       max_likelihood_distace_(max_likelihood_distace),
                                       sigma_hit_(sigma_hit),
                                       z_max_(z_max),
                                       z_min_(z_min),
                                       z_rand_(z_rand),
                                       node_(node),
                                       map_(map),
                                       laser_pose_(laser_pose)
{
	preComputeLikelihoodField();	// TODO: is this the best place for this?
}

BFL::Probability LikelihoodFieldPdf::ProbabilityGet(const sensor_msgs::msg::LaserScan &measurement) const
{
    double q = 1;

	// in case the user specfies more beams than is avaiable
	int max_number_of_beams = std::min(max_number_of_beams, static_cast<int>(measurement.ranges.size()));
	for(int i=0; i<max_number_of_beams; i++)
	{
		if(measurement.ranges[i]<z_max_ || measurement.ranges[i]>z_min_) // within sensor range
		{
			geometry_msgs::msg::TransformStamped curr_pose = ConditionalArgumentGet(0);
			double x_z_kt = curr_pose.transform.translation.x +
							laser_pose_.transform.translation.x * cos(tf2::getYaw(curr_pose.transform.rotation)) -
							laser_pose_.transform.translation.y * sin(tf2::getYaw(curr_pose.transform.rotation)) +
							measurement.ranges[i] * 
							cos(tf2::getYaw(curr_pose.transform.rotation) + tf2::getYaw(laser_pose_.transform.rotation));
			double y_z_kt = curr_pose.transform.translation.y +
							laser_pose_.transform.translation.y * cos(tf2::getYaw(curr_pose.transform.rotation)) +
							laser_pose_.transform.translation.x * sin(tf2::getYaw(curr_pose.transform.rotation)) +
							measurement.ranges[i] * 
							sin(tf2::getYaw(curr_pose.transform.rotation) + tf2::getYaw(laser_pose_.transform.rotation));
			int8_t end_point_index = MapUtils::coordinates_to_index(x_z_kt, y_z_kt, map_->info.width);
			double dist_prob = pre_computed_likelihood_field_.at(end_point_index);

			q *= z_hit_* dist_prob + (z_rand_/z_max_);
		}
	}
	return BFL::Probability(q);
}

void LikelihoodFieldPdf::preComputeLikelihoodField()
{
	std::vector<int> occupied_cells;

	// Identify all the occupied cells
	for(auto index=0; index < map_->info.width*map_->info.height; index++)
	{
		if(map_->data[index]==100) // the cell is occupied
		{
			pre_computed_likelihood_field_[index] = 0.0;
			occupied_cells.push_back(index);
		}
		else
			pre_computed_likelihood_field_[index] = max_likelihood_distace_;
	}

	// Depth first search for other cells
	std::vector<bool> visited(map_->info.width*map_->info.height, false);
	for(auto index : occupied_cells)
		DFS(index, index, visited);

	// Apply zero-mean norrmal distribution
	for(auto index=0; index < map_->info.width*map_->info.height; index++)
		pre_computed_likelihood_field_[index] = (1.0/(sqrt(2*M_PI)*sigma_hit_))*exp(-0.5*((pre_computed_likelihood_field_[index]*pre_computed_likelihood_field_[index])/(sigma_hit_*sigma_hit_)));
}

void LikelihoodFieldPdf::DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited)
{
	visited[index_curr] = true;
	std::pair<uint32_t, uint32_t> coord_curr = MapUtils::index_to_coordinates(index_curr, map_->info.width);
	std::pair<uint32_t, uint32_t> coord_obs = MapUtils::index_to_coordinates(index_of_obstacle, map_->info.width);

	// This cell is NOT an obstacle
	if(pre_computed_likelihood_field_[index_curr]!=0.0)	
	{
		double distance_to_obstacle = MapUtils::distance_between_two_points(coord_curr.first, coord_curr.second, coord_obs.first, coord_obs.second)*map_->info.resolution;

		// Getting far from the obstacle
		if(distance_to_obstacle >= max_likelihood_distace_)
			return;
		
		// Found a closer obstacle
		if(distance_to_obstacle < pre_computed_likelihood_field_[index_curr])
			pre_computed_likelihood_field_[index_curr] = distance_to_obstacle;
	}

	// left
	if(coord_curr.first > 0)
	{
		int left_cell_index =  MapUtils::coordinates_to_index(coord_curr.first-1, coord_curr.second, map_->info.width);
		if(!visited[left_cell_index])
			DFS(left_cell_index, index_of_obstacle, visited);
	}

	// right
	if(coord_curr.first < map_->info.width-1)
	{
		int right_cell_index =  MapUtils::coordinates_to_index(coord_curr.first+1, coord_curr.second, map_->info.width);
		if(!visited[right_cell_index])
			DFS(right_cell_index, index_of_obstacle, visited);
	}

	// up
	if(coord_curr.second > 0)
	{
		int up_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second-1, map_->info.width);
		if(!visited[up_cell_index])
			DFS(up_cell_index, index_of_obstacle, visited);
	}

	// down
	if(coord_curr.second < map_->info.height-1)
	{
		int down_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second+1, map_->info.width);
		if(!visited[down_cell_index])
			DFS(down_cell_index, index_of_obstacle, visited);
	}
}

}   // nav2_localization