#include "better_pathfinder.h"
#include "nodedef.h"
#include "Map.h"
#include "gamedef.h"
#include <algorithm>

BetterPathfinder::BetterPathfinder(ServerEnvironment *env, unsigned maximum_distance, v3s16 start, v3s16 end) :
env(env), maximum_distance(maximum_distance), start(start), end(end)
{
	max_drop_height = 1;
	max_jump_height = 1;
	swimming_above = false;
	swimming_surface = false;
	swimming_below = false;
	height_required = 1;
	visited_nodes.clear();
	nodes_data.clear();
}

void BetterPathfinder::set_max_drop_height(unsigned height)
{
	max_drop_height = height;
}

void BetterPathfinder::set_max_jump_height(unsigned height)
{
	max_jump_height = height;
}

void BetterPathfinder::set_height_required(unsigned height)
{
	height_required = height;
}

void BetterPathfinder::set_swimming_above(bool swimming_above)
{
	this->swimming_above = swimming_above;
}

void BetterPathfinder::set_swimming_surface(bool swimming_surface)
{
	this->swimming_surface = swimming_surface;
}

void BetterPathfinder::set_swimming_below(bool swimming_below)
{
	this->swimming_below = swimming_below;
}

void BetterPathfinder::set_swimming_wading(bool swimming_wading)
{
	this->swimming_wading = swimming_wading;
}

bool BetterPathfinder::find_path(std::vector<v3s16>& nodes)
{
	add_if_unvisited(start, start, 0);

	while (!to_visit.empty())
	{
		v3s16 node = to_visit.front().second;
		to_visit.pop_front();

		if (node == end)
		{
			NodeData* nd = &nodes_data[node];

			nodes.reserve(nd->cost);

			while (node != nd->previous)
			{
				nodes.push_back(node);
				node = nd->previous;
				nd = &nodes_data[node];
			}

			nodes.push_back(node);

			return true;
		}

		visit_node(node);
	}

	return false;
}

BetterPathfinder::InvalidPositionReason BetterPathfinder::is_invalid_position(const v3s16& node)
{
	INodeDefManager *ndef = env->getGameDef()->ndef();

	enum Submergence
	{
		UNKNOWN, NONE, PARTIAL, FULL
	} submergence;
	submergence = NONE;


	for (unsigned h = 0; h != height_required; ++h)
	{
		// Check to make sure this is not solid
		MapNode node_info = env->getMap().getNodeNoEx(node - v3s16(0,-h,0));
		const ContentFeatures & node_cf = ndef->get(node_info);
		if (node_cf.walkable)
			return SOLID;

		// If it is a liquid we need to check all our swimming rules
		if (node_cf.liquid_type != LIQUID_NONE)
		{
			// If this is the topmost height (the head) better make sure we can swim at the
			// surface, or below the surface
			if ((h == height_required - 1) && !swimming_surface && !swimming_below)
				return WILL_DROWN;

			// If this is not the head, make sure we are allowed to wade through water
			if ((h != height_required - 1) && !swimming_wading)
				return SOLID;

			// If we can swim below surface, we don't care what's above us
			if (!swimming_below)
			{
				if (h == height_required - 1)
				{
					// To get here, we must have swimming_surface
					// If this is the topmost height, make sure there is "walkable" above

					MapNode above_head = env->getMap().getNodeNoEx(node - v3s16(0, -h, 0) + v3s16(0, 1, 0));
					if (ndef->get(above_head).walkable || !swimming_surface)
						return WILL_DROWN;
				}
			}

			if (submergence == UNKNOWN)
				submergence = FULL;
			if (submergence == NONE)
				submergence = PARTIAL;
		}
		else
		{
			if (submergence == FULL)
				submergence = PARTIAL;
			if (submergence == UNKNOWN)
				submergence = NONE;
		}
	}

	MapNode below_node_info = env->getMap().getNodeNoEx(node + v3s16(0, -1, 0));
	// If we are partially submerged, make sure we can wade and are standing on a solid
	// or make sure we can swim at the surface
	if (submergence == PARTIAL)
	{
		if (!swimming_surface)
		{
			if (!(ndef->get(below_node_info).walkable && swimming_wading))
				return CANNOT_STAND;
		}
	}
	else {
		const ContentFeatures & below_node_cf = ndef->get(below_node_info);
		if (!below_node_cf.walkable)
		{
			// Can't stand on it, but maybe it's water and we can float
			if (below_node_cf.liquid_type == LIQUID_NONE || !swimming_above)
				return CANNOT_STAND;
		}
	}

	return NOT_INVALID;
}

void BetterPathfinder::visit_node(v3s16& node)
{
	visited_nodes.insert(node);
	NodeData& nd = nodes_data[node];

	v3s16 directions[4];

	directions[0] = v3s16(1, 0, 0);
	directions[1] = v3s16(-1, 0, 0);
	directions[2] = v3s16(0, 0, 1);
	directions[3] = v3s16(0, 0, -1);

	INodeDefManager *ndef = env->getGameDef()->ndef();

	//Basic move
	for (v3s16* dir = directions; dir != directions+4; ++dir)
	{
		v3s16 next_node = node + *dir;

		for (int drop_distance = 0; drop_distance <= max_drop_height; ++drop_distance)
		{
			v3s16 offset;
			offset.Y = -drop_distance;

			InvalidPositionReason reason = is_invalid_position(next_node + offset);

			if (reason == SOLID)
				break;

			if (reason == CANNOT_STAND)
				continue;

			if (reason == WILL_DROWN)
				break;

			unsigned next_node_cost = nd.cost + 1 + calculate_heuristic(next_node + offset);
			add_if_unvisited(next_node + offset, node, next_node_cost);
			break;
		}

		for (int jump_distance = 1; jump_distance <= max_jump_height; ++jump_distance)
		{
			v3s16 offset;
			offset.Y = jump_distance;

			// We don't want to hit our head when we jump
			MapNode above_head = env->getMap().getNodeNoEx(node + offset);
			if (ndef->get(above_head).walkable)
				break;

			InvalidPositionReason reason = is_invalid_position(next_node + offset);

			if (reason != NOT_INVALID)
				continue;

			unsigned next_node_cost = nd.cost + 1 + calculate_heuristic(next_node + offset);
			add_if_unvisited(next_node + offset, node, next_node_cost);
		}
	}
}

unsigned BetterPathfinder::calculate_heuristic(v3s16& node)
{
	int min_x = std::min(end.X, node.X);
	int max_x = std::max(end.X, node.X);
	int min_z = std::min(end.Z, node.Z);
	int max_z = std::max(end.Z, node.Z);

	return (max_x - min_x) + (max_z - min_z);
}

void BetterPathfinder::add_if_unvisited(v3s16& node, v3s16& from, unsigned cost)
{
	// Check to see if we've already visited this node
	if (visited_nodes.count(node) == 1)
		return;

	// Try to find existing
	std::list<std::pair<unsigned, v3s16>>::iterator tv = to_visit.begin();
	while (tv != to_visit.end())
	{
		if (tv->second == node)
			break;
		++tv;
	}

	// If it already exists, see if we need to update cost if lower and re-insert
	if (tv != to_visit.end())
	{
		if (cost >= tv->first)
			return;

		to_visit.erase(tv);
	}

	// Insert it in the queue at the right place
	std::list<std::pair<unsigned, v3s16>>::iterator insert_position = to_visit.begin();
	while (insert_position != to_visit.end())
	{
		if (insert_position->first > cost)
			break;
		++insert_position;
	}
	to_visit.insert(insert_position, std::make_pair(cost, node));

	// Update/insert node data
	NodeData& nd = nodes_data[node];
	nd.previous = from;
	nd.cost = cost;
}

BetterPathfinder::NodeSet BetterPathfinder::visited_nodes;
BetterPathfinder::NodeDataMap BetterPathfinder::nodes_data;