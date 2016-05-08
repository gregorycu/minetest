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
	swimming_wading = false;
	diagonally = false;
	height_required = 1;
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

void BetterPathfinder::set_diagonally(bool diagonally)
{
	this->diagonally = diagonally;
}

bool BetterPathfinder::find_path(std::vector<v3s16>& nodes)
{
 	add_if_unvisited(start, start, 0, calculate_heuristic(end));

 	while (!to_visit.empty())
	{
		const to_visit_record& record = to_visit.top();
		visit_node(record);

		if (record.pos == end)
		{
			NodeData* nd = &nodes_data[record.pos];

			nodes.reserve(100);

			v3s16 node = record.pos;

			while (node != nd->previous)
			{
				nodes.push_back(node);
				node = nd->previous;
				nd = &nodes_data[node];
			}

			nodes.push_back(node);

			return true;
		}
		to_visit.pop();
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

void BetterPathfinder::visit_node(const to_visit_record& record)
{
	//visited_nodes.insert(node);

	NodeData& nd = nodes_data[record.pos];
	bool visited = nd.cost != UINT_MAX;

	if (record.cost < nd.cost)
	{
		nd.previous = record.prev;
		nd.cost = record.cost;
	}

	if (visited)
		return;

	v3s16 node = record.pos;

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

			double next_node_cost = nd.cost + 10;
			add_if_unvisited(next_node + offset, node, next_node_cost, next_node_cost + calculate_heuristic(next_node + offset));
			break;
		}

		for (int jump_distance = 1; jump_distance <= max_jump_height; ++jump_distance)
		{
			v3s16 offset;
			offset.Y = jump_distance;

			// We don't want to hit our head when we jump
			MapNode above_head = env->getMap().getNodeNoEx(node + offset + v3s16(0, height_required-1,0));
			if (ndef->get(above_head).walkable)
				break;

			InvalidPositionReason reason = is_invalid_position(next_node + offset);

			if (reason != NOT_INVALID)
				continue;

			double next_node_cost = nd.cost + 10;
			add_if_unvisited(next_node + offset, node, next_node_cost, next_node_cost + calculate_heuristic(next_node + offset));
		}
	}

	if (diagonally)
	{
		v3s16 diagonal_directions[4];
		diagonal_directions[0] = v3s16(1, 0, -1);
		diagonal_directions[1] = v3s16(-1, 0, -1);
		diagonal_directions[2] = v3s16(1, 0, 1);
		diagonal_directions[3] = v3s16(-1, 0, 1);

		for (v3s16* dir = diagonal_directions; dir != diagonal_directions + 4; ++dir)
		{
			v3s16 next_node = node + *dir;

			if (is_invalid_position(next_node) != NOT_INVALID)
				continue;

			InvalidPositionReason x_reason = is_invalid_position(node + v3s16(dir->X, 0, 0));
			if (x_reason != NOT_INVALID && x_reason != CANNOT_STAND)
				continue;

			InvalidPositionReason z_reason = is_invalid_position(node + v3s16(0, 0, dir->Z));
			if (z_reason != NOT_INVALID && z_reason != CANNOT_STAND)
				continue;

			double next_node_cost = nd.cost + 15;
			add_if_unvisited(next_node, node, next_node_cost, next_node_cost + calculate_heuristic(next_node));
		}
	}


}

unsigned BetterPathfinder::calculate_heuristic(v3s16& node)
{
	int min_x = std::min(end.X, node.X);
	int max_x = std::max(end.X, node.X);
	int min_z = std::min(end.Z, node.Z);
	int max_z = std::max(end.Z, node.Z);

	if (diagonally)
	{
		int dx = max_x - min_x;
		int dy = max_z - min_z;

		if (dx > dy)
			return (dx - dy) * 10 + dy * 15;
		else
			return (dy - dx) * 10 + dx * 15;
	}

	return ((max_x - min_x) + (max_z - min_z)) * 10;
}

void BetterPathfinder::add_if_unvisited(v3s16& node, v3s16& from, unsigned cost, unsigned estimated_total_cost)
{
	// Insert it in the queue at the right place

	if (estimated_total_cost > maximum_distance)
		return;

	to_visit.push(to_visit_record(cost, estimated_total_cost, node, from));
}

BetterPathfinder::NodeDataMap BetterPathfinder::nodes_data;