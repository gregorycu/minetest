#include <vector>
#include <list>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include "irr_v3d.h"
#include "environment.h"

namespace std
{
	template <>
	struct hash<v3s16>
	{
		size_t operator()(const v3s16& value) const
		{
			return ((value.X * 128) + value.Y * 128) + value.Z;
		}
	};
} 

class BetterPathfinder
{



public:
	BetterPathfinder(ServerEnvironment *env, unsigned maximum_distance, v3s16 start, v3s16 end);
	void set_max_drop_height(unsigned height);
	void set_max_jump_height(unsigned height);
	void set_height_required(unsigned height);
	void set_swimming_above(bool swimming_above);
	void set_swimming_surface(bool swimming_surface);
	void set_swimming_below(bool swimming_below);
	void set_swimming_wading(bool swimming_wading);
	bool find_path(std::vector<v3s16>& nodes);
private:
	void visit_node(v3s16& node);
	unsigned calculate_heuristic(v3s16& node);
	void add_if_unvisited(v3s16& node, v3s16& from, unsigned cost);


	enum InvalidPositionReason
	{
		NOT_INVALID,
		CANNOT_STAND,
		SOLID,
		WILL_DROWN
	};

	InvalidPositionReason is_invalid_position(const v3s16& node);

	unsigned maximum_distance;
	v3s16 start;
	v3s16 end;
	ServerEnvironment *env;

	unsigned max_drop_height;
	unsigned max_jump_height;
	bool swimming_above;
	bool swimming_surface;
	bool swimming_below;
	bool swimming_wading;
	unsigned height_required;

	struct NodeData
	{
		v3s16 current;
		v3s16 previous;
		unsigned cost;
	};
	class NodeDataMap
	{
	public:
		NodeDataMap()
		{
			m_data.resize(number_buckets);

		}
		void clear()
		{
			for (std::vector<std::vector<NodeData> >::iterator iter = m_data.begin(); iter != m_data.end(); ++iter)
				iter->clear();
		}

		NodeData& operator[](const v3s16& p)
		{
			std::vector<NodeData>& bucket = m_data[std::hash<v3s16>()(p) % number_buckets];

			for (std::vector<NodeData>::iterator iter = bucket.begin(); iter != bucket.end(); ++iter)
			{
				if (iter->current == p)
					return *iter;
			}

			bucket.push_back(NodeData());
			bucket.rbegin()->current = p;
			return *bucket.rbegin();
		}

	private:
		const int number_buckets = 227;
		std::vector<std::vector<NodeData> > m_data;
	};
	class NodeSet
	{
	public:
		NodeSet(){
			m_data.resize(number_buckets);
		}
		void clear()
		{
			for (std::vector<std::vector<v3s16> >::iterator iter = m_data.begin(); iter != m_data.end(); ++iter)
				iter->clear();
		}
		int count(const v3s16& p) const
		{
			const std::vector<v3s16>& bucket = m_data[std::hash<v3s16>()(p) % number_buckets];
			return std::find(bucket.begin(), bucket.end(), p) != bucket.end() ? 1 : 0;
		}
		void insert(const v3s16& p)
		{
			std::vector<v3s16>& bucket = m_data[std::hash<v3s16>()(p) % number_buckets];
			if (std::find(bucket.begin(), bucket.end(), p) != bucket.end())
				return;
			bucket.push_back(p);
		}
	private:
		const int number_buckets = 227;
		std::vector<std::vector<v3s16> > m_data;
	};


	static NodeSet visited_nodes;
	std::list<std::pair<unsigned, v3s16>> to_visit;
	static NodeDataMap nodes_data;
};