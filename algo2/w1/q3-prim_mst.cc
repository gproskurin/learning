#include <exception>
#include <iostream>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include <boost/optional.hpp>

#include <assert.h>

using vertex_t = size_t;

struct adj_item_t {
	vertex_t v1;
	vertex_t v2;
	long cost;
	adj_item_t(size_t vv1, size_t vv2, long c) : v1(vv1), v2(vv2), cost(c) {}
};
using adj_list_t = std::vector<adj_item_t>;
using adj_idx_t = adj_list_t::size_type;
using vtx_set_t = std::unordered_set<vertex_t>;
using adj_idx_set_t = std::unordered_set<adj_idx_t>;

class graph_t {
	const adj_list_t adj_;
	mutable boost::optional<size_t> num_vertexes_;
public:
	graph_t(const adj_list_t& al) : adj_(al) {}

	const adj_item_t& item_at(size_t i) const { return adj_.at(i); }
	bool empty() const { return adj_.empty(); }

	size_t count_edges() const { return adj_.size(); }
	size_t count_vertexes() const {
		if (!num_vertexes_) {
			std::unordered_set<vertex_t> vtxs;
			for (const auto& item : adj_) {
				vtxs.insert(item.v1);
				vtxs.insert(item.v2);
			}
			num_vertexes_ = vtxs.size();
		}
		return num_vertexes_.get();
	}

	vertex_t get_any_vertex() const { assert(!adj_.empty()); return adj_.front().v1; }

	adj_idx_t find_min_edge_from_vertexes(const vtx_set_t& V) const
	{
		boost::optional<size_t> min_idx;
		for (size_t i=0; i<adj_.size(); ++i)
		{
			const auto& item = adj_.at(i);
			if (
				exits_from(V, item.v1, item.v2)
				&&
				(
					!min_idx
					||
					(item.cost < adj_[min_idx.get()].cost)
				)
			)
			{
					min_idx = i;
			}
		}
		return min_idx.get();
	}
private:
	static bool exists(const vtx_set_t& V, vertex_t v) { return (V.find(v) != V.cend()); }
	static bool exits_from(const vtx_set_t& V, vertex_t v1, vertex_t v2) { return exists(V,v1) ^ exists(V,v2); }
};

adj_idx_set_t prim_mst(const graph_t& gr)
{
	assert(!gr.empty());

	adj_idx_set_t MSTe;
	vtx_set_t Xv;

	assert(gr.count_vertexes() != 0);

	Xv.insert(gr.get_any_vertex());
	while (Xv.size() != gr.count_vertexes()) {
		size_t e_idx = gr.find_min_edge_from_vertexes(Xv);
		const adj_item_t& new_edge = gr.item_at(e_idx);
		const bool p1 = Xv.insert(new_edge.v1).second;
		const bool p2 = Xv.insert(new_edge.v2).second;
		assert( (p1 && !p2) || (!p1 && p2) );
		MSTe.insert(e_idx);
	}

	return MSTe;
}

adj_list_t load_graph(std::istream& is)
{
	adj_list_t res;
	size_t num_nodes, num_edges;
	is >> num_nodes >> num_edges;
	res.reserve(num_edges);
	for (size_t e=0; e<num_edges; ++e) {
		size_t v1, v2;
		long c;
		is >> v1 >> v2 >> c;
		res.emplace_back(v1, v2, c);
	}
	if (!is)
		throw std::runtime_error("Unexpected EOF");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("EOF expected");
	std::cout << "nodes:" << num_nodes << " edges:" << num_edges << "," << res.size() << "\n";
	assert(res.size() == num_edges);
	return res;
}



void run()
{
	const adj_list_t al = load_graph(std::cin);
	const graph_t gr(al);
	std::cout << "count_edges:" << gr.count_edges() << " count_vertexes:" << gr.count_vertexes() << "\n";

	const adj_idx_set_t mst = prim_mst(gr);
	long mst_cost = 0;
	for (auto idx : mst) {
		mst_cost += gr.item_at(idx).cost;
	}

	std::cout << "mst_edges: " << mst.size() << " mst_cost: " << mst_cost << "\n";
}

int main()
{
	try {
		run();
	} catch (const std::exception& e) {
		std::cerr << "Ex: " << e.what() << "\n";
		return 1;
	}
	return 0;
}
