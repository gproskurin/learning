#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <future>
#include <vector>

#include <assert.h>

typedef int vertex_t;
typedef long cost_t;
const cost_t pos_inf_cost = std::numeric_limits<cost_t>::max();

struct item_t {
	vertex_t v1;
	vertex_t v2;
	cost_t cost;
	item_t(vertex_t vv1, vertex_t vv2, cost_t c) : v1(vv1), v2(vv2), cost(c) {}
};
typedef std::vector<item_t> adj_list_t;


class vertices_info_t {
	const vertex_t idx_shift_;

	struct bf_item_t {
		static const size_t no_idx = std::numeric_limits<size_t>::max();
		size_t prev_idx;
		cost_t sum_cost;
		bf_item_t() : prev_idx(no_idx), sum_cost(pos_inf_cost) {}
	};
	typedef std::vector<bf_item_t> bf_data_t;
	mutable bf_data_t data_;
private:
	size_t v2idx(vertex_t v) const { return static_cast<size_t>(v - idx_shift_); }
	vertex_t idx2v(size_t i) const { return static_cast<vertex_t>(i) + idx_shift_; }
public:
	vertices_info_t(vertex_t min, vertex_t max) : idx_shift_(min)
	{
		if (max < min)
			throw std::runtime_error("vertex_wrong_min_max");
		const size_t size = max - min + 1;
		data_.resize(size);
		//std::cout << "data_size: " << data_.size() << "\n";
	}
	bf_item_t& set(vertex_t v) { return data_.at(v2idx(v)); }
	const bf_item_t& get(vertex_t v) const { return data_.at(v2idx(v)); }
	cost_t get_shortest_cost(vertex_t s) const {
		const size_t s_idx = v2idx(s);
		cost_t min_cost = pos_inf_cost;
		for (size_t i=0; i<data_.size(); ++i) {
			if (i!=s_idx) {
				const auto cur_cost = data_.at(i).sum_cost;
				assert(cur_cost < pos_inf_cost);
				if (cur_cost < min_cost) {
					min_cost = cur_cost;
					//std::cout << "   - new_min_cost: " << cur_cost << "\n";
				}
			}
		}
		assert(min_cost < pos_inf_cost);
		//std::cout << " - cur_min_cost: " << min_cost << "\n";
		return min_cost;
	}
};

std::pair<vertex_t,vertex_t> minmax_vertex(const adj_list_t& g)
{
	assert(!g.empty());

	// assign to something
	vertex_t min = g.front().v1;
	vertex_t max = g.front().v1;

	for (const auto& e : g) {
		min = std::min(min, std::min(e.v1, e.v2));
		max = std::max(max, std::max(e.v1, e.v2));
	}
	return std::make_pair(min, max);
}

vertices_info_t bellman_ford(const adj_list_t& g, vertex_t s)
{
	const auto mmv = minmax_vertex(g);
	vertices_info_t res(mmv.first, mmv.second);

	res.set(s).sum_cost = 0;

	for (size_t bf_iter=1; bf_iter<g.size()-1; ++bf_iter) {
		//if (bf_iter % 1000 == 0)
		//	std::cout << " - iter: " << bf_iter << "/" << g.size() << "\n";
		for (const auto& e : g) {
			const cost_t cost_prev = res.get(e.v1).sum_cost;
			const cost_t cost_from_prev = (cost_prev==pos_inf_cost) ? pos_inf_cost : (cost_prev + e.cost);
			auto& cur = res.set(e.v2);
			if (cost_from_prev < cur.sum_cost) {
				cur.sum_cost = cost_from_prev;
				cur.prev_idx = e.v1;
			}
		}
	}

	//std::cout << "Checking for negative cycles...\n";
	for (const auto& e : g) {
		const cost_t cost_from_prev = res.get(e.v1).sum_cost + e.cost;
		const cost_t cost_cur = res.get(e.v2).sum_cost;
		if (cost_from_prev < cost_cur)
			throw std::runtime_error("neg_cycle");
	}
	//std::cout << "No negative cycles\n";

	return res;
}

adj_list_t load_graph(std::istream& is)
{
	adj_list_t res;
	size_t num_v, num_e;
	is >> num_v >> num_e;
	res.reserve(num_e);
	for (size_t e=0; e<num_e; ++e) {
		vertex_t v1, v2;
		cost_t c;
		is >> v1 >> v2 >> c;
		if (v1>static_cast<vertex_t>(num_v) || v2>static_cast<vertex_t>(num_v))
			throw std::runtime_error("invalid_vertex");
		res.emplace_back(v1, v2, c);
	}
	assert(res.size()==num_e);
	if (!is)
		throw std::runtime_error("early_EOF");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("no_EOF");
	return res;
}
typedef std::future<vertices_info_t> bf_future_t;

bf_future_t bellman_ford_async(const adj_list_t& g, vertex_t s)
{
	return std::async(std::launch::async, bellman_ford, std::cref(g), s);
}

void run()
{
	const adj_list_t gr = load_graph(std::cin);
	const auto mmv = minmax_vertex(gr);
	std::cout << "min_vertex:" << mmv.first << " max_vertex:" << mmv.second << "\n";
	std::cout << "Inf cost: " << pos_inf_cost << "\n";
	std::cout << "Edges: " << gr.size() << "\n";
	std::cout << "\n";

	typedef std::unordered_map<vertex_t, bf_future_t> async_bf_t;
	async_bf_t async_bf;

	for (vertex_t s = mmv.first; s<=mmv.second; ++s) {
		const auto p = async_bf.emplace(s, bellman_ford_async(gr, s));
		assert(p.second == true);
	}

	cost_t min_cost = pos_inf_cost;
	for (auto& cur : async_bf) {
		const vertex_t s = cur.first;
		std::cout << "* waiting for vertex " << s << "..." << std::flush;
		cur.second.wait();
		std::cout << " done. " << std::flush;
		const auto cur_gr = cur.second.get();
		const cost_t min_cur = cur_gr.get_shortest_cost(s);
		std::cout << "MIN=" << min_cur << std::endl;
		if (min_cur < min_cost) {
			min_cost = min_cur;
			std::cout << " - new min cost: " << min_cost << std::endl;
		}
	}
	std::cout << "Total min cost: " << min_cost << "\n";
}

int main()
{
	try {
		run();
		return 0;
	} catch (const std::exception& ex) {
		std::cerr << "Ex: " << ex.what() << "\n";
	}
	return 1;
}
