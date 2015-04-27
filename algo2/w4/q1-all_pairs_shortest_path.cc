#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
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

struct bf_item_t {
	static const size_t no_idx = std::numeric_limits<size_t>::max();
	size_t prev_idx;
	cost_t sum_cost;
	bf_item_t() : prev_idx(no_idx), sum_cost(pos_inf_cost) {}
};
typedef std::unordered_map<vertex_t, bf_item_t> bf_result_t;

bf_result_t bellman_ford(const adj_list_t& g, size_t s_idx)
{
	bf_result_t res; // init with defaults
	res[s_idx].sum_cost = 0;

	for (size_t bf_iter=1; bf_iter<g.size()-1; ++bf_iter) {
		if (bf_iter % 1000 == 0)
			std::cout << " - iter: " << bf_iter << "/" << g.size() << "\n";
		for (const auto& e : g) {
			const cost_t cost_from_prev = res[e.v1].sum_cost + e.cost;
			bf_item_t& cur = res[e.v2];
			if (cost_from_prev < cur.sum_cost) {
				cur.sum_cost = cost_from_prev;
				cur.prev_idx = e.v1;
			}
		}
	}

	std::cout << "Checking for negative cycles...\n";
	for (const auto& e : g) {
		const cost_t cost_from_prev = res.at(e.v1).sum_cost + e.cost;
		const cost_t cost_cur = res.at(e.v2).sum_cost;
		if (cost_from_prev < cost_cur)
			throw std::runtime_error("neg_cycle");
	}
	std::cout << "No negative cycles\n";

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

void run()
{
	const adj_list_t gr = load_graph(std::cin);
	std::cout << "Inf cost: " << pos_inf_cost << "\n";
	std::cout << "Edges: " << gr.size() << "\n";

	const bf_result_t bf = bellman_ford(gr, 0);
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
