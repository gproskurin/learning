#include <exception>
#include <iostream>
#include <fstream>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include <boost/graph/adjacency_list.hpp>

#include <assert.h>

using vertex_t = size_t;

struct adj_item_t {
	vertex_t v1;
	vertex_t v2;
	long cost;
	adj_item_t(size_t vv1, size_t vv2, long c) : v1(vv1), v2(vv2), cost(c) {}
};
using adj_list_t = std::vector<adj_item_t>;

using graph_t = boost::adjacency_list<
	boost::setS,
	boost::setS,
	boost::undirectedS,
	boost::property<boost::edge_weight_t, long>
>;

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

graph_t convert_graph(const adj_list_t& al)
{
	graph_t gr;
	using vd_t = boost::graph_traits<graph_t>::vertex_descriptor;
	//boost::property_map<graph_t, boost::edge_weight_t>::type weight_map = boost::get(boost::edge_weight, gr);
	for (const adj_item_t& a : al) {
		const auto p = boost::add_edge(vd_t(a.v1), vd_t(a.v2), gr);
		assert(p.second==true);
		const auto& ed = p.first;
		//ed
	}

	return gr;
}


void run()
{
	std::ifstream in;
	in.open("q3-edges.txt");
	const adj_list_t al = load_graph(in);
	const graph_t gr = convert_graph(al);

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
