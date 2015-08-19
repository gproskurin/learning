#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>

#include <exception>
#include <iostream>
#include <set>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <assert.h>

typedef unsigned vertex_t;

typedef boost::container::flat_set<vertex_t> vertices_t;
typedef std::vector<vertices_t> adj_t;
typedef std::vector<bool> v_set_t;

size_t v2i(vertex_t v) { return v-1; }
vertex_t i2v(size_t i) { return i+1; }

void add_edge(adj_t& adj, vertex_t v1, vertex_t v2)
{
	const auto i = v2i(v1);
	if (i >= adj.size())
		adj.resize(i+1);
	auto& vv = adj.at(i);
	const auto p = vv.insert(v2);
	assert(p.second==true);
}

adj_t reverse(const adj_t& adj)
{
	adj_t res;
	for (size_t i=0; i<adj.size(); ++i) {
		const auto v1 = i2v(i);
		for (const auto& v2 : adj.at(i)) {
			add_edge(res, v2, v1);
		}
	}
	assert(res.size() <= adj.size());
	if (res.size() < adj.size())
		res.resize(adj.size());
	return res;
}

std::string vv2str(const vertices_t& vv)
{
	std::ostringstream os;
	os << "{";
	bool first=true;
	for (const auto& v : vv) {
		if (!first)
			os << ",";
		else
			first = false;
		os << v;
	}
	os << "}";
	return os.str();
}

void print_stat(const adj_t& adj)
{
	std::cout << "SIZE: " << adj.size() << "\n";
	size_t empty = 0, nempty = 0;
	for (const auto& vv : adj) {
		if (vv.empty())
			++empty;
		else
			++nempty;
	}
	std::cout << "EMPTY: " << empty << " NON_EMPTY: " << nempty << " " << " fill_ratio:" << (nempty*100/adj.size()) << "%\n";

	const size_t sz = std::min(adj.size(), size_t(5));
	for (size_t i=0; i<sz; ++i) {
		std::cout << i2v(i) << " => " << vv2str(adj.at(i)) << "\n";
	}
	std::cout << "...\n";
	for (auto i=adj.size()-sz; i<adj.size(); ++i) {
		std::cout << i2v(i) << " => " << vv2str(adj.at(i)) << "\n";
	}
}

adj_t read_data(std::istream& is)
{
	adj_t res;
	//res.reserve(875714);
	while (is) {
		vertex_t v1, v2;
		is >> v1 >> v2;
		if (is) {
			//std::cout << "E: " << v1 << " -> " << v2 << "\n";
			add_edge(res, v1, v2);
		}
	}
	return res;
}

namespace detail {
	template <typename Pred>
	void DFS_impl(const adj_t& g, const vertex_t v, v_set_t& v_used, const Pred& pred)
	{
		const auto i = v2i(v);
		assert(v_used.at(i) == true);
		const auto& vv = g.at(i);
		for (const auto& v2 : vv) {
			const auto i2 = v2i(v2);
			if (v_used.at(i2)==false) {
				v_used[i2] = true;
				DFS_impl(g, v2, v_used, pred);
			}
		}
		pred(v);
	}
}

template <typename Pred>
void DFS(const adj_t& g, const Pred& pred)
{
	v_set_t v_used(g.size(), false);
	for (size_t i=0; i<g.size(); ++i) {
		if (v_used.at(i)==false) {
			v_used[i] = true;
			const auto v = i2v(i);
			detail::DFS_impl(g, v, v_used, pred);
		}
	}
}

void run()
{
	const auto adj = read_data(std::cin);
	std::cout << "GRAPH:\n";
	print_stat(adj);
	std::cout << "\n";

	std::cout << "REVERSE_GRAPH:\n";
	const auto adj_rev = reverse(adj);
	print_stat(adj_rev);
	std::cout << "\n";

	DFS(
		adj,
		[](vertex_t v) {
			std::cout << v << "\n";
		}
	);
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
