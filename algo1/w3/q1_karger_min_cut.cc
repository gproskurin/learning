#include <exception>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>
#include <vector>

#include <assert.h>
#include <stdlib.h>

#include "../../algo2/w2/sets.h"

typedef unsigned vertex_t;

typedef std::unordered_set<vertex_t> vertices_t;
typedef std::map<vertex_t, vertices_t> adj_t;

void add_edge(adj_t& adj, vertex_t v, vertex_t vv)
{
	// XXX
	assert(v != vv); // XXX exception?
	if (v < vv) {
		const auto p = adj[v].emplace(vv);
		assert(p.second==true);
		//std::cout << "Add: " << v << " " << vv << "\n";
	} else {
		const auto p = adj[v].emplace(vv);
		/*
		const adj_t::const_iterator vv_iter = adj_.find(vv);
		assert(vv_iter!=adj_.cend());
		assert(vv_iter->second.find(v)!=vv_iter->second.cend());
		*/
	}
}

void print1(const adj_t& adj)
{
	std::vector<vertex_t> vvv;
	for (const auto& p : adj) {
		vvv.push_back(p.first);
	}
	std::sort(vvv.begin(), vvv.end());
	std::cout << "Print1: size=" << vvv.size() << "\n";
	for (const auto& i : vvv) {
		std::cout << i << "\n";
	}
}

bool exists_directed_pair(const adj_t& adj, vertex_t u, vertex_t v)
{
	//std::cout << "EX: " << u << " " << v << "\n";
	const auto set4u_iter = adj.find(u);
	if (set4u_iter == adj.cend())
		return false;
	assert(set4u_iter->first == u);
	const auto& set4u = set4u_iter->second;
	const auto v_iter = set4u.find(v);
	const bool exists = (v_iter != set4u.cend());
	return exists;
}

template <typename Pred>
void for_each_edge(const adj_t& adj, const Pred& pred)
{
	for (const auto& u_pair : adj) {
		const auto& u = u_pair.first;
		const auto& set4u = u_pair.second;
		for (const auto& v : set4u) {
			pred(u,v);
		}
	}
}

bool check_all(const adj_t& adj)
{
	bool res = true;
	for_each_edge(
		adj,
		[&adj, &res](vertex_t u, vertex_t v) {
			assert(exists_directed_pair(adj, u, v));
			if (!exists_directed_pair(adj, v, u)) {
				res = false;
				// TODO return immediately
			}
		}
	);
	return res;
}

adj_t read_data(std::istream& is)
{
	adj_t res;
	while (is) {
		std::string line;
		std::getline(is, line);
		if (!is)
			break;
		std::istringstream is_line(line);
		vertex_t v;
		is_line >> v;
		if (!is_line)
			break;
		while (is_line) {
			vertex_t vv;
			is_line >> vv;
			if (!is_line)
				break;
			add_edge(res, v, vv);
		}
		//std::cout << "V: " << v << "\n";
	}
	return res;
}

typedef sets_t<vertex_t> vertex_union_t;
typedef std::pair<vertex_t, vertex_t> v2_t;

v2_t find_random_edge(const adj_t& adj, const vertex_union_t& U)
{
	std::vector<v2_t> edges;
	for_each_edge(
		adj,
		[&U,&edges](vertex_t v1, vertex_t v2) {
			if (U.are_separated(v1, v2)) {
				edges.emplace_back(v1, v2);
			}
		}
	);
	const size_t rnd = random() % edges.size();
	const auto& e = edges.at(rnd);
	//std::cout << "rnd_edge: " << e.first << " " << e.second << "\n";
	return e;
}

size_t count_rnd_cut(const adj_t& adj)
{
	const auto v_min = adj.cbegin()->first;
	const auto v_max = adj.crbegin()->first;
	//std::cout << "min:" << v_min << " max:" << v_max << "\n";

	vertex_union_t U(v_min, v_max);

	while (U.count_sets_gt(2)) {
		const auto e = find_random_edge(adj, U);
		const auto& v1 = e.first;
		const auto& v2 = e.second;
		assert(exists_directed_pair(adj,v1,v2));
		assert(exists_directed_pair(adj,v2,v1));
		U.union_sets(v1, v2);
	}
	//std::cout << "U_size: " << U.count_sets() << "\n";
	assert(U.count_sets()==2);

	size_t count_cut_size = 0;
	for_each_edge(
		adj,
		[&U, &count_cut_size](vertex_t v1, vertex_t v2) {
			if (U.are_separated(v1, v2)) {
				++count_cut_size;
				//std::cout << "CUT_EDGE: " << v1 << " " << v2 << "\n";
			}
		}
	);
	// each edge accurs twice in adjacency list
	assert((count_cut_size & 1) == 0);
	return count_cut_size>>1;
}

void run()
{
	const auto adj = read_data(std::cin);
	std::cout << "size1: " << adj.size() << "\n";
	//adj.print1();
	assert(check_all(adj));
	assert(!adj.empty());
	size_t min_cut = static_cast<size_t>(-1);
	const int iters = 100;
	for (int i=0; i<iters; ++i) {
		const auto cut_size = count_rnd_cut(adj);
		if (cut_size < min_cut) {
			min_cut = cut_size;
			std::cout << "min_cut: " << min_cut << "\n";
		}
	}
}

int main()
{
	try {
		const auto seed = time(nullptr);
		std::cout << "seed: " << seed << "\n";
		srandom(seed);
		run();
		return 0;
	} catch (const std::exception& ex) {
		std::cerr << "Ex: " << ex.what() << "\n";
	}
	return 1;
}
