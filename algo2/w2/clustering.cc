#include "../../lib/sets/sets.h"

#include <algorithm>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <boost/optional.hpp>

#include <assert.h>

typedef size_t node_t;
typedef long cost_t;


class adj_list_t {
	struct item_t {
		node_t n1;
		node_t n2;
		cost_t cost;
		item_t(node_t nn1, node_t nn2, cost_t c) : n1(nn1), n2(nn2), cost(c) {}
	};
	typedef std::vector<item_t> item_list_t;
	item_list_t items_;

	// index
	struct hash_index {
		static size_t hash_node(node_t n) { return std::hash<node_t>()(n); }
		static size_t hash_nodes(node_t n1, node_t n2) { return hash_node(n1) ^ hash_node(n2); }
		typedef std::unordered_multimap<size_t, item_list_t::size_type> index_t;
	};
	hash_index::index_t index_;

public:
	size_t count_size() const { return items_.size(); }
	size_t index_size() const { return index_.size(); }

	// find index of pair (with ends in different sets) with minimum cost
	template <typename CLM>
	item_list_t::size_type min_cost_diff_idx(const CLM& clm) const
	{
		boost::optional<size_t> min_item_idx;
		clm.foreach_pair_in_different_sets(
			[&min_item_idx,this](node_t n1, node_t n2) {
				if (!min_item_idx || get_cost(n1,n2) < items_.at(min_item_idx.get()).cost)
					min_item_idx = search(n1,n2);
			}
		);
		return min_item_idx.get();
	}

	void test_clustering() const
	{
		const size_t K = 4;
		typedef sets_t<node_t> clm_t;
		clm_t clm(1,500);
		std::cout << "CLM_COUNT: " << clm.count_sets() << "\n";

		while (clm.count_sets() > K) {
			const item_list_t::size_type min_item_idx = min_cost_diff_idx(clm);
			const item_t& item = items_.at(min_item_idx);
			assert(!clm.in_one_set(item.n1, item.n2));
			const size_t old_count = clm.count_sets();
			clm.union_sets(item.n1, item.n2);
			const size_t new_count = clm.count_sets();
			std::cout << "SETS: " << old_count << " -> " << new_count << "\n";
		}
	}

	void add(node_t n1, node_t n2, cost_t c)
	{
		assert(find_eq_n1_n2(n1,n2) == items_.cend());
		const size_t next_index = items_.size();
		items_.emplace_back(n1, n2, c);
		index_.emplace(
			hash_index::hash_nodes(n1, n2),
			next_index
		);
		//std::cout << "added: n1:" << n1 << " n2:" << n2 << " cost:" << c << "\n";
	}
	cost_t get_cost(node_t n1, node_t n2) const
	{
		const item_list_t::const_iterator iter = find_eq_n1_n2(n1, n2);
		if (iter == items_.cend())
			throw std::runtime_error("not_found");
		return iter->cost;
	}

	item_list_t::size_type search(node_t n1, node_t n2) const
	{
		for (size_t i=0; i<items_.size(); ++i) {
			const auto& item = items_[i];
			if ((item.n1==n1 && item.n2==n2)
				|| (item.n2==n1 && item.n1==n2))
			{
				return i;
			}
		}
		throw std::runtime_error("search_not_found");
	}

private:
	item_list_t::const_iterator find_eq_n1_n2(node_t n1, node_t n2) const
	{
		const auto eq_n1_n2 = [n1,n2](const item_t& i) -> bool {
			return (n1==i.n1 && n2==i.n2) || (n1==i.n2 && n2==i.n1);
		};

		const auto idx_range = index_.equal_range(hash_index::hash_nodes(n1,n2));
		for (auto idx_iter = idx_range.first; idx_iter!=idx_range.second; ++idx_iter) {
			const auto item_idx = idx_iter->second;
			if (eq_n1_n2(items_.at(item_idx)))
				return items_.cbegin() + item_idx;
		}

		return items_.cend();
	}
};

adj_list_t load_graph(std::istream& is)
{
	adj_list_t res;
	size_t num_nodes;
	is >> num_nodes;
	for (node_t n1=1; n1<=num_nodes; ++n1) {
		for (node_t n2=n1+1; n2<=num_nodes; ++n2) {
			cost_t c;
			node_t n1_value, n2_value;
			is >> n1_value >> n2_value >> c;
			if (n1!= n1_value || n2!=n2_value)
				throw std::runtime_error("unexpected_node_values");
			res.add(n1, n2, c);
			assert(res.get_cost(n1,n2) == c);
			assert(res.get_cost(n2,n1) == c);
		}
	}
	if (!is)
		throw std::runtime_error("Unexpected EOF");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("EOF expected");
	std::cout << "nodes:" << num_nodes << " total_size:" << res.count_size() << " idx_size:" << res.index_size() << "\n";
	//assert(res.size() == num_edges);
	return res;
}

void run()
{
	const adj_list_t al = load_graph(std::cin);
	std::cout << "\n";
	al.test_clustering();
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
