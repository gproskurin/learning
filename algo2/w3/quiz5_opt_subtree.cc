#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include <assert.h>

template<typename T, typename Index>
class sparse_matrix_t {
	typedef std::map<Index, T> map2_t;
	typedef std::map<Index, map2_t> data_t;
	data_t data_;
public:
	mutable size_t get_miss_ = 0;
	mutable size_t get_hit_ = 0;
	mutable size_t set_ = 0;

	void set_new_at(Index a, Index b, T value) {
		++set_;
		auto iter1 = data_.find(a);
		if (iter1 == data_.end()) {
			const auto p = data_.emplace(a, map2_t());
			assert(p.second == true);
			iter1 = p.first;
		}
		auto& map2 = iter1->second;
		const auto p = map2.emplace(b, value);
		if (p.second == false)
			throw std::runtime_error("set_new_at_exists");
	};
		
	const T* get_ptr_at(Index a, Index b) const noexcept {
		const auto iter1 = data_.find(a);
		if (iter1 == data_.cend()) {
			++get_miss_;
			return nullptr;
		}
		const map2_t& map2 = iter1->second;
		const auto iter2 = map2.find(b);
		if (iter2 == map2.cend()) {
			++get_miss_;
			return nullptr;
		}
		++get_hit_;
		return &iter2->second;
	}
};

template <typename Sol, typename Iter>
typename std::iterator_traits<Iter>::value_type get_best(
	Sol& sol,
	const Iter first,
	const Iter last)
{
	if (first == last)
		return 0;

	if (auto const P = sol.get_ptr_at(first,last)) {
		return *P;
	}

	typedef typename std::iterator_traits<Iter>::value_type value_type;
	typedef std::vector<value_type> results_t;
	results_t ress;
	for (auto root=first; root!=last; ++root) {
		const auto left_sub = get_best(sol, first, root);
		const auto left_my_add = std::accumulate(first, root, value_type(0));
		const auto right_sub = get_best(sol, root+1, last);
		const auto right_my_add = std::accumulate(root+1, last, value_type(0));
		const auto my = left_sub + left_my_add + *root + right_sub + right_my_add;
		ress.emplace_back(my);
		std::cout << "l_sub:" << left_sub << " l_add:" << left_my_add << " l_size:" << std::distance(first,root)
			<< " root:" << *root
			<< " r_sub:" << right_sub << " r_add:" << right_my_add << " r_size:" << std::distance(root+1,last)
			<< " total:" << my << "\n";
	}
	const auto min_iter = std::min_element(ress.cbegin(), ress.cend());
	assert(min_iter != ress.cend());
	const auto res = *min_iter;
	sol.set_new_at(first, last, *min_iter);
	return res;
}

int main()
{
	typedef std::vector<double> data_t;
	const data_t data{ 0.05, 0.4, 0.08, 0.04, 0.1, 0.1, 0.23 };
	assert(data.size()==7);
	sparse_matrix_t<data_t::value_type, data_t::const_iterator> sol;
	const auto R = get_best(sol, data.cbegin(), data.cend());
	std::cout << "Result: " << R << "\n";
}
