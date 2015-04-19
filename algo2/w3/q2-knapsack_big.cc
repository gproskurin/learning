#include <exception>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <assert.h>

struct item_t {
	size_t v;
	size_t w;
	item_t(size_t vv, size_t ww) : v(vv), w(ww) {}
};
typedef std::vector<item_t> items_t;

template<typename T>
class sparse_matrix_t {
	typedef std::unordered_map<size_t, T> map2_t;
	typedef std::unordered_map<size_t, map2_t> data_t;
	data_t data_;
	const size_t a_;
	const size_t b_;
public:
	mutable size_t get_miss_ = 0;
	mutable size_t get_hit_ = 0;
	mutable size_t set_ = 0;

	sparse_matrix_t(size_t a, size_t b) : a_(a), b_(b) {}

	void set_new_at(size_t a, size_t b, T value) {
		++set_;
		check_idx(a,b);
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
		
	const size_t* get_ptr_at(size_t a, size_t b) const noexcept {
		check_idx(a,b);
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
private:
	void check_idx(size_t a, size_t b) const {
		if (a >= a_ || b>=b_)
			throw std::range_error("matrix_at_range");
	}
};
typedef sparse_matrix_t<size_t> sol_matr_t;

size_t solve_knapsack_at(const items_t& data, sol_matr_t& sol, size_t i, size_t x)
{
	if (i==0)
		return 0;
	if (const size_t* const R = sol.get_ptr_at(i,x)) {
		return *R;
	}
	const item_t& item = data.at(i-1);
	const size_t wi = item.w;
	const size_t sol_prev_1 = solve_knapsack_at(data, sol, i-1, x);
	if (wi > x) {
		sol.set_new_at(i, x, sol_prev_1);
		return sol_prev_1;
	}
	const size_t vi = item.v;
	const size_t R = std::max(sol_prev_1, solve_knapsack_at(data, sol, i-1, x-wi)+vi);
	sol.set_new_at(i, x, R);
	return R;
}

size_t knapsack(const items_t& data, const size_t W)
{
	sol_matr_t sol(data.size()+1, W+1);
	const size_t R = solve_knapsack_at(data, sol, data.size(), W);
	std::cout << "get_hit:" << sol.get_hit_ << " get_miss:" << sol.get_miss_ << " set:" << sol.set_ << "\n";
	return R;
}

std::tuple<size_t, items_t> load_data(std::istream& is)
{
	items_t res;
	size_t W, n;
	is >> W >> n;
	res.reserve(n);
	for (size_t i=0; i<n; ++i) {
		size_t v, w;
		is >> v >> w;
		res.emplace_back(v,w);
	}
	if (!is)
		throw std::runtime_error("early_EOF");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("no_EOF");
	if (res.size()!=n)
		throw std::runtime_error("size_mismatch");
	return std::make_tuple(W, std::move(res));
}


void run()
{
	size_t W;
	items_t data;
	std::tie(W, data) = load_data(std::cin);
	std::cout << "W:" << W << " size:" << data.size() << "\n";
	const size_t R = knapsack(data, W);
	std::cout << "Result: " << R << "\n";
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
