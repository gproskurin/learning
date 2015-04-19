#include <exception>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <assert.h>

struct item_t {
	size_t v;
	size_t w;
	item_t(size_t vv, size_t ww) : v(vv), w(ww) {}
};
typedef std::vector<item_t> items_t;

template<typename T>
class matrix_t {
	std::vector<T> data_;
	const size_t a_;
	const size_t b_;
public:
	matrix_t(size_t a, size_t b) : data_(a*b), a_(a), b_(b) {}

	size_t& set_at(size_t a, size_t b) { return data_.at(get_idx(a,b)); }
	size_t get_at(size_t a, size_t b) const { return data_.at(get_idx(a,b)); }
private:
	void check_idx(size_t a, size_t b) const {
		if (a >= a_ || b>=b_)
			throw std::range_error("matrix_at_range");
	}
	size_t get_idx(size_t a, size_t b) const {
		check_idx(a,b);
		return a * b_ + b;
	}
};

size_t knapsack(const items_t& data, const size_t W)
{
	matrix_t<size_t> sol(data.size()+1, W+1);
	for (size_t w=0; w<=W; ++w) {
		sol.set_at(0,w) = 0;
	}
	for (size_t x=0; x<=W; ++x) {
		for (size_t i=1; i<=data.size(); ++i) {
			const item_t& item = data.at(i-1);
			const size_t wi = item.w;
			const size_t sol_prev_1 = sol.get_at(i-1, x);
			if (wi > x) {
				sol.set_at(i, x) = sol_prev_1;
			} else {
				const size_t vi = item.v;
				sol.set_at(i, x) = std::max(sol_prev_1, sol.get_at(i-1,x-wi) + vi);
			}
		}
	}
	return sol.get_at(data.size(), W);
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
