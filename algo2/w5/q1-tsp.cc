#include <array>
#include <bitset>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <sstream>

#include <math.h>
#include <assert.h>

// for speed. To use bitset
const size_t N = 3;

static_assert(std::numeric_limits<double>::has_infinity, "no_inf_for_double");
static const double pos_inf = std::numeric_limits<double>::infinity();

struct point_t {
	double x;
	double y;
	point_t(double xx, double yy) : x(xx), y(yy) {}
	point_t() : point_t(0.0, 0.0) {}
	std::string to_string() const { std::ostringstream os; os << "("<<x<<","<<y<<")"; return os.str(); }
};
typedef std::array<point_t, N> points_t;
typedef std::bitset<N> bitset_t;

double dist2(const point_t& p1, const point_t& p2)
{
	const auto dx = p1.x - p2.x;
	const auto dy = p1.y - p2.y;
	return dx*dx + dy*dy;
}

double dist(const point_t& p1, const point_t& p2)
{
	const double d = sqrt(dist2(p1,p2));
	//std::cout << "dist: " << p1.to_string() << " " << p2.to_string() << " = " << d << "\n";
	return d;
}

points_t load_graph(std::istream& is)
{
	points_t res;
	size_t n;
	is >> n;
	if (!is)
		throw std::runtime_error("early_EOF");
	if (n != N)
		throw std::runtime_error("unsupported_N");
	for (size_t i=0; i<n; ++i) {
		double x, y;
		is >> x >> y;
		if (!is)
			throw std::runtime_error("early_EOF");
		res.at(i) = point_t(x,y);
	}
	if (res.size() != n)
		throw std::runtime_error("size_mismatch");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("no_EOF");
	return res;
}

class subset_generator_t {
	typedef std::vector<bitset_t> subsets_t;
	typedef std::array<subsets_t, N+1> data_by_count1_t; // index: 0..N
	data_by_count1_t data_;
public:
	subset_generator_t() { generate_all_subsets(); }
	const subsets_t& get_subsets_with_1_count(size_t c) const {
		assert(c >= 2);
		return data_.at(c);
	}
	size_t total_count() const {
		size_t s = 0;
		for (const auto& a : data_) {
			s += a.size();
		}
		return s;
	}
private:
	void generate_all_subsets() {
		static const unsigned long long max_val = (1ULL << N) - 1;
		const std::string max_str = bitset_t(max_val).to_string() ;
		std::cout << "max_val: " << max_str << " len:" << max_str.size() << "\n";
		for (unsigned long long i=0; i<=max_val; ++i) {
			bitset_t S(i);
			if (S[0]==false)
				continue;
			const auto count1 = S.count();
			if (count1 < 2)
				continue;
			if (i==max_val)
				assert(count1==N);
			else
				assert(count1 < N);
			data_[count1].emplace_back(std::move(S));
		}
		std::cout << "Subsets: " << total_count() << "\n";
	}
};

class results_t {
	typedef std::unordered_map<size_t, double> map2_t;
	typedef std::unordered_map<bitset_t, map2_t> res_cache_t;
	res_cache_t data_;
public:
	const double* get_ptr_at(const bitset_t& S, size_t j) const noexcept {
		//std::cout << " ** get_ptr_at(\"" << S.to_string() << "\", " << j << ")" << std::flush;
		assert(S[0]==true);
		if (j==0) { // path to initial point
			if (S.count()==1) {
				static const double zero = 0.0;
				//std::cout << " -> zero (j==0, S.count()==1)\n";
				return &zero;
			}
			//std::cout << " -> pos_inf (j==0, S.count()!=1)\n";
			return &pos_inf; // TODO: don't use global
		}
		const auto iter1 = data_.find(S);
		if (iter1==data_.cend()) {
			//std::cout << " -> nullptr (S not found)\n";
			return nullptr;
		}
		const auto& D = iter1->second;
		const auto iter2 = D.find(j);
		if (iter2==D.cend()) {
			//std::cout << " -> nullptr (S found, data not)\n";
			return nullptr;
		}
		//std::cout << " -> " << iter2->second << "\n";
		return &iter2->second;
	}

	void set_at(const bitset_t& S, size_t j, double val) {
		//std::cout << " ** set_at(\"" << S.to_string() << "\", " << j << ", " << val << ")\n";
		assert(S[0]==true);
		//assert(get_ptr_at(S,j) == nullptr);
		const auto iter1 = data_.find(S);
		if (iter1 == data_.end()) {
			const auto p = data_.emplace(S, map2_t{ {j,val} });
			assert(p.second == true);
			return;
		}
		map2_t& map2 = iter1->second;
		const auto p = map2.emplace(j, val);
		assert(p.second == true);
	}
};

void run()
{
	std::cout << "bitset_t size: " << sizeof(bitset_t) << "\n";
	const points_t pp = load_graph(std::cin);
	std::cout << "Data size: " << pp.size() << "\n";
	assert(pp.size() == N);
#if 0
	// print
	std::cout.precision(10);
	for (const auto& p : pp) {
		std::cout << "\t" << p.x << " " << p.y << "\n";
	}
#endif

	subset_generator_t Sgen;
	results_t A;
	for (size_t m=2; m<=N; ++m) { // m - subproblem size
		const auto& SS = Sgen.get_subsets_with_1_count(m);
		std::cout << " * subproblem_size:" << m << " subsets:" << SS.size() << "\n";
		for (const bitset_t& S : SS) {
			assert(S[0]==true);
			assert(S.count()==m);
			for (size_t j=1; j<=N; ++j) {
				if (S[j]==false)
					continue;
				double min = pos_inf;
				for (size_t k=0; k<=N; ++k) {
					if (k!=j && S[k]==true) {
						bitset_t S_minus_j = S;
						assert(S_minus_j[j] == true);
						S_minus_j[j] = false;
						const double* const cur_ptr = A.get_ptr_at(S_minus_j, k);
						assert(cur_ptr != nullptr);
						const double cur = *cur_ptr + dist(pp.at(k), pp.at(j));
						if (cur < min)
							min = cur;
					}
				}
				A.set_at(S, j, min);
			}
		}
	}

	const bitset_t Sall((1ULL << N) - 1);
	std::cout << "Sall: " << Sall.to_string() << "\n";
	double min = pos_inf;
	for (size_t j=1; j<=N; ++j) {
		const double* const cur_ptr = A.get_ptr_at(Sall, j);
		assert(cur_ptr != nullptr);
		const double cur = *cur_ptr + dist(pp.at(j), pp.at(0));
		if (cur < min)
			min = cur;
	}
	std::cout << "Result: " << min << "\n";
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
