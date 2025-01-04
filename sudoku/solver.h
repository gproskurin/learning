#ifndef _my_solver_h_included_
#define _my_solver_h_included_


//#include <boost/iterator/iterator_facade.hpp> // TODO

#include <array>
#include <bitset>
#include <exception>
#include <iostream>
#include <iterator>
#include <optional>
#include <stdexcept>

#include <assert.h>


using num_t = char;
using idx_t = char;


template <num_t N> num_t num_parse(char c);
template<> num_t num_parse<4>(char c)
{
	if (c >= '1' && c <= '4')
		return c - '0' - 1;
	throw std::runtime_error("Num[1..4] parsing error");
}
template<> num_t num_parse<9>(char c)
{
	if (c >= '1' && c <= '9')
		return c - '0' - 1;
	throw std::runtime_error("Num[1..9] parsing error");
}
template<> num_t num_parse<16>(char c)
{
	if (c >= '1' && c <= '9')
		return c - '0' - 1;
	if (c >= 'a' && c <= 'g')
		return c - 'a' + 10 - 1;
	throw std::runtime_error("Num[1..a..g] parsing error");
}

template <num_t N>
char num_print(num_t n)
{
	assert(n < N);
	static_assert(N == 4 || N == 9 || N == 16);
	if ((N==4 && n<4) || (N==9 && n<9) || (N==16 && n<9)) {
		return '0' + n + 1;
	}
	if (N==16 && n<16) {
		return 'a' + n - 10 + 1;
	}
	throw std::runtime_error("Num printing error");
}


template <num_t N> struct sqrt_t;
template<> struct sqrt_t<4> { static constexpr num_t value = 2; };
template<> struct sqrt_t<9> { static constexpr num_t value = 3; };
template<> struct sqrt_t<16> { static constexpr num_t value = 4; };


template <num_t N>
using bitset_t = std::bitset<N>;

template <num_t N>
constexpr bitset_t<N> bitset_make_full()
{
	return bitset_t<N>(static_cast<unsigned long long>((1 << N) - 1));
}

template <num_t N>
constexpr bitset_t<N> bitset_make_empty()
{
	return bitset_t<N>(0ULL);
}

template <num_t N>
bitset_t<N> bitset_make_solved(num_t num)
{
	return bitset_t<N>(static_cast<unsigned long long>(1 << num));
}


template <num_t N>
bool bitset_is_solved(const bitset_t<N>& bs)
{
	assert(bs.any());
	return bs.count() == 1;
}


template <num_t N>
bool bitset_try_exclude(bitset_t<N>& bs, num_t n)
{
	if (bs.test(n)) {
		bs.reset(n);
		return true;
	}
	return false;
}


template <num_t N>
size_t bitset_exclude_set(bitset_t<N>& bs, const bitset_t<N>& excl)
{
	assert(bs.any());
	size_t r = 0;
	for (num_t n=0; n<N; ++n) {
		if (excl.test(n) && bitset_try_exclude<N>(bs, n)) {
			++r;
		}
	}
	assert(bs.any());
	return r;
}


template <num_t N>
std::optional<num_t> bitset_get_solved_opt(const bitset_t<N>& bs)
{
	assert(bs.any());
	if (bs.count() != 1) {
		return std::nullopt;
	}
	for (num_t n=0; n<N; ++n) {
		if (bs.test(n)) {
			return n;
		}
	}
	assert("unreacheable code");
	return std::nullopt; // FIXME
}


template <num_t N>
size_t bitset_assign_set(bitset_t<N>& bs, const bitset_t<N>& set)
{
	// return number of 1->0 transitions
	// TODO bit ops
	size_t r = 0;
	for (num_t i=0; i<N; ++i) {
		if (set.test(i)) {
			assert(bs.test(i)); // cannot "add" to set
		} else {
			if (bs.test(i)) {
				// 1->0 transition
				bs.reset(i);
				++r;
			}
		}
	}
	return r;
}


template <num_t N>
char bitset_parser_print(const bitset_t<N>& bs)
{
	assert(bs.any());
	switch (bs.count()) {
		case N:
			return '*';
		case 1:
			return num_print<N>(bitset_get_solved_opt<N>(bs).value());
		default:
			return '.';
	}
}


template <num_t N>
class sudoku_t {
public:
	using data_t = std::array<std::array<bitset_t<N>, N>, N>;
public:
	sudoku_t(std::istream&);
	void print(std::ostream&) const;
	void print_detailed(std::ostream&) const;
	void solve();
	void verify(); // TODO make const
//private: // FIXME public for tests
	data_t data_;
};

// TODO iterator_facade
template <num_t N>
struct iterator_base_t {
	bool is_valid() const { return mutable_idx_ < N; }
	void next() { ++mutable_idx_; }
public: // FIXME
	iterator_base_t(sudoku_t<N>::data_t& data, idx_t idx) : data_(data), fixed_idx_(idx) {}
	sudoku_t<N>::data_t& data_;
	idx_t const fixed_idx_;
	idx_t mutable_idx_ = 0;
};

template <num_t N>
struct iterator_over_row_t : public iterator_base_t<N> {
	iterator_over_row_t(sudoku_t<N>::data_t& data, idx_t row) : iterator_base_t<N>(data, row) {}
	bitset_t<N>& deref() { return iterator_base_t<N>::data_.at(iterator_base_t<N>::fixed_idx_).at(iterator_base_t<N>::mutable_idx_); } // row is fixed
	const bitset_t<N>& const_deref() const { return iterator_base_t<N>::data_.at(iterator_base_t<N>::fixed_idx_).at(iterator_base_t<N>::mutable_idx_); }
	void print_info(std::ostream& os) const { os << "iterator_row<" << iterator_base_t<N>::fixed_idx_ << ">"; }
};

template <num_t N>
struct iterator_over_column_t : public iterator_base_t<N> {
	iterator_over_column_t(sudoku_t<N>::data_t& data, idx_t col) : iterator_base_t<N>(data, col) {}
	bitset_t<N>& deref() { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
	const bitset_t<N>& const_deref() const { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
	void print_info(std::ostream& os) const { os << "iterator_column<" << iterator_base_t<N>::fixed_idx_ << ">"; }
};

template <num_t N>
struct iterator_over_sq_t {
	iterator_over_sq_t(sudoku_t<N>::data_t& data, idx_t row, idx_t col)
		: data_(data)
		, fixed_row_(row)
		, fixed_col_(col)
		, r_(row)
		, c_(col)
		{}
	bitset_t<N>& deref() { return data_.at(r_).at(c_); }
	const bitset_t<N>& const_deref() const { return data_.at(r_).at(c_); }
	bool is_valid() const
	{
		return (c_ < fixed_col_ + Ns) && (r_ < fixed_row_ + Ns);
	}
	void next()
	{
		++c_;
		if (c_ >= fixed_col_ + Ns) {
			c_ = fixed_col_;
			++r_;
		}
	}
	void print_info(std::ostream& os) const { os << "iterator_sq<" << fixed_row_ << "," << fixed_col_ << ">"; }
private:
	static constexpr auto Ns = sqrt_t<N>::value;
	sudoku_t<N>::data_t& data_;
	idx_t const fixed_row_;
	idx_t const fixed_col_;
	idx_t r_;
	idx_t c_;
};


// collect all "solved" values in the row, exclude them from each "unsolved" cell in the same row
template <num_t N, typename Iter>
size_t solve_exclusions_iterate(Iter const iter_begin)
{
	size_t progress_count = 0;

	bitset_t<N> solved_values(bitset_make_empty<N>());

	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		// collect all solved nums in the row
		auto const& cell = iter.const_deref();
		if (auto const n = bitset_get_solved_opt<N>(cell)) {
			solved_values.set(n.value());
		}
	}

	// exclude each "solved" num from all cells
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (!bitset_is_solved<N>(cell)) {
			progress_count += bitset_exclude_set<N>(cell, solved_values);
		}
	}

	return progress_count;
}


// Try to find a SINGLE cell which can contain the number. If found, solve number there.
template <num_t N, typename Iter>
size_t solve_emplace(Iter const iter_begin, num_t num)
{
	bitset_t<N> *eligible_cell = nullptr;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (auto const ns = bitset_get_solved_opt<N>(cell)) {
			// if "num" is already solved in some cell, exit early
			if (ns.value() == num) {
				return 0;
			}
			// skip other solved values
		}
		if (cell.test(num)) {
			if (eligible_cell) {
				// already have eligible cell, and current is the second
				return 0;
			}
			eligible_cell = &cell;
		}
	}

	if (!eligible_cell) {
		return 0;
	}

	assert(eligible_cell->count() >= 2);
	assert(eligible_cell->test(num));
	*eligible_cell = bitset_make_solved<N>(num);
	return 1; // TODO return number of changed bits
}


template <num_t N, typename Iter>
size_t solve_clusters_exact_match(Iter const iter_begin, const bitset_t<N>& test_set)
{
	assert(test_set.count() >= 2);

	size_t count_exact_match = 0;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto const& cell = iter.const_deref();
		if (cell == test_set) {
			++count_exact_match;
		}
	}
	assert(count_exact_match <= test_set.count());

	if (count_exact_match != test_set.count()) {
		return 0;
	}

	size_t count_exact_match_check = 0;
	size_t count_solved = 0;
	size_t count_cell_upd = 0;
	size_t progress = 0;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (cell == test_set) {
			++count_exact_match_check;
		} else if (bitset_is_solved<N>(cell)) {
			++count_solved;
		} else {
			progress += bitset_exclude_set<N>(cell, test_set);
			++count_cell_upd;
		}
	}
	assert(count_exact_match_check == count_exact_match);
	assert(count_exact_match_check + count_solved + count_cell_upd == N);
	return progress;
}


template <num_t N>
sudoku_t<N>::sudoku_t(std::istream& is)
{
	// TODO more robust
	auto iter = std::istream_iterator<char>(is);
	for (size_t row = 0; row < N; ++row) {
		for (size_t col = 0; col < N; ++col) {
			char const c = *iter;
			++iter;
			if (c == '*') {
				data_.at(row).at(col) = bitset_make_full<N>();
			} else {
				data_.at(row).at(col) = bitset_make_solved<N>(num_parse<N>(c));
			}
		}
	}
}


template <num_t N>
void sudoku_t<N>::print(std::ostream& os) const
{
	constexpr auto Ns = sqrt_t<N>::value;

	for (size_t row = 0; row < N; ++row) {
		for (size_t col = 0; col < N; ++col) {
			os << bitset_parser_print<N>(data_.at(row).at(col));
			if (col < N-1) {
				os << ' ';
				if ((col % Ns) == (Ns - 1)) {
					os << ' ';
				}
			}
		}
		os << std::endl;
		if ((row % Ns) == (Ns - 1)) {
			os << std::endl;
		}
	}
}


template <num_t N>
void sudoku_t<N>::print_detailed(std::ostream& os) const
{
	constexpr auto Ns = sqrt_t<N>::value;
	std::array<std::array<char, N*Ns>, N*Ns> data_sets; // TODO without additional storage
	for(idx_t r=0; r<N; ++r) {
		for(idx_t c=0; c<N; ++c) {
			const auto& cell = data_.at(r).at(c);
			for (num_t n=0; n<N; ++n) {
				data_sets.at(r*Ns + n/Ns).at(c*Ns + n%Ns) = (cell.test(n) ? num_print<N>(n) : ' ');
			}
		}
	}
	for (size_t r=0; r<data_sets.size(); ++r) {
		const auto& row = data_sets.at(r);
		os << " ";
		for (size_t c=0; c<row.size(); ++c) {
			os << row.at(c);
			if (c%N == (N-1)) {
				if (c < N*Ns-1) {
					os << "  |||  ";
				}
			} else if (c%Ns == Ns-1) {
				os << " | ";
			}
		}
		os << "\n";
		constexpr size_t div_len = (N==4 ? 23 : N==9 ? 61 : 123);
		if (r%N == N-1) {
			if (r < N*Ns-1) {
				os << "\n";
				for (size_t i=0; i<div_len; ++i) os << "=";
				os << "\n\n";
			}
		} else if (r%Ns == Ns-1) {
			for (size_t i=0; i<div_len; ++i) os << "-";
			os << "\n";
		}
	}
}


template <num_t N>
void sudoku_t<N>::solve()
{
	constexpr unsigned long long test_cluster_last = (1 << N) - 2;
	static_assert(test_cluster_last == ((1 << (N-1)) - 1) << 1); // N-1 1's then 0
	static_assert(bitset_t<N>(test_cluster_last).count() == N-1);
	static_assert(bitset_t<N>(test_cluster_last).test(0) == 0);

	size_t updates_total = 0;
	for (size_t iter = 1; ; ++iter) {
		size_t updates_current = 0;

		// iterate over over rows & columns
		for (idx_t idx = 0; idx < N; ++idx) {
			auto const iter_row_begin = iterator_over_row_t<N>(data_, idx);
			auto const iter_column_begin = iterator_over_column_t<N>(data_, idx);

			// simple exclusions
			updates_current += solve_exclusions_iterate<N>(iter_row_begin);
			updates_current += solve_exclusions_iterate<N>(iter_column_begin);

			// emplace
			for (num_t n=0; n<N; ++n) {
				updates_current += solve_emplace<N>(iter_row_begin, n);
				updates_current += solve_emplace<N>(iter_column_begin, n);
			}

			// exact clusters
			// iterate over all possible sets of N bits
			static_assert(N <= 16); // conservative, make sure std::bitset doesn't overflow
			for (unsigned long long n = 1; n <= test_cluster_last; ++n) {
				bitset_t<N> const test_cluster(n);
				if (test_cluster.count() >= 2) {
					updates_current += solve_clusters_exact_match<N>(iter_row_begin, test_cluster);
					updates_current += solve_clusters_exact_match<N>(iter_column_begin, test_cluster);
				}
			}
		}

		// iterate over 3*3 (Ns*Ns) squares
		constexpr auto Ns = sqrt_t<N>::value;
		for (idx_t r = 0; r < N; r += Ns) {
			for (idx_t c = 0; c < N; c += Ns) {
				auto const iter_sq_begin = iterator_over_sq_t<N>(data_, r, c);

				// simple exclusions
				updates_current += solve_exclusions_iterate<N>(iter_sq_begin);

				// emplace
				for (num_t n=0; n<N; ++n) {
					updates_current += solve_emplace<N>(iter_sq_begin, n);
				}

				// exact clusters
				for (unsigned long long n = 1; n <= test_cluster_last; ++n) {
					bitset_t<N> const test_cluster(n);
					if (test_cluster.count() >= 2) {
						updates_current += solve_clusters_exact_match<N>(iter_sq_begin, test_cluster);
					}
				}
			}
		}

		std::cout << "Iteration=" << iter << " updates:" << updates_total << "+" << updates_current << "\n";
		updates_total += updates_current;
		if (updates_current == 0 /* || iter>20*/) {
			std::cout << "No progress, terminating\n";
			return;
		}
	}
}


template <num_t N, typename Iter>
bool verify_impl(Iter&& iter_begin)
{
	auto nums = bitset_make_empty<N>();
	for (auto iter = iter_begin; iter.is_valid(); iter.next()){
		auto const ns = bitset_get_solved_opt<N>(iter.const_deref());
		if (!ns) {
			std::cout << "UNSOLVED\n";
			return false;
		}
		nums.set(ns.value());
	}
	if (nums.all()) {
		return true;
	} else {
		std::cout << "- verify_set:" << nums.to_string() << " iterator:";
		iter_begin.print_info(std::cout);
		std::cout << "\n";
		return false;
	}
}


template <num_t N>
void sudoku_t<N>::verify()
{
	bool verified = true;
	for (idx_t idx=0; idx<N; ++idx) {
		verified = verify_impl<N>(iterator_over_row_t<N>(data_, idx)) && verified;
		verified = verify_impl<N>(iterator_over_column_t<N>(data_, idx)) && verified;
	}

	constexpr auto Ns = sqrt_t<N>::value;
	for (idx_t r = 0; r < N; r += Ns) {
		for (idx_t c = 0; c < N; c += Ns) {
			verified = verify_impl<N>(iterator_over_sq_t<N>(data_, r, c)) && verified;
		}
	}
	if (verified) {
		std::cout << "VERIFY_CORRECT\n";
	} else {
		std::cout << "!!! VERIFY_INCORRECT\n";
	}
}

#endif

