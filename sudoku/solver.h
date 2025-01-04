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
template<> num_t num_parse<9>(char c)
{
	if (c >= '1' && c <= '9')
		return c - '0' - 1;
	throw std::runtime_error("Num[1..9] parsing error");
}
template<> num_t num_parse<4>(char c)
{
	if (c >= '1' && c <= '4')
		return c - '0' - 1;
	throw std::runtime_error("Num[1..4] parsing error");
}


template <num_t N> struct sqrt_t;
template<> struct sqrt_t<9> { static constexpr num_t value = 3; };
template<> struct sqrt_t<4> { static constexpr num_t value = 2; };


template <num_t N>
class numset_t {
public:
	using bitset_t = std::bitset<N>;

public:
	static constexpr bitset_t make_bitset_full()
	{
		bitset_t r;
		r.set();
		return r;
	}

	static constexpr bitset_t make_bitset_empty()
	{
		return bitset_t();
	}

public:
	numset_t() : bits_(make_bitset_full()) {}

	void assign(num_t num)
	{
		// TODO return number of changed bits
		bits_.reset();
		bits_.set(num);
	}

	size_t assign_set(const bitset_t& b)
	{
		// return number of 1->0 transitions
		// TODO bit ops
		size_t r = 0;
		for (idx_t i=0; i<N; ++i) {
			if (b.test(i)) {
				assert(bits_.test(i)); // cannot "add" to set
			} else {
				if (bits_.test(i)) {
					// 1->0 transition
					bits_.reset(i);
					++r;
				}
			}
		}
		return r;
	}

	static bool eq(const numset_t& a, const numset_t& b)
	{
		return a.bits_ == b.bits_;
	}

	size_t count() const { return bits_.count(); }

	bool has(num_t n) const
	{
		return bits_.test(n);
	}

	bool try_exclude(num_t n)
	{
		if (bits_.test(n)) {
			bits_.reset(n);
			return true;
		}
		return false;
	}

	// exclude "solved_nums" from cell
	size_t try_exclude_set(bitset_t b)
	{
		size_t r = 0;
		for (num_t n=0; n<N; ++n) {
			if (b.test(n) && try_exclude(n)) {
				++r;
			}
		}
		assert(bits_.count() >= 1);
		return r;
	}

	bool contains_all(const bitset_t& b) const
	{
		// TODO bit operation
		for (num_t n=0; n<N; ++n) {
			if (b.test(n) && !bits_.test(n)) {
				return false;
			}
		}
		return true;
	}

	std::optional<num_t> is_solved() const
	{
		// TODO better implementation
		assert(bits_.count() >= 1);
		ssize_t idx = -1;
		for (size_t i = 0; i<N; ++i) {
			if (bits_.test(i)) {
				if (idx != -1) {
					return std::nullopt;
				}
				idx = i;
			}
		}
		assert(idx != -1);
		return idx;
	}

	char print() const
	{
		auto const ns = is_solved();
		if (ns.has_value()) {
			return ns.value() + '0' + 1;
		}
		if (bits_.count() == N) {
			return '*';
		}
		return '.';
	}

	// for tests
	std::string to_string() const { return bits_.to_string(); }
	auto to_ulong() const { return bits_.to_ulong(); }

public: // FIXME
	bitset_t bits_;
};



template <num_t N>
class sudoku_t {
public:
	using data_t = std::array<std::array<numset_t<N>, N>, N>;
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
	numset_t<N>& deref() { return iterator_base_t<N>::data_.at(iterator_base_t<N>::fixed_idx_).at(iterator_base_t<N>::mutable_idx_); } // row is fixed
	const numset_t<N>& const_deref() const { return iterator_base_t<N>::data_.at(iterator_base_t<N>::fixed_idx_).at(iterator_base_t<N>::mutable_idx_); }
	void print_info(std::ostream& os) const { os << "iterator_row<" << iterator_base_t<N>::fixed_idx_ << ">"; }
};

template <num_t N>
struct iterator_over_column_t : public iterator_base_t<N> {
	iterator_over_column_t(sudoku_t<N>::data_t& data, idx_t col) : iterator_base_t<N>(data, col) {}
	numset_t<N>& deref() { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
	const numset_t<N>& const_deref() const { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
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
	numset_t<N>& deref() { return data_.at(r_).at(c_); }
	const numset_t<N>& const_deref() const { return data_.at(r_).at(c_); }
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

	typename numset_t<N>::bitset_t solved_values(numset_t<N>::make_bitset_empty());

	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		// collect all solved nums in the row
		auto const& cell = iter.const_deref();
		if (auto const n = cell.is_solved()) {
			solved_values.set(n.value());
		}
	}

	// exclude each "solved" num from all cells
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (!cell.is_solved()) {
			progress_count += cell.try_exclude_set(solved_values);
		}
	}

	return progress_count;
}


// Try to find a SINGLE cell which can contain the number. If found, solve number there.
template <num_t N, typename Iter>
size_t solve_emplace(Iter const iter_begin, num_t num)
{
	numset_t<N> *eligible_cell = nullptr;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (auto const ns = cell.is_solved()) {
			// if "num" is already solved in some cell, exit early
			if (ns.value() == num) {
				return 0;
			}
			// skip other solved values
		}
		if (cell.bits_.test(num)) {
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

	assert(eligible_cell->bits_.count() >= 2);
	assert(eligible_cell->bits_.test(num));
	eligible_cell->assign(num);
	return 1; // TODO return number of changed bits
}


template <num_t N, typename Iter>
size_t solve_clusters_exact_match(Iter const iter_begin, const typename numset_t<N>::bitset_t& test_set)
{
	assert(test_set.count() >= 2);

	size_t count_exact_match = 0;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		auto const& cell = iter.const_deref();
		if (cell.bits_ == test_set) {
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
		if (cell.bits_ == test_set) {
			++count_exact_match_check;
		} else if (cell.is_solved()) {
			++count_solved;
		} else {
			progress += cell.try_exclude_set(test_set);
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
				// do nothing, numset_t is "full" by default
			} else {
				data_.at(row).at(col).assign(num_parse<N>(c));
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
			os << data_.at(row).at(col).print();
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
				data_sets.at(r*Ns + n/Ns).at(c*Ns + n%Ns) = (cell.has(n) ? '0'+n+1 : ' ');
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
		constexpr size_t div_len = (N==9 ? 61 : 23);
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
	static_assert(typename numset_t<N>::bitset_t(test_cluster_last).count() == N-1);
	static_assert(typename numset_t<N>::bitset_t(test_cluster_last).test(0) == 0);

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
			static_assert(N < 16); // conservative, make sure std::bitset doesn't overflow
			for (unsigned long long n = 1; n <= test_cluster_last; ++n) {
				typename numset_t<N>::bitset_t const test_cluster(n);
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
					typename numset_t<N>::bitset_t const test_cluster(n);
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
	typename numset_t<N>::bitset_t nums = numset_t<N>::make_bitset_empty();
	for (auto iter = iter_begin; iter.is_valid(); iter.next()){
		auto const ns = iter.const_deref().is_solved();
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

