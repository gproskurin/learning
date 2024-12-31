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
		bits_.reset();
		bits_.set(num);
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

	std::optional<num_t> is_solved() const
	{
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

	std::string to_string() const { return bits_.to_string(); } // used for tests

private:
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
//private: // FIXME public for tests
	data_t data_;
};

// TODO iterator_facade
template <num_t N>
struct iterator_base_t {
	bool is_valid() const { return mutable_idx_ < N; }
	void next() { ++mutable_idx_; }
protected:
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
};

template <num_t N>
struct iterator_over_column_t : public iterator_base_t<N> {
	iterator_over_column_t(sudoku_t<N>::data_t& data, idx_t col) : iterator_base_t<N>(data, col) {}
	numset_t<N>& deref() { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
	const numset_t<N>& const_deref() const { return iterator_base_t<N>::data_.at(iterator_base_t<N>::mutable_idx_).at(iterator_base_t<N>::fixed_idx_); }
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
			// exclude "solved_nums" from cell
			// TODO bits operations instead of loop
			for (num_t n = 0; n < N; ++n) {
				if (solved_values.test(n) && cell.try_exclude(n)) {
					++progress_count;
				}
			}
		}
	}

	return progress_count;
}


template <num_t N, typename Iter>
size_t solve_clusters(Iter const iter_begin)
{
	size_t count_unsolved = 0;
	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		if (!iter.const_deref().is_solved()) {
			++count_unsolved;
		}
	}

	for (auto iter = iter_begin; iter.is_valid(); iter.next()) {
		if (iter.const_deref().is_solved()) {
			continue;
		}

		auto const iter_cluster_begin = iter;
		const auto& cell_cluster_begin = iter.const_deref();
		size_t count = 0;
		for (auto iter2 = iter_cluster_begin; iter2.is_valid(); iter2.next()) {
			if (iter2.const_deref().is_solved()) {
				continue;
			}
			if (numset_t<N>::eq(iter2.const_deref(), cell_cluster_begin)) {
				++count;
			}
		}
		assert(count >= 1);
		assert(count <= count_unsolved);

		if ((count < count_unsolved) && (count == cell_cluster_begin.count())) {
			size_t count_updates = 0;
			for (auto iter_update = iter_begin; iter_update.is_valid(); iter_update.next()) {
				auto& cell_update = iter_update.deref();
				if (!cell_update.is_solved()) {
					if (!numset_t<N>::eq(iter_update.const_deref(), cell_cluster_begin)) {
						for(num_t n=0; n<N; ++n) {
							if (cell_cluster_begin.has(n)) {
								if (cell_update.try_exclude(n)) {
									++count_updates;
								}
							}
						}
					}
				}
			}
			//std::cout << "FOUND_CLUSTER: size=" << count << " cells_updated=" << count_cells_updated << "\n";
			if (count_updates > 0) {
				// made some progress and updated some cells
				return count_updates;
			}
		}
	}
	return 0;
}

#endif


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
	size_t updates_total = 0;
	for (size_t iter = 1; ; ++iter) {
		size_t updates_current = 0;

		// iterate over over rows & columns
		for (idx_t idx = 0; idx < N; ++idx) {
			// simple exclusions
			updates_current += solve_exclusions_iterate<N>(iterator_over_row_t<N>(data_, idx));
			updates_current += solve_exclusions_iterate<N>(iterator_over_column_t<N>(data_, idx));

			// clusters
			updates_current += solve_clusters<N>(iterator_over_row_t<N>(data_, idx));
			updates_current += solve_clusters<N>(iterator_over_column_t<N>(data_, idx));
		}

		// iterate over 3*3 (Ns*Ns) squares
		constexpr auto Ns = sqrt_t<N>::value;
		for (idx_t r = 0; r < N; r += Ns) {
			for (idx_t c = 0; c < N; c += Ns) {
				// simple exclusions
				updates_current += solve_exclusions_iterate<N>(iterator_over_sq_t<N>(data_, r, c));

				// clusters
				updates_current += solve_clusters<N>(iterator_over_sq_t<N>(data_, r, c));
			}
		}

		std::cout << "Iteration=" << iter << " updates:" << updates_total << "+" << updates_current << "\n";
		updates_total += updates_current;
		if (updates_current == 0) {
			std::cout << "No progress, terminating\n";
			return;
		}
	}
}

