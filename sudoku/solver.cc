//#include <boost/iterator/iterator_facade.hpp> // TODO

#include <array>
#include <bitset>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <assert.h>


constexpr size_t N = 9;
using num_t = char;
using idx_t = char;

using bitset_t = std::bitset<9>;

constexpr bitset_t make_bitset_full()
{
	bitset_t r;
	r.set();
	return r;
}

constexpr bitset_t make_bitset_empty()
{
	return bitset_t();
}

num_t num_parse(char c)
{
	if (c >= '1' && c <= '9')
		return c - '0' - 1;
	throw std::runtime_error("Num parsing error");
}

class numset_t {
	bitset_t bits_;
public:
	numset_t() : bits_(make_bitset_full()) {}

	void assign(num_t num)
	{
		bits_.reset();
		bits_.set(num);
	}

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
};


using sudoku_data_t = std::array<std::array<numset_t, N>, N>;

class sudoku_t {
	sudoku_data_t data_;
public:
	sudoku_t(std::istream&);
	void print(std::ostream&) const;
	void solve();
};

// TODO iterator_facade
struct iterator_base_t {
	bool is_valid() const { return mutable_idx_ < N; }
	void next() { ++mutable_idx_; }
protected:
	iterator_base_t(sudoku_data_t& data, idx_t idx) : data_(data), fixed_idx_(idx) {}
	sudoku_data_t& data_;
	idx_t const fixed_idx_;
	idx_t mutable_idx_ = 0;
};

struct iterator_over_row_t : public iterator_base_t {
	iterator_over_row_t(sudoku_data_t& data, idx_t row) : iterator_base_t(data, row) {}
	numset_t& deref() { return data_.at(fixed_idx_).at(mutable_idx_); } // row is fixed
	const numset_t& const_deref() const { return data_.at(fixed_idx_).at(mutable_idx_); }
};

struct iterator_over_column_t : public iterator_base_t {
	iterator_over_column_t(sudoku_data_t& data, idx_t col) : iterator_base_t(data, col) {}
	numset_t& deref() { return data_.at(mutable_idx_).at(fixed_idx_); }
	const numset_t& const_deref() const { return data_.at(mutable_idx_).at(fixed_idx_); }
};

struct iterator_over_sq_t {
	iterator_over_sq_t(sudoku_data_t& data, idx_t row, idx_t col)
		: data_(data)
		, fixed_row_(row)
		, fixed_col_(col)
		, r_(row)
		, c_(col)
		{}
	numset_t& deref() { return data_.at(r_).at(c_); }
	const numset_t& const_deref() const { return data_.at(r_).at(c_); }
	bool is_valid() const { return (c_ < fixed_col_ + 3) && (r_ < fixed_row_ + 3); }
	void next() {
		++c_;
		if (c_ >= fixed_col_ + 3) {
			c_ = fixed_col_;
			++r_;
		}
	}
private:
	sudoku_data_t& data_;
	idx_t const fixed_row_;
	idx_t const fixed_col_;
	idx_t r_;
	idx_t c_;
};


sudoku_t::sudoku_t(std::istream& is)
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
				data_.at(row).at(col).assign(num_parse(c));
			}
		}
	}
}


void sudoku_t::print(std::ostream& os) const
{
	for (size_t row = 0; row < N; ++row) {
		for (size_t col = 0; col < N; ++col) {
			os << data_.at(row).at(col).print();
			if (col < N-1) {
				os << ' ';
				if ((col % 3) == 2) {
					os << ' ';
				}
			}
		}
		os << std::endl;
		if ((row % 3) == 2) {
			os << std::endl;
		}
	}
}


// collect all "solved" values in the row, exclude them from each "unsolved" cell in the same row
template <typename IterMaker>
bool solve_exclusions_iterate(IterMaker&& iter_maker)
{
	bool progress = false;

	bitset_t solved_values(make_bitset_empty());

	for (auto iter = iter_maker(); iter.is_valid(); iter.next()) {
		// collect all solved nums in the row
		auto const& cell = iter.const_deref();
		if (auto const n = cell.is_solved()) {
			solved_values.set(n.value());
		}
	}

	// exclude each "solved" num from all cells
	for (auto iter = iter_maker(); iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (!cell.is_solved()) {
			// exclude "solved_nums" from cell
			// TODO bits operations instead of loop
			for (num_t n = 0; n < N; ++n) {
				if (solved_values.test(n) && cell.try_exclude(n)) {
					progress = true;
				}
			}
		}
	}

	return progress;
}


template <typename Iter>
bool solve_emplace(num_t num, Iter iter)
{
	// try to emplace number to current row
	// process current row, count number of eligible places where the number could be emplaced
	// if this count is 1, emplace it (solve)
	size_t eligible_cells_count = 0;
	numset_t* eligible_cell_ptr = nullptr;
	for (; iter.is_valid(); iter.next()) {
		auto& cell = iter.deref();
		if (auto const sv = cell.is_solved()) {
			if (sv.value() == num) {
				// this num is already "solved" in this iterator
				return false;
			}
			// ignore other solved values
		} else {
			if (cell.has(num)) {
				++eligible_cells_count;
				eligible_cell_ptr = &cell;
			}
		}
	}

	if (eligible_cells_count == 1) {
		// TODO asserts
		//assert(eligible_cell_ptr);
		//assert(eligible_cell_ptr->count() >= 1);
		//assert(eligible_cell_ptr->test(num));
		eligible_cell_ptr->assign(num);
		return true;
	}

	return false;
}


void sudoku_t::solve()
{
	struct iterator_over_row_maker_t {
		iterator_over_row_maker_t(sudoku_data_t& data, idx_t idx) : data_(data), idx_(idx) {}
		iterator_over_row_t operator()() const { return iterator_over_row_t(data_, idx_); }
	private:
		sudoku_data_t& data_;
		idx_t const idx_;
	};

	struct iterator_over_column_maker_t {
		iterator_over_column_maker_t(sudoku_data_t& data, idx_t idx) : data_(data), idx_(idx) {}
		iterator_over_column_t operator()() const { return iterator_over_column_t(data_, idx_); }
	private:
		sudoku_data_t& data_;
		idx_t const idx_;
	};

	struct iterator_over_sq_maker_t {
		iterator_over_sq_maker_t(sudoku_data_t& data, idx_t row, idx_t col) : data_(data), row_(row), col_(col) {}
		iterator_over_sq_t operator()() const { return iterator_over_sq_t(data_, row_, col_); }
	private:
		sudoku_data_t& data_;
		idx_t const row_;
		idx_t const col_;
	};

	for (size_t iter = 1; ; ++iter) {
		bool progress = false;

		// iterate over over rows & columns
		for (idx_t idx = 0; idx < N; ++idx) {
			// simple exclusions
			progress = solve_exclusions_iterate(iterator_over_row_maker_t(data_, idx)) || progress;
			progress = solve_exclusions_iterate(iterator_over_column_maker_t(data_, idx)) || progress;

			// emplace
			for (num_t num = 0; num < N; ++num) {
				progress = solve_emplace(num, iterator_over_row_t(data_, idx)) || progress;
				progress = solve_emplace(num, iterator_over_column_t(data_, idx)) || progress;
			}
		}

		// iterate over 3*3 squares
		for (idx_t r = 0; r < N; r += 3) {
			for (idx_t c = 0; c < N; c += 3) {
				// simple exclusions
				progress = solve_exclusions_iterate(iterator_over_sq_maker_t(data_, r,c)) || progress;

				// emplace
				for (num_t num = 0; num < N; ++num) {
					progress = solve_emplace(num, iterator_over_sq_t(data_, r, c)) || progress;
				}
			}
		}

		if (!progress) {
			std::cout << "No progress, iterations: " << iter << std::endl;
			return;
		}
	}
}


int main()
{
try {
	sudoku_t s(std::cin);
	std::cout << "INPUT\n";
	s.print(std::cout);
	std::cout << std::endl;

	s.solve();
	std::cout << "OUTPUT\n";
	s.print(std::cout);
	std::cout << std::endl;

} catch (const std::exception& e) {
	std::cerr << "Exception: " << e.what() << "\n";
	return 1;
}
}

