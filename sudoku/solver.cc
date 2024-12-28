#include <array>
#include <bitset>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <assert.h>


constexpr size_t N = 9;
using num_t = char;


num_t num_parse(char c)
{
	if (c >= '1' && c <= '9')
		return c - '0' - 1;
	throw std::runtime_error("Num parsing error");
}

class numset_t {
	using bitset_t = std::bitset<9>;
	bitset_t bits_;
public:
	numset_t()
	{
		bits_.set();
	}

	void assign(num_t num)
	{
		bits_.reset();
		bits_.set(num);
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

class sudoku_t {
	std::array<
		std::array<numset_t, N>,
		N
	> data_;
public:
	sudoku_t(std::istream& is)
	{
		// TODO more robust
		auto iter = std::istream_iterator<char>(is);
		for (size_t row = 0; row < N; ++row) {
			for (size_t col = 0; col < N; ++col) {
				char const c = *iter;
				++iter;
				if (c == '*') {
					// do nothing, numset_t is "full" by default
				} else if (c >= '1' && c <= '9') {
					data_.at(row).at(col).assign(num_parse(c));
				} else {
					throw std::runtime_error("Unexpected symbol");
				}
			}
		}
	}

	void print(std::ostream& os) const
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

	bool solve_once(size_t row, size_t col)
	{
		auto& xset = data_.at(row).at(col);
		bool progress = false;

		// iterate over the row specified
		for (size_t c = 0; c < N; ++c) {
			if (c != col) {
				if (auto const cur = data_.at(row).at(c).is_solved()) {
					if (xset.try_exclude(cur.value())) {
						progress = true;
					}
				}
			}
		}

		// iterate over the column specified
		for (size_t r = 0; r < N; ++r) {
			if (r != row) {
				if (auto const cur = data_.at(r).at(col).is_solved()) {
					if (xset.try_exclude(cur.value())) {
						progress = true;
					}
				}
			}
		}

		// iterate over 3*3 square
		auto const row_begin = row - (row % 3);
		auto const row_end = row_begin + 3;
		auto const col_begin = col - (col % 3);
		auto const col_end = col_begin + 3;
		for (size_t r = row_begin; r < row_end; ++r)
			for (size_t c = col_begin; c < col_end; ++c) {
				if ((r != row) || (c != col)) {
					if (auto const cur = data_.at(r).at(c).is_solved()) {
						if (xset.try_exclude(cur.value())) {
							progress = true;
						}
					}
				}
		}

		//std::cout << "SOLVING: [" << row << "," << col << "]: " << progress << "\n";
		return progress;

	}

	void solve()
	{
		for (size_t iter = 1; ; ++iter) {
			bool progress = false;
			for (size_t row = 0; row < N; ++row) {
				for (size_t col = 0; col < N; ++col) {
					progress = solve_once(row, col) || progress;
				}
			}
			if (!progress) {
				std::cout << "No progress, iterations: " << iter << std::endl;
				return;
			}
		}
	}
};


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

