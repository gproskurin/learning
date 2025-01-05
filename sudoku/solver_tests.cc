#define BOOST_TEST_MODULE solver_tests
#include <boost/test/included/unit_test.hpp>

#include "solver.h"

#include <iostream>
#include <sstream>

BOOST_AUTO_TEST_CASE(test_parse_print)
{
	constexpr std::array<std::pair<num_t, char>, 18> prn{ {
		{0, '1'},
		{1, '2'},
		{2, '3'},
		{3, '4'},
		//
		{4, '5'},
		{5, '6'},
		{6, '7'},
		{7, '8'},
		{8, '9'},
		//
		{9, 'a'},
		{10, 'b'},
		{11, 'c'},
		{12, 'd'},
		{13, 'e'},
		{14, 'f'},
		{15, 'g'},
		//
		{16, 'h'},
		{24, 'p'}
	} };
	for (num_t n = 0; n<prn.size(); ++n) {
		auto const num_internal = prn.at(n).first;
		auto const num_printed = prn.at(n).second;
		if (n < 4) {
			BOOST_TEST(num_parse<4>(num_printed) == num_internal);
			BOOST_TEST(num_print<4>(num_internal) == num_printed);
		}
		if (n < 9) {
			BOOST_TEST(num_parse<9>(num_printed) == num_internal);
			BOOST_TEST(num_print<9>(num_internal) == num_printed);
		}
		if (n < 16) {
			BOOST_TEST(num_parse<16>(num_printed) == num_internal);
			BOOST_TEST(num_print<16>(num_internal) == num_printed);
		}
		if (n < 25) {
			BOOST_TEST(num_parse<25>(num_printed) == num_internal);
			BOOST_TEST(num_print<25>(num_internal) == num_printed);
		}
	}
}

BOOST_AUTO_TEST_CASE(test_bitset)
{
	constexpr num_t N = 4;
	{
		auto const bs = bitset_make_empty<N>();
		BOOST_TEST(bs.count() == 0);
		BOOST_TEST(bs.none());
		BOOST_TEST(bs.to_ulong() == 0);
	}
	{
		auto const bs = bitset_make_full<N>();
		BOOST_TEST(bs.count() == 4);
		BOOST_TEST(bs.all());
		BOOST_TEST(bs.to_ulong() == 0b1111);
	}
	{
		auto const bs = bitset_make_solved<N>(0);
		BOOST_TEST(bs.count() == 1);
		BOOST_TEST(!bs.all());
		BOOST_TEST(!bs.none());
		BOOST_TEST(bs.to_ulong() == 0b0001);
	}
	{
		auto const bs = bitset_make_solved<N>(1);
		BOOST_TEST(bs.count() == 1);
		BOOST_TEST(!bs.all());
		BOOST_TEST(!bs.none());
		BOOST_TEST(bs.to_ulong() == 0b0010);
	}
	{
		auto const bs = bitset_make_solved<N>(3);
		BOOST_TEST(bs.count() == 1);
		BOOST_TEST(!bs.all());
		BOOST_TEST(!bs.none());
		BOOST_TEST(bs.to_ulong() == 0b1000);
	}
}

BOOST_AUTO_TEST_CASE(test_bitset_exclude_set)
{
	constexpr num_t N = 4;

	auto bs = bitset_make_full<N>();
	BOOST_TEST(bs.to_ulong() == 0b1111);

	{
		bitset_t<N> const bs1(0b0011ULL);
		BOOST_TEST(bs1.to_ulong() == 0b0011);

		BOOST_TEST(bitset_exclude_set<N>(bs, bs1) == true);
		BOOST_TEST(bs.to_ulong() == 0b1100);
		BOOST_TEST(bitset_exclude_set<N>(bs, bs1) == false);
		BOOST_TEST(bs.to_ulong() == 0b1100);
	}

	{
		BOOST_TEST(bs.to_ulong() == 0b1100);

		bitset_t<N> const bs2(0b0110ULL);
		BOOST_TEST(bs2.to_ulong() == 0b0110);
		BOOST_TEST(bitset_exclude_set<N>(bs, bs2) == true);
		BOOST_TEST(bs.to_ulong() == 0b1000);
		BOOST_TEST(bitset_exclude_set<N>(bs, bs2) == false);
		BOOST_TEST(bs.to_ulong() == 0b1000);
	}
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_row)
{
	constexpr num_t N = 4;
	std::istringstream is(
		"4*21\n"
		"****\n"
		"****\n"
		"****\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(0).at(1)) == '3');
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_column)
{
	constexpr num_t N = 4;
	std::istringstream is(
		"4***\n"
		"****\n"
		"2***\n"
		"1***\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(1).at(0)) == '3');
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_sq)
{
	constexpr num_t N = 4;
	std::istringstream is(
		"4***\n"
		"21**\n"
		"****\n"
		"****\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(0).at(1)) == '3');
}

BOOST_AUTO_TEST_CASE(test_emplace_sq)
{
	constexpr num_t N = 4;
	std::istringstream is(
		"*2**\n" /* 1 can be placed only in the first column of this row */
		"***1\n"
		"****\n"
		"****\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(0).at(0)) == '1');
}

BOOST_AUTO_TEST_CASE(test_emplace_column)
{
	constexpr num_t N = 9;
	std::istringstream is(
		"*********\n"
		"********1\n"
		"*********\n"
		"*********\n"
		"*******2*\n"
		"*******3*\n"
		"******1**\n"
		"*********\n"
		"*********\n"
	);
	sudoku_t<9> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(3).at(7)) == '1');
}

BOOST_AUTO_TEST_CASE(test_cluster_column_size_2)
{
	constexpr num_t N = 9;
	std::istringstream is(
		// first column (2 elements) miss {8,9}, this is a cluster of size 2
		"*********\n"
		"*********\n"
		"1********\n"
		"2********\n"
		"3********\n"
		"4********\n"
		"5********\n"
		"6********\n"
		"7********\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	// can contain {8,9} only
	BOOST_TEST(s.data_.at(0).at(0).to_ulong() == 0b110000000);
	BOOST_TEST(s.data_.at(1).at(0).to_ulong() == 0b110000000);

	// {8,9} is excluded from top-left square
	BOOST_TEST(s.data_.at(0).at(1).to_ulong() == 0b001111110);
	BOOST_TEST(s.data_.at(1).at(1).to_ulong() == 0b001111110);
	BOOST_TEST(s.data_.at(2).at(1).to_ulong() == 0b001111110);
	BOOST_TEST(s.data_.at(0).at(2).to_ulong() == 0b001111110);
	BOOST_TEST(s.data_.at(1).at(2).to_ulong() == 0b001111110);
	BOOST_TEST(s.data_.at(2).at(2).to_ulong() == 0b001111110);
}

BOOST_AUTO_TEST_CASE(test_solve_16)
{
	constexpr num_t N = 16;
	std::istringstream is(
		"*23456789abcdefg\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
		"****************\n"
	);
	sudoku_t<N> s(is);
	s.solve();
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(0).at(0)) == '1');
}

BOOST_AUTO_TEST_CASE(test_solve_25)
{
	constexpr num_t N = 25;
	std::istringstream is(
		"*23456789abcdefghijklmnop\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
		"*************************\n"
	);
	sudoku_t<N> s(is);
	//s.solve(); // too long
	solve_exclusions<N>(iterator_over_row_t<N>(s.data_, 0));
	BOOST_TEST(bitset_parser_print<N>(s.data_.at(0).at(0)) == '1');
}
const std::array<std::string, 3> sudoku_samples{
	// easy 299
	"**8*362**\n"
	"*****29**\n"
	"*2*718*5*\n"
	"**92*4*35\n"
	"*3**7***9\n"
	"******6**\n"
	"******84*\n"
	"**51*937*\n"
	"64***75*1\n",

	// moderate 108237
	"*********\n"
	"*65***741\n"
	"3****8***\n"
	"**46*****\n"
	"**67**42*\n"
	"**2*5**36\n"
	"******15*\n"
	"********3\n"
	"591*7****\n",

	// moderate 119032
	"1*****5**\n"
	"*57******\n"
	"***3*8**4\n"
	"*45******\n"
	"*********\n"
	"**6**7*89\n"
	"****9*842\n"
	"*****2*9*\n"
	"69**3***1\n"
};

BOOST_AUTO_TEST_CASE(test_solve_full_samples)
{
	constexpr num_t N = 9;
	for (const auto& str : sudoku_samples) {
		std::istringstream is(str);
		sudoku_t<N> s(is);
		s.solve();
		// test it is solved
		for (const auto& row : s.data_) {
			for (const auto& cell : row) {
				BOOST_TEST(bitset_is_solved<N>(cell));
			}
		}
	}
}

