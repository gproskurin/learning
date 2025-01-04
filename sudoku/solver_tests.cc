#define BOOST_TEST_MODULE solver_tests
#include <boost/test/included/unit_test.hpp>

#include "solver.h"

#include <iostream>
#include <sstream>


BOOST_AUTO_TEST_CASE(test_bitset_exclude_1)
{
	numset_t<4> ns;
	BOOST_TEST(ns.to_ulong() == 0b1111);

	BOOST_TEST(ns.try_exclude(1) == true);
	BOOST_TEST(ns.to_ulong() == 0b1101);

	BOOST_TEST(ns.try_exclude(1) == false);
	BOOST_TEST(ns.to_ulong() == 0b1101);
}

BOOST_AUTO_TEST_CASE(test_bitset_exclude_2)
{
	numset_t<4> ns;
	BOOST_TEST(ns.to_ulong() == 0b1111);

	numset_t<4>::bitset_t bs1(0b0011);
	BOOST_TEST(bs1.to_ulong() == 0b0011);

	BOOST_TEST(ns.try_exclude_set(bs1) == 2);
	BOOST_TEST(ns.to_ulong() == 0b1100);
	BOOST_TEST(ns.try_exclude_set(bs1) == 0);
	BOOST_TEST(ns.to_ulong() == 0b1100);

	numset_t<4>::bitset_t bs2(0b0110);
	BOOST_TEST(bs2.to_ulong() == 0b0110);
	BOOST_TEST(ns.try_exclude_set(bs2) == 1);
	BOOST_TEST(ns.to_ulong() == 0b1000);
	BOOST_TEST(ns.try_exclude_set(bs2) == 0);
	BOOST_TEST(ns.to_ulong() == 0b1000);
}

BOOST_AUTO_TEST_CASE(test_bitset_assign_set_1)
{
	numset_t<4> ns;
	BOOST_TEST(ns.to_ulong() == 0b1111);

	BOOST_TEST(ns.assign_set(numset_t<4>::bitset_t(0b1111)) == 0);
	BOOST_TEST(ns.to_ulong() == 0b1111);

	BOOST_TEST(ns.assign_set(numset_t<4>::bitset_t(0b1110)) == 1);
	BOOST_TEST(ns.to_ulong() == 0b1110);

	BOOST_TEST(ns.assign_set(numset_t<4>::bitset_t(0b0010)) == 2);
	BOOST_TEST(ns.to_ulong() == 0b0010);

	BOOST_TEST(ns.assign_set(numset_t<4>::bitset_t(0b0010)) == 0);
	BOOST_TEST(ns.to_ulong() == 0b0010);
}

BOOST_AUTO_TEST_CASE(test_contains_all_1)
{
	constexpr num_t N = 4;

	numset_t<N> ns;
	BOOST_TEST(ns.to_ulong() == 0b1111);
	numset_t<N>::bitset_t const bs_all(0b1111ULL);

	BOOST_TEST(ns.contains_all(bs_all));

	numset_t<N>::bitset_t bs1(0b1100ULL);
	BOOST_TEST(ns.contains_all(bs1));

	BOOST_TEST(ns.assign_set(bs1) == 2);
	BOOST_TEST(ns.to_ulong() == 0b1100);
	BOOST_TEST(!ns.contains_all(bs_all));

	BOOST_TEST(ns.contains_all(numset_t<N>::bitset_t(0b0000ULL)));
	BOOST_TEST(ns.contains_all(numset_t<N>::bitset_t(0b0100ULL)));
	BOOST_TEST(ns.contains_all(numset_t<N>::bitset_t(0b1000ULL)));
	BOOST_TEST(ns.contains_all(numset_t<N>::bitset_t(0b1100ULL)));

	BOOST_TEST(!ns.contains_all(numset_t<N>::bitset_t(0b0001ULL)));
	BOOST_TEST(!ns.contains_all(numset_t<N>::bitset_t(0b1001ULL)));
	BOOST_TEST(!ns.contains_all(numset_t<N>::bitset_t(0b1101ULL)));
	BOOST_TEST(!ns.contains_all(numset_t<N>::bitset_t(0b1110ULL)));
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_row)
{
	std::istringstream is(
		"4*21\n"
		"****\n"
		"****\n"
		"****\n"
	);
	sudoku_t<4> s(is);
	s.solve();
	BOOST_TEST(s.data_.at(0).at(1).print() == '3');
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_column)
{
	std::istringstream is(
		"4***\n"
		"****\n"
		"2***\n"
		"1***\n"
	);
	sudoku_t<4> s(is);
	s.solve();
	BOOST_TEST(s.data_.at(1).at(0).print() == '3');
}

BOOST_AUTO_TEST_CASE(test_exclusions_solve_sq)
{
	std::istringstream is(
		"4***\n"
		"21**\n"
		"****\n"
		"****\n"
	);
	sudoku_t<4> s(is);
	s.solve();
	BOOST_TEST(s.data_.at(0).at(1).print() == '3');
}

BOOST_AUTO_TEST_CASE(test_emplace_sq)
{
	std::istringstream is(
		"*2**\n" /* 1 can be placed only in the first column of this row */
		"***1\n"
		"****\n"
		"****\n"
	);
	sudoku_t<4> s(is);
	s.solve();
	BOOST_TEST(s.data_.at(0).at(0).print() == '1');
}

BOOST_AUTO_TEST_CASE(test_emplace_column)
{
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
	BOOST_TEST(s.data_.at(3).at(7).print() == '1');
}

BOOST_AUTO_TEST_CASE(test_cluster_column_size_2)
{
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
	sudoku_t<9> s(is);
	s.solve();
	// can contain {8,9} only
	BOOST_TEST(s.data_.at(0).at(0).to_string() == "110000000");
	BOOST_TEST(s.data_.at(1).at(0).to_string() == "110000000");

	// {8,9} is excluded from top-left square
	BOOST_TEST(s.data_.at(0).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(1).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(2).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(0).at(2).to_string() == "001111110");
	BOOST_TEST(s.data_.at(1).at(2).to_string() == "001111110");
	BOOST_TEST(s.data_.at(2).at(2).to_string() == "001111110");
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
		for (idx_t r=0; r<N; ++r) {
			for (idx_t c=0; c<N; ++c) {
				BOOST_TEST(s.data_.at(r).at(c).bits_.count() == 1);
			}
		}
	}
}

