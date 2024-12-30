#define BOOST_TEST_MODULE solver_tests
#include <boost/test/included/unit_test.hpp>

#include "solver.h"

#include <iostream>
#include <sstream>


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

BOOST_AUTO_TEST_CASE(test_emplace_row)
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

