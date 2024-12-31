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

BOOST_AUTO_TEST_CASE(test_cluster_size_1)
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
	// {8,9} is excluded from the following
	BOOST_TEST(s.data_.at(0).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(1).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(2).at(1).to_string() == "001111110");
	BOOST_TEST(s.data_.at(0).at(2).to_string() == "001111110");
	BOOST_TEST(s.data_.at(1).at(2).to_string() == "001111110");
	BOOST_TEST(s.data_.at(2).at(2).to_string() == "001111110");
}

