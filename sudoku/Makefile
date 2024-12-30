BOOST_INCLUDE ?= /usr/include

.PHONY: test

solver: solver.h main.cc Makefile
	c++ main.cc -std=c++23 -o solver -Werror -Wall -Wno-sign-compare -O3

test:
	c++ solver_tests.cc -std=c++23 -o solver_tests -Werror -Wall -Wno-sign-compare -O3 -I${BOOST_INCLUDE}
	./solver_tests

