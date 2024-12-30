#include "solver.h"

int main()
{
	try {
		sudoku_t<9> s(std::cin);
		std::cout << "INPUT\n";
		s.print(std::cout);
		std::cout << std::endl;

		s.solve();

		std::cout << "OUTPUT_DETAILED\n";
		s.print_detailed(std::cout);
		std::cout << std::endl;

		std::cout << "OUTPUT\n";
		s.print(std::cout);
		std::cout << std::endl;

		return 0;

	} catch (const std::exception& e) {
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}
}

