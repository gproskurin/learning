#include "sets.h"

#include <iostream>

int main()
{
	{
		sets_t<int> sets(100, 105);
		sets.print(std::cout);

		sets._reparent_set_to(101, 103);
		sets.print(std::cout);

		sets._reparent_set_to(103, 104);
		sets.print(std::cout);

		sets.foreach_pair_in_different_sets(
			[](int s1, int s2) { std::cout << "(" << s1 << "," << s2 << ")\n"; }
		);
	}

	if (0) {
		sets_t<int> sets(-3, 2);
		sets.print(std::cout);

		sets._reparent_set_to(-2, -1);
		sets.print(std::cout);

		sets._reparent_set_to(-1, 2);
		sets.print(std::cout);

	}

	return 0;
}
