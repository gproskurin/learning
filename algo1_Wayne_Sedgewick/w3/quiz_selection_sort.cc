#include <algorithm>
#include <array>
#include <iostream>

#include <assert.h>

template <typename C>
void print_array(const C& c, std::ostream& os)
{
	for (const auto& i : c) {
		os << " " << i;
	}
	os << "\n";
}

template <typename C>
void selection_sort_steps(C& c, size_t const steps)
{
	size_t steps_done = 0;
	auto first_unsorted = c.begin();
	while (steps_done < steps) {
		assert(first_unsorted != c.end());
		auto min_iter = std::min_element(first_unsorted, c.end());
		assert(min_iter != c.end());
		if (min_iter != first_unsorted) {
			using std::swap;
			swap(*first_unsorted, *min_iter);
		}
		++first_unsorted;
		++steps_done;
	}
}

int main()
{
	std::array<int,10> a { 69,64,38,84,86,83,24,79,59,97 };
	print_array(a, std::cout);
	selection_sort_steps(a, 4);
	print_array(a, std::cout);
}
