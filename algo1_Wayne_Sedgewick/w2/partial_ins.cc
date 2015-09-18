#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>

template <typename C>
void print(const C& c)
{
	for (const auto& i : c) {
		std::cout << " " << i;
	}
	std::cout << "\n";
}

template <typename C>
void partial_sort(C& c, unsigned exch)
{
	for (size_t u=1; u<c.size()-1; ++u) {
		size_t j = u - 1;
		while (j>0 && c.at(j) < c.at(j-1)) {
			using std::swap;
			swap(c[j], c[j-1]);
			--exch;
			--j;
			std::cout << exch << "\n";
			print(c);
			if (exch == 0)
				return;
		}
	}
}

int main()
{
	std::array<int, 10> a { {19,28,50,84,93,96,82,12,66,74} };
	print(a);
	partial_sort(a, 6);
	std::cout << "---\n";
	print(a);
}
