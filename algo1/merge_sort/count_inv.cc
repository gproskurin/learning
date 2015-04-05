#include <algorithm>
#include <deque>
#include <iterator>
#include <iostream>

#include <boost/range/iterator_range.hpp>

#include <stdlib.h>

namespace detail {

	template <typename T>
	using tmp_container_t = std::deque<T>;

	template <typename RangeIn, typename IterOut, typename Pred>
	size_t merge(
			RangeIn left,
			RangeIn right,
			IterOut result,
			Pred less
		)
	{
		if (left.empty()) {
			std::copy(right.begin(), right.end(), result);
			return 0;
		}
		if (right.empty()) {
			std::copy(left.begin(), left.end(), result);
			return 0;
		}

		size_t count_inv = 0;
		while (true) {
			if (less(left.front(), right.front())) {
				*result = left.front();
				left.advance_begin(1);
				if (left.empty()) {
					std::copy(right.begin(), right.end(), result);
					return count_inv;
				}
			} else {
				*result = right.front();
				right.advance_begin(1);
				count_inv += left.size();
				if (right.empty()) {
					std::copy(left.begin(), left.end(), result);
					return count_inv;
				}
			}
		}
	}

	template <typename IterIn, typename IterOut, typename Pred>
	size_t merge_sort_impl(IterIn first, IterIn last, IterOut result, Pred less)
	{
		if (first==last)
			return 0;
		if (first+1==last) {
			*result = *first;
			return 0;
		}

		tmp_container_t<typename std::iterator_traits<IterIn>::value_type>
			left, right;
		const IterIn half = first + std::distance(first, last) / 2;

		size_t count_inv = 0;
		count_inv += merge_sort_impl(first, half, std::back_inserter(left), less);
		count_inv += merge_sort_impl(half, last, std::back_inserter(right), less);
		count_inv += merge(
			boost::make_iterator_range(left.cbegin(), left.cend()),
			boost::make_iterator_range(right.cbegin(), right.cend()),
			result,
			less
		);
		return count_inv;
	}

}

template <typename IterIn, typename IterOut, typename Pred>
size_t merge_sort_count_inv(IterIn first, IterIn last, IterOut result, Pred less)
{
	return detail::merge_sort_impl(first, last, result, less);
}

template <typename IterIn, typename Pred>
size_t count_inv_simple(IterIn first, IterIn last, Pred less)
{
	size_t count_inv = 0;
	for (IterIn i=first; i!=last; ++i) {
		for (IterIn j=i+1; j!=last; ++j) {
			if (!less(*i, *j)) {
				++count_inv;
			}
		}
	}
	return count_inv;
}

int gen() { return random() % 1000000; }

std::vector<int> gen_vector(size_t sz)
{
	srandom(time(nullptr));
	std::vector<int> res(sz);
	std::generate(res.begin(), res.end(), gen);
	return res;
}

template <typename C>
void print(const C& c, std::ostream& os)
{
	std::copy(c.cbegin(), c.cend(), std::ostream_iterator<typename C::value_type>(os, " "));
	os << "\n";
}

int main()
{
	std::vector<int> v1 = gen_vector(50000);
	//print(v1, std::cout);
	std::cout << "InvSimple:\t" << count_inv_simple(v1.cbegin(), v1.cend(), std::less<int>()) << "\n";
	std::vector<int> vs;
	const size_t inv2 = merge_sort_count_inv(v1.cbegin(), v1.cend(), std::back_inserter(vs), std::less<int>());
	std::cout << "Count:\t\t" << inv2 << "\n";
}
