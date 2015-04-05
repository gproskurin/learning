#include <algorithm>
#include <deque>
#include <iterator>

namespace detail {

	template <typename T>
	using tmp_container_t = std::deque<T>;

	template <typename IterIn, typename IterOut, typename Pred>
	void merge(
			IterIn first1,
			IterIn last1,
			IterIn first2,
			IterIn last2,
			IterOut result,
			Pred less
		)
	{
		if (first1==last1) {
			std::copy(first2, last2, result);
			return;
		}
		if (first2==last2) {
			std::copy(first1, last1, result);
			return;
		}
		while (true) {
			if (less(*first1,*first2)) {
				*result = *first1;
				++first1;
				if (first1==last1) {
					std::copy(first2, last2, result);
					return;
				}
			} else {
				*result = *first2;
				++first2;
				if (first2==last2) {
					std::copy(first1, last1, result);
					return;
				}
			}
		}
	}

	template <typename IterIn, typename IterOut, typename Pred>
	void merge_sort_impl(IterIn first, IterIn last, IterOut result, Pred less)
	{
		if (first==last)
			return;
		if (first+1==last) {
			*result = *first;
			return;
		}

		tmp_container_t<typename std::iterator_traits<IterIn>::value_type>
			left, right;
		const IterIn half = first + std::distance(first, last) / 2;

		merge_sort_impl(first, half, std::back_inserter(left), less);
		merge_sort_impl(half, last, std::back_inserter(right), less);
		merge(left.cbegin(), left.cend(), right.cbegin(), right.cend(), result, less);
	}

}

template <typename IterIn, typename IterOut, typename Pred>
void merge_sort(IterIn first, IterIn last, IterOut result, Pred less)
{
	detail::merge_sort_impl(first, last, result, less);
}

