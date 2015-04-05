
namespace quick_sort {

	namespace impl {
		template <typename Iter>
		Iter select_pivot(Iter first, Iter last)
		{
			return first;
		}

		// returns iterator to pivot
		template <typename Iter, typename Swap>
		Iter partition(Iter first, Iter last, const Swap& swap_func)
		{
			{
				const auto pivot_iter = select_pivot(first, last);
				if (pivot_iter != first)
					swap_func(*pivot_iter, *first);
			}
		}
	}

	template <typename RangeIn, typename Pred, typename Swap>
	void sort(const RangeIn& in, const Pred& is_less, const Swap& swap_func)
	{
		if (in.empty() || in.size()==1)
			return;

		const auto pivot_iter = partition(in.begin(), in.end());
		sort(boost::make_iterator_range(in, pivot), is_less, swap_func);
		sort(boost::make_iterator_range(pivot+1, in.end()), is_less, swap_func);
	}

}
