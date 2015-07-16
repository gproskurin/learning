#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <vector>

#include <assert.h>

namespace quick_sort {

	enum class partition_method {
		pivot_first,
		pivot_last,
		med3,
	};

	namespace impl {
		// returns iterator to pivot
		template <typename Iter, typename Pred>
		Iter partition(Iter const first, Iter const last, Iter const pivot_iter, const Pred& is_less)
		{
			assert(first != last);
			assert(pivot_iter != last);

			if (pivot_iter != first) {
				using std::swap;
				swap(*first, *pivot_iter);
			}
			const auto& p = *first;
			Iter gt_begin = first+1;
			for (Iter unp_begin = first+1; unp_begin!=last; ++unp_begin) {
				if (is_less(*unp_begin,p)) { // current element is less than pivot
					using std::swap;
					swap(*unp_begin, *gt_begin);
					++gt_begin;
				}
			}
			{
				using std::swap;
				swap(*first, *(gt_begin-1));
			}
			return gt_begin-1;
		}

		template <typename Iter>
		Iter middle(Iter first, Iter last)
		{
			const auto size = last-first;
			assert(size >= 3);
			Iter mid = first + size/2;
			if ((size & 1) == 0) {
				--mid;
			}
			return mid;
		}

		template <typename Iter>
		Iter med3(Iter i1, Iter i2, Iter i3)
		{
			if (*i1==*i2 || *i1==*i3 || *i2==*i3) {
				assert(! "med3: eq"); // breaks condition of the task
				return i1;
			}
			Iter m;
			if (*i1 < *i2) {
				if (*i2 < *i3)
					m = i2;
				else if (*i3 < *i1)
					m = i1;
				else
					m = i3;
			} else {
				if (*i1 < *i3)
					m = i1;
				else if (*i3 < *i2)
					m = i2;
				else
					m = i3;
			}
#ifndef NDEBUG
			{
				std::array<Iter, 3> a{ {i1, i2, i3} };
				std::sort(
					a.begin(),
					a.end(),
					[](Iter a, Iter b) { return *a < *b; }
				);
				if (m!=a[1]) {
					std::cout << "med3: " << *i1 << " " << *i2 << " " << *i3 << " m:" << *m << "\n";
				}
				assert(m==a[1]);
			}
#endif
			return m;
		}
	}

	template <typename Iter, typename Pred>
	Iter do_partition(Iter const first, Iter const last, const Pred& is_less, partition_method pm)
	{
		assert(first != last);
		switch (pm) {
			case partition_method::pivot_first:
				return impl::partition(first, last, first, is_less);
			case partition_method::pivot_last:
				return impl::partition(first, last, last-1, is_less);
			case partition_method::med3: {
				const auto size = last-first;
				assert(size > 2);
				const Iter mid = impl::middle(first, last);
				//std::cout << "size:" << size << " mid:" << (mid-first) << "\n";
				return impl::partition(first, last, impl::med3(first, mid, last-1), is_less);
			}
		}
		assert(! "Unknown partition method");
	}

	template <typename Iter, typename Pred>
	unsigned long sort(Iter const first, Iter const last, const Pred& is_less, partition_method pm)
	{
		const auto size = last - first;
		if (size <= 1)
			return 0;
		if (size == 2) {
			// swap two elements, if necessary
			assert(first+1 == last-1);
			if (is_less(*(first+1), *first)) {
				using std::swap;
				swap(*(first+1), *first);
			}
			return 1;
		}

		const auto pivot_iter = do_partition(first, last, is_less, pm);
		unsigned long cmp_count = size - 1; // partitioning size elements requires size-1 comparisons

		// not including pivot in halfs
		cmp_count += sort(
			first,
			pivot_iter,
			is_less,
			pm
		);
		cmp_count += sort(
			pivot_iter+1,
			last,
			is_less,
			pm
		);

		return cmp_count;
	}

}

template <typename T>
std::vector<T> read_data(std::istream& is)
{
	std::vector<T> res;
	while (is) {
		T n;
		is >> n;
		if (is)
			res.push_back(n);
	}
	std::cout << "Size: " << res.size() << "\n";
	return res;
}

struct cmp_count_t {
	cmp_count_t() : cmp_count(0) {}
	template <typename T>
	bool operator()(const T& t1, const T& t2) const {
		++cmp_count;
		return std::less<T>()(t1, t2);
	}
	mutable unsigned long cmp_count;
};

void run()
{
	const auto data = read_data<int>(std::cin);

	// pivot first
	{
		auto d = data;
		cmp_count_t cmp;
		const auto cmp_count = quick_sort::sort(d.begin(), d.end(), cmp, quick_sort::partition_method::pivot_first);
		std::cout << "pivot_first: cmp_func:" << cmp.cmp_count << " cmp_R:" << cmp_count << "\n";
		assert(std::is_sorted(d.cbegin(), d.cend()));
		assert(cmp_count == cmp.cmp_count);
	}

	// pivot last
	{
		auto d = data;
		cmp_count_t cmp;
		const auto cmp_count = quick_sort::sort(d.begin(), d.end(), cmp, quick_sort::partition_method::pivot_last);
		std::cout << "pivot_last: cmp_func:" << cmp.cmp_count << " cmp_R:" << cmp_count << "\n";
		assert(std::is_sorted(d.cbegin(), d.cend()));
		assert(cmp_count == cmp.cmp_count);
	}

	// med3
	{
		auto d = data;
		cmp_count_t cmp;
		const auto cmp_count = quick_sort::sort(d.begin(), d.end(), cmp, quick_sort::partition_method::med3);
		std::cout << "pivot_med3: cmp_func:" << cmp.cmp_count << " cmp_R:" << cmp_count << "\n";
		assert(std::is_sorted(d.cbegin(), d.cend()));
		assert(cmp_count == cmp.cmp_count);
	}
}

int main()
{
	try {
		run();
		return 0;
	} catch (const std::exception& ex) {
		std::cerr << "Ex: " << ex.what() << "\n";
	}
	return 1;
}
