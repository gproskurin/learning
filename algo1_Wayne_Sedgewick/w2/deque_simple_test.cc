#include "deque_simple.h"

#include <deque>
#include <iostream>

#include <assert.h>
#include <stdlib.h>

bool rnd_bool()
{
	return random() & 1;
}

bool rnd_bool_more_false()
{
	return (random() % 100) < 48;
}

bool rnd_bool_more_true()
{
	return !rnd_bool_more_false();
}

int rnd_int()
{
	return static_cast<int>(random());
}

template <typename C1, typename C2>
void run_test(C1& c1, C2& c2)
{
	bool inc = true;
	for (size_t i=0; i<100000; ++i) {
		assert((c1.size()==0) == c1.empty());
		assert((c2.size()==0) == c2.empty());
		assert(c1.size() == c2.size());
		assert(c1.empty() == c2.empty());

		if (c1.empty())
			inc = true;
		if (c1.size() > 1000)
			inc = false;

		const bool push_or_pop = c1.empty() || (inc ? rnd_bool_more_true() : rnd_bool_more_false());

		const bool front_or_back = rnd_bool();
		//std::cout << (push_or_pop ? "push" : "pop") << " " << (front_or_back ? "front" : "back") << "\n";
		if (push_or_pop) {
			const int rnd_val = rnd_int();
			if (front_or_back) {
				c1.push_back(rnd_val);
				c2.push_back(rnd_val);
				assert(c1.back() == c2.back());
			} else {
				c1.push_front(rnd_val);
				c2.push_front(rnd_val);
				assert(c1.front() == c2.front());
			}
		} else {
			assert(!c1.empty());
			assert(!c2.empty());
			assert(c1.front() == c2.front());
			assert(c1.back() == c2.back());
			if (front_or_back) {
				c1.pop_front();
				c2.pop_front();
			} else {
				c1.pop_back();
				c2.pop_back();
			}
			//std::cout << "c1s:" << c1.size() << " c2s:" << c2.size() << "\n";
			//std::cout << "c1e?" << c1.empty() << " c2e:" << c2.empty() << "\n";
			assert(c1.size() == c2.size());
			assert(c1.empty() == c2.empty());
			if (!c1.empty()) {
				assert(c1.front() == c2.front());
				assert(c1.back() == c2.back());
			}
		}
		//const auto c1s = c1.size();
		//const auto c2s = c2.size();
		//std::cout << "c1_size:" << c1s << " c2size:" << c2s << "\n";
		assert(c1.size() == c2.size());
		assert(c1.empty() == c2.empty());
		//std::cout << c1.size() << "\n";
	}
}

template <typename C1, typename C2>
void test1(C1& c1, C2& c2)
{
	for (size_t i=0; i<1000; ++i) {
		//std::cout << "before push_back: c1s:" << c1.size() << " c2s:" << c2.size() << "\n";
		c1.push_back(i);
		c2.push_back(i);
		assert(c1.back() == c2.back());
		assert(c1.size() == c2.size());
		assert(c1.empty() == c2.empty());
	}

	assert(c1.size()==c2.size());
	const size_t sz = c1.size();
	for (size_t i=0; i<sz; ++i) {
		assert(c1.at(i) == c2.at(i));
	}
}

template <typename C1, typename C2>
void test2(C1& c1, C2& c2)
{
	for (size_t i=0; i<1000; ++i) {
		//std::cout << "before push_front: c1s:" << c1.size() << " c2s:" << c2.size() << "\n";
		c1.push_front(i);
		c2.push_front(i);
		assert(c1.front() == c2.front());
		assert(c1.size() == c2.size());
		assert(c1.empty() == c2.empty());
	}
}

int main()
{
	if (1) {
		std::deque<int> d_std;
		deque<int> d_my;
		std::cout << "start_test1\n";
		test1(d_my, d_std);
		std::cout << "end_test1\n";
	}

	if (1) {
		std::deque<int> d_std;
		deque<int> d_my;
		std::cout << "start_test2\n";
		test2(d_my, d_std);
		std::cout << "end_test2\n";
	}

	std::deque<int> d_std;
	deque<int> d_my;

	assert(d_my.empty());
	assert(d_my.size()==0);
	run_test(d_my, d_std);
	d_my.clear();
	d_std.clear();
	assert(d_my.size()==0);
	run_test(d_my, d_std);
}
