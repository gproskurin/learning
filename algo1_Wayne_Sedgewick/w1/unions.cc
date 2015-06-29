#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <assert.h>

typedef std::array<size_t, 10> union_t;
typedef std::array<size_t, 10> sizes_t;

typedef std::tuple<union_t, sizes_t> union_with_sizes_t;

union_t new_union()
{
	union_t u;
	for (size_t i=0; i<u.size(); ++i) {
		u.at(i) = i;
	}
	return u;
}

union_t union_from_str(const std::string& s)
{
	std::vector<size_t> uv;
	uv.reserve(10);
	for (auto i=boost::algorithm::make_split_iterator(s, boost::algorithm::token_finder(boost::algorithm::is_space(), boost::algorithm::token_compress_on)); !i.eof(); ++i) {
		if (i->empty())
			continue;
		const size_t n = boost::lexical_cast<size_t>(*i);
		uv.push_back(n);
	}
	union_t u;
	assert(uv.size()==u.size());
	for (size_t i=0; i<uv.size(); ++i) {
		u.at(i) = uv.at(i);
	}
	return u;
}

union_with_sizes_t new_union_with_sizes()
{
	sizes_t s;
	for (size_t i=0; i<s.size(); ++i) {
		s.at(i) = 1;
	}
	return std::make_tuple(new_union(), s);
}

size_t get_root(const union_t& u, size_t i)
{
	while (i != u.at(i)) {
		i = u.at(i);
	}
	return i;
}

void union_combine_q1(union_t& u, size_t p, size_t q)
{
	const auto id_p = u.at(p);
	const auto id_q = u.at(q);
	for (size_t i=0; i<u.size(); ++i) {
		if (u.at(i) == id_p)
			u.at(i) = id_q;
	}
}

void union_combine_q2(union_with_sizes_t& us, size_t p, size_t q)
{
	union_t& u = std::get<0>(us);
	sizes_t& sz = std::get<1>(us);
	const auto rp = get_root(u,p);
	const auto rq = get_root(u,q);
	if (sz.at(rp) < sz.at(rq)) {
		assert(u.at(rp)==rp);
		u.at(rp) = rq;
		sz.at(rq) += sz.at(rp);
	} else {
		assert(u.at(rq)==rq);
		u.at(rq) = rp;
		sz.at(rp) += sz.at(rq);
	}
}

void union_print(const union_t& u, std::ostream& os)
{
	for (const auto& i : u) {
		os << " " << i;
	}
	os << "\n";
}

std::vector<std::pair<size_t,size_t>> to_vp(const std::string& s)
{
	std::vector<std::pair<size_t,size_t>> res;
	for (auto i=boost::algorithm::make_split_iterator(s, boost::algorithm::token_finder(boost::algorithm::is_space(), boost::algorithm::token_compress_on)); !i.eof(); ++i) {
		if (i->empty())
			continue;
		const std::string::value_type hy = '-';
		auto hy_iter = std::find(i->begin(), i->end(), hy);
		assert(hy_iter != i->end());
		assert(*hy_iter == hy);
		const auto n1 = boost::lexical_cast<size_t>(boost::make_iterator_range(i->begin(), hy_iter));
		const auto n2 = boost::lexical_cast<size_t>(boost::make_iterator_range(hy_iter+1, i->end()));
		res.emplace_back(n1, n2);
	}
	return res;
}

size_t union_height(const union_t& u, size_t p)
{
	size_t h = 1;
	while (p != u.at(p)) {
		++h;
		p = u.at(p);
	}
	//std::cout << "h("<<p<<"):" << h << "\n";
	return h;
}

size_t union_max_height(const union_t& u)
{
	size_t mh = union_height(u, 0);
	for (auto p : u) {
		mh = std::max(mh, union_height(u, p));
	}
	return mh;
}

bool union_is_same_or_parent_of(const union_t& u, size_t c, const size_t p)
{
	bool res;
	const auto old_c = c;
	while (true) {
		if (c==p) {
			res = true;
			break;
		}
		if (c==u.at(c)) {
			res = false;
			break;
		}
		c = u.at(c);
	}
	//std::cout << "is_same_or_parent_of(" << old_c << "," << p << "): " << (res?"TRUE":"FALSE") << "\n";
	return res;
}

size_t union_sub_size(const union_t& u, size_t const p)
{
	size_t count = 0;
	for (size_t i=0; i<u.size(); ++i) {
		if (union_is_same_or_parent_of(u,i,p))
			++count;
	}
	//std::cout << "sub_size(" << p << "): " << count << "\n";
	return count;
}

bool union_has_cycle_from(const union_t& u, size_t p)
{
	const auto start = p;
	while (true) {
		if (p==u.at(p))
			return false;
		p = u.at(p);
		if (start == p)
			return true;
	}
}

bool union_has_cycles(const union_t& u)
{
	for (const auto x : u) {
		if (union_has_cycle_from(u,x))
			return true;
	}
	return false;
}

namespace union_tests {
	bool height_ok(const union_t& u) {
		const auto mh = union_max_height(u);
		const bool res = mh < 5;
		if (!res)
			std::cout << "HEIGHT_FAIL h:" << mh << "\n";
		return res;

	}

	bool cycles_ok(const union_t& u) {
		const bool res = !union_has_cycles(u);
		if (!res)
			std::cout << "CYCLE_FAIL\n";
		return res;
	}

	bool balance_ok(const union_t& u) {
		for (const auto x : u) {
			if (x==u.at(x))
				continue;
			const auto size_of_tree_rooted_at_parent_x = union_sub_size(u, u.at(x));
			const auto size_of_tree_rooted_at_x = union_sub_size(u, x);
			//std::cout << "size_of_tree_rooted_at_parent_x(" << x << "): " << size_of_tree_rooted_at_parent_x << "\n";
			//std::cout << "size_of_tree_rooted_at_x(" << x << "): " << size_of_tree_rooted_at_x << "\n";
			if (size_of_tree_rooted_at_parent_x < 2 * size_of_tree_rooted_at_x) {
				std::cout << "BALANCE_FAIL: " << x << "\n";
				return false;
			}
		}
		return true;
	}

	void run_all(const union_t& u) {
		union_print(u, std::cout);
		bool ok = true;
		ok = cycles_ok(u) && ok;
		if (ok) {
			ok = height_ok(u) && ok;
			ok = balance_ok(u) && ok;
		}
		std::cout << " *** " << (ok?"OK":"FAIL") << "\n\n";
	}
}

int main()
{
	if (1) {
		union_t u = new_union();
		union_print(u, std::cout);

		for (const auto& p : to_vp("4-0 1-7 7-4 7-8 3-6 9-0")) {
			union_combine_q1(u, p.first, p.second);
		}

		union_print(u, std::cout);
	}

	std::cout << "---\n";

	if (1) {
		auto us = new_union_with_sizes();
		union_print(std::get<0>(us), std::cout);


		for (const auto& p : to_vp("9-7 2-5 4-8 5-1 3-5 6-5 9-4 7-5 7-0")) {
			union_combine_q2(us, p.first, p.second);
		}

		union_print(std::get<0>(us), std::cout);
	}

	std::cout << "---\n";

	if (1) {
		union_tests::run_all(union_from_str("3 5 3 5 5 5 5 2 5 5"));
		union_tests::run_all(union_from_str("0 1 5 4 5 6 1 8 5 5"));
		union_tests::run_all(union_from_str("6 6 6 4 1 1 6 6 1 1"));
		union_tests::run_all(union_from_str("7 7 5 0 7 8 7 2 7 8"));
		union_tests::run_all(union_from_str("0 1 2 3 6 6 6 7 8 6"));
	}
}
