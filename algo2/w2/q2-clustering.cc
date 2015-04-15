#include <stdint.h>
#include <assert.h>

#include <algorithm>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

typedef uint32_t number_t;
const number_t N24 = 24;

typedef std::vector<number_t> num_list_t;

typedef std::unordered_map<number_t, number_t> numset_t;

size_t count_1_bits(number_t n)
{
	size_t count = 0;
	for ( ; n!=0; count++, n &= n-1)
		;
	return count;
}

num_list_t generate_1bit(size_t num_bits)
{
	if (num_bits > sizeof(number_t)*8)
		throw std::runtime_error("out_of_bits");
	num_list_t res;
	res.reserve(1 << num_bits);
	for (number_t shift=0; shift<num_bits; ++shift) {
		const number_t n = 1 << shift;
		assert(count_1_bits(n)==1);
		res.push_back(n);
	}
	assert(res.size()==num_bits);
	return res;
}

num_list_t generate_2bit(size_t num_bits)
{
	if (num_bits > sizeof(number_t)*8)
		throw std::runtime_error("out_of_bits");
	num_list_t res;
	for (number_t shift1=0; shift1<num_bits; ++shift1) {
		const number_t num1 = (1 << shift1);
		for (number_t shift2=shift1+1; shift2<num_bits; ++shift2) {
			const number_t num2 = (1 << shift2);
			const number_t n = num1 | num2;
			assert(count_1_bits(n)==2);
			res.push_back(n);
		}
	}
	return res;
}

template <typename NumSet, typename Iter>
Iter get_root_impl(NumSet&& ns, Iter iter)
{
	//std::cout << "start get_root\n";
	assert(iter != ns.cend());
	while (iter->first != iter->second) {
		iter = ns.find(iter->second); // search for parent of current element
		assert(iter != ns.cend());
	}
	//std::cout << "end get_root\n";
	return iter;
}

numset_t::const_iterator get_root(const numset_t& ns, numset_t::const_iterator iter)
{
	const auto root_iter = get_root_impl(ns, iter);
	assert(root_iter->first == root_iter->second);
	return root_iter;
}

numset_t::iterator get_root(numset_t& ns, numset_t::iterator iter)
{
	numset_t::iterator const root_iter = get_root_impl(ns, iter);
	assert(root_iter->first == root_iter->second);
	iter->second = root_iter->first; // path compression
	return root_iter;
}

void reparent(numset_t& ns, numset_t::iterator iter, number_t new_parent)
{
	const numset_t::iterator root_iter = get_root(ns, iter);
	assert(root_iter->first == root_iter->second);
	root_iter->second = new_parent; // set new parent
}

void union_hamming_dist(numset_t& ns, const num_list_t& dists)
{
	//size_t count = 0;
	for (numset_t::iterator cur_iter=ns.begin(); cur_iter!=ns.end(); ++cur_iter) {
		const number_t cur_num = cur_iter->first;
		const number_t cur_num_root = get_root(ns, cur_iter)->second;
		//std::cout << "NUM: " << cur_num << " PAR:" << cur_iter->second << " ROOT:" << cur_num_root << "\n";
		for (const number_t d : dists) {
			//std::cout << "\tdist:" << d << "\n";
			const number_t search_num = cur_num ^ d;
			numset_t::iterator const iter = ns.find(search_num);
			if (iter != ns.cend()) {
				reparent(ns, iter, cur_num_root);
			}
		}
		//++count;
		//if (count % 100 == 0)
		//	std::cout << count << "\n";
	}
}

num_list_t generate_2bit_brute(size_t num_bits)
{
	num_list_t res;
	for (size_t num=0; num<(size_t(1)<<num_bits); ++num) {
		if (count_1_bits(num)==2)
			res.push_back(num);
	}
	std::sort(res.begin(), res.end());
	return res;
}


numset_t load_data(std::istream& is)
{
	numset_t res;
	size_t num_lines;
	size_t num_bits;
	is >> num_lines >> num_bits;
	if (num_bits > (sizeof(number_t)*8))
		throw std::runtime_error("num_bits_unsupported");
	for (size_t line=1; line<=num_lines; ++line) {
		number_t num = 0;
		for (unsigned bit_num=0; bit_num<num_bits; ++bit_num) {
			number_t bit;
			is >> bit;
			if (bit!=0 && bit!=1)
				throw std::runtime_error("bit_not_0_1");
			if (bit_num==0)
				num = bit;
			else
				num = (num << 1) | bit;
		}
		res.emplace(num,num);
	}
	if (!is)
		throw std::runtime_error("early_EOF");
	char c;
	is >> c;
	if (is)
		throw std::runtime_error("no_EOF");
	std::cout << "Loading OK, distinct numbers: " << res.size() << " buckets:" << res.bucket_count() << "\n";
	return res;
}

size_t count_clusters(const numset_t& ns)
{
	std::unordered_set<number_t> roots;
	for (auto i=ns.cbegin(); i!=ns.cend(); ++i) {
		const numset_t::const_iterator root_iter = get_root(ns, i);
		assert(root_iter != ns.cend());
		assert(root_iter->first == root_iter->second);
		roots.emplace(root_iter->first);
	}
	return roots.size();
}

void run()
{
	numset_t nums = load_data(std::cin);
	std::cout << nums.size() << "\n";

	const num_list_t b1 = generate_1bit(N24);
	const num_list_t b2 = generate_2bit(N24);
	std::cout << "b1:" << b1.size() << " b2:" << b2.size() << "\n";

	std::cout << "Clusters: " << count_clusters(nums) << "\n";

	union_hamming_dist(nums, b1);
	std::cout << "Hamming distance 1 done. Clusters: " << count_clusters(nums) << "\n";

	union_hamming_dist(nums, b2);
	std::cout << "Hamming distance 2 done. Clusters: " << count_clusters(nums) << "\n";
}

int main()
{
	try {
		run();
		return 0;
	} catch (const std::exception& e) {
		std::cerr << "Ex: " << e.what() << "\n";
	}
	return 1;
}
