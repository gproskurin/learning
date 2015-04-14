#include <stdint.h>
#include <assert.h>

#include <algorithm>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

typedef uint32_t number_t;
const number_t N24 = 24;

typedef std::vector<number_t> num_list_t;

struct number_info_t {
	number_t num;
	number_t parent;

	explicit number_info_t(number_t n) : num(n), parent(n) {}

	struct hasher {
		size_t operator()(const number_info_t& ni) const { return std::hash<number_t>()(ni.num); }
	};
	struct key_equal {
		bool operator()(const number_info_t& ni1, const number_info_t& ni2) const { return ni1.num==ni2.num; }
	};
};

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

num_list_t generate_2bit_brute(size_t num_bits)
{
	num_list_t res;
	for (size_t num=0; num<(1<<num_bits); ++num) {
		if (count_1_bits(num)==2)
			res.push_back(num);
	}
	return res;
}


typedef std::unordered_set<
		number_info_t,
		number_info_t::hasher,
		number_info_t::key_equal
	>
	input_set_t;


input_set_t load_data(std::istream& is)
{
	input_set_t res;
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
		res.emplace(num);
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

void run()
{
	const input_set_t nums = load_data(std::cin);
	std::cout << nums.size() << "\n";

	const num_list_t b1 = generate_1bit(N24);
	const num_list_t b2 = generate_2bit(N24);
	std::cout << "b1:" << b1.size() << " b2:" << b2.size() << "\n";
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
