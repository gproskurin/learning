#include <array>
#include <iostream>
#include <vector>

#include <assert.h>
#include <stdlib.h>

struct literal_t {
	bool negate;
	size_t varnum;
};

typedef std::array<literal_t, 2> clause_t;

typedef std::vector<clause_t> expr_t;

typedef std::vector<bool> ass_t;

bool calc_l(const literal_t& l, const ass_t& ass)
{
	const bool var = ass.at(l.varnum);
	const bool res1 = l.negate ? !var : var;
	//const bool res2 = var ^ l.negate;
	//assert(res1 == res2);
	return res1;
}

bool calc_c(const clause_t& c, const ass_t& ass)
{
	// OR
	for (const auto& l : c) {
		const bool l_val = calc_l(l, ass);
		if (l_val == true)
			return true;
	}
	return false;
}

bool calc_e(const expr_t& e, const ass_t& ass)
{
	// AND
	for (const auto& c : e) {
		const bool c_val = calc_c(c, ass);
		if (c_val == false)
			return false;
	}
	return true;
}

ass_t gen_rnd_ass(size_t sz)
{
	ass_t ass(sz);
	for (ass_t::reference b : ass) {
		if (random() & 1) {
			b = !b;
		}
	}
	return ass;
}

void print_ass_stat(const ass_t& ass)
{
	size_t count_true = 0, count_false = 0;
	for (const bool b : ass) {
		if (b)
			++count_true;
		else
			++count_false;
	}
	assert(count_false+count_true == ass.size());
	std::cout << "ass size:" << ass.size() << " false:" << count_false << " true:" << count_true << "\n";
}

literal_t read_literal(std::istream& is)
{
	ssize_t n;
	is >> n;
	if (!is)
		throw std::runtime_error("early_EOF");
	if (n==0)
		throw std::runtime_error("zero_varnum");
	literal_t l;
	if (n < 0) {
		size_t num = static_cast<size_t>(-n);
		assert(num >= 1);
		l.varnum = num-1;
		l.negate = true;
	} else {
		const size_t num = static_cast<size_t>(n);
		assert(num >= 1);
		l.varnum = num-1;
		l.negate = false;
	}
	return l;
}

std::pair<expr_t,size_t> load_expr(std::istream& is)
{
	expr_t res;
	size_t sz;
	is >> sz;
	if (!is)
		throw std::runtime_error("early_EOF");
	res.reserve(sz);
	for (size_t i=0; i<sz; ++i) {
		clause_t c;
		static_assert(c.size()==2, "wrong size");
		for (size_t j=0; j<c.size(); ++j) {
			c.at(j) = read_literal(is);
		}
		res.emplace_back(std::move(c));
	}
	if (!is)
		throw std::runtime_error("early_EOF");
	char c;
	is >> c;
	if (c)
		throw std::runtime_error("no_EOF");
	return std::make_pair(res, sz);
}

void print_stat(const expr_t& e) {
	std::cout << "Expr size: " << e.size() << "\n";
	if (!e.empty()) {
		size_t min_var = e.at(0).at(0).varnum;
		size_t max_var = e.at(0).at(0).varnum;
		for (const auto& c : e) {
			for (const auto& l : c) {
				if (l.varnum > max_var)
					max_var = l.varnum;
				if (l.varnum < min_var)
					min_var = l.varnum;
			}
		}
		std::cout << "min_var:" << min_var << " max_var:" << max_var << "\n";
	}
}

void run()
{
	srandom(time(nullptr));
	const auto p = load_expr(std::cin);
	const expr_t& e = p.first;
	const size_t N = p.second;
	print_stat(e);
	for (int i=0; i<10; ++i) {
		const ass_t ass = gen_rnd_ass(N);
		print_ass_stat(ass);
		if (calc_e(e, ass)) {
			std::cout << "TRUE\n";
			return;
		}
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
