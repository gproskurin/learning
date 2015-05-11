#include <algorithm>
#include <array>
#include <deque>
#include <future>
#include <iostream>
#include <iterator>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <assert.h>
#include <stdlib.h>
#include <unistd.h>

struct literal_t {
	bool negate;
	size_t varnum;
};

typedef std::array<literal_t, 2> clause_t;

typedef std::vector<clause_t> expr_t;

typedef std::vector<bool> ass_t;

typedef std::unordered_map<size_t, bool> fixed_ass_t;

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

template <typename C>
void calc_e_get_falses(const expr_t& e, const ass_t& ass, C& falses)
{
	for (size_t i=0; i<e.size(); ++i) {
		const clause_t& c = e.at(i);
		const bool c_val = calc_c(c, ass);
		if (c_val == false) {
			falses.push_back(i);
		}
	}
}

int32_t get_rnd(random_data* const rd_buf)
{
	int32_t rnd;
	const auto r = random_r(rd_buf, &rnd);
	assert(r==0);
	return rnd;
}

ass_t gen_rnd_ass(const size_t sz, random_data* const rd_buf, const fixed_ass_t& fix_ass)
{
	ass_t ass;
	ass.reserve(sz);
	for (size_t varnum=0; varnum<sz; ++varnum) {
		const auto fa_iter = fix_ass.find(varnum);
		if (fa_iter!=fix_ass.cend()) {
			ass.push_back(fa_iter->second);
		} else {
			ass.push_back(get_rnd(rd_buf) & 1);
		}
	}
	assert(ass.size()==sz);
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

class my_log {
	std::ostream& os_;
	std::ostringstream os_str_;
public:
	my_log(std::ostream& os) : os_(os) {}
	template <typename T>
	std::ostream& operator<<(const T& t) {
		return os_str_ << t;
	}
	~my_log() {
		os_ << os_str_.str() << std::flush;
	}
};

std::pair<expr_t, fixed_ass_t> expr_remove_unique_vars(expr_t e)
{
	typedef std::unordered_set<size_t> vars_set_t;

	fixed_ass_t fix_ass;

	expr_t new_e;
	vars_set_t pos, neg;

again:
	new_e.clear();
	pos.clear();
	neg.clear();

	for (const clause_t& c : e) {
		for (const literal_t& l : c) {
			if (l.negate) {
				neg.emplace(l.varnum);
			} else {
				pos.emplace(l.varnum);
			}
		}
	}

	for (const clause_t& c : e) {
		bool copy_cur_clause = true;
		for (const literal_t& l : c) {
			const size_t varnum = l.varnum;

			// all occurrences are negative (or none), fix var to be false
			if (pos.find(varnum)==pos.cend()) {
				fix_ass.emplace(varnum, false);
				copy_cur_clause = false;
				break;
			}

			// all occurrences are positive (or none), fix var to be true
			if (neg.find(varnum)==neg.cend()) {
				fix_ass.emplace(varnum, true);
				copy_cur_clause = false;
				break;
			}
		}
		if (copy_cur_clause)
			new_e.push_back(c);
	}


	assert(e.size() >= new_e.size());
	if (e.size() > new_e.size()) {
		e = std::move(new_e);
		goto again;
	}

	return std::make_pair(std::move(new_e), std::move(fix_ass));
}

bool papadimitriou_inner(const size_t ass_sz, const expr_t& e, const fixed_ass_t& fa, const size_t inner_iter)
{
	struct random_data rd_buf;
	char state[32];
	const auto seed = time(nullptr);// + std::this_thread::get_id();
	if (initstate_r(seed, state, sizeof(state), &rd_buf) != 0) {
		static const std::string err = "Error in srandom_r()";
		std::cerr << err << std::endl;
		throw std::runtime_error(err);
	}

	ass_t ass = gen_rnd_ass(ass_sz, &rd_buf, fa);

	for (size_t j=0; j<inner_iter; ++j) {
		std::deque<size_t> falses;
		calc_e_get_falses(e, ass, falses);
		if (falses.empty()) {
			print_ass_stat(ass);
			assert(calc_e(e, ass) == true);
			if (calc_e(e, ass) != true) {
				my_log(std::cout) << "BAD_PROGRAMMER\n";
				throw std::runtime_error("bad_programmer");
			}
			my_log(std::cout) << "TRUE!!!\n";
			return true;
		}
		if (j % 10 == 0) {
			my_log(std::cout)
				<< "iter:" << j << "/" << inner_iter
				<< " false_clauses:" << falses.size() << "\n";
		}
		// negate variables for random false clause
		const size_t rnd = static_cast<size_t>(get_rnd(&rd_buf)) % falses.size();
		const size_t false_c_pos = falses.at(rnd);
		const clause_t& false_c = e.at(false_c_pos);
		assert(calc_c(false_c, ass) == false);
		for (const literal_t& l : false_c) {
			const size_t varnum = l.varnum;
			ass_t::reference b = ass.at(varnum);
			b = !b;
		}
	}
	return false;
}

bool papadimitriou_parallel(size_t ass_sz, const expr_t& e, const fixed_ass_t& fa)
{
	const size_t outer_iter = 100;
	const size_t inner_iter = 2*e.size()*e.size();
	std::deque< std::future<bool> > fut;
	for (size_t i=0; i<outer_iter; ++i) {
		fut.emplace_back(
			std::async(
				std::launch::async,
				papadimitriou_inner, 
				ass_sz,
				std::cref(e),
				std::cref(fa),
				inner_iter
			)
		);
	}

	while (!fut.empty()) {
		auto& f = fut.front();
		const bool res = f.get();
		if (res==true)
			return true;
		fut.pop_front();
	}
	my_log(std::cout) << "ALL_FALSE\n";
	return false;
}

void run()
{
	const auto p = load_expr(std::cin);
	const expr_t& e = p.first;
	const size_t N = p.second;
	print_stat(e);
	std::cout << "\n";

	const auto e_fa = expr_remove_unique_vars(e);
	const expr_t& new_e = e_fa.first;
	const fixed_ass_t& fa = e_fa.second;
	print_stat(new_e);
	std::cout << "\n";

	const bool res = papadimitriou_parallel(N, new_e, fa);
	if (res)
		std::cout << "Result: TRUE!!!\n";
	else
		std::cout << "Result: may be false...\n";
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
