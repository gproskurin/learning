#include <algorithm>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <assert.h>

struct job_t {
	long w;
	long l;
	job_t(long ww, long ll) : w(ww), l(ll) {}
	long prio_diff_wl() const { return w - l; }
};
typedef std::vector<job_t> all_jobs_t;
typedef std::vector<all_jobs_t::size_type> indexes_t;

long weighted_sum(const all_jobs_t& jobs, const indexes_t& idx)
{
	long end_time = 0;
	long wsum = 0;
	size_t cur = 0;
	for (auto i : idx) {
		const job_t& j = jobs.at(i);
		end_time += j.l;
		//std::cout << "job " << cur << ", idx:" << i << " w:" << j.w << " l:" << j.l << " end:" << end_time << " delta_wsum:" << (j.w * end_time) << "\n";
		wsum += j.w * end_time;
		++cur;
	}
	return wsum;
}

all_jobs_t load_jobs(std::istream& is)
{
	all_jobs_t res;
	size_t num;
	is >> num;
	res.reserve(num);
	for (size_t j=1; j<=num; ++j) {
		long w, l;
		is >> w >> l;
		if (w<0 || l<0)
			throw std::runtime_error("Negative");
		res.emplace_back(w,l);
	}
	assert(res.size() == num);
	if (!is)
		throw std::runtime_error("Unexpected EOF");
	char check;
	is >> check;
	if (is)
		throw std::runtime_error("No expected EOF");
	return res;
}



void run(const all_jobs_t& jobs)
{
	indexes_t idx1;
	idx1.reserve(jobs.size());
	for (size_t i=0; i<jobs.size(); ++i) {
		idx1.push_back(i);
	}

	auto idx2 = idx1;

	std::cout << "wsum0(orig): " << weighted_sum(jobs, idx1) << "\n";

	std::sort(
		idx1.begin(),
		idx1.end(),
		[&jobs](size_t i1, size_t i2) {
			const auto& j1 = jobs.at(i1);
			const auto& j2 = jobs.at(i2);
			const auto wl1 = j1.w - j1.l;
			const auto wl2 = j2.w - j2.l;
			if (wl1 > wl2)
				return true;
			if (wl1 < wl2)
				return false;
			return (j1.w > j2.w);
		}
	);

	std::cout << "wsum1(diff): " << weighted_sum(jobs, idx1) << "\n";

	std::sort(
		idx2.begin(),
		idx2.end(),
		[&jobs](size_t i1, size_t i2) {
			const auto& j1 = jobs.at(i1);
			const auto& j2 = jobs.at(i2);
			const auto w1l2 = j1.w * j2.l;
			const auto w2l1 = j2.w * j1.l;
			if (w1l2 > w2l1)
				return true;
			if (w1l2 < w2l1)
				return false;
			return (j1.w > j2.w);
		}
	);
	std::cout << "wsum2(div) : " << weighted_sum(jobs, idx2) << "\n";
}

int main()
{
	try {
		auto jobs = load_jobs(std::cin);
		run(jobs);
	} catch (const std::exception& e) {
		std::cerr << "Ex: " << e.what() << "\n";
		return 1;
	}
	return 0;
}
