#ifndef COURSERA_LIB_SETS_H_INCLUDED
#define COURSERA_LIB_SETS_H_INCLUDED

#include <algorithm>
#include <unordered_set>
#include <vector>
#include <iterator>
#include <iosfwd>

#include <assert.h>

template <typename set_id_t>
class sets_t {
	const set_id_t min_item_;
	const set_id_t max_item_;
	size_t size_() const { return max_item_ - min_item_ + 1; }

	typedef std::vector<set_id_t> membership_t;
	membership_t m_;

	size_t set_to_idx(set_id_t s) const {
		assert(s >= min_item_);
		assert(s <= max_item_);
		const size_t idx = s - min_item_;
		return idx;
	}
	set_id_t idx_to_set(size_t idx) const {
		const set_id_t s = idx + min_item_;
		assert(s >= min_item_);
		assert(s <= max_item_);
		return s;
	}

public:
	sets_t(set_id_t min_item, set_id_t max_item) : min_item_(min_item), max_item_(max_item)
	{
		m_.reserve(size_());
		for (size_t idx=0; idx<size_(); ++idx) {
			m_.push_back(idx_to_set(idx));
		}
	}

	void _reparent_set_to(const set_id_t s, const set_id_t new_parent)
	{
		const set_id_t root = get_root(s);
		const size_t root_idx = set_to_idx(root);
		m_.at(root_idx) = new_parent;
	}

	void union_sets(const set_id_t s1, const set_id_t s2)
	{
		_reparent_set_to(s1, s2);
	}

	bool are_separated(const set_id_t s1, const set_id_t s2) const
	{
		return get_root(s1) != get_root(s2);
	}

	size_t count_sets() const
	{
		std::unordered_set<set_id_t> roots;
		std::transform(
			m_.cbegin(),
			m_.cend(),
			std::inserter(roots, roots.begin()),
			[this](set_id_t s) { return get_root(s); }
		);
		return roots.size();
	}

	// return true if count_sets() is greater then k (optimized)
	bool count_sets_gt(size_t k) const
	{
		assert(k>0);
		std::unordered_set<set_id_t> roots;
		for (const set_id_t s : m_) {
			roots.insert(get_root(s));
			if (roots.size() > k)
				return true;
		}
		return false;
	}

	void print(std::ostream& os)
	{
		os << " size:" << m_.size() << " : ";
		for (const auto s : m_) {
			os << " " << s;
		}
		os << "\n";
		os << "roots:" << count_sets() << " : ";
		for (const auto s : m_) {
			os << " " << get_root(s);
		}
		os << "\n\n";
	}

private:
	set_id_t get_root(set_id_t s) const
	{
		set_id_t par = m_.at(set_to_idx(s));
		while (par != s) {
			s = par;
			par = m_.at(set_to_idx(s));
		}
		return par;
	}
};

#endif
