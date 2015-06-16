#include <array>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <vector>

#include <assert.h>

template <typename T>
class deque_simple {
	typedef std::vector<T> storage_t;
	typedef typename storage_t::iterator storage_iterator_t;

	storage_t storage_;
	storage_iterator_t data_begin_;
	storage_iterator_t data_end_;

public:
	deque_simple() noexcept
		: data_begin_(storage_.begin())
		, data_end_(storage_.end())
	{}

	void push_back(T new_value) {
		reserve_end();
		assert(data_end_ != storage_.end());
		*data_end_ = std::move(new_value);
		++data_end_;
	}

	void push_front(T new_value) {
		reserve_begin();
		assert(data_begin_ != storage_.begin());
		--data_begin_;
		*data_begin_ = std::move(new_value);
	}

	void pop_back() noexcept {
		assert(!empty());
		--data_end_;
		*data_end_ = T();
	}

	void pop_front() noexcept {
		assert(!empty());
		*data_begin_ = T();
		++data_begin_;
	}

	T& front() { assert(!empty()); return *data_begin_; }
	const T& front() const { assert(!empty()); return *data_begin_; }
	T& back() { assert(!empty()); return *(data_end_ - 1); }
	const T& back() const { assert(!empty()); return *(data_end_ - 1); }

	size_t size() const noexcept { return data_end_ - data_begin_; }
	bool empty() const noexcept { return data_begin_ == data_end_; }

	void clear() noexcept {
		storage_.clear();
		data_begin_ = storage_.begin();
		data_end_ = storage_.end();
	}

	void swap(deque_simple<T>& oth) noexcept {
		storage_.swap(oth.storage_);
		std::swap(data_begin_, oth.data_begin_);
		std::swap(data_end_, oth.data_end_);
	}

	T& at(size_t idx) {
		if (idx >= size())
			throw std::range_error("deque_simple::at()");
		const storage_iterator_t iter = data_begin_ + idx;
		assert(iter < data_end_);
		return *iter;
	}

private:
	void reserve_end() {
		if (storage_.end() == data_end_) {
			assert(storage_.begin() <= data_begin_);
			const auto shift = data_begin_ - storage_.begin();
			const auto data_size = this->size();
			storage_.resize(storage_.size() + reserve_size());
			data_begin_ = storage_.begin() + shift;
			data_end_ = data_begin_ + data_size;
		}
	}

	void reserve_begin() {
		if (data_begin_ == storage_.begin()) {
			const auto new_shift = reserve_size();
			const auto data_size = this->size();
			storage_t new_stor;
			new_stor.reserve(new_shift + storage_.size());
			new_stor.resize(new_shift); // reserved free space
			new_stor.insert(
				new_stor.end(),
				std::make_move_iterator(data_begin_),
				std::make_move_iterator(data_end_)
			);

			storage_.swap(new_stor);
			data_begin_ = storage_.begin() + new_shift;
			data_end_ = data_begin_ + data_size;
		}
	}

	size_t reserve_size() const { return 4; }
};

template <typename T>
class deque {
	static const size_t chunk_size_ = 32;
	typedef std::array<T, chunk_size_> chunk_t;
	typedef deque_simple<std::unique_ptr<chunk_t>> chunk_list_t;
	//typedef std::deque<std::unique_ptr<chunk_t>> chunk_list_t;
	typedef typename chunk_t::iterator data_iterator_t;
	chunk_list_t chunks_;
	data_iterator_t first_begin_;
	data_iterator_t last_end_;
public:
	bool empty() const { return chunks_.empty(); }

	void clear() { chunks_.clear(); }

	size_t size() const {
		assert_invariants();
		if (chunks_.empty())
			return 0;
		if (chunks_.size()==1)
			return last_end_ - first_begin_;

		//assert(chunks_.size()>=2);

		const chunk_t& first_chunk = *chunks_.front();
		const chunk_t& last_chunk = *chunks_.back();

		return
			(chunks_.size()-2) * chunk_size_ // size of full chunks in middle
			+ first_chunk.end() - first_begin_ // size of the first chunk
			+ last_end_ - last_chunk.begin(); // size of the last chunk
	}

	T& at(size_t const idx) {
		assert_invariants();
		chunk_t& first_chunk = *chunks_.at(0);
		const size_t shift = first_begin_ - first_chunk.begin();
		assert(shift < chunk_size_);

		{
			// idx is in first chunk?
			const size_t idx_first = idx + shift;
			if (idx_first < chunk_size_)
				return first_chunk.at(idx_first);
		}

		const size_t chunk_number = (idx - shift) / chunk_size_ + 1; // skip first chunk
		const size_t offset_in_chunk = (idx - shift) % chunk_size_;
		chunk_t& chunk = *chunks_.at(chunk_number);

		if (chunk_number == chunks_.size()-1) {
			assert(&chunk == &*chunks_.back());
			const size_t last_used = last_end_ - chunk.begin();
			if (offset_in_chunk >= last_used) {
				assert(idx >= this->size());
				throw std::range_error("");
			}
		}

		assert(idx < this->size());
		return chunk.at(offset_in_chunk);
	}

	void push_back(T new_item) {
		assert_invariants();
		if (chunks_.empty())
			reserve_first();
		else
			reserve_back();
		*last_end_ = std::move(new_item);
		++last_end_;
		assert_invariants();
	}

	void push_front(T new_item) {
		assert_invariants();
		if (chunks_.empty())
			reserve_first();
		else
			reserve_front();
		--first_begin_;
		*first_begin_ = std::move(new_item);
		assert_invariants();
	}

	void pop_front() {
		assert(!empty());
		assert_invariants();
		*first_begin_ = T();
		++first_begin_;
		if (first_begin_ == chunks_.front()->end()) {
			chunks_.pop_front();
			if (!chunks_.empty())
				first_begin_ = chunks_.front()->begin();
		}
		if (chunks_.size()==1 && first_begin_==last_end_) {
			chunks_.clear();
		}
		assert_invariants();
	}

	void pop_back() {
		assert(!empty());
		assert_invariants();
		--last_end_;
		*last_end_ = T();
		if (last_end_ == chunks_.back()->begin()) {
			chunks_.pop_back();
			if (!chunks_.empty())
				last_end_ = chunks_.back()->end();
		}
		if (chunks_.size()==1 && first_begin_==last_end_) {
			chunks_.clear();
		}
		assert_invariants();
	}

	T& front() {
		assert(!empty());
		assert_invariants();
		return *first_begin_;
	}

	T& back() {
		assert(!empty());
		assert_invariants();
		return *(last_end_ - 1);
	}

private:
	void reserve_first() {
		assert(chunks_.empty());
		chunks_.push_back(std::make_unique<chunk_t>());
		first_begin_ = last_end_ = chunks_.front()->begin() + (chunk_size_ / 2); // point to middle of the chunk
	}
	void reserve_back() {
		if (last_end_ == chunks_.back()->end()) {
			chunks_.push_back(std::make_unique<chunk_t>());
			last_end_ = chunks_.back()->begin();
		}
	}
	void reserve_front() {
		if (first_begin_ == chunks_.front()->begin()) {
			chunks_.push_front(std::make_unique<chunk_t>());
			first_begin_ = chunks_.front()->end();
		}
	}
	void assert_invariants() const {
		if (chunks_.empty())
			return;

		const chunk_t& first_chunk = *chunks_.front();
		assert(first_chunk.begin() <= first_begin_);
		assert(first_begin_ < first_chunk.end()); // should not be empty

		const chunk_t& last_chunk = *chunks_.back();
		assert(last_chunk.begin() < last_end_);
		assert(last_end_ <= last_chunk.end()); // should not be empty

		if (chunks_.size()==1) {
			assert(first_begin_ < last_end_); // should not be empty
		}
	}
};

