#include <cstdlib>
#include <cstdint>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

typedef uint32_t item_t;
typedef std::vector<item_t> data_t;


template <typename T>
void write_output(const T& data, std::ostream& os)
{
    for(const auto& i : data) {
        os << " " << std::hex << i;
    }
    os << "\n";
}


item_t get_byte_0(const item_t& item)
{
    return (item & 0xff000000) >> 24;
}

item_t get_byte_1(const item_t& item)
{
    return (item & 0xff0000) >> 16;
}

item_t get_byte_2(const item_t& item)
{
    return (item & 0xff00) >> 8;
}

item_t get_byte_3(const item_t& item)
{
    return item & 0xff;
}


template <typename T, typename C, typename F>
void radix_sort_count(const T& data, F&& func, C& counters)
{
    for (const auto& i : data) {
        const auto p = func(i);
        assert(p >= 0 && p < counters.size());
        ++counters[p];
    }
}


template<typename C>
void prefix_sum(C& c)
{
    auto p = c.begin();
    if (p == c.end())
        return;
    auto i = p;
    ++i;
    while (i != c.end()) {
        *i += *p;
        p = i;
        ++i;
    }
}


template <typename T, typename C, typename F>
void radix_sort_copy(const T& data, T& dest, C& counters, F&& func)
{
    for (auto i = data.crbegin(); i != data.crend(); ++i) {
        const auto item = func(*i);

        assert(item >= 0 && item <= 255);
        auto& psum = counters[item];

        assert(psum > 0);
        --psum;

        assert(psum < dest.size());
        dest[psum] = *i;
    }
}


template <typename T, typename F>
void radix_sort_step(T& data, T& tmp, F&& func)
{
    std::array<unsigned long, 256> counters;
    std::fill(counters.begin(), counters.end(), 0);

    radix_sort_count(data, std::forward<F>(func), counters);
    prefix_sum(counters);

    radix_sort_copy(data, tmp, counters, std::forward<F>(func));
    using std::swap;
    swap(data, tmp);
}


template <typename T>
void radix_sort(T& data)
{
    T tmp(data.size());
    radix_sort_step(data, tmp, get_byte_3);
    radix_sort_step(data, tmp, get_byte_2);
    radix_sort_step(data, tmp, get_byte_1);
    radix_sort_step(data, tmp, get_byte_0);
}


template <typename T>
void check_sorted(const T& data)
{
    auto b = data.cbegin();
    if (b == data.cend())
        return;
    auto e = b;
    ++e;
    while (e != data.cend()) {
        //std::cout << "b=" << *b << " e=" << *e << "\n";
        if (*b > *e)
            throw std::runtime_error("Unsorted");
        b = e;
        ++e;
    }
}


data_t read_input()
{
    //return data_t{0x45000000, 0x45123456, 0x11123456, 0x11123465};
    data_t d(10000000);
    for (auto& i : d) {
        i = random();
        i ^= (random() & 0xff) << 24; // randomize first byte
    }
    return d;

    //return data_t{1,2,3,4,5};
    //return data_t{3,1,4};
    //return data_t{5};
    //return data_t{0xa1b2c3d4};
}


int main()
{
    auto data = read_input();

    data_t sorted(data);
    std::sort(sorted.begin(), sorted.end());

    //write_output(data, std::cout);
    radix_sort(data);
    //check_sorted(data);
    //write_output(data, std::cout);
    if (sorted != data)
        throw std::runtime_error("Not equal");
}

