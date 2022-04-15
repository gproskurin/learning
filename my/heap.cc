#include <iostream>
#include <vector>

namespace my_heap{

template <typename Iter>
void push_heap(Iter begin, Iter end)
{
    if (end - begin <= 1)
        return;
    auto cur_idx = end - begin - 1;
    while (1) {
        const auto parent_idx = (cur_idx-1) / 2;
        if (! (*(begin+parent_idx) < *(begin+cur_idx)))
            return;
        std::iter_swap(begin+parent_idx, begin+cur_idx);
        if (parent_idx == 0)
            return;
        cur_idx = parent_idx;
    }
}


template <typename Iter>
void pop_heap(Iter first, Iter last)
{
    if (last - first <= 1)
        return;
    --last;
    std::iter_swap(first, last);

    // TODO
    // sift down first element
    //{
    //    const auto size = last - first - 1;
    //    auto ch1 = 
}


} // namespace my_heap


template <typename T>
void print(const T& a)
{
    std::cout << "[" << a.size() << "]";
    for (const auto& i : a) {
        std::cout << " " << i;
    }
    std::cout << "\n";
}


int main()
{
    std::vector<int> v;

    std::cout << "init\n";
    print(v);

    std::cout << "push_heap empty";
    my_heap::push_heap(v.begin(), v.end());
    print(v);

    std::cout << "push_heap first";
    print(v);
    v.push_back(20);
    print(v);
    std::cout << "---\n";

    v.push_back(5);
    print(v);
    my_heap::push_heap(v.begin(), v.end());
    print(v);

    v.push_back(10);
    print(v);
    my_heap::push_heap(v.begin(), v.end());
    print(v);
}
