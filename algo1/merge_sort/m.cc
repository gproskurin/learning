#include "merge_sort.h"

#include <functional>
#include <iostream>
#include <iterator>
#include <vector>

template <typename C>
void print(const C& c, std::ostream& os)
{
	std::copy(c.cbegin(), c.cend(), std::ostream_iterator<typename C::value_type>(os, " "));
	os << "\n";
}

struct Point {
	double x;
	double y;
	Point(double xx, double yy) : x(xx), y(yy) {}
};

std::ostream& operator<<(std::ostream& os, const Point& p)
{
	os << "(" << p.x << "," << p.y << ")";
	return os;
}

bool pnt_less_x(const Point& p1, const Point& p2) { return p1.x < p2.x; }
bool pnt_less_y(const Point& p1, const Point& p2) { return p1.y < p2.y; }

int main()
{
	typedef std::vector<Point> pts_t;

	pts_t p1 = { Point(1,2), Point(3,4), Point(2,5), Point(7,2), Point(5,3) };
	//std::random_shuffle(p1.begin(), p1.end());
	print(p1, std::cout);

	pts_t vs;
	merge_sort(p1.cbegin(), p1.cend(), std::back_inserter(vs), pnt_less_x);
	print(vs, std::cout);
}
