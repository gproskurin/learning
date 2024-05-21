#include <stddef.h>


void* memset(void *dst, int c, size_t n)
{
	for (size_t i=0; i<n; ++i) {
		((char*)dst)[i] = c;
	}
	return dst;
}

void* memcpy(void* restrict dst, const void* restrict src, size_t n)
{
	for (size_t i=0; i<n; ++i) {
		((char*)dst)[i] = ((const char*)src)[i];
	}
	return dst;
}

char *stpcpy(char *dst, const char *src)
{
	while ((*dst++ = *src++) != 0) {}
	return dst-1;
}

size_t strlen(const char* s)
{
	size_t r = 0;
	while (*s) {
		++s;
		++r;
	}
	return r;
}

