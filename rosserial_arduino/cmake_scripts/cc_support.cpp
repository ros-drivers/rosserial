#include <stdlib.h>

#define __weak __attribute__((weak))

extern "C"
__weak
void __cxa_pure_virtual(void)
{
	for (;;)
		;
}

__weak
void *operator new(size_t size)
{
	return malloc(size);
}

__weak
void *operator new[] (size_t size)
{
	return ::operator new(size);
}

__weak
void operator delete(void *ptr)
{
	free(ptr);
}

__weak
void operator delete[](void *ptr)
{
	::operator delete(ptr);
}

