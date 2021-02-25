#include "sun_vector_reference.h"
#include <iostream>
int main() {
	
	int Y, M, D;
	double UT;

	Y = 2019;
	M = 11;
	D = 27;
	UT = 14.55365;

	std::cout << sun_vector_reference(Y, M, D, UT);
}