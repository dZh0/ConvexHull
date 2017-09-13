// Dummy spaceship thruster implementation

#pragma once
#ifndef _IOSTREAM_
#include <iostream>
#endif // !_IOSTREAM_

#include "FVector.h"

struct AThruster {
	std::string name;
	FVector position;
	FVector thrust;
};