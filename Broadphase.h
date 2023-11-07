#pragma once
#include <vector>
#include "Body.h"

struct CollisionPair
{
	int a;
	int b;

	bool operator==(const CollisionPair& rhs) const 
	{
		return (((a == rhs.a) && (b == rhs.b)) || ((a == rhs.b) && (b == rhs.a)));
	}
	bool operator!=(const CollisionPair& rhs) const 
	{
		return !(*this == rhs);
	}
};

struct PseudoBody
{
	int id;
	float value;
	bool ismin;
};

void BroadPhase(const Body* bodies, const int num, std::vector<CollisionPair>& finalPairs, const float dt_sec);
