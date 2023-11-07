#pragma once
#include "code/Math/Vector.h"
#include "Body.h"

class Contact
{
public:

	Vec3 ptOnAWorldSpace;
	Vec3 ptOnALocalSpace;
	Vec3 ptOnBWorldSpace;
	Vec3 ptOnBLocalSpace;
	Vec3 normal;

	float separationDistance;
	float timeOfImpact;

	Body* a{ nullptr };
	Body* b{ nullptr };

	static void ResolveContact(Contact& contact);
	static int CompareContact(const void* p1, const void* p2);
};