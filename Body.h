#pragma once
#include "code/Math/Vector.h"
#include "code/Renderer/model.h"
#include "code/Math/Quat.h"

class Body
{
public:
	Vec3 position;
	Quat orientation;
	Shape* shape;
};

