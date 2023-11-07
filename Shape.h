#pragma once
class Shape {
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
	};

	virtual ShapeType GetType() const = 0;
};

class ShapeSphere : public Shape {
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
	}

	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	float radius;
};

