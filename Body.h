#pragma once
#include "code/Math/Vector.h"
#include "code/Renderer/model.h"
#include "code/Math/Quat.h"

class Body
{
public:

	void Update(const float dt_sec);

	Vec3 position;
	Quat orientation;
	Vec3 linearVelocity;
	Vec3 angularVelocity;
	float inverseMass;
	float elasticity;
	float friction;
	Shape* shape;

	void ApplyImpulseLinear(const Vec3& impulse);
	void ApplyImpulseAngular(const Vec3& impulse);

	/// <summary>
	/// Apply impulse on a specific world space
	/// </summary>
	/// <param name="impulsePoint">
	/// The world space location of the application of the impulse
	/// </param>
	/// <param name="impulse">
	/// The world space direction and magnitude of the impulse
	///</param>
	void ApplyImpulse(const Vec3 & impulsePoint, const Vec3 & impulse);


	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassBodySpace() const;
	Vec3 WorldSpaceToBodySpace(const Vec3& worldPoint);
	Vec3 BodySpaceToWorldSpace(const Vec3& bodyPoint);

	Mat3 GetInverseInertiaTensorBodySpace() const;
	Mat3 GetInverseInertiaTensorWorldSpace() const;
};

