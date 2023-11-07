#include "Contact.h"

void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;
	const float invMassA = a->inverseMass;
	const float invMassB = b->inverseMass;
	const float elasticityA = a->elasticity;
	const float elasticityB = b->elasticity;
	const float elasticity = elasticityA * elasticityB;
	const Vec3 ptOnA = contact.ptOnAWorldSpace;
	const Vec3 ptOnB = contact.ptOnBWorldSpace;
	const Mat3 inverseWorldInertiaA = a->GetInverseInertiaTensorWorldSpace();
	const Mat3 inverseWorldInertiaB = b->GetInverseInertiaTensorWorldSpace();
	const Vec3 n = contact.normal;
	const Vec3 rA = ptOnA - a->GetCenterOfMassWorldSpace();
	const Vec3 rB = ptOnB - b->GetCenterOfMassWorldSpace();
	const Vec3 angularJA = (inverseWorldInertiaA * rA.Cross(n)).Cross(rA);
	const Vec3 angularJB = (inverseWorldInertiaB * rB.Cross(n)).Cross(rB);
	const float angularFactor = (angularJA + angularJB).Dot(n);
	// Get world space velocity of the motion and rotation
	const Vec3 velA = a->linearVelocity + a->angularVelocity.Cross(rA);
	const Vec3 velB = b->linearVelocity + b->angularVelocity.Cross(rB);
	// Collision impulse
	const Vec3& velAb = velA - velB;
	// -- Sign is changed here
	const float impulseValueJ = (1.0f + elasticity) * velAb.Dot(n) / (invMassA + invMassB + angularFactor);

	const Vec3 impulse = n * impulseValueJ;
	a->ApplyImpulse(ptOnA, impulse * -1.0f); // ...And here
	b->ApplyImpulse(ptOnB, impulse * 1.0f); // ...And here

	// Friction-caused impulse
	const float frictionA = a->friction;
	const float frictionB = b->friction;

	const float friction = frictionA * frictionB;
	// -- Find the normal direction of the velocity
	// -- with respect to the normal of the collision
	const Vec3 velNormal = n * n.Dot(velAb);
	// -- Find the tengent direction of the velocity
	// -- with respect to the normal of the collision
	const Vec3 velTengent = velAb - velNormal;
	// -- Get the tengential velocities relative to the other body
	Vec3 relativVelTengent = velTengent;
	relativVelTengent.Normalize();
	const Vec3 inertiaA = (inverseWorldInertiaA * rA.Cross(relativVelTengent)).Cross(rA);
	const Vec3 inertiaB = (inverseWorldInertiaB * rB.Cross(relativVelTengent)).Cross(rB);
	const float inverseInertia = (inertiaA + inertiaB).Dot(relativVelTengent);

	// -- Tengential impulse for friction
	const float reducedMass = 1.0f / (a->inverseMass + b->inverseMass + inverseInertia);
	const Vec3 impulseFriction = velTengent * reducedMass * friction;

	// -- Apply kinetic friction
	a->ApplyImpulse(ptOnA, impulseFriction * -1.0f);
	b->ApplyImpulse(ptOnB, impulseFriction * 1.0f);

	// If object are interpenetrating, use this to set them on contact
	if (contact.timeOfImpact == 0.0f)
	{
		const float tA = invMassA / (invMassA + invMassB);
		const float tB = invMassB / (invMassA + invMassB);
		const Vec3 d = ptOnB - ptOnA;
		a->position += d * tA;
		b->position -= d * tB;
	}
}

int Contact::CompareContact(const void* p1, const void* p2)
{
	const Contact& a = *(Contact*)p1;
	const Contact& b = *(Contact*)p1;

	if (a.timeOfImpact < b.timeOfImpact) 
	{
		return -1;
	}
	else if (a.timeOfImpact == b.timeOfImpact) 
	{
		return -0;
	}
	return 1;
}