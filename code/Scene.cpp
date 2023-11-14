#include "Scene.h"
#include <thread>
#include "../Intersection.h"
#include "../Shape.h"
#include "../Broadphase.h"
#include <iostream>

Scene::~Scene()
{
	for (int i = 0; i < bodies.size(); i++)
	{
		delete bodies[i].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset()
{
	for (int i = 0; i < bodies.size(); i++)
	{
		delete bodies[i].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
	srand(time(NULL));

	Body body;
	for (int i = 0; i < 7; i++)
	{
		/* Sphere */
		body.position = Vec3(1, 1, 2.5);
		body.orientation = Quat(0, 0, 0, 1);

		if (i == 0)
		{
			body.shape = new ShapeSphere(.4f);
			body.inverseMass = 0.9f;
			body.friction = 1.f;

		}
		else
		{
			body.shape = new ShapeSphere(1.0f);
			body.inverseMass = 0.5f;
			body.friction = .8f;
		}

		body.elasticity = 0.1f;
		bodies.push_back(body);
	}

	float incrementalAngle = 0;
	float radiusArena = 5;
	float gap = 9;
	float n_balls = 30;

	/* Arena */
	for (int i = 0; i < n_balls; i++)
	{
		body.position = Vec3(cos(incrementalAngle) * radiusArena * gap, sin(incrementalAngle) * radiusArena * gap, 0);
		body.orientation = Quat(0, 0, 0, 1);
		body.shape = new ShapeSphere(radiusArena);
		body.inverseMass = 0.00f;
		body.elasticity = 0.5f;
		body.friction = 0.05f;
		body.linearVelocity = Vec3(0, 0, 0);
		incrementalAngle += 2 * 3.14159265 / n_balls;
		bodies.push_back(body);
	}

	/* Earth */
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float radius = 800.0f;
			float x = (i - 1) * radius * 0.25f;
			float y = (j - 1) * radius * 0.25f;
			body.position = Vec3(x, y, -radius);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 0.0f;
			body.elasticity = 0.99f;
			body.friction = 0.5f;
			bodies.push_back(body);
		}
	}
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
	// Gravity
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);

		if (body.elasticity == 0.1f)
		{
			float length = body.linearVelocity.GetLengthSqr();

			if (length > 0.05f)
			{
				body.linearVelocity *= 0.99f;
				body.angularVelocity *= 0.98f;
			}
			else
			{
				body.linearVelocity = Vec3(0, 0, 0);
				body.angularVelocity = Vec3(0, 0, 0);
			}
		}
	}

	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);

	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)malloc(sizeof(Contact) * maxContacts);

	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		Contact contact;

		if (Intersection::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}
	// Sort times of impact
	if (numContacts > 1)
	{
		qsort(contacts, numContacts, sizeof(Contact),
			Contact::CompareContact);
	}
	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;

		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j)
		{
			bodies[j].Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}

	free(contacts);

	// Other physics behaviours, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i)
		{
			bodies[i].Update(timeRemaining);
		}
	}
}

void Scene::ThrowBallPetanque(Vec3 positionP, Vec3 cameraRotation)
{
	if (round >= 7)
	{
		round = 0;
	}

	Body& body = bodies[round];
	if (body.elasticity == 0.1f)
	{
		body.position = positionP;
		body.linearVelocity = cameraRotation * power;
		body.angularVelocity = Vec3(0, -4, 0);
	}
	round++;
}

void Scene::SetPower(float amountP)
{
	power += amountP;

	if (power < .5f)
		power = 0.5f;

	std::cout << power << std::endl;
}