#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "PlatformPhysics.h"
#include "MyMathUtils.h"

#include<chrono>
#include<thread>

using namespace JesseRussell;
using namespace JesseRussell::vectors;

class Actor : public phy::DynamicEntity {
public:
	Actor(const fvector2& pointA, const fvector2& pointB, float mass) : DynamicEntity(pointA, pointB, mass){}
};

// Override base class with your custom functionality
class Example : public olc::PixelGameEngine
{
public:
	Example()
	{
		// Name you application
		sAppName = "Example";
	}

public:
	phy::Engine engine;
	Actor *player;
	fvector2 createPointA;

public:
	void DrawEntity(const phy::CollisionBox& e, olc::Pixel color = olc::WHITE) {
		DrawRect((int32_t)e.GetPosition().x, (int32_t)e.GetPosition().y, std::abs((int32_t)e.GetSize().x), std::abs((int32_t)e.GetSize().y), color);
	}

public:
	bool OnUserCreate() override
	{
		player = new Actor({ 100, 100 }, { 120, 101 }, 5);
		engine.AddEntity(new phy::CollisionBox({0, 390}, {399, 399}));
		engine.AddEntity(new phy::CollisionBox({390, 0}, {399, 390}));
		engine.AddEntity(player);
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{

		if (GetMouse(0).bPressed) createPointA = { (float)GetMouseX(), (float)GetMouseY() };

		if (GetMouse(0).bReleased) engine.AddEntity
		(new phy::CollisionBox(createPointA, fvector2((float)GetMouseX(), (float)GetMouseY())));


		if (GetKey(olc::UP).bHeld) player->AddForce({ 0, -10 });
		if (GetKey(olc::DOWN).bHeld) player->AddForce({ 0, 10 });
		if (GetKey(olc::LEFT).bHeld) player->AddForce({ -10, 0 });
		if (GetKey(olc::RIGHT).bHeld) player->AddForce({ 10, 0 });


		player->AddForce({ 0, player->GetVelocity().y * -0.01f });
		player->AddForce({ player->GetVelocity().x * -0.01f, 0 });

		player->ApplyNetForce(phy::Cardinal::EAST);
		engine.RunCollisions(fElapsedTime, phy::Cardinal::EAST);
		player->ApplyVelocity(fElapsedTime, phy::Cardinal::EAST);
		player->ClearNetForce(phy::Cardinal::EAST);

		player->ApplyNetForce(phy::Cardinal::SOUTH);
		engine.RunCollisions(fElapsedTime, phy::Cardinal::SOUTH);
		player->ApplyVelocity(fElapsedTime, phy::Cardinal::SOUTH);
		player->ClearNetForce(phy::Cardinal::SOUTH);

		Clear(olc::BLACK);

		auto iter = engine.entities_cbegin();
		while (iter != engine.entities_cend()) {
			DrawEntity(**iter);
			iter++;
		}
		

		return true;
	}
};

int main()
{
	Example demo;
	if (demo.Construct(400, 400, 1, 1))
		demo.Start();
	return 0;
}