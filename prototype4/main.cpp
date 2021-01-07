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
		DrawRect((int32_t)e.Position().x, (int32_t)e.Position().y, (int32_t)e.Size().x, (int32_t)e.Size().y, color);
	}

public:
	bool OnUserCreate() override
	{
		player = new Actor({ 100, 100 }, { 20, 20 }, 10);
		player->Bounce(-0.5);
		player->Drag(0.01);
		engine.AddEntity(new phy::CollisionBox({ 0, 390 }, { 399, 9 }));
		engine.AddEntity(new phy::CollisionBox({ 390, 0 }, { 9, 390 }));
		engine.AddEntity(new phy::CollisionBox({ 0, 0 }, { 9, 390 }));
		engine.AddEntity(new phy::CollisionBox({ 9, 0 }, { 381, 9 }));
		engine.AddEntity(player);
		engine.Gravity({ 0, 9.8 * 20 });
		engine.AirDensity(0.05);
		
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		
		if (GetMouse(0).bPressed) createPointA = { (float)GetMouseX(), (float)GetMouseY() };

		if (GetMouse(0).bReleased) engine.AddEntity(
			new phy::CollisionBox(phy::CollisionBox::ByPoints(createPointA, fvector2((float)GetMouseX(), (float)GetMouseY())))
		);

		float pushForce = 3000;
		float jumpForce = 1000000;
		if (GetKey(olc::UP).bHeld) player->AddForce({ 0, -pushForce });
		if (GetKey(olc::DOWN).bHeld) player->AddForce({ 0, pushForce });
		if (GetKey(olc::LEFT).bHeld) player->AddForce({ -pushForce, 0 });
		if (GetKey(olc::RIGHT).bHeld) player->AddForce({ pushForce, 0 });
		if (GetKey(olc::SPACE).bPressed && player->Touching(phy::Cardinal::SOUTH)) player->AddForce({ 0, -jumpForce});

		engine.TimeScale(fElapsedTime);
		engine.Update();


		// o----------o
		// | Drawing: |
		// o----------o

		Clear(olc::BLACK);

		auto iter = engine.entities_cbegin();
		while (iter != engine.entities_cend()) {
			DrawEntity(**iter);
			iter++;
		}

		DrawString(10, 10, "velocity:" + player->Velocity().toString(), olc::Pixel(0xff00ffff));

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