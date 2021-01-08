// This is in the debugging stage. This code doesn't need to be neat.
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "PlatformPhysics.h"
#include "MyMathUtils.h"
#include "Stopwatch.h"

#include<chrono>
#include<thread>

using namespace JesseRussell;
using namespace JesseRussell::vectors;
using namespace JesseRussell::Diagnostics;

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
	phy::DynamicEntity* block;
	fvector2 createPointA;
	std::chrono::milliseconds drawTime = std::chrono::milliseconds(10);
	Stopwatch drawWatch;

public:
	void DrawEntity(const phy::CollisionBox& e, olc::Pixel color = olc::WHITE) {
		DrawRect((int32_t)e.Position().x, (int32_t)e.Position().y, (int32_t)e.Size().x, (int32_t)e.Size().y, color);
	}

public:
	bool OnUserCreate() override
	{
		player = new Actor({ 100, 150 }, { 20, 20 }, 100);
		block = new phy::DynamicEntity({ 120, 100 }, { 20, 20 }, 10);
		engine.AddEntity(player);
		//engine.AddEntity(block);

		//// Newton's cradle:
		//engine.AddEntity(new phy::DynamicEntity({ 140,100 }, { 20, 20 }, 10));
		//engine.AddEntity(new phy::DynamicEntity({ 160,100 }, { 20, 20 }, 10));
		//engine.AddEntity(new phy::DynamicEntity({ 180,100 }, { 20, 20 }, 10));
		//engine.AddEntity(new phy::DynamicEntity({ 200,100 }, { 20, 20 }, 10));
		//phy::DynamicEntity newtonHammer({ 370, 100 }, { 20, 20 }, 10);
		//phy::DynamicEntity newtonHammer2({ 350, 100 }, { 20, 20 }, 10);
		//newtonHammer.Velocity({-200, 0});
		//newtonHammer2.Velocity({-200, 0});
		//engine.AddEntity(new phy::DynamicEntity(newtonHammer));
		//engine.AddEntity(new phy::DynamicEntity(newtonHammer2));

		engine.AddEntity(new phy::Entity({ 0, 390 }, { 399, 9 }));
		engine.AddEntity(new phy::Entity({ 390, 0 }, { 9, 390 }));
		engine.AddEntity(new phy::Entity({ 0, 0 }, { 9, 390 }));
		engine.AddEntity(new phy::Entity({ 9, 0 }, { 381, 9 }));
		//phy::MovableEntity movable = phy::MovableEntity({ 100, 200 }, { 20, 20 });
		//movable.Velocity({ 50, 0 });
		//engine.AddEntity(new phy::MovableEntity(movable));


		drawWatch.start();
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		if (GetMouse(0).bPressed || GetMouse(1).bPressed) createPointA = { (float)GetMouseX(), (float)GetMouseY() };

		if (GetMouse(0).bReleased) {
			phy::DynamicEntity* newent = new phy::DynamicEntity(phy::DynamicEntity::ByPoints(createPointA, fvector2((float)GetMouseX(), (float)GetMouseY()), player->Mass()));
			//newent->Mass(newent->Volume() * 0.01);
			engine.AddEntity(newent);
		}

		if (GetMouse(1).bReleased) engine.AddEntity(
			new phy::Entity(phy::Entity::ByPoints(createPointA, fvector2((float)GetMouseX(), (float)GetMouseY())))
		);

		float pushForce = 30000;
		float jumpForce = 1000000;
		if (GetKey(olc::UP).bHeld) player->AddForce({ 0, -pushForce });
		if (GetKey(olc::DOWN).bHeld) player->AddForce({ 0, pushForce });
		if (GetKey(olc::LEFT).bHeld) player->AddForce({ -pushForce, 0 });
		if (GetKey(olc::RIGHT).bHeld) player->AddForce({ pushForce, 0 });

		engine.Update(Cardinal::EAST, fElapsedTime);
		engine.Update(Cardinal::SOUTH, fElapsedTime);



		if (drawWatch.getElapsed() > drawTime) {
			drawWatch.restart();

			// o----------o
			// | Drawing: |
			// o----------o
			Clear(olc::BLACK);

			auto iter = engine.Entities_cbegin();
			while (iter != engine.Entities_cend()) {
				DrawEntity(**iter);
				iter++;
			}

			DrawString(10, 10, "velocity:" + player->Velocity().ToString(), olc::Pixel(0xff00ffff));
			DrawString(10, 20, "velocity:" + block->Velocity().ToString(), olc::Pixel(0xff00ffff));
			auto cent = player->GetCenter();
			DrawCircle(cent.x, cent.y, cmp::min(player->Width(), player->Height()) / 2);
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