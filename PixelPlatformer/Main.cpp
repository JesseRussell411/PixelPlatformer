#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "PlatformPhysics.h"
#include "Stopwatch.h"
#include "MyMathUtils.h"

using namespace JesseRussell;
using namespace JesseRussell::vectors;
using namespace JesseRussell::Diagnostics;

#include <thread>
#include <chrono>
#include <iostream>

class Player : public phy::DynamicEntity {
public:
	Player(fvector2 position, fvector2 size, float mass)
		: phy::DynamicEntity(position, phy::Box(size), mass) {}
public:
	void pre_update_horizontal(phy::Engine& engine) override {
		updateVelocity_horizontal(engine.getTimeScale());
	}
	void post_update_horizontal(phy::Engine& engine) override {

		updatePosition_horizontal(engine.getTimeScale());
	}
	void pre_update_vertical(phy::Engine& engine) override {
		updateVelocity_vertical(engine.getTimeScale());
	}

	void post_update_vertical(phy::Engine& engine) override {

		updatePosition_vertical(engine.getTimeScale());

		// update overFloor:
		canJump = false;
		auto iter = engine.entities_cbegin();
		do {
			if (*iter == this) continue;
			const phy::DynamicEntity& e = **iter;
			if (phy::vectorRangeIntersection(getPosition(), getPosition2().plusY(5), e.getPosition(), e.getPosition2())) {
				canJump = true;
				break;
			}
		} while (++iter != engine.entities_cend());
	}
	void post_update(phy::Engine& engine) override {}

public:
	bool CanJump() const { return canJump; }
	void CanJump(bool value) { canJump = value; }

private:
	bool canJump = false;
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
	bool OnUserCreate() override
	{
		// Called once at the start, so create things here
		player = new Player({ 200, 200 }, { 20, 20 }, 30);
		player->setBounciness(.2);
		engine.entities_add(player);
		engine.entities_add(new phy::DynamicEntity({ 0,390 }, phy::Box(400, 10)));
		engine.entities_add(new phy::DynamicEntity({ 0,0 }, phy::Box(10, 400)));
		engine.entities_add(new phy::DynamicEntity({ 390,0 }, phy::Box(10, 400)));
		engine.entities_add(new phy::DynamicEntity({ 10, 310 }, phy::Box(100, 10)));
		return true;
	}

	fvector2 createA, createB;
	float moveSpeed = 80;
	phy::Engine engine;
	Player* player;

	bool OnUserUpdate(float fElapsedTime) override
	{


		//fElapsedTime = 0.01;

		//std::this_thread::sleep_for(std::chrono::milliseconds(200));
		Clear(olc::BLACK);
		player->setNetForce({ 0, 0 });
		// add gravity.
		player->addForce({ 0, 30 });
		if (player->getVelocity_x() < 0) {
			bool trash = true;
		}

		// add air resistance.
		player->addForce(player->getVelocity()
			.timesX(std::abs(player->getVelocity_x()))
			.timesY(std::abs(player->getVelocity_y())) 
			* 0.5 * -0.0005
		);
		

		// add controls:
		bool moveLeft = GetKey(olc::LEFT).bHeld;
		bool moveRight = GetKey(olc::RIGHT).bHeld;
		if (moveLeft) player->subtractForce_x(player->isTouchingSouth() ? moveSpeed : moveSpeed * 0.3f);
		if (moveRight) player->addForce_x(player->isTouchingSouth() ? moveSpeed : moveSpeed * 0.3f);
		if (GetKey(olc::SPACE).bPressed) {
			if (player->CanJump()) {
				player->CanJump(false);
				player->addForce({ 0, -9600});
			}
		}


		if (GetMouse(0).bPressed) {
			createA = fvector2(GetMouseX(), GetMouseY());
		}

		if (GetMouse(0).bReleased) {
			createB = fvector2(GetMouseX(), GetMouseY());

			engine.entities_add(new phy::DynamicEntity(createA, createB - createA));
		}

		if (GetMouse(0).bHeld) {
			DrawRect(createA.x, createA.y, GetMouseX() - createA.x, GetMouseY() - createA.y, olc::GREEN);
		}

		if (GetKey(olc::R).bPressed) {
			player->setPosition({100,100});
		}
		//
		
		fvector2 player_force = player->getNetForce();

		engine.update(fElapsedTime);


		//// add friction:
		//if (!moveLeft && !moveRight) {
		//	if (player->isTouchingSouth() || player->isTouchingNorth()) {
		//		float frictionForce = std::abs(player_force.y) * 0.9;
		//		if (frictionForce > std::abs(player_force.x)) {
		//			player->addForce_x(frictionForce * cmp::sign(player_force.x));
		//		}
		//	}

		//	if (player->isTouchingEast() || player->isTouchingWest()) {
		//		float frictionForce = std::abs(player_force.x) * 0.9;
		//		if (frictionForce > std::abs(player_force.y)) {
		//			player->addForce_y(frictionForce * cmp::sign(player_force.y));
		//		}
		//	}
		//}
		////

		auto iter = engine.entities_cbegin();
		do  {
			DrawCollider(**iter);
		} while (++iter != engine.entities_cend());
		return true;
	}
public:
	void DrawCollider(const phy::Collider& c, olc::Pixel color = olc::WHITE) {
		DrawRect(c.getX1(), c.getY1(), c.getWidth(), c.getHeight(), color);
	}
};

int main()
{
	Example demo;
	if (demo.Construct(400, 400, 1, 1))
		demo.Start();
	return 0;
}
