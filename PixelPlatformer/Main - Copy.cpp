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
		return true;
	}
	phy::DynamicEntity bob = phy::DynamicEntity(fvector2(100, 150), fvector2(30, 30), 10);
	phy::Entity steve = phy::Entity(fvector2(100, 100), fvector2(30, 30));
	phy::Entity frank = phy::Entity(fvector2(69, 100), fvector2(5, 5));

	void DrawEntity(const phy::Entity& entity, olc::Pixel color = olc::WHITE) {
		DrawRect(round(entity.getX1()), round(entity.getY1()), round(entity.getWidth()), round(entity.getHeight()), color);
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		fElapsedTime = .017;
		Clear(olc::BLACK);
		//
		//// set bob's future position to mouse position.
		//bob.setVelocity(mmu::fv2d(GetMouseX(), GetMouseY()) - bob.getPosition());

		std::this_thread::sleep_for(std::chrono::nanoseconds((long long)std::round(ft * 1000000000)));
		// draw bob's future position.
		/*mmu::fv2d fp = bob.getPosition() + bob.getVelocity() * fElapsedTime;
		DrawRect(round(fp.x), round(fp.y), round(bob.getWidth()), round(bob.getHeight()));*/


		// draw bob.
		//DrawRect(bob.getX1(), bob.getY1(), bob.getWidth(), bob.getHeight(), olc::BLUE);
		DrawEntity(bob, olc::BLUE);
		DrawEntity(steve, olc::RED);
		DrawEntity(frank, olc::GREEN);




		// air resistance:
		fvector2 airRes = bob.getVelocity();
		airRes.x *= -abs(airRes.x);
		airRes.y *= -abs(airRes.y);
		bob.addForce(airRes * k);
		//


		// mouse follow:
		fvector2 follow = fvector2(GetMouseX(), GetMouseY()) - bob.getCenter();
		
		bob.addForce(follow * f);
		//

		// tweak values:
		bool shift = GetKey(olc::SHIFT).bHeld;
		bool ctrl = GetKey(olc::CTRL).bHeld;
		if (shift && ctrl) {
			ft += 0.00001 * GetMouseWheel();
		}
		else if (shift) {
			k += 0.0001 * GetMouseWheel();
		}
		else if (ctrl) {
			bob.setMass(bob.getMass() + 0.001 * GetMouseWheel());
		}
		else {
			f += 0.003 * GetMouseWheel();
		}
		//

		// reset bob:
		if (GetKey(olc::R).bPressed) {
			bob.setPosition({ 100,150 });
			bob.setVelocity({ 0,0 });
		}
		//

		// twang bob.
		if (GetKey(olc::D).bPressed) bob.setMass(0.0f);

		
		// draw values on screen:
		DrawString(10, 20, "follow force: " + std::to_string(f));
		DrawString(10, 30, "drag force: "   + std::to_string(k));
		DrawString(10, 40, "mass: "   + std::to_string(bob.getMass()));
		DrawString(10, 50, "frame time : "   + std::to_string(ft));
		//


		// apply force
		bob.applyForce(fElapsedTime);


		// collision:
		Stopwatch sw;

		sw.start();
		fvector2 steve_collisionSpot = { -100, -100 };
		phy::CardinalDirection steve_collisionSide;
		fvector2 frank_collisionSpot = { -100, -100 };
		phy::CardinalDirection frank_collisionSide;
		fvector2 collisionSpot;
		bool steve_col = false;
		bool frank_col = false;

		if (bob.collisionStaticCheck(steve, fElapsedTime, steve_collisionSpot, steve_collisionSide)) {
			DrawString(10, 10, "col with steve", olc::RED);
			steve_col = true;
		}

		if (bob.collisionStaticCheck(frank, fElapsedTime, frank_collisionSpot, frank_collisionSide)) {
			DrawString(10, 10, "col with frank", olc::GREEN);
			frank_col = true;
		}
		sw.stop();

		if (frank_col) {
			frank_collisionSpot.x = std::round(frank_collisionSpot.x);
			frank_collisionSpot.y = std::round(frank_collisionSpot.y);
		}

		if (steve_col) {
			steve_collisionSpot.x = std::round(steve_collisionSpot.x);
			steve_collisionSpot.y = std::round(steve_collisionSpot.y);
		}

		float lowest_x_col;
		bool lowest_x_defined = false;
		float lowest_y_col;
		bool lowest_y_defined = false;
		float highest_x_col;
		bool highest_x_defined = false;
		float highest_y_col;
		bool highest_y_defined = false;

		if (frank_col) {
			switch (frank_collisionSide)
			{
			case phy::NORTH:
				highest_y_col = frank_collisionSpot.y;
				highest_y_defined = true;
				break;
			case phy::EAST:
				lowest_x_col = frank_collisionSpot.x;
				lowest_x_defined = true;
				break;
			case phy::SOUTH:
				lowest_y_col = frank_collisionSpot.y;
				lowest_y_defined = true;
				break;
			case phy::WEST:
				highest_x_col = frank_collisionSpot.x;
				highest_x_defined = true;
				break;
			}
		}

		if (steve_col) {
			switch (steve_collisionSide) {
			case phy::NORTH:
				highest_y_col = highest_y_defined ? comp::min(steve_collisionSpot.y, highest_y_col) : steve_collisionSpot.y;
				highest_y_defined = true;
				break;
			case phy::EAST:
				lowest_x_col = lowest_x_defined ? comp::min(steve_collisionSpot.x, lowest_x_col) : steve_collisionSpot.x;
				lowest_x_defined = true;
				break;
			case phy::SOUTH:
				lowest_y_col = lowest_y_defined ? comp::max(steve_collisionSpot.y, lowest_y_col) : steve_collisionSpot.y;
				lowest_y_defined = true;
				break;
			case phy::WEST:
				highest_x_col = highest_x_defined ? comp::min(steve_collisionSpot.x, highest_x_col) : steve_collisionSpot.x;
				highest_x_defined = true;
				break;
			}
		}


		if (lowest_x_defined) {
			if (frank_collisionSpot.x < lowest_x_col) frank_col = false;
			if (steve_collisionSpot.x < lowest_x_col) steve_col = false;
		}
		if (highest_x_defined) {
			if (frank_collisionSpot.x > highest_x_col) frank_col = false;
			if (steve_collisionSpot.x > highest_x_col) steve_col = false;
		}
		if (lowest_y_defined) {
			if (frank_collisionSpot.y < lowest_y_col) frank_col = false;
			if (steve_collisionSpot.y < lowest_y_col) steve_col = false;
		}
		if (highest_y_defined) {
			if (frank_collisionSpot.y > highest_y_col) frank_col = false;
			if (steve_collisionSpot.y > highest_y_col) steve_col = false;
		}
		if (steve_col) DrawEntity({ steve_collisionSpot, bob.getSize() }, olc::DARK_RED);
		if (frank_col) DrawEntity({ frank_collisionSpot, bob.getSize()}, olc::DARK_GREEN);

		bool horizontalCollision = false;
		bool verticalCollision = false;
		if (steve_col && frank_col) {
			if (phy::isHorizontal(frank_collisionSide)) {
				horizontalCollision = true;

				if (phy::isHorizontal(steve_collisionSide)) {
					collisionSpot.y = steve_collisionSpot.y;
					collisionSpot.x = frank_collisionSpot.x;
				}
				else {
					verticalCollision = true;
					collisionSpot.y = steve_collisionSpot.y;
					collisionSpot.x = frank_collisionSpot.x;
				}
			}
			else {
				verticalCollision = true;
				if (phy::isHorizontal(steve_collisionSide)) {
					horizontalCollision = true;
					collisionSpot.y = frank_collisionSpot.y;
					collisionSpot.x = steve_collisionSpot.x;
				}
				else {
					collisionSpot.y = frank_collisionSpot.y;
					collisionSpot.x = steve_collisionSpot.x;
				}
			}
		}
		else if (steve_col) {
			collisionSpot = steve_collisionSpot;
			if (phy::isHorizontal(steve_collisionSide)) horizontalCollision = true;
			if (phy::isVertical(steve_collisionSide)) verticalCollision = true;
		}
		else if (frank_col) {
			collisionSpot = frank_collisionSpot;
			if (phy::isHorizontal(frank_collisionSide)) horizontalCollision = true;
			if (phy::isVertical(frank_collisionSide)) verticalCollision = true;
		}
		if (frank_col || steve_col) {
			bob.setPosition(collisionSpot);
			if (horizontalCollision) bob.setVelocity({0, bob.getVelocity().y});
			if (verticalCollision) bob.setVelocity({bob.getVelocity().x, 0});
		}


		DrawString(10, 60, "time for collision:" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(sw.getElapsed()).count()));
		//


		bob.applyVelocity(fElapsedTime);


		
		return true;
	}
	float k = 0;
	float f = 500;
	float ft = .01666666666;
};

int main()
{
	Example demo;
	if (demo.Construct(256, 240, 4, 4))
		demo.Start();
	return 0;
}
