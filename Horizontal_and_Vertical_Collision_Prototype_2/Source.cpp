#include "MyMathUtils.h"
#include "PlatformPhysics.h"

#include<iostream>

using namespace JesseRussell;
using namespace JesseRussell::vectors;
using namespace platformPhysics;

int main() {
	phy::Collider coll = phy::Collider({20,20}, 10, 10);
	phy::Cardinal card = phy::Cardinal::WEST | phy::Cardinal::NORTH | phy::Cardinal::EAST | phy::Cardinal::SOUTH;
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
	card = card.Rotated_counterClockwise();
	std::cout << card << std::endl;
}