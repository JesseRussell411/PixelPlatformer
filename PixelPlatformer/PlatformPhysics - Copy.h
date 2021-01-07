#pragma once

#include "MyMathUtils.h"

#include<iostream>
#include<cmath>
#include<string>

namespace phy {
	using namespace JesseRussell::vectors;

	namespace intersectionCheck {
		static bool rangeIntersection(const float& min1, const float& max1, const float& min2, const float& max2) {
			return min2 <= max1 && min1 <= max2;
		}

		static bool vectorRangeIntersection(const fvector2& min1, const fvector2& max1, const fvector2& min2, const fvector2 max2) {
			return rangeIntersection(min1.x, max1.x, min2.x, max2.x) &&
				rangeIntersection(min1.y, max1.y, min2.y, max2.y);
		}
	}
	using namespace intersectionCheck;


	enum CardinalDirection {
		NORTH = 0b00,
		EAST =  0b01,
		SOUTH = 0b10,
		WEST =  0b11
	};

	bool isHorizontal(const CardinalDirection& dir) {
		return dir & 0b01;
	}

	bool isVertical(const CardinalDirection& dir) {
		return !(dir & 0b01);
	}


	struct Box {
		fvector2 size;
		float getWidth() const { return size.x; }
		void  setWidth(const float& value) { size.x = value; }

		float getHeight() const { return size.y; }
		void  setHeight(const float& value) { size.y = value; }

		fvector2 getCenter() const { return size / 2; }
	};

	struct Collider {
		// fields:
		fvector2 position;
		Box collisionBox;

		// getters & setters:
		fvector2 getNW() const { return position; }
		fvector2 getNE() const { return position.plusX(collisionBox.size); }
		fvector2 getSE() const { return position + collisionBox.size; }
		fvector2 getSW() const { return position.plusY(collisionBox.size); }

		fvector2 getSize() const { return collisionBox.size; }
		void     setSize(const fvector2& value) { collisionBox.size = value; }

		fvector2 getCenter() const { return position + collisionBox.size / 2; }

		float getWidth() const { return collisionBox.size.x; }
		void  setWidth(const float& value) { collisionBox.size.x = value; }

		float getHeight() const { return collisionBox.size.y; }
		void  setHeight(const float& value) { collisionBox.size.y = value; }

		fvector2 getPosition2() const { return position + collisionBox.size; }

		float getX1() const { return position.x; }
		float getX2() const { return position.x + collisionBox.size.x; }
		float getY1() const { return position.y; }
		float getY2() const { return position.y + collisionBox.size.y; }

		// methods:
		bool intersects(const Collider& other) const {
			return vectorRangeIntersection(position, getSE(), other.position, other.getSE());
		}
		
		bool intersectsHorizontal(const Collider& other) const {
			return rangeIntersection(position.x, getX2(), other.position.x, other.getX2());
		}

		bool intersectsVertical(const Collider& other) const {
			return rangeIntersection(position.y, getY2(), other.position.y, other.getY2());
		}
	};

	using namespace intersectionCheck;

	class Entity {
	public: // fields
		fvector2 position;
		fvector2 size;

	public: // constructors
		Entity(fvector2 position, fvector2 size) {
			this->position = position;
			this->size = size;
		}

	public: // properties
		float getX1() const { return position.x; }
		void  setX1(const float& value) { position.x = value; }

		float getY1() const { return position.y; }
		void  setY1(const float& value) { position.y = value; }

		float getX2() const { return position.x + size.x; }
		void  setX2(const float& value) { size.x = value - position.x; }

		float getY2() const { return position.y + size.y; }
		void  setY2(const float& value) { size.y = value - position.y; }

		fvector2 getSize() const { return size; }
		void setSize(const fvector2& value) { size = value; }

		fvector2 getPosition() const { return position; }
		void setPosition(const fvector2& value) { position = value; }

		fvector2 getPosition2() const { return position + size; }
		void setPosition2(fvector2 value) { size = value - position; }

		fvector2 getCenter() const { return position + size / 2; }

		float getWidth() const { return size.x; }
		void setWidth(float value) { size.x = value; }

		float getHeight() const { return size.y; }
		void setHeight(float value) { size.y = value; }

		fvector2 getNWCorner() const { return position; }
		fvector2 getNECorner() const { return fvector2(position.x + size.x, position.y); }
		fvector2 getSECorner() const { return position + size; }
		fvector2 getSWCorner() const { return fvector2(position.x, position.y + size.y); }

	public: //methods
		bool intersectionHorizontal(const Entity& other, const float& x, const float& other_x) const {
			return rangeIntersection(x, x + size.x, other_x, other_x + other.size.x);
		}

		bool intersectionHorizontal(const Entity& other, const float& x) const {
			return rangeIntersection(x, x + size.x, other.position.x, other.getX2());
		}

		bool intersectionHorizontal(const Entity& other) const {
			return rangeIntersection(position.x, getX2(), other.position.x, other.getX2());
		}

		bool intersectionVertical(const Entity& other, const float& y, const float& other_y) const {
			return rangeIntersection(y, y + size.y, other_y, other_y + other.size.y);
		}

		bool intersectionVertical(const Entity& other, const float& y) const {
			return rangeIntersection(y, y + size.y, other.position.y, other.getY2());
		}

		bool intersectionVertical(const Entity& other) const {
			return rangeIntersection(position.y, getY2(), other.position.y, other.getY2());
		}

		bool intersection(const Entity& other, const fvector2& position, const fvector2& other_position) const {
			return vectorRangeIntersection(position, position + size, other_position, other_position + other.size);
		}

		bool intersection(const Entity& other, const fvector2& position) const {
			return vectorRangeIntersection(position, position + size, other.position, other.position + other.size);
		}

		bool intersection(const Entity& other) const {
			return vectorRangeIntersection(position, position + size, other.position, other.position + other.size);
		}

	protected: // static Functions:
		static bool rangeIntersection(const float& min1, const float& max1, const float& min2, const float& max2) {
			return min2 <= max1 && min1 <= max2;
		}

		static bool vectorRangeIntersection(const fvector2& min1, const fvector2& max1, const fvector2& min2, const fvector2 max2) {
			return rangeIntersection(min1.x, max1.x, min2.x, max2.x) &&
				rangeIntersection(min1.y, max1.y, min2.y, max2.y);
		}

	public: // virtual methods
		void virtual update(float timeScale) { return; };
	};






	class DynamicEntity : public Entity {
	public:
		DynamicEntity(const fvector2& position, const fvector2& size, const float& mass)
			: Entity(position, size){
			this->mass = mass;
		}

	public: // Properties:
		fvector2 getForce() const { return force; }
		void setForce(const fvector2& value) { force = value; }

		fvector2 getAcceleration() const { return acceleration; }
		void setAcceleration(const fvector2& value) { acceleration = value; }

		fvector2 getVelocity() const { return velocity; }
		void setVelocity(const fvector2& value) { velocity = value; }

		fvector2 getSize() const { return size; }
		void setSize(const fvector2& value) { size = value; }

		float getMass() const { return mass; }
		void  setMass(const float& value) { mass = value; }

	public: // Methods:
		void addForce(const fvector2 &value) {
			force += value;
		}
		void subForce(const fvector2& value) {
			force -= value;
		}
		
		void update(float timeScale) override {
			acceleration = force / mass; force = { 0, 0 };
			velocity += acceleration * timeScale;
			position += velocity * timeScale;
		}

		void applyForce(float timeScale){
			acceleration = force / mass; force = { 0, 0 };
			velocity += acceleration * timeScale;
		}

		void applyVelocity(float timeScale) {
			position += velocity * timeScale;
		}

		bool collisionStaticCheck(const Entity& other, float timeScale) {
			fvector2 trash;
			CardinalDirection alsoTrash;
			return internal_collisionStaticCheck<false> (other, timeScale, trash, alsoTrash);
		}

		bool collisionStaticCheck(const Entity& other, float timeScale, fvector2& collisionSpot, CardinalDirection& collisionSide) {
			return internal_collisionStaticCheck<true>(other, timeScale, collisionSpot, collisionSide);
		}
		

	private: // Fields
		// position and size from entity.
		fvector2 velocity = { 0, 0 };
		fvector2 acceleration = { 0, 0 };
		float mass;
		fvector2 force = { 0, 0 };




	private: // Properties:
		float& x1() { return position.x; }
		float& y1() { return position.y; }
		float x2() { return position.x + size.x; }
		float y2() { return position.y + size.y; }
	protected: // static functions:
		static float getSlopeY(const fvector2& a, const fvector2& b, const float& c_x) {
			fvector2 s = b - a;
			return (c_x - a.x) / s.x * s.y + a.y;
		}

	private: // Methods
		template<bool calculateCollisionSpot>
		bool internal_collisionStaticCheck(const Entity& other, float timeScale, fvector2& out_collisionSpot, CardinalDirection& out_collisionSide) const {

			const float offset = 1;

			// adjust velocity with timeScale; using shadowing because I am lazy.
			fvector2 velocity = this->velocity * timeScale;

			//special cases:
				// clipping inside other...
			/*if (intersection(other)) {
				if (calculateCollisionSpot) out_collisionSpot = position;

				return true;
			}*/
				// perfectly vertical velocity...
			if (this->velocity.x == 0.0f) {
				if (velocity.y != 0 && std::abs(velocity.y) < 1) {
					velocity.y = velocity.y < 0 ? -1 : 1;
				}
				if (velocity.y >= 0) {
					if (vectorRangeIntersection(position, getPosition2() + velocity, other.position, other.getPosition2())) {
						if (calculateCollisionSpot) {
							out_collisionSpot.set(position.x, other.position.y - size.y - offset);
							out_collisionSide = NORTH;
						}
						return true;
					} else return false;
				}
				else {
					if (vectorRangeIntersection(position + velocity, getPosition2(), other.position, other.getPosition2())) {
						if (calculateCollisionSpot) {
							out_collisionSpot.set(position.x, other.position.y + other.size.y + offset);
							out_collisionSide = SOUTH;
						}
						return true;
					} else return false;
				}
			}
				//perfectly horizontal velocity...
			if (velocity.y == 0.0f) {
				if (velocity.x != 0 && std::abs(velocity.x) < 1) {
					velocity.x = velocity.x < 0 ? -1 : 1;
				}
				if (velocity.x >= 0) {
					if (vectorRangeIntersection(position, getPosition2() + velocity, other.position, other.getPosition2())) {
						if (calculateCollisionSpot) {
							out_collisionSpot.set(other.position.x - size.x - offset, position.y);
							out_collisionSide = WEST;
						}
						return true;
					} else return false;
				}
				else {
					if (vectorRangeIntersection(position + velocity, getPosition2(), other.position, other.getPosition2())) {
						if (calculateCollisionSpot) {
							out_collisionSpot.set(other.position.x + other.size.x + offset, position.y);
							out_collisionSide = EAST;
						}
						return true;
					} else return false;
				}
			}
			//




			// To do the diagonal collisions, I have 4 scenarios that are almost identical: moving South East, North East, South West, North West
			// now, this isn't the best way of doing this for future maintenance, but this is the easiest way, and this method shouldn't need to be changed or added to much.

			// see figure 1.0 for one of the scenarios (moving South East). Square a represents the entity before the movement and b represents the entity after the movement.
			// The diagonal section is the area to check for collisions during the movement. The large dotted square covering everything is the area in which a collision might happen;
			// any entity outside of this area will definitely not collide.
			// If the entity is inside this area, it might collide. To check whether is does or does not collide, we can check to see if it's south-west corner is within the
			// triangular region 1, or if it's north-east corner is in region 2. In either case, we know that no collision has occured; otherwise, a collision has occured.

   //         o-------o- - - - o
   //         |       |\
   //         |   a   | \   1  |
   //         |       |  \
   //         o-------o   \    |
   //         |\           \
   //           \           \  |
   //         |  \           \
   //             \           \|
   //         |    \   o-------o
   //               \  |       |
   //         | 2    \ |   b   |
   //                 \|       |    figure 1.0
   //         o - - -  o-------o



   //                        o--------------o
   //                        |              |
   //                        |              |
   //                        |              |
   //         o-------o- - - | o            |
   //         |       |\     |              |
   //         |   a   | \   1| |            |
   //         |       |  \   o--------------o
   //         o-------o   \    |
   //         |\           \
   //           \           \  |
   //         |  \           \
   //             \           \|
   //         |    \   o-------o
   //               \  |       |
   //         | 2    \ |   b   |
   //                 \|       |                 figure 1.1: no collision
   //         o - - -  o-------o




   //                    o--------------o
   //         o-------o- | - - o        |
   //         |       |\ |              |
   //         |   a   | \|  1  |        |
   //         |       |  |              |
   //         o-------o  |\    |        |
   //         |\         | \            |
   //           \        o--------------o
   //         |  \           \
   //             \           \|
   //         |    \   o-------o
   //               \  |       |
   //         | 2    \ |   b   |
   //                 \|       |             figure 1.2: collision
   //         o - - -  o-------o

			// complicated diagonal collisions...
			if (velocity.x >= 0) {
				if (velocity.y >= 0) {
					// *moving SE ===============================================================================================================
					float xdist = (other.getNWCorner().x - getSECorner().x);
					float ydist = (other.getNWCorner().y - getSECorner().y);

					if ((xdist / velocity.x) > (ydist / velocity.y)) {
						out_collisionSide = WEST;
						if (velocity.x != 0 && std::abs(velocity.x) < 1) {
							velocity.x = velocity.x < 0 ? -1 : 1;
						}
					}
					else {
						out_collisionSide = NORTH;
						if (velocity.y != 0 && std::abs(velocity.y) < 1) {
							velocity.y = velocity.y < 0 ? -1 : 1;
						}
					}

					if (vectorRangeIntersection(position, getPosition2() + velocity, other.position, other.getPosition2())) {
						fvector2 other_SW = other.getSWCorner();
						fvector2 other_NE = other.getNECorner();

						if (other_SW.y >= getSlopeY(getNECorner(), getNECorner() + velocity, other_SW.x) &&
							other_NE.y <= getSlopeY(getSWCorner(), getSWCorner() + velocity, other_NE.x))
						{

							if (calculateCollisionSpot) {
								if (out_collisionSide == WEST) {
									// hit on west edge
									out_collisionSpot.x = other.position.x - size.x - offset;
									out_collisionSpot.y = position.y + (xdist / velocity.x) * velocity.y;
								}
								else {
									// hit on north edge
									out_collisionSpot.x = position.x + (ydist / velocity.y) * velocity.x;
									out_collisionSpot.y = other.position.y - size.y - offset;
								}
							}
							return true;
						} else return false;
					}
					else return false;
				}
				else {
					// *moving NE ===============================================================================================================
					float xdist = (other.getSWCorner().x - getNECorner().x);
					float ydist = -(other.getSWCorner().y - getNECorner().y);

					if ((xdist / velocity.x) > (ydist / -velocity.y)) {
						out_collisionSide = WEST;
						if (velocity.x != 0 && std::abs(velocity.x) < 1) {
							velocity.x = velocity.x < 0 ? -1 : 1;
						}
					}
					else {
						out_collisionSide = SOUTH;
						if (velocity.y != 0 && std::abs(velocity.y) < 1) {
							velocity.y = velocity.y < 0 ? -1 : 1;
						}
					}

					if (vectorRangeIntersection(position.plusY(velocity), getSECorner().plusX(velocity), other.position, other.getPosition2())) {
						fvector2 other_SE = other.getSECorner();
						fvector2 other_NW = other.getNWCorner();

						if (other_SE.y >= getSlopeY(getNWCorner(), getNWCorner() + velocity, other_SE.x) &&
							other_NW.y <= getSlopeY(getSECorner(), getSECorner() + velocity, other_NW.x))
						{

							if (calculateCollisionSpot) {
								if (out_collisionSide == WEST) {
									// hit on west edge
									out_collisionSpot.x = other.position.x - size.x - offset;
									out_collisionSpot.y = position.y + (xdist / velocity.x) * velocity.y;
								}
								else {
									// hit on south edge
									out_collisionSpot.x = position.x + (ydist / -velocity.y) * velocity.x;
									out_collisionSpot.y = other.position.y + other.size.y + offset;
								}
							}
							return true;
						} else return false;
					}
					else return false;
				}
			}
			else {
				if (velocity.y >= 0) {
					// *moving SW ===============================================================================================================
					float xdist = -(other.getNECorner().x - getSWCorner().x);
					float ydist = (other.getNECorner().y - getSWCorner().y);

					if ((xdist / -velocity.x) > (ydist / velocity.y)) {
						out_collisionSide = EAST;
						if (velocity.x != 0 && std::abs(velocity.x) < 1) {
							velocity.x = velocity.x < 0 ? -1 : 1;
						}
					}
					else {
						out_collisionSide = NORTH;
						if (velocity.y != 0 && std::abs(velocity.y) < 1) {
							velocity.y = velocity.y < 0 ? -1 : 1;
						}
					}

					if (vectorRangeIntersection(getNWCorner().plusX(velocity), getSECorner().plusY(velocity), other.position, other.getPosition2())) {
						fvector2 other_SE = other.getSECorner();
						fvector2 other_NW = other.getNWCorner();

						if (other_SE.y >= getSlopeY(getNWCorner() + velocity, getNWCorner(), other_SE.x) &&
							other_NW.y <= getSlopeY(getSECorner() + velocity, getSECorner(), other_NW.x)) 
						{

							if (calculateCollisionSpot) {
								if (out_collisionSide == EAST) {
									// hit on east edge
									out_collisionSpot.x = other.position.x + other.size.x + offset;
									out_collisionSpot.y = position.y + (xdist / -velocity.x) * velocity.y;
								}
								else {
									// hit on north edge
									out_collisionSpot.x = position.x + (ydist / velocity.y) * velocity.x;
									out_collisionSpot.y = other.position.y - size.y - offset;
								}
							}
							return true;
						} else return false;
					}
					else return false;
				}
				else {
					// *moving NW ===============================================================================================================
					float xdist = -(other.getSECorner().x - getNWCorner().x);
					float ydist = -(other.getSECorner().y - getNWCorner().y);

					if ((xdist / -velocity.x) > (ydist / -velocity.y)) {
						out_collisionSide = EAST;
						if (velocity.x != 0 && std::abs(velocity.x) < 1) {
							velocity.x = velocity.x < 0 ? -1 : 1;
						}
					}
					else {
						out_collisionSide = SOUTH;
						if (velocity.y != 0 && std::abs(velocity.y) < 1) {
							velocity.y = velocity.y < 0 ? -1 : 1;
						}
					}

					if (vectorRangeIntersection(position + velocity, getPosition2(), other.position, other.getPosition2())) {
						fvector2 other_SW = other.getSWCorner();
						fvector2 other_NE = other.getNECorner();

						if (other_SW.y >= getSlopeY(getNECorner() + velocity, getNECorner(), other_SW.x) &&
							other_NE.y <= getSlopeY(getSWCorner() + velocity, getSWCorner(), other_NE.x))
						{

							if (calculateCollisionSpot) {
								float xdist = -(other.getSECorner().x - getNWCorner().x);
								float ydist = -(other.getSECorner().y - getNWCorner().y);

								if (out_collisionSide == EAST) {
									// hit on east edge
									out_collisionSpot.x = other.position.x + other.size.x + offset;
									out_collisionSpot.y = position.y + (xdist / -velocity.x) * velocity.y;
								}
								else {
									// hit on south edge
									out_collisionSpot.x = position.x + (ydist / -velocity.y) * velocity.x;
									out_collisionSpot.y = other.position.y + other.size.y + offset;
								}
							}
							return true;
						} else return false;
					}
					else return false;
				}
			}
		}
	};
}

//
//#include<iostream>
//#include<map>
//#include<queue>
//#include<vector>
//#include<list>
//#include<set>
//
//namespace phy {
//	struct mmu::fv2d {
//		float x, y;
//		mmu::fv2d() { x = 0, y = 0; }
//		mmu::fv2d(float x, float y) { this->x = x; this->y = y; }
//
//		void set(const mmu::fv2d& value) { x = value.x; y = value.y; }
//
//		mmu::fv2d& transform(const mmu::fv2d& i, const mmu::fv2d& j) {
//			float oldx = x;
//			x = x * i.x + y * j.x;
//			y = oldx * i.y + y * j.y;
//			return *this;
//		}
//
//		mmu::fv2d clone() const { return *this; }
//
//		mmu::fv2d& operator+=(const mmu::fv2d& other) { x += other.x; y += other.y; return *this; }
//		mmu::fv2d& operator-=(const mmu::fv2d& other) { x -= other.x; y -= other.y; return *this; }
//		mmu::fv2d operator+(const mmu::fv2d& other) const { return mmu::fv2d(x + other.x, y + other.y); }
//		mmu::fv2d operator-(const mmu::fv2d& other) const { return mmu::fv2d(x - other.x, y - other.y); }
//	};
//	std::ostream& operator<<(std::ostream& os, const mmu::fv2d& val) { os << "[" << val.x << ", " << val.y << "]"; return os; }
//
//	class Entity {
//	public:
//		Entity(const float& mass, const mmu::fv2d& position, const mmu::fv2d& size) {
//			this->mass = mass;
//			this->position = position;
//			this->size = size;
//		}
//		mmu::fv2d& Position() { return position; }
//		mmu::fv2d& Acceleration() { return acceleration; }
//		mmu::fv2d& Velocity() { return velocity; }
//		float Mass() { return mass; }
//		float setMass(float value) { mass = value; }
//		float x1() const { return position.x; }
//		float y1() const { return position.y; }
//		float x2() const { return position.x + size.y; }
//		float y2() const { return position.y + size.y; }
//		float width() const { return size.x; }
//		float height() const { return size.y; }
//		mmu::fv2d Position2() { return position + size; }
//
//		bool intersectionWithHorizontal(const Entity& other, float x, float other_x) const {
//			return rangeIntersection(x, x + width(), other_x, other_x + other.width());
//		}
//		bool intersectionWithHorizontal(const Entity& other, float x) const {
//			return rangeIntersection(x, x + width(), other.position.x, other.x2());
//		}
//		bool intersectionWithHorizontal(const Entity& other) const {
//			return rangeIntersection(position.x, x2(), other.position.x, other.x2());
//		}
//
//		bool intersectionWithVertical(const Entity& other, float y, float other_y) const {
//			return rangeIntersection(y, y + height(), other_y, other_y + other.height());
//		}
//		bool intersectionWithVertical(const Entity& other, float y) const {
//			return rangeIntersection(y, y + height(), other.position.y, other.y2());
//		}
//		bool intersectionWithVertical(const Entity& other) const {
//			return rangeIntersection(position.y, y2(), other.position.y, other.y2());
//		}
//
//		bool intersectionWith(const Entity& other, const mmu::fv2d& position, const mmu::fv2d& other_position) const {
//			return intersectionWithHorizontal(other, position.x, other_position.x) &&
//				intersectionWithVertical(other, position.y, other_position.y);
//		}
//		bool intersectionWith(const Entity& other, const mmu::fv2d& position) const {
//			return intersectionWithHorizontal(other, position.x) &&
//				intersectionWithVertical(other, position.y);
//		}
//		bool intersectionWith(const Entity& other) const {
//			return intersectionWithHorizontal(other) && intersectionWithVertical(other);
//		}
//
//		friend class Engine;
//
//	protected:
//		float mass;
//		mmu::fv2d position;
//		mmu::fv2d size;
//		mmu::fv2d acceleration = { 0, 0 };
//		mmu::fv2d velocity = { 0, 0 };
//
//	private:
//		static bool rangeIntersection(const float& min1, const float& max1, const float& min2, const float& max2) {
//			return  max2 >= min1 && max1 >= min2;
//		}
//	};
//
//	struct CollisionEvent {
//		Entity* master, * slave;
//		bool vertical = false;
//	};
//
//	class Engine {
//	public:
//		std::map<size_t, Entity*>::const_iterator getEntityIterator() {
//			return entities.cbegin();
//		}
//
//		Entity* getEntity(size_t id) {
//			return entities[id];
//		}
//
//		void addEntity(Entity* e) {
//			entities.insert(std::pair<long, Entity*>(newId(), e));
//		}
//
//		void removeEntity(size_t id) {
//			entities.erase(id);
//		}
//
//		void deleteEntity(size_t id) {
//			delete entities[id];
//			removeEntity(id);
//		}
//
//		void update() {
//			checkCollisions();
//			calculateCollisions();
//			updateEntityValues();
//		}
//		float elapsedSeconds;
//	protected:
//		std::map<size_t, Entity*> entities;
//		std::queue<CollisionEvent*> collisions;
//
//		void checkCollisions() {
//			std::list<Entity*>* ents = new std::list<Entity*>;
//
//			// Generate collision check list...
//			for (auto ep : entities) {
//				ents->push_back(ep.second);
//			}
//
//			auto iter = ents->begin();
//			do {
//				Entity* e = *iter;
//				ents->erase(iter++);
//
//				for (Entity* e2 : *ents) {
//					if (e->intersectionWith(*e2, { e->position.x + e->velocity.x, 0 }, { e2->position.x + e2->velocity.x, 0 }))
//						if (std::abs(e->velocity.x) < std::abs(e2->velocity.x))
//							collisions.push(new CollisionEvent{ e2, e });
//						else
//							collisions.push(new CollisionEvent{ e, e2 });
//
//					if (e->intersectionWith(*e2, { 0, e->position.y + e->velocity.y }, { 0, e2->position.y + e2->velocity.y }))
//						if (std::abs(e->velocity.y) < std::abs(e2->velocity.y))
//							collisions.push(new CollisionEvent{ e2, e, true});
//						else
//							collisions.push(new CollisionEvent{ e, e2, true});
//				}
//			} while (iter != ents->end());
//
//			delete ents;
//		}
//
//		void calculateCollisions() {
//			while (!collisions.empty()) {
//				CollisionEvent ce = *collisions.front();
//				collisions.pop();
//				if (ce.vertical) {
//					if (ce.master->velocity.x > 0)
//						ce.master->position.x = ce.slave->position.x - ce.master->size.x - 1;
//					else
//						ce.master->position.x = ce.slave->position.x + ce.slave->size.x + 1;
//
//					collide(ce.master->mass, ce.slave->mass, ce.master->velocity.x, ce.slave->velocity.x);
//				}
//				else {
//					if (ce.master->velocity.x > 0)
//						ce.master->position.y = ce.slave->position.y - ce.master->size.y - 1;
//					else
//						ce.master->position.y = ce.slave->position.y + ce.slave->size.y + 1;
//
//					collide(ce.master->mass, ce.slave->mass, ce.master->velocity.y, ce.slave->velocity.y);
//				}
//			}
//		}
//
//		void updateEntityValues() {
//			for (auto ep : entities) {
//				Entity& e = *ep.second;
//
//				e.velocity += e.acceleration;
//				e.acceleration = { 0, 0 };
//				e.position += e.velocity;
//			}
//		}
//
//		static void collide(const float& mass1, const float& mass2, float& vel1, float& vel2) {
//			float vel1_final = (mass1 - mass2) / (mass1 + mass2) * vel1 + (2 * mass2) / (mass1 + mass2) * vel2;
//			vel2 = (2 * mass1) / (mass1 + mass2) * vel1 + (mass2 - mass1) / (mass1 + mass2) * vel2;
//			vel1 = vel1_final;
//		}
//
//		size_t nextId = 0;
//		std::queue<size_t> freeIds;
//
//		size_t newId() {
//			if (freeIds.empty()) {
//				return nextId++;
//			}
//			else
//			{
//				auto result = freeIds.front();
//				freeIds.pop();
//				return result;
//			}
//		}
//	};
//}