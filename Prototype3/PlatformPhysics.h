#pragma once

#include<cmath>
#include<vector>
#include<set>

#include "MyMathUtils.h"
using namespace JesseRussell;
using namespace vectors;

namespace phy {
	// o----------o
	// | Cardinal |
	// o----------o

	// Enum wrapper representing cardinal directions.
	class Cardinal {
	public:
		enum Value : uint8_t
		{
			NORTH = 0B1000,
			EAST  = 0B0100,
			SOUTH = 0B0010,
			WEST  = 0B0001,

			NORTH_WEST = 0b1001,
			NORTH_EAST = 0b1100,
			SOUTH_EAST = 0b0110,
			SOUTH_WEST = 0b0011,

			NORTH_SOUTH = 0b1010,
			EAST_WEST   = 0b0101,

			NORTH_EAST_SOUTH = 0b1110,
			EAST_SOUTH_WEST  = 0b0111,
			NORTH_SOUTH_WEST = 0b1011,
			NORTH_EAST_WEST  = 0b1101,

			NORHT_EAST_SOUTH_WEST = 0b1111,

			NONE = 0b0000
		};

	public:
		Cardinal() = default;
		Cardinal(const Value& value) { this->value = value; }
		Cardinal(const uint8_t& value) { this->value = static_cast<Value>(0b00001111 & value); }

	public: // conversions:
		operator Value() const { return value; }
		operator uint8_t() const { return value; }
		operator bool() = delete;

	public: // operators:
		bool operator ==(const Cardinal& other) const { return value == other.value; }
		bool operator !=(const Cardinal& other) const { return value != other.value; }
		bool operator ==(const Cardinal::Value& other) const { return value == value; }
		bool operator !=(const Cardinal::Value& other) const { return value != value; }

		Cardinal operator &(const Cardinal& other) const { return value & other.value; }
		Cardinal operator |(const Cardinal& other) const { return value | other.value; }
		Cardinal operator ^(const Cardinal& other) const { return value ^ other.value; }

	public: // methods:
		uint8_t SelectNorth() const { return value & NORTH; }
		uint8_t SelectEast()  const { return value & EAST; }
		uint8_t SelectSouth() const { return value & SOUTH; }
		uint8_t SelectWest()  const { return value & WEST; }

		bool has(const Cardinal& other) const {
			if (other.SelectNorth() && !SelectNorth()) return false;
			if (other.SelectEast() && !SelectEast())  return false;
			if (other.SelectSouth() && !SelectSouth()) return false;
			if (other.SelectWest() && !SelectWest())  return false;

			return true;
		}

		bool isVertical()  const { return (0b1010 & value) && !(0b0101 & value); }
		bool isHorizontl() const { return (0b0101 & value) && !(0b1010 & value); }

		Cardinal Flipped() const {
			return
				((0b1100 & value) >> 2) |
				((0b0011 & value) << 2);
		}

		Cardinal Flipped_horizontally() const {
			return
				((0b0100 & value) >> 2) |
				((0b0001 & value) << 2);
		}
		Cardinal Flipped_vertically() const {
			return
				((0b1000 & value) >> 2) |
				((0b0010 & value) << 2);
		}

		Cardinal Rotated_clockwise() const {
			return
				((0b1110 & value) >> 1) |
				((0b0001 & value) << 3);
		}

		Cardinal Rotated_counterClockwise() const {
			return
				((0b0111 & value) << 1) |
				((0b1000 & value) >> 3);
		}

		const Value& GetValue() const { return value; }
		uint8_t bitwise() const { return value; }

	private:
		Value value;
	};

	// Useful function for working with cardinals and vectors:
	float& ref_vectorAxis(Cardinal axis, fvector2& v) {
		switch (axis.GetValue()) {
		case Cardinal::EAST: return v.x;
		case Cardinal::SOUTH: return v.y;
		default: throw std::invalid_argument("Expected east or south.");
		}
	}
	//



	// o--------o
	// | Entity |
	// o--------o

	class CollisionBox {
		friend class DynamicEntity;
		friend class Engine;
	public: // Constructors:
		CollisionBox(const fvector2& pointA, const fvector2& pointB) { A = pointA, B = pointB; }
		CollisionBox() = default;

	public: // properties:
		fvector2 GetPosition() const { return GetCorner(Cardinal::NORTH_WEST); }
		void     SetPosition(const fvector2& value) {
			ref_side_x(Cardinal::WEST)  = value.x;
			ref_side_y(Cardinal::NORTH) = value.y;
		}
		void     SetPosition_x(float value) { ref_side_x(Cardinal::WEST)  = value; }
		void     SetPosition_y(float value) { ref_side_y(Cardinal::NORTH) = value; }

		fvector2 GetSize() const { return B - A; }
		void     SetSize(const fvector2& value) { B - value + A; }
		void     SetSize_x(float value) { B.x = value + A.x; }
		void     SetSize_y(float value) { B.y = value + A.y; }

		float GetSize_1d(Cardinal axis) const {
			return GetSide(axis) - GetSide(axis.Flipped());
		}


	public: // methods:
		fvector2 GetCorner(Cardinal corner) const {
			switch (corner.GetValue()) {
			case Cardinal::NORTH_WEST: return { GetSide_x(Cardinal::WEST), GetSide_y(Cardinal::NORTH) };
			case Cardinal::NORTH_EAST: return { GetSide_x(Cardinal::EAST), GetSide_y(Cardinal::NORTH) };
			case Cardinal::SOUTH_EAST: return { GetSide_x(Cardinal::EAST), GetSide_y(Cardinal::SOUTH) };
			case Cardinal::SOUTH_WEST: return { GetSide_x(Cardinal::WEST), GetSide_y(Cardinal::SOUTH) };
			default: throw std::invalid_argument("The provided Cardinal direction must be either: NW, NE, SE, or SW");
			}
		}

		bool Intersects(const CollisionBox& other) const { return vectorRangeIntersection(A, B, other.A, other.B); }

		void Move(const fvector2& distance) {
			A += distance;
			B += distance;
		}

		void SetPositionOnSide(float position, Cardinal side) {
			float& main = ref_side(side);
			float& reverse = ref_side(side.Flipped());
			reverse = position - GetSize_1d(side);
			main = position;
		}

	private: // Fields:
		fvector2
			A = fvector2(),
			B = fvector2();

	private: // ref side
		float& ref_side_x(Cardinal side) {
			switch (side.GetValue()) {
			case Cardinal::WEST: return cmp::ref_min(A.x, B.x);
			case Cardinal::EAST: return cmp::ref_max(A.x, B.x);
			default: throw std::invalid_argument("The provided side must be either east or west.");
			}
		}
		float& ref_side_y(Cardinal side) {
			switch (side.GetValue()) {
			case Cardinal::NORTH: return cmp::ref_min(A.y, B.y);
			case Cardinal::SOUTH: return cmp::ref_max(A.y, B.y);
			default: throw std::invalid_argument("The provided side must be either east or west.");
			}
		}
		float& ref_side(Cardinal side) {
			switch (side.GetValue()) {
				case Cardinal::WEST: return cmp::ref_min(A.x, B.x);
				case Cardinal::EAST: return cmp::ref_max(A.x, B.x);
				case Cardinal::NORTH: return cmp::ref_min(A.y, B.y);
				case Cardinal::SOUTH: return cmp::ref_max(A.y, B.y);
				default: throw std::invalid_argument("The provided side must be either east or west.");
			}
		}
	public: // non-ref side
		float GetSide_x(Cardinal side) const {
			switch (side.GetValue()) {
			case Cardinal::WEST: return cmp::min(A.x, B.x);
			case Cardinal::EAST: return cmp::max(A.x, B.x);
			default: throw std::invalid_argument("The provided side must be either east or west.");
			}
		}
		float GetSide_y(Cardinal side) const {
			switch (side.GetValue()) {
			case Cardinal::NORTH: return cmp::min(A.y, B.y);
			case Cardinal::SOUTH: return cmp::max(A.y, B.y);
			default: throw std::invalid_argument("The provided side must be either north or south.");
			}
		}
		float GetSide(Cardinal side) const {
			switch (side.GetValue()) {
			case Cardinal::WEST: return cmp::min(A.x, B.x);
			case Cardinal::EAST: return cmp::max(A.x, B.x);
			case Cardinal::NORTH: return cmp::min(A.y, B.y);
			case Cardinal::SOUTH: return cmp::max(A.y, B.y);
			default: throw std::invalid_argument("The provided side must be either east or west.");
			}
		}
	public:
		bool IsDynamic() const { return isDynamic(); }
	private:
		virtual bool isDynamic() const { return false; }
	};





	// o---------------o
	// | DynamicEntity |
	// o---------------o

	class DynamicEntity : public CollisionBox {
		friend class Engine;
	public: // constructors:
		DynamicEntity(const fvector2& pointA, const fvector2& pointB, float mass) : CollisionBox(pointA, pointB) { this->mass = mass; }
		DynamicEntity() = default;

	public: // properties:
		const fvector2& GetVelocity() const { return velocity; }
		void            SetVelocity(const fvector2& value) { velocity = value; }
		void            SetVelocity_x(float value) { velocity.x = value; }
		void            SetVelocity_y(float value) { velocity.y = value; }

		void            SetVelocity(Cardinal axis, float value) {
			ref_vectorAxis(axis, velocity) = value;
		}

		const fvector2& GetNetForce() const { return netForce; }
		void            SetNetForce(const fvector2& value) { netForce = value; }
		void            SetNetForce_x(float value) { netForce.x = value; }
		void            SetNetForce_y(float value) { netForce.y = value; }


		const float& GetBounce() const { return bounce; }
		void         SetBounce(float value) { bounce = value; }

		const float& GetMass() const { return mass; }
		void         SetMass(float value) { mass = value; }
	public: // methods:
		void AddForce(const fvector2& force) { netForce += force; }

		void ApplyVelocity(float timeScale, Cardinal axis = Cardinal::SOUTH_EAST) {
			switch (axis.GetValue()) {
			case Cardinal::EAST:  Move(velocity.selectX() * timeScale); break;
			case Cardinal::SOUTH: Move(velocity.selectY() * timeScale);	break;
			case Cardinal::SOUTH_EAST: Move(velocity * timeScale);		break;
			default: throw std::invalid_argument("The axis must be either: south, east, or south-east.");
			}
		}

		void ApplyNetForce(Cardinal axis = Cardinal::SOUTH_EAST){
			switch (axis.GetValue()) {
			case Cardinal::EAST:  velocity.x += netForce.x / mass;  break;
			case Cardinal::SOUTH: velocity.y += netForce.y / mass;  break;
			case Cardinal::SOUTH_EAST: velocity += netForce / mass; break;
			default: throw std::invalid_argument("The axis must be either: south, east, or south-east.");
			}
		}
		
		void ClearNetForce(Cardinal axis = Cardinal::SOUTH_EAST) {
			switch (axis.GetValue()) {
			case Cardinal::EAST: netForce.x = 0;			break;
			case Cardinal::SOUTH: netForce.y = 0;			break;
			case Cardinal::SOUTH_EAST: netForce = { 0, 0 }; break;
			default: throw std::invalid_argument("The axis must be either: south, east, or south-east.");
			}
		}

		Cardinal GetHeading() {
			uint8_t result = 0;
			if (velocity.x < 0) result |= Cardinal::WEST;
			else                result |= Cardinal::EAST;

			if (velocity.y < 0) result |= Cardinal::NORTH;
			else                result |= Cardinal::SOUTH;
			return result;
		}

		CollisionBox GetVelocitySmear(float timeScale, Cardinal axis) {
			switch (axis.GetValue()) {
			case Cardinal::EAST:
				if (velocity.x < 0) return CollisionBox(GetPosition().plusX(velocity * timeScale), GetCorner(Cardinal::SOUTH_EAST));
				else return CollisionBox(GetPosition(), GetCorner(Cardinal::SOUTH_EAST).plusX(velocity * timeScale));
				break;
			case Cardinal::SOUTH:
				if (velocity.y < 0) return CollisionBox(GetCorner(Cardinal::NORTH_WEST).plusY(velocity * timeScale), GetCorner(Cardinal::SOUTH_EAST));
				else return CollisionBox(GetPosition(), GetCorner(Cardinal::SOUTH_EAST).plusY(velocity * timeScale));
				break;
			}
		}

		// collisions:
		bool Collides(const CollisionBox& other, Cardinal axis, float timeScale, float& out_collisionSpot, Cardinal& out_collisionSide) {
			if (Collides(other, axis, timeScale)) {
				out_collisionSide = (axis | axis.Flipped()) & GetHeading();
				out_collisionSpot = other.GetSide((out_collisionSide).Flipped());
				return true;
			}
			else return false;
		}

		bool Collides(const CollisionBox& other, Cardinal axis, float timeScale) {
			if ((axis != Cardinal::EAST) || (axis != Cardinal::SOUTH)) throw std::invalid_argument("The provided axis must be either East or South.");

			return (GetVelocitySmear(timeScale, axis).Intersects(other));
		}


	private: // fields:
		fvector2
			velocity = fvector2(),
			netForce = fvector2();
		float
			mass = 1,
			bounce = -0.5;

	private:
		bool isDynamic() override { return true; }
	};




	// o--------o
	// | Engine |
	// o--------o

	class Engine {
	public: // entity management:
		void AddEntity(CollisionBox* e) {
			if (e->IsDynamic()) dynamicEntities.insert((DynamicEntity*) e);
			entities.push_back(e);
		}

		std::vector<CollisionBox*>::const_iterator entities_cbegin() const { return entities.cbegin(); }
		std::vector<CollisionBox*>::const_iterator entities_cend() const { return entities.cend(); }

	public: // collisions:
		void RunCollisions(float timeScale, Cardinal axis) {
			this->timeScale = timeScale; // record timescale

			// Variables that will be needed later:
			float out_collisionSpot;
			Cardinal out_collisionSide;
			Cardinal closestSide;
			float closestSpot = 0;
			bool collided = false;
			//

			for (DynamicEntity* dep : dynamicEntities) {
				DynamicEntity& d = *dep;

				for (CollisionBox* ep : entities) {
					if (ep == dep) continue; // Don't check for collisions against yourself.
					CollisionBox& e = *ep;

					if (d.Collides(e, axis, timeScale, out_collisionSpot, out_collisionSide)) {
						if (collided) { closestSpot = cmp::closest(closestSpot, out_collisionSpot, d.GetSide(out_collisionSide)); closestSide = out_collisionSide; }
						else { closestSpot = out_collisionSpot; closestSide = out_collisionSide; }
						collided = true;
					}
				}

				if (collided) {
					d.SetPositionOnSide(closestSpot, closestSide);
					ref_vectorAxis(axis, d.velocity) *= d.bounce;
				}
			}
		}

	private:
		std::vector<CollisionBox*> entities;
		std::set<DynamicEntity*> dynamicEntities;
		float timeScale = 1;
	};
}