#pragma once

#include "MyMathUtils.h"

#include<cmath>
#include<vector>
#include<set>


namespace phy {
	using namespace JesseRussell::vectors;
	using namespace JesseRussell;
	// o==========o
	// | Cardinal |
	// o==========o

	// Enum wrapper representing cardinal directions.
	class Cardinal {
	public:
		enum Value : uint8_t
		{
			NORTH = 0B1000,
			EAST =  0B0100,
			SOUTH = 0B0010,
			WEST =  0B0001,

			NORTH_WEST = 0b1001,
			NORTH_EAST = 0b1100,
			SOUTH_EAST = 0b0110,
			SOUTH_WEST = 0b0011,

			NORTH_SOUTH = 0b1010,
			EAST_WEST =   0b0101,

			NORTH_EAST_SOUTH = 0b1110,
			EAST_SOUTH_WEST =  0b0111,
			NORTH_SOUTH_WEST = 0b1011,
			NORTH_EAST_WEST =  0b1101,

			NORTH_EAST_SOUTH_WEST = 0b1111,

			NONE = 0b0000
		};

	public:
		Cardinal() = default;
		Cardinal(const Value& value) { this->value = value; }
		Cardinal(const uint8_t& value) { this->value = static_cast<Value>(0b00001111 & value); }

	public: // conversions:
		//operator Value() const { return value; }
		//operator uint8_t() const { return value; }
		explicit operator bool() { return (bool)value; }

	public: // operators:
		bool operator ==(const Cardinal& other) const { return value == other.value; }
		bool operator !=(const Cardinal& other) const { return value != other.value; }
		bool operator ==(const Cardinal::Value& other) const { return value == value; }
		bool operator !=(const Cardinal::Value& other) const { return value != value; }

		Cardinal operator &(const Cardinal& other) const { return value & other.value; }
		Cardinal operator |(const Cardinal& other) const { return value | other.value; }
		Cardinal operator ^(const Cardinal& other) const { return value ^ other.value; }

		Cardinal operator &=(const Cardinal& other) { return value = (Value)(value & other.value); }
		Cardinal operator |=(const Cardinal& other) { return value = (Value)(value | other.value); }
		Cardinal operator ^=(const Cardinal& other) { return value = (Value)(value ^ other.value); }

		Cardinal operator ~() const { return ~value; }

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

		Cardinal Mirrored_horizontally() const {
			return
				((0b0100 & value) >> 2) |
				((0b0001 & value) << 2);
		}
		Cardinal Mirrored_vertically() const {
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
		Value value = NONE;
	};



	// o=========================================================o
	// | Useful function for working with cardinals and vectors: |
	// o=========================================================o

	float& ref_vectorAxis(Cardinal axis, fvector2& v);

	const float& vectorAxis(Cardinal axis, const fvector2& v);

	fvector2 selectVectorAxis(Cardinal axis, const fvector2& v);

	Cardinal cardinalAxis(const Cardinal& c);

	fvector2 cardinalAxisSign(const Cardinal& c);

	Cardinal toggleAxis(const Cardinal& c);



	// o========o
	// | Entity |
	// o========o
	
	class CollisionBox {
		friend class Engine;
	public: // Constructors:
		CollisionBox(const fvector2& position, const fvector2& size) { this->position = position, this->size = size; }
		CollisionBox() = default;

	public: // factories:
		static CollisionBox ByPoints(const fvector2& pointA, const fvector2& pointB) {
			CollisionBox result;
			result.PointA(pointA);
			result.PointB(pointB);
			return result;
		}

	public: // Properties:
		const fvector2& Position() const { return position; }
		void            Position(const fvector2& value) { position = value; }

		const fvector2& Size() const { return size; }
		void            Size(const fvector2& value) { size = value; }


		const fvector2& PointA() const { return position; }
		void            PointA(const fvector2& value) { size += position - value; position = value; }

		fvector2 PointB() const { return position + size; }
		void     PointB(const fvector2& value) { size = value - position; }


		float Position_x() const { return position.x; }
		void  Position_x(const float& value) { position.x = value; }
		float Position_y() const { return position.x; }
		void  Position_y(const float& value) { position.x = value; }

		float Size_x() const { return size.x; }
		void  Size_x(const float& value) { size.x = value; }
		float Size_y() const { return size.x; }
		void  Size_y(const float& value) { size.x = value; }

		float PointA_x() const { return position.x; }
		void  PointA_x(const float& value) { position.x = value; }

		float PointB_x() const { return position.x + size.x; }
		void  PointB_x(const float& value) { size.x = value - position.x; }

		float PointA_y() const { return position.y; }
		void  PointA_y(const float& value) { position.y = value; }
					 
		float PointB_y() const { return position.y + size.y; }
		void  PointB_y(const float& value) { size.y = value - position.y; }


		float Position(const Cardinal& axis) const { return vectorAxis(axis, position); }
		void  Position(const Cardinal& axis, const float& value) { ref_vectorAxis(axis, position) = value; }

		float Size(const Cardinal& axis) const { return vectorAxis(axis, size); }
		void  Size(const Cardinal& axis, const float& value) { ref_vectorAxis(axis, size) = value; }

		float PointA(const Cardinal& axis) const { return Position(axis); }
		void  PointA(const Cardinal& axis, const float& value) { Position(axis, value); }

		float PointB(const Cardinal& axis) const { return Position(axis) + Size(axis); }
		void  PointB(const Cardinal& axis, const float& value) { Size(axis, value - Position(axis)); }


		float Volume() const { return size.x * size.y; }

		const float& Width() const { return size.x; }
		void         Width(const float& value) { size.x = value; }
		
		const float& Height() const { return size.y; }
		void         Height(const float& value) { size.y = value; }

	public:
		float GetSide(const Cardinal& side) const {
			switch (side.GetValue()) {
			case Cardinal::NORTH: return cmp::min(PointA_y(), PointB_y());
			case Cardinal::EAST:  return cmp::max(PointA_x(), PointB_x());
			case Cardinal::SOUTH: return cmp::max(PointA_y(), PointB_y());
			case Cardinal::WEST:  return cmp::min(PointA_x(), PointB_x());
			default: throw std::invalid_argument("Expected exclusively north, south, east, or west.");
			}
		}

		fvector2 GetCorner(const Cardinal& corner) const {
			return { GetSide(corner & Cardinal::EAST_WEST), GetSide(corner & Cardinal::NORTH_SOUTH) };
		}

		bool Intersects(const CollisionBox& other) const {
			return vectorRangeIntersection(PointA(), PointB(), other.PointA(), other.PointB());
		}

		CollisionBox Smear(const Cardinal& axis, const float& offset) {
			return Smear(axis, { offset, offset });
		}

		CollisionBox Smear(const Cardinal& directions, const fvector2& offset) {
			switch (directions.GetValue()) {
			case Cardinal::EAST:
				if (offset.x >= 0)
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).plusX(offset.x));
				else
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).plusX(offset.x), GetCorner(Cardinal::SOUTH_EAST));
			case Cardinal::SOUTH:
				if (offset.y >= 0)
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).plusY(offset.y));
				else
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).plusY(offset.y), GetCorner(Cardinal::SOUTH_EAST));
			case Cardinal::WEST:
				if (-offset.x >= 0)
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).plusX(-offset.x));
				else
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).plusX(-offset.x), GetCorner(Cardinal::SOUTH_EAST));
			case Cardinal::NORTH:
				if (-offset.y >= 0)
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).plusY(-offset.y));
				else
					return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).plusY(-offset.y), GetCorner(Cardinal::SOUTH_EAST));
			default: throw std::invalid_argument("Expected exclusively east or south.");
			}
		}

	public:
		void Move(const fvector2& distance) { position += distance; }
		void Move(const Cardinal& axis, const float& distance) {
			Position(axis, Position(axis) + distance);
		}

		void SetPositionOnSide(const Cardinal& side, const float& value) {
			Cardinal axis = cardinalAxis(side);
			if (Size(axis) >= 0) {
				switch (side.GetValue()) {
				case Cardinal::NORTH: position.y = value; break;
				case Cardinal::EAST:  position.x = value - size.x; break;
				case Cardinal::SOUTH: position.y = value - size.y; break;
				case Cardinal::WEST:  position.x = value; break;
				default: throw std::invalid_argument("Expected exclusively north, east, south, or west");
				}
			}
			else {
				switch (side.GetValue()) {
				case Cardinal::NORTH: position.y = value - size.y; break;
				case Cardinal::EAST:  position.x = value; break;
				case Cardinal::SOUTH: position.y = value; break;
				case Cardinal::WEST:  position.x = value - size.x; break;
				default: throw std::invalid_argument("Expected exclusively north, east, south, or west");
				}
			}
		}

	private:
		fvector2
			position = { 0, 0 },
			size = { 0, 0 };

		// o-----------------o
		// | identification: |
		// o-----------------o

	public:
		bool IsDynamic() const { return isDynamic(); }
	private:
		virtual bool isDynamic() const { return false; }
	};




	// o===============o
	// | DynamicEntity |
	// o===============o

	class DynamicEntity : public CollisionBox {
		friend class Engine;
	public: // Constructors:
		DynamicEntity(const fvector2& position, const fvector2& size, const float& mass) : CollisionBox(position, size) { this->mass = mass; }
		DynamicEntity() = default;

	public: // Properties:
		const fvector2& NetForce() const { return netForce; }
		void            NetForce(const fvector2& value) { netForce = value; }

		const fvector2& Velocity() const { return velocity; }
		void            Velocity(const fvector2& value) { velocity = value; }

		const float& Mass() const { return mass; }
		void         Mass(const float& value) { mass = value; }

		const float& Bounce() const { return bounce; }
		void         Bounce(const float& value) { bounce = value; }

		const float& Drag() const { return drag; }
		void         Drag(const float& value) { drag = value; }

		const float& NetForce(const Cardinal& axis) const { return vectorAxis(axis, netForce); }
		void         NetForce(const Cardinal& axis, const float& value) { ref_vectorAxis(axis, netForce) = value; }

		const float& Velocity(const Cardinal& axis) const { return vectorAxis(axis, velocity); }
		void         Velocity(const Cardinal& axis, const float& value) { ref_vectorAxis(axis, velocity) = value; }
		
		bool Hitting(const Cardinal& side) const { return (bool)(hitting & side); }
		bool Touching(const Cardinal& side) const { return (bool)(touching & side); }


		float Density() const { mass / Volume(); }
		void  Density(const float& value) { mass = value * Volume(); }
		
	public:
		void AddForce(const fvector2& force) { netForce += force; }

		void ApplyNetForce(Cardinal axis, float timeScale) {
			switch (axis.GetValue()) {
			case Cardinal::EAST:  velocity.x += netForce.x * timeScale; break;
			case Cardinal::SOUTH: velocity.y += netForce.y * timeScale; break;
			case Cardinal::SOUTH_EAST: Move(velocity += netForce * timeScale); break;
			default: throw std::invalid_argument("Expected exclusively east, south, or south-east.");
			}
		}

		void ApplyVelocity(Cardinal axis, float timeScale) {
			switch (axis.GetValue()) {
			case Cardinal::EAST:  Move(velocity.selectX() * timeScale); break;
			case Cardinal::SOUTH: Move(velocity.selectY() * timeScale); break;
			case Cardinal::SOUTH_EAST: Move(velocity * timeScale); break;
			default: throw std::invalid_argument("Expected exclusively east, south, or south-east.");
			}
		}

		void ClearNetForce(Cardinal axis) {
			switch (axis.GetValue()) {
			case Cardinal::EAST: netForce.x = 0; break;
			case Cardinal::SOUTH: netForce.y = 0; break;
			case Cardinal::SOUTH_EAST: netForce = { 0, 0 }; break;
			default: throw std::invalid_argument("Expected exclusively east, south, or south-east.");
			}
		}

		CollisionBox GetVelocitySmear(Cardinal axis, float timeScale) {
			return Smear(axis, velocity * timeScale);
		}

		Cardinal GetHeading() {
			uint8_t result = 0;
			if (velocity.x >= 0) result |= Cardinal::EAST;
			else                 result |= Cardinal::WEST;

			if (velocity.y >= 0) result |= Cardinal::SOUTH;
			else                 result |= Cardinal::NORTH;
			return result;
		}

	public: // Collision methods:
		bool Collides(Cardinal axis, const CollisionBox& other, float timeScale, float& out_collisionSpot, Cardinal& out_collisionSide) {
			out_collisionSide = (axis | axis.Flipped()) & GetHeading();
			out_collisionSpot = other.GetSide((out_collisionSide).Flipped());

			return Collides(axis, other, timeScale);
		}

		bool Collides(Cardinal axis, const CollisionBox& other, float timeScale) {
			if ((axis != Cardinal::EAST) || (axis != Cardinal::SOUTH)) throw std::invalid_argument("Expected exclusively east or south.");

			return (GetVelocitySmear(axis, timeScale).Intersects(other));
		}

		bool Touches(Cardinal side, const CollisionBox& other) {
			return Smear(side, touching_threshold).Intersects(other);
		}

	private: // Fields:
		fvector2
			netForce = { 0, 0 },
			velocity = { 0, 0 };
		float
			mass = 1,
			bounce = -0.5,
			friction = 0.1,
			drag = 0.1;

		Cardinal
			hitting,
			touching;
		// hitting-touching (bits):
		// 3 north
		// 2 east
		// 1 south
		// 0 west
		// same as Cardinal::Value

		static const float touching_threshold; // defined in cpp file.

	private:
		void setHitting(const Cardinal& side, const bool& value) {
			if (value)
				hitting = hitting | side;
			else
				hitting = hitting & ~side;
		}

		void setTouching(const Cardinal& side, const bool& value) {
			if (value)
				touching = touching | side;
			else
				touching = touching & ~side;
		}

	private:
		bool isDynamic() const override { return true; }
	};



	// o========o
	// | Engine |
	// o========o

	class Engine {
	public: // properties:
		const fvector2& Gravity() { return gravity; }
		void            Gravity(const fvector2& value) { gravity = value; }

		const float& AirDensity() { return airDensity; }
		void         AirDensity(const float& value) { airDensity = value; }

		const float& TimeScale() { return timeScale; }
		void         TimeScale(const float& value) { timeScale = value; }
	public: // entity management:
		void AddEntity(CollisionBox* e) {
			if (e->IsDynamic()) dynamicEntities.insert((DynamicEntity*)e);
			entities.insert(e);
		}

		void RemoveEntity(CollisionBox* e) {
			if (e->IsDynamic()) dynamicEntities.erase((DynamicEntity*)e);
			entities.erase(e);
		}

		void DeleteEntity(CollisionBox* e) {
			RemoveEntity(e);
			delete e;
		}

		std::set<CollisionBox*>::const_iterator entities_cbegin() const { return entities.cbegin(); }
		std::set<CollisionBox*>::const_iterator entities_cend() const { return entities.cend(); }

		std::set<DynamicEntity*>::const_iterator dynamicEntities_cbegin() const { return dynamicEntities.cbegin(); }
		std::set<DynamicEntity*>::const_iterator dynamicEntities_cend() const { return dynamicEntities.cend(); }

	public: // collisions:
		void ApplyGravity(Cardinal axis) {
			for (auto dep : dynamicEntities) {
				dep->AddForce(selectVectorAxis(axis, gravity) * dep->Mass());
			}
		}

		void ApplyAirResistance(Cardinal axis) {
			for (auto dep : dynamicEntities) {
				fvector2 force = { 0, 0 };
				float velocity = dep->Velocity(axis);
				ref_vectorAxis(axis, force) = velocity * std::abs(velocity) * -0.5 * airDensity * dep->Drag() * dep->Size(toggleAxis(axis));

				dep->AddForce(force);
			}
		}

		void ApplyNetForce(Cardinal axis, float timeScale) {
			for (auto dep : dynamicEntities) {
				dep->ApplyNetForce(axis, timeScale);
			}
		}

		void ApplyVelocity(Cardinal axis, float timeScale) {
			for (auto dep : dynamicEntities) {
				dep->ApplyVelocity(axis, timeScale);
			}
		}
		
		void ClearNetForce(Cardinal axis) {
			for (auto dep : dynamicEntities) {
				dep->ClearNetForce(axis);
			}
		}

		void RunCollisions(Cardinal axis, float timeScale) {
			this->timeScale = timeScale; // record timescale

			// Variables that will be needed later:
			float out_collisionSpot;
			Cardinal out_collisionSide;
			Cardinal closestSide;
			float closestSpot = 0;
			bool collided = false;
			bool out_touching = false;
			Cardinal touchingSide = Cardinal::NONE;
			//

			for (DynamicEntity* dep : dynamicEntities) {
				DynamicEntity& d = *dep;

				for (CollisionBox* ep : entities) {
					if (ep == dep) continue; // Don't check for collisions against yourself.
					CollisionBox& e = *ep;

					if (d.Collides(axis, e, timeScale, out_collisionSpot, out_collisionSide)) {
						if (collided) {
							closestSpot = cmp::closest(closestSpot, out_collisionSpot, d.GetSide(out_collisionSide));
							closestSide = out_collisionSide; 
						}
						else { 
							closestSpot = out_collisionSpot;
							closestSide = out_collisionSide;
							collided = true;
						}
					}

					if (d.Touches(axis, e)) touchingSide |= axis;
					if (d.Touches(axis.Flipped(), e)) touchingSide |= axis.Flipped();

				}

				if (collided) {
					d.SetPositionOnSide(closestSide, closestSpot);
					d.setHitting(closestSide, true);
					ref_vectorAxis(axis, d.velocity) *= d.bounce;
				}
				else {
					d.setHitting(axis | axis.Flipped(), false);
				}

				d.setTouching(axis | axis.Flipped(), false);
				d.setTouching(touchingSide, true);
			}
		}

		void Update(Cardinal axis = Cardinal::SOUTH_EAST) {
			if (axis.has(Cardinal::EAST)) {
				ApplyGravity(Cardinal::EAST);
				ApplyAirResistance(Cardinal::EAST);
				ApplyNetForce(Cardinal::EAST, timeScale);
				RunCollisions(Cardinal::EAST, timeScale);
				ApplyVelocity(Cardinal::EAST, timeScale);
				ClearNetForce(Cardinal::EAST);
			}

			if (axis.has(Cardinal::SOUTH)) {
				ApplyGravity(Cardinal::SOUTH);
				ApplyAirResistance(Cardinal::SOUTH);
				ApplyNetForce(Cardinal::SOUTH, timeScale);
				RunCollisions(Cardinal::SOUTH, timeScale);
				ApplyVelocity(Cardinal::SOUTH, timeScale);
				ClearNetForce(Cardinal::SOUTH);
			}
		}

	private:
		std::set<CollisionBox*> entities;
		std::set<DynamicEntity*> dynamicEntities;
		float timeScale = 1;


		fvector2 gravity = { 0, 0 };
		float airDensity = 1;
	};
}