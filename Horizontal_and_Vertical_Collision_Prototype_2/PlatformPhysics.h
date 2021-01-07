#pragma once
#include "MyMathUtils.h"

#include<cmath>
#include<stdexcept>
#include<iostream>

namespace platformPhysics {
	namespace phy {
		using namespace JesseRussell;
		using namespace vectors;

		// o----------o
		// | Cardinal |
		// o----------o

		// Enum wrapper representing cardinal directions.
		class Cardinal {
		public:
			enum Value : uint8_t
			{
				NORTH       = 0B1000,
				EAST        = 0B0100,
				SOUTH       = 0B0010,
				WEST        = 0B0001,
				NORTH_WEST  = 0b1001,
				NORTH_EAST  = 0b1100,
				SOUTH_EAST  = 0b0110,
				SOUTH_WEST  = 0b0011,
			};

		public:
			Cardinal() = default;
			Cardinal(const Value& value) { this->value = value; }
			Cardinal(const uint8_t& value) { this->value = static_cast<Value>(0b00001111 & value); }

		public: // Properties
			uint8_t SelectNorth() const { return value & NORTH; }
			uint8_t SelectEast()  const { return value & EAST; }
			uint8_t SelectSouth() const { return value & SOUTH; }
			uint8_t SelectWest()  const { return value & WEST; }

		public: // conversions:
			 operator Value() const { return value; }
			 operator bool() = delete;
			 operator uint8_t() const { return value; }

		public: // operators:
			 bool operator ==(const Cardinal& other) const { return value == other.value; }
			 bool operator !=(const Cardinal& other) const { return value != other.value; }

		public: // methods:
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

		private:
			Value value;
		};

		std::ostream& operator<< (std::ostream& io, const Cardinal& c) {
			bool space = false;
			if (c.has(Cardinal::NORTH)) {
				io << "north";
				space = true;
			}
			if (c.has(Cardinal::SOUTH)) {
				if (space) io << "-";
				io << "south";
				space = true;
			}
			if (c.has(Cardinal::WEST)) {
				if (space) io << "-";
				io << "west";
				space = true;
			}
			if (c.has(Cardinal::EAST)) {
				if (space) io << "-";
				io << "east";
			}

			return io;
		}


		// o--------------------o
		// | Usefull Functions: |
		// o--------------------o
		Cardinal getAxis(const fvector2& v) {
			uint8_t result = 0;
			if (v.y < 0) result |= Cardinal::NORTH;
			else if (v.y > 0) result |= Cardinal::SOUTH;

			if (v.x < 0) result |= Cardinal::WEST;
			else if (v.x > 0) result |= Cardinal::EAST;

			return result;
		}

		fvector2 selectAxis(const Cardinal& d, const fvector2& v) {
			switch (d.GetValue())
			{
			case Cardinal::NORTH: return -v.selectY();
			case Cardinal::SOUTH: return  v.selectY();
			case Cardinal::WEST:  return -v.selectX();
			case Cardinal::EAST:  return  v.selectX();
			case Cardinal::NORTH_WEST: return (v * -1);
			case Cardinal::NORTH_EAST: return v.timesY(-1);
			case Cardinal::SOUTH_EAST: return v;
			case Cardinal::SOUTH_WEST: return v.timesX(-1);
			default: throw std::invalid_argument("The provided direction was not one of the 8 cardinal directions.");
			}
		}


		// o----------o
		// | Collider |
		// o----------o

		// A square with size and position defined by two points in 2d space.
		class Collider {
			friend class Entity;
			friend class DynamicEntity;
		public: // Constructors:
			Collider(const fvector2& pointA, const fvector2& pointB) {
				A = pointA;
				B = pointB;
			}

			Collider(const fvector2& pointA, const float& width, const float& height) {
				A = pointA;
				B = pointA + fvector2(width, height);
			}

			Collider(const float& x, const float& y, const float& width, const float& height) {
				A = { x, y };
				B = A + fvector2(width, height);
			}

			Collider() {
				A = { 0, 0 };
				B = { 0, 0 };
			}

		public: // Properties:
			const fvector2& GetPointA() const { return A; }
			void            SetPointA(const fvector2& value) { A = value; }
			void            SetPointA_x(const float& value) { A.x = value; }
			void            SetPointA_y(const float& value) { A.y = value; }

			const fvector2& GetPointB() const { return B; }
			void            SetPointB(const fvector2& value) { B = value; }
			void            SetPointB_x(const float& value) { B.x = value; }
			void            SetPointB_y(const float& value) { B.y = value; }

				// corners
			fvector2 GetNW() const { return { GetWest_x(), GetNorth_y()}; }
			fvector2 GetNE() const { return { GetEast_x(), GetNorth_y()}; }
			fvector2 GetSE() const { return { GetEast_x(), GetSouth_y()}; }
			fvector2 GetSW() const { return { GetWest_x(), GetSouth_y()}; }

			fvector2 GetCorner(const Cardinal& at) const {
				switch (at.GetValue())
				{
				case Cardinal::NORTH_WEST: return GetNW();
				case Cardinal::NORTH_EAST: return GetNE();
				case Cardinal::SOUTH_EAST: return GetSE();
				case Cardinal::SOUTH_WEST: return GetSW();
				default: throw std::invalid_argument("The Cardinal provided does not represent exclusively NW, NE, SE, or SW.");
				}
			}

			bool IsPointA_NW() const { return A.x <= B.x && A.y <= B.y; }
			bool IsPointA_NE() const { return A.x >  B.x && A.y <= B.y; }
			bool IsPointA_SE() const { return A.x >  B.x && A.y >  B.y; }
			bool IsPointA_SW() const { return A.x <= B.x && A.y >  B.y; }

			bool IsPointB_NW() const { return B.x <  A.x && B.y <  A.y; }
			bool IsPointB_NE() const { return B.x >= A.x && B.y <  A.y; }
			bool IsPointB_SE() const { return B.x >= A.x && B.y >= A.y; }
			bool IsPointB_SW() const { return B.x <  A.x && B.y >= A.y; }

			bool IsPointA(const Cardinal& corner) {
				switch (corner.GetValue())
				{
				case Cardinal::NORTH_WEST: return IsPointA_NW();
				case Cardinal::NORTH_EAST: return IsPointA_NE();
				case Cardinal::SOUTH_EAST: return IsPointA_SE();
				case Cardinal::SOUTH_WEST: return IsPointA_SW();
				default: throw std::invalid_argument("The Cardinal provided does not represent exclusively NW, NE, SE, or SW.");
				}
			}

			bool IsPointB(const Cardinal& corner) {
				switch (corner.GetValue())
				{
				case Cardinal::NORTH_WEST: return IsPointB_NW();
				case Cardinal::NORTH_EAST: return IsPointB_NE();
				case Cardinal::SOUTH_EAST: return IsPointB_SE();
				case Cardinal::SOUTH_WEST: return IsPointB_SW();
				default: throw std::invalid_argument("The Cardinal provided does not represent exclusively NW, NE, SE, or SW.");
				}
			}
				//

				// size, width, height
			fvector2 GetSize() const { return B - A; }
			void     SetSize(const fvector2& value) { B = A + value; }

			float GetWidth() const  { return B.x - A.x; }
			void  SetWidth(const float& value) { B.x = A.x + value; }

			float GetHeight() const { return B.y - B.y; }
			void  SetHeight(const float& value) { B.y = A.y + value; }
				//

				// sides:
			float GetNorth_y() const { return cmp::min(A.y, B.y); }
			float GetSouth_y() const { return cmp::max(B.y, A.y); }
			float GetWest_x()  const { return cmp::min(A.x, B.x); }
			float GetEast_x()  const { return cmp::max(B.x, A.x); }

			void  SetNorth_y(const float value) { ref_North_y() = value; }
			void  SetSouth_y(const float value) { ref_South_y() = value; }
			void  SetWest_x(const float value)  { ref_West_x()  = value; }
			void  SetEast_x(const float value)  { ref_East_x()  = value; }

			float GetSide(const Cardinal& side) const {
				switch (side.GetValue())
				{
				case Cardinal::NORTH: return GetNorth_y();
				case Cardinal::EAST:  return GetEast_x();
				case Cardinal::SOUTH: return GetSouth_y();
				case Cardinal::WEST:  return GetWest_x();
				default: throw std::invalid_argument("The side specified was not exclusively horizontal or vertical.");
				}
			}

			void  SetSide(const Cardinal& side, const float& value) { ref_Side(side) = value; }
				//

		public: // Methods:
				// in place modification:
			void Move(const fvector2& distance) {
				A += distance;
				B += distance;
			}

			void Move_horizontally(const float& distance) { 
				A.x += distance; 
				B.x += distance;
			}

			void Move_vertically(const float& distance) { 
				A.y += distance; 
				B.y += distance;
			}

			void SetPosition_NW(const fvector2& value) {
				ref_East_x()  = value.x + GetWidth();
				ref_South_y() = value.y + GetHeight();
				ref_West_x()  = value.x;
				ref_North_y() = value.y;
			}

			void SetPosition_NE(const fvector2& value) {
				ref_West_x()  = value.x + GetWidth();
				ref_South_y() = value.y + GetHeight();
				ref_East_x()  = value.x;
				ref_North_y() = value.y;
			}

			void SetPosition_SE(const fvector2& value) {
				ref_West_x()  = value.x + GetWidth();
				ref_North_y() = value.y + GetHeight();
				ref_East_x()  = value.x;
				ref_South_y() = value.y;
			}

			void SetPosition_SW(const fvector2& value) {
				ref_East_x()  = value.x + GetWidth();
				ref_North_y() = value.y + GetHeight();
				ref_West_x()  = value.x;
				ref_South_y() = value.y;
			}

			void SetPosition_on(const fvector2& position, const Cardinal& at = Cardinal::NORTH_WEST) {
				switch (at.GetValue())
				{
				case Cardinal::NORTH_WEST: SetPosition_NW(position); break;
				case Cardinal::NORTH_EAST: SetPosition_NE(position); break;
				case Cardinal::SOUTH_EAST: SetPosition_SE(position); break;
				case Cardinal::SOUTH_WEST: SetPosition_SW(position); break;
				default: throw std::invalid_argument("The Cardinal provided does not represent NW, NE, SE, or SW.");
				}
			}
				//

			bool Intersects(const Collider& other) const { return vectorRangeIntersection(A, B, other.A, other.B); }

			bool Intersects_horizontal(const Collider& other) const { return cmp::rangeIntersection(A.x, B.x, other.A.x, other.B.x); }
			bool Intersects_vertical(const Collider& other)   const { return cmp::rangeIntersection(A.y, B.y, other.A.y, other.B.y); }

				// out of place modification:
			Collider Clone() const { return *this; }

			Collider Moved(const fvector2& distance) const {
				Collider clone = Clone();
				clone.Move(distance);
				return clone;
			}

			Collider Moved_horizontally(const float& distance) const {
				Collider clone = Clone();
				clone.Move_horizontally(distance);
				return clone;
			}

			Collider Moved_vertically(const float& distance) const {
				Collider clone = Clone();
				clone.Move_vertically(distance);
				return clone;
			}

			Collider At_NW(const fvector2& position) {
				Collider result = Clone();
				result.SetPosition_NW(position);
				return result;
			}

			Collider At_NE(const fvector2& position) {
				Collider result = Clone();
				result.SetPosition_NE(position);
				return result;
			}

			Collider At_SE(const fvector2& position) {
				Collider result = Clone();
				result.SetPosition_SE(position);
				return result;
			}

			Collider At_SW(const fvector2& position) {
				Collider result = Clone();
				result.SetPosition_SW(position);
				return result;
			}

			Collider At(const fvector2& position, const Cardinal& originCorner = Cardinal::NORTH_WEST) {
				Collider result = Clone();
				result.SetPosition_on(position, originCorner);
				return result;
			}
				//

			bool IsEntity()  const { return isEntity(); }
			bool IsDynamic() const { return isDynamic(); }
		private: // Fields:
			fvector2 
				A = { 0, 0 },
				B = { 0, 0 };

		private: // Methods:
			float& ref_North_y() { return cmp::ref_min(A.y, B.y); }
			float& ref_South_y() { return cmp::ref_max(B.y, A.y); }
			float& ref_West_x()  { return cmp::ref_min(A.x, B.x); }
			float& ref_East_x()  { return cmp::ref_max(B.x, A.x); }
			float& ref_Side(const Cardinal& side) {
				switch (side.GetValue()){
				case Cardinal::NORTH: return ref_North_y();
				case Cardinal::SOUTH: return ref_South_y();
				case Cardinal::WEST:  return ref_West_x();
				case Cardinal::EAST:  return ref_East_x();
				}
			}

			virtual bool isEntity()  const { return false; }
			virtual bool isDynamic() const  { return false; }
		};


		

		// o--------o
		// | Entity |
		// o--------o

		// Collider with velocity.
		class Entity : public Collider{
			friend class DynamicEntity;
		public: // Constructors:
			Entity(const fvector2& pointA, const fvector2& pointB) : Collider(pointA, pointB) {}
			Entity(const fvector2& pointA, const float& width, const float& height) : Collider(pointA, width, height) {}
			Entity(const float& x, const float& y, const float& width, const float& height) : Collider(x, y, width, height) {}
			Entity() = default;
		public: // Properties:
				// velocity
			const fvector2& GetVelocity() const { return velocity; }
			void            SetVelocity(const fvector2& value) { velocity = value; }
			void            SetVelocity_x(const float& value) { velocity.x = value; }
			void            SetVelocity_y(const float& value) { velocity.y = value; }

			float GetVelocity(const Cardinal& direction) {
				switch (direction.GetValue())
				{
				case Cardinal::NORTH: return -velocity.y;
				case Cardinal::SOUTH: return  velocity.y;
				case Cardinal::WEST:  return -velocity.x;
				case Cardinal::EAST:  return  velocity.x;
				case Cardinal::NORTH_WEST: return (velocity * -1).getMagnitude();
				case Cardinal::NORTH_EAST: return velocity.timesY(-1).getMagnitude();
				case Cardinal::SOUTH_EAST: return velocity.getMagnitude();
				case Cardinal::SOUTH_WEST: return velocity.timesX(-1).getMagnitude();
				default: throw std::invalid_argument("The provided direction was not one of the 8 cardinal directions.");
				}
			}

			void SetVelocity(const Cardinal& axis, const float& value) {
				switch (axis.GetValue()) {
				case Cardinal::EAST:  velocity.x =  value; break;
				case Cardinal::WEST:  velocity.x = -value; break;
				case Cardinal::SOUTH: velocity.y =  value; break;
				case Cardinal::NORTH: velocity.y = -value; break;
				case Cardinal::NORTH_WEST:
					velocity.x = -value / sqrt_2;
					velocity.y = -value / sqrt_2;
					break;
				case Cardinal::NORTH_EAST: 
					velocity.x =  value / sqrt_2;
					velocity.y = -value / sqrt_2;
					break;
				case Cardinal::SOUTH_EAST: 
					velocity.x =  value / sqrt_2;
					velocity.y =  value / sqrt_2;
					break;
				case Cardinal::SOUTH_WEST: 
					velocity.x = -value / sqrt_2;
					velocity.y =  value / sqrt_2;
					break;
				default: throw std::invalid_argument("The provided axis was not one of the 8 cardinal directions.");
				}
			}
				//

				//heading:
			fvector2 GetHeadingNorthCorner() {
				if (velocity.x < 0)
					return GetNW();
				else
					return GetNE();
			}
			fvector2 GetHeadingSouthCorner() {
				if (velocity.x < 0)
					return GetSW();
				else
					return GetSE();
			}
			fvector2 GetHeadingWestCorner() {
				if (velocity.y < 0)
					return GetNW();
				else
					return GetSW();
			}

			Cardinal GetHeading() {
				uint8_t result = 0;
				if (velocity.y < 0) result |= Cardinal::NORTH;
				else if (velocity.y > 0) result |= Cardinal::SOUTH;

				if (velocity.x < 0) result |= Cardinal::WEST;
				else if (velocity.x > 0) result |= Cardinal::EAST;

				return result;
			}
				//

			bool IsDynamic() const { return isDynamic(); }

		public: // Methods:
				// out of place modification:
			Entity Clone() { return *this; }
				//

			// Returns a collider represnting the area carved out by this object plus it's velocity.
			Collider GetCollisionSmear(const float& timeScale, const Cardinal& axis) const {
				switch (axis.GetValue())
				{
				case Cardinal::EAST:  return Collider(GetNW(), GetSE().plusX(velocity));
				case Cardinal::WEST:  return Collider(GetNW().minusX(velocity), GetSE());
				case Cardinal::SOUTH: return Collider(GetNW(), GetSE().plusY(velocity));
				case Cardinal::NORTH: return Collider(GetNW().minusY(velocity), GetSE());
				default: throw std::invalid_argument("The axis given was not exclusively north, east, south, or west.");
				}
			}

		public: // static Methods:
			// factories:
			static Entity FromCollider(const Collider& col) { return Entity(col.A, col.B); }

		private: // Fields:
			fvector2 velocity = { 0, 0 };

		private: // Constants:
			static const float sqrt_2;

		private: // Properties
			virtual bool isEntity() const override { return true; }
		};




		// o----------------o
		// | Dynamic Entity |
		// o----------------o

		// Entity with acceleration, mass, and netForce
		class DynamicEntity : public Entity {
		public: // Constructors:
			DynamicEntity(const fvector2& pointA, const fvector2& pointB, const float& mass) : Entity(pointA, pointB) { this->mass = mass; }
			DynamicEntity(const fvector2& pointA, const float& width, const float& height, const float& mass) : Entity(pointA, width, height) { this->mass = mass; }
			DynamicEntity(const float& x, float& y, const float& width, const float& height, const float& mass) : Entity(x, y, width, height) { this->mass = mass; }
			DynamicEntity() = default;

		public: // Properties:
			const fvector2& GetNetForce() const { return netForce; }
			void            SetNetForce(const fvector2& value) { netForce = value; }
			void            SetNetForce_x(const float& value) { netForce.x = value; }
			void            SetNetForce_y(const float& value) { netForce.y = value; }

			const float& GetMass() const { return mass; }
			void         SetMass(const float& value) { mass = value; }

			fvector2 GetAcceleration() const { return netForce / mass; }
			void     SetAcceleration(const fvector2& value) { netForce = value * mass; }
			void     SetAcceleration_x(const float& value) { netForce.x = value * mass; }
			void     SetAcceleration_y(const float& value) { netForce.y = value * mass; }

		public: // Methods:
			void AddForce(const fvector2& force) { netForce += force; }
			void ResetNetForce() { netForce = { 0, 0 }; }
			void ApplyNetForce() { velocity += netForce / mass; }
			void ApplyVelocity(const float& timeScale = 1.0f) { Move(velocity * timeScale); }

			void ApplyNetForce(const Cardinal& axis) {
				switch (axis.GetValue())
				{
				case Cardinal::SOUTH: velocity.y += netForce.y; break;
				case Cardinal::NORTH: velocity.y -= netForce.y; break;
				case Cardinal::EAST:  velocity.y += netForce.y; break;
				case Cardinal::WEST:  velocity.y -= netForce.y; break;
				case Cardinal::SOUTH_EAST:
					velocity.x += netForce.y;
					velocity.y += netForce.x;
					break;
				case Cardinal::NORTH_EAST:
					velocity.x += netForce.y;
					velocity.y -= netForce.x;
					break;
				case Cardinal::SOUTH_WEST:
					velocity.x -= netForce.y;
					velocity.y += netForce.x;
					break;
				case Cardinal::NORTH_WEST:
					velocity.x -= netForce.y;
					velocity.y -= netForce.x;
					break;
				default: throw std::invalid_argument("The provided axis was not one of the 8 cardinal directions.");
				}
			}

			void ApplyVelocity(const Cardinal& axis, const float& timeScale) {
				switch (axis.GetValue())
				{
				case Cardinal::SOUTH: Move(velocity.withX(0)); break;
				case Cardinal::NORTH: Move(velocity.withX(0).timesY(-1)); break;
				case Cardinal::EAST:  Move(velocity.withY(0)); break;
				case Cardinal::WEST:  Move(velocity.withY(0).timesX(-1)); break;
				case Cardinal::SOUTH_EAST: Move({  velocity.x,  velocity.y }); break;
				case Cardinal::NORTH_EAST: Move({  velocity.x, -velocity.y }); break;
				case Cardinal::SOUTH_WEST: Move({ -velocity.x,  velocity.y }); break;
				case Cardinal::NORTH_WEST: Move({ -velocity.x, -velocity.y }); break;
				default: throw std::invalid_argument("The provided axis was not one of the 8 cardinal directions.");
				}
			}

			DynamicEntity Clone() { return *this; }

			// Collisions:
			bool collides_static(const Collider* other, const Cardinal& axis, const float& timeScale, float &out_collisionSpot_NW, Cardinal& out_collisionSide) const {
				Entity other_ent;
				if (other->IsEntity()) other_ent = *(Entity*)other;
				else other_ent = Entity::FromCollider(*other);

				Collider colsmr = GetCollisionSmear(timeScale, axis);
				Collider other_colsmr = other_ent.GetCollisionSmear(timeScale, axis);

				if (colsmr.Intersects(other_colsmr)) {
					out_collisionSide = (other_ent);
					
				}
				
			};
			//

		public: // static Methods:
			static DynamicEntity FromCollider(const Collider& col, const float& mass) { return DynamicEntity(col.GetPointA(), col.GetPointB(), mass); }
			static DynamicEntity FromEntity(const Entity& e, const float& mass) { return DynamicEntity(e.GetPointA(), e.GetPointB(), mass); }

		private: // Fields:
			fvector2 netForce = { 0.0f, 0.0f };
			float mass = 1.0f;

		private: // Properties:
			virtual bool isDynamic() const override { return true; }
		};
	}
}