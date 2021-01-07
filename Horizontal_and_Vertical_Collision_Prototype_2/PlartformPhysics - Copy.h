#pragma once
#include "MyMathUtils.h"

#include<cmath>
#include<stdexcept>

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
			constexpr Cardinal(const Value& value) { this->value = value; }
			constexpr Cardinal(const uint8_t& value) { this->value = static_cast<Value>(0b00001111 & value); }

		public:
			constexpr operator Value() { return value; }
			constexpr operator bool() = delete;
			constexpr bool operator ==(const Cardinal& other) const { return value == other.value; }
			constexpr bool operator !=(const Cardinal& other) const { return value != other.value; }

			constexpr bool is(const Cardinal& other) const { return value & other.value; }

			constexpr Cardinal Flipped() {
				return
					((0b1100 & value) >> 2) |
					((0b0011 & value) << 2);
			}

			constexpr Cardinal Flipped_horizontally() {
				return
					((0b0100 & value) >> 2) |
					((0b0001 & value) << 2);
			}
			constexpr Cardinal Flipped_vertically() {
				return
					((0b1000 & value) >> 2) |
					((0b0010 & value) << 2);
			}

			constexpr Cardinal Clockwise() {
				return
					((0b1110 & value) >> 1) |
					((0b0001 & value) << 3);
			}

			constexpr Cardinal CounterClockwise() {
				return
					((0b0111 & value) << 1) |
					((0b1000 & value) >> 3);
			}

			constexpr const Value& GetValue() const { return value; }
		private:
			Value value;
		};




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

			fvector2 GetSize() const { return B - A; }
			void     SetSize(const fvector2& value) { B = A + value; }

			fvector2 GetSize_abs() const { return (B - A).abs(); }

			float GetWidth() const  { return B.x - A.x; }
			void  SetWidth(const float& value) { B.x = A.x + value; }

			float GetHeight() const { return B.y - B.y; }
			void  SetHeight(const float& value) { B.y = A.y + value; }

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

			float GetNorth_y() const { return cmp::min(A.y, B.y); }
			float GetSouth_y() const { return cmp::max(B.y, A.y); }
			float GetWest_x()  const { return cmp::min(A.x, B.x); }
			float GetEast_x()  const { return cmp::max(B.x, A.x); }

			void  SetNorth_y(const float value) { ref_North_y() = value; }
			void  SetSouth_y(const float value) { ref_South_y() = value; }
			void  SetWest_x(const float value)  { ref_West_x()  = value; }
			void  SetEast_x(const float value)  { ref_East_x()  = value; }

			float GetSide_x(const Cardinal& at) const {
				if      (at == Cardinal::WEST) return GetWest_x();
				else if (at == Cardinal::EAST) return GetEast_x();
				else throw std::invalid_argument("The Cardinal provided does not represent exclusively East or West");
			}

			void  SetSide_x(const Cardinal& at, const float value) {
				if (at == Cardinal::WEST) return SetWest_x(value);
				else if (at == Cardinal::EAST) return SetEast_x(value);
				else throw std::invalid_argument("The Cardinal provided does not represent exclusively East or West");
			}

			float GetSide_y(const Cardinal& at) const {
				if      (at == Cardinal::NORTH) return GetNorth_y();
				else if (at == Cardinal::SOUTH) return GetSouth_y();
				else throw std::invalid_argument("The Cardinal provided does not represent exclusively North or South");
			}

			void  SetSide_y(const Cardinal& at, const float value) {
				if (at == Cardinal::NORTH) return SetNorth_y(value);
				else if (at == Cardinal::SOUTH) return SetSouth_y(value);
				else throw std::invalid_argument("The Cardinal provided does not represent exclusively East or West");
			}

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

		public: // Methods:
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

			void SetPosition_on(const Cardinal& at, const fvector2& position) {
				switch (at.GetValue())
				{
				case Cardinal::NORTH_WEST: SetPosition_NW(position); break;
				case Cardinal::NORTH_EAST: SetPosition_NE(position); break;
				case Cardinal::SOUTH_EAST: SetPosition_SE(position); break;
				case Cardinal::SOUTH_WEST: SetPosition_SW(position); break;
				default: throw std::invalid_argument("The Cardinal provided does not represent NW, NE, SE, or SW.");
				}
			}

			bool Intersects(const Collider& other) const { return vectorRangeIntersection(A, B, other.A, other.B); }

			bool Intersects_horizontal(const Collider& other) const { return cmp::rangeIntersection(A.x, B.x, other.A.x, other.B.x); }
			bool Intersects_vertical(const Collider& other)   const { return cmp::rangeIntersection(A.y, B.y, other.A.y, other.B.y); }

		private: // Fields:
			fvector2 
				A,
				B;

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
		};


		

		// o--------o
		// | Entity |
		// o--------o

		// Collider with velocity.
		class Entity : public Collider{
			friend class DynamicEntity;
		public: // Constructors:
			Entity(const fvector2& pointA, const fvector2& size) : Collider(pointA, pointA + size) {}
			Entity() : Collider() {}

		public: // Properties:
			const fvector2& GetVelocity() const { return velocity; }
			void            SetVelocity(const fvector2& value) { velocity = value; }
			void            SetVelocity_x(const float& value) { velocity.x = value; }
			void            SetVelocity_y(const float& value) { velocity.y = value; }

			float GetDirectionalVelocity(const Cardinal& direction) {
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
				default: throw std::invalid_argument("");
				}
			}

		public: // Methods:
			Entity Clone() { return *this; }
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
				if      (velocity.y < 0) result &= Cardinal::NORTH;
				else if (velocity.y > 0) result &= Cardinal::SOUTH;
				
				if      (velocity.x < 0) result &= Cardinal::WEST;
				else if (velocity.x > 0) result &= Cardinal::EAST;

				return result;
			}

			bool IsDynamic() { return isDynamic(); }

		private: // Fields:
			fvector2 velocity = { 0, 0 };

		private: // Properties
			virtual bool isDynamic() { return false; }
		};




		// o----------------o
		// | Dynamic Entity |
		// o----------------o

		// Entity with acceleration, mass, and netForce
		class DynamicEntity : public Entity {
		public: // Constructors:
			DynamicEntity(const fvector2& pointA, const fvector2& size, const float& mass) : Entity(pointA, size) {
				this->mass = mass;
			}

		public: // Properties:
			const fvector2& GetNetForce() const { return netForce; }
			void            SetNetForce(const fvector2 value) { netForce = value; }

			const float& GetMass() const { return mass; }
			void         SetMass(const float& value) { mass = value; }

			fvector2 GetAcceleration() const { return netForce / mass; }
			void     SetAcceleration(const fvector2& value) { netForce = value * mass; }

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
				default: throw std::invalid_argument("The provided axis was not exclusively north, south, east, or west.");
				}
			}

			void ApplyVelocity(const Cardinal& axis, const float& timeScale) {
				switch (axis.GetValue())
				{
				case Cardinal::SOUTH: Move(velocity.withY(0); break;
				case Cardinal::NORTH: velocity.y -= netForce.y; break;
				case Cardinal::EAST:  velocity.y += netForce.y; break;
				case Cardinal::WEST:  velocity.y -= netForce.y; break;
				default: throw std::invalid_argument("The provided axis was not exclusively north, south, east, or west.");
				}
			}

			DynamicEntity Clone() { return *this; }

		private: // Fields:
			fvector2 netForce = { 0, 0 };
			float mass;

		private: // Properties:
			virtual bool isDynamic() { return true; }
		};
	}
}