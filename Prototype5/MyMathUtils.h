#pragma once
#include<string>
#include<iostream>

namespace JesseRussell {
	namespace cmp {
		bool rangeIntersection(const float& a1, const float& b1, const float& a2, const float& b2);
		float min(const float& a, const float& b);
		float max(const float& a, const float& b);

		float& ref_min(float& a, float& b);
		float& ref_max(float& a, float& b);

		float closest(const float& a, const float& b, const float& from);
		float farthest(const float& a, const float& b, const float& from);

		bool closer(const float& a, const float& b, const float& from);
		bool further(const float& a, const float& b, const float& from);

		int sign(const float& num);

		int sign(const double& num);

		int sign(const int& num);
	}

	namespace math {
		float square(const float& value);
	}

	namespace vectors {
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

				NORTH_EAST = 0b1100,
				SOUTH_EAST = 0b0110,
				SOUTH_WEST = 0b0011,
				NORTH_WEST = 0b1001,

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
			operator Value() const { return value; }
			operator uint8_t() const { return value; }
			operator bool() { return (bool)value; }

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

			bool isVertical()  const { return value & NORTH_SOUTH; }
			bool isHorizontal() const { return value & EAST_WEST; }

			Cardinal SelectAxis(const Cardinal& axis) const {
				return
					(axis.isHorizontal() ? value & EAST_WEST : Cardinal::NONE) |
					(axis.isVertical() ? value & NORTH_SOUTH : Cardinal::NONE);
			}

			Cardinal Flipped() const {
				return
					((value & 0b1100) >> 2) |
					((value & 0b0011) << 2);
			}

			Cardinal Mirrored_horizontally() const {
				return
					((value & 0b0100) >> 2) |
					((value & 0b0001) << 2);
			}
			Cardinal Mirrored_vertically() const {
				return
					((value & 0b1000) >> 2) |
					((value & 0b0010) << 2);
			}

			Cardinal Rotated_clockwise() const {
				return
					((value & 0b1110) >> 1) |
					((value & 0b0001) << 3);
			}

			Cardinal Rotated_counterClockwise() const {
				return
					((value & 0b0111) << 1) |
					((value & 0b1000) >> 3);
			}


			Cardinal Mirrored(const Cardinal& axis) const {
				return
					(axis.isHorizontal() ? Mirrored_horizontally() : SelectAxis(Cardinal::EAST_WEST)) |
					(axis.isVertical()   ? Mirrored_vertically()   : SelectAxis(Cardinal::NORTH_SOUTH));
			}

			Cardinal AxisSwapped() const {
				return
					((value & 0b1010) >> 1) |
					((value & 0b0101) << 1);
			}
			const Value& GetValue() const { return value; }

		private:
			Value value = NONE;
		};

		struct fvector2 {
			// fields:
			float
				x = 0,
				y = 0;

			// Constructors:
			fvector2() = default;
			fvector2(float x, float y) { this->x = x; this->y = y; }

			// Methods:
				// in-place modification:
			void Set(const fvector2& value) { x = value.x; y = value.y; }
			void Set(const float& x, const float& y) { this->x = x; this->y = y; }

			void Transform(const fvector2& i, const fvector2& j) {
				float oldx = x;
				x = x * i.x + y * j.x;
				y = oldx * i.y + y * j.y;
			}

			void Normalize() {
				float mag = GetMagnitude();
				x /= mag;
				y /= mag;
			}

			void AddAxis(const Cardinal& axis, const float& value) {
				if (axis.isHorizontal()) x += value;
				if (axis.isVertical()) y += value;
			}
			void AddAxis(const Cardinal& axis, const fvector2& other) {
				if (axis.isHorizontal()) x += other.x;
				if (axis.isVertical()) y += other.y;
			}

			void SubtractAxis(const Cardinal& axis, const float& value) {
				if (axis.isHorizontal()) x -= value;
				if (axis.isVertical()) y -= value;
			}
			void SubtractAxis(const Cardinal& axis, const fvector2& other) {
				if (axis.isHorizontal()) x -= other.x;
				if (axis.isVertical()) y -= other.y;
			}

			void MultiplyAxis(const Cardinal& axis, const float& value) {
				if (axis.isHorizontal()) x *= value;
				if (axis.isVertical()) y *= value;
			}
			void MultiplyAxis(const Cardinal& axis, const fvector2& other) {
				if (axis.isHorizontal()) x *= other.x;
				if (axis.isVertical()) y *= other.y;
			}

			void DivideAxis(const Cardinal& axis, const float& value) {
				if (axis.isHorizontal()) x /= value;
				if (axis.isVertical()) y /= value;
			}
			void DivideAxis(const Cardinal& axis, const fvector2& other) {
				if (axis.isHorizontal()) x /= other.x;
				if (axis.isVertical()) y /= other.y;
			}

			void ReplaceAxis(const Cardinal& axis, const float& value) {
				if (axis.isHorizontal()) x = value;
				if (axis.isVertical()) y = value;
			}

			void ReplaceAxis(const Cardinal& axis, const fvector2& other) {
				if (axis.isHorizontal()) x = other.x;
				if (axis.isVertical()) y = other.y;
			}
				// out-of-place modification:
			fvector2 Transformed(const fvector2& i, const fvector2& j) const {
				fvector2 result = Clone();
				result.Transform(i, j);
				return result;
			}

			fvector2 Normalized() {
				fvector2 result = Clone();
				result.Normalize();
				return result;
			}

			fvector2 Clone() const { return *this; }

			fvector2 WithX(const float& x) const { return fvector2(x, y); }
			fvector2 WithY(const float& y) const { return fvector2(x, y); }
			fvector2 WithX(const fvector2& other) const { return fvector2(other.x, y); }
			fvector2 WithY(const fvector2& other) const { return fvector2(x, other.y); }

			fvector2 PlusX(const float& x) const { return fvector2(this->x + x, y); }
			fvector2 PlusY(const float& y) const { return fvector2(x, this->y + y); }
			fvector2 PlusX(const fvector2& other) const { return fvector2(x + other.x, y); }
			fvector2 PlusY(const fvector2& other) const { return fvector2(x, y + other.y); }

			fvector2 MinusX(const float& x) const { return fvector2(this->x - x, y); }
			fvector2 MinusY(const float& y) const { return fvector2(x, this->y - y); }
			fvector2 MinusX(const fvector2& other) const { return fvector2(x - other.x, y); }
			fvector2 MinusY(const fvector2& other) const { return fvector2(x, y - other.y); }

			fvector2 TimesX(const float& x) const { return fvector2(this->x * x, y); }
			fvector2 TimesY(const float& y) const { return fvector2(x, this->y * y); }
			fvector2 TimesX(const fvector2& other) const { return fvector2(x * other.x, y); }
			fvector2 TimesY(const fvector2& other) const { return fvector2(x, y * other.y); }

			fvector2 DivbyX(const float& x) const { return fvector2(this->x / x, y); }
			fvector2 DivbyY(const float& y) const { return fvector2(x, this->y / y); }
			fvector2 DivbyX(const fvector2& other) const { return fvector2(x / other.x, y); }
			fvector2 DivbyY(const fvector2& other) const { return fvector2(x, y / other.y); }

			fvector2 SelectX() const { return { x, 0 }; }
			fvector2 SelectY() const { return { 0, y }; }

			fvector2 Abs() const { return { std::abs(x), std::abs(y) }; }
				//


			float GetMagnitudeSquared() const { return x * x + y * y; }

			float GetMagnitude() const { return std::sqrt(x * x + y * y); }

			float DistanceSquared(const fvector2& other) const { return (*this - other).GetMagnitudeSquared(); }
			float Distance(const fvector2& other) const { return (*this - other).GetMagnitude(); }

			std::string ToString() const { return "[ " + std::to_string(x) + ", " + std::to_string(y) + " ]"; }

			// operators:
			fvector2& operator+=(const fvector2& other) { x += other.x; y += other.y; return *this; }
			fvector2& operator-=(const fvector2& other) { x -= other.x; y -= other.y; return *this; }
			fvector2& operator*=(const float& scaler) { x *= scaler; y *= scaler; return *this; }
			fvector2& operator/=(const float& scaler) { x /= scaler; y /= scaler; return *this; }


			fvector2 operator+(const fvector2& other) const { return fvector2(x + other.x, y + other.y); }
			fvector2 operator-(const fvector2& other) const { return fvector2(x - other.x, y - other.y); }
			fvector2 operator*(const float& scaler) const { return fvector2(x * scaler, y * scaler); }
			fvector2 operator/(const float& scaler) const { return fvector2(x / scaler, y / scaler); }

			fvector2 operator-() const { return fvector2(-x, -y); }
			fvector2 operator+() const { return *this; }

			bool operator==(const fvector2& other) const { return x == other.x && y == other.y; }
			bool operator!=(const fvector2& other) const { return x != other.x || y != other.y; }

			// Cardinal integration:
			const float& Axis(const Cardinal& axis) const {
				if (axis.isHorizontal()) {
					if (axis.isVertical())
						throw std::invalid_argument("The axis was horizontal and vertical simultaneously when exclusively horizontal or vertical was expected.");
					else
						return x;
				}
				else {
					if (axis.isVertical())
						return y;
					else
						throw std::invalid_argument("The axis was NONE.");
				}
			}

			float& ref_Axis(const Cardinal& axis) {
				if (axis.isHorizontal()) {
					if (axis.isVertical())
						throw std::invalid_argument("The axis was horizontal and vertical simultaneously when exclusively horizontal or vertical was expected.");
					else
						return x;
				}
				else {
					if (axis.isVertical())
						return y;
					else
						throw std::invalid_argument("The axis was NONE.");
				}
			}

			fvector2 SelectAxis(const Cardinal& axis) const {
				return {
					axis.isHorizontal() ? x : 0,
					axis.isVertical() ? y : 0
				};
			}

			fvector2 WithAxis(const Cardinal& axis, const fvector2& other) const {
				return {
					axis.isHorizontal() ? other.x : x,
					axis.isVertical() ? other.y : y
				};
			}
			fvector2 WithAxis(const Cardinal& axis, const float& value) const {
				return {
					axis.isHorizontal() ? value : x,
					axis.isVertical() ? value : y
				};
			}

			fvector2 PlusAxis(const Cardinal& axis, const fvector2& other) const {
				return {
					axis.isHorizontal() ? x + other.x : x,
					axis.isVertical() ? y + other.y : y
				};
			}
			fvector2 PlusAxis(const Cardinal& axis, const float& value) const {
				return {
					axis.isHorizontal() ? x + value : x,
					axis.isVertical() ? y + value : y
				};
			}

			fvector2 MinusAxis(const Cardinal& axis, const fvector2& other) const {
				return {
					axis.isHorizontal() ? x - other.x : x,
					axis.isVertical() ? y - other.y : y
				};
			}
			fvector2 MinusAxis(const Cardinal& axis, const float& value) const {
				return {
					axis.isHorizontal() ? x - value : x,
					axis.isVertical() ? y - value : y
				};
			}

			fvector2 TimesAxis(const Cardinal& axis, const fvector2& other) const {
				return {
					axis.isHorizontal() ? x * other.x : x,
					axis.isVertical() ? y * other.y : y
				};
			}
			fvector2 TimesAxis(const Cardinal& axis, const float& value) const {
				return {
					axis.isHorizontal() ? x * value : x,
					axis.isVertical() ? y * value : y
				};
			}

			fvector2 DivbyAxis(const Cardinal& axis, const fvector2& other) const {
				return {
					axis.isHorizontal() ? x / other.x : x,
					axis.isVertical() ? y / other.y : y
				};
			}
			fvector2 DivbyAxis(const Cardinal& axis, const float& value) const {
				return {
					axis.isHorizontal() ? x / value : x,
					axis.isVertical() ? y / value : y
				};
			}
		};

		fvector2 operator*(const float& scaler, const fvector2& vector);
		fvector2 operator/(const float& scaler, const fvector2& vector);

		std::ostream& operator<< (std::ostream& os, const fvector2& v);

		bool vectorRangeIntersection(const fvector2& a1, const fvector2& b1, const fvector2& a2, const fvector2 b2);
	}
}