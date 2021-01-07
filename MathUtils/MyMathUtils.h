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

		int sign(const float& num);

		int sign(const double& num);

		int sign(const int& num);
	}
	namespace vectors {
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
			void set(const fvector2& value) { x = value.x; y = value.y; }
			void set(const float& x, const float& y) { this->x = x; this->y = y; }

			void transform(const fvector2& i, const fvector2& j) {
				float oldx = x;
				x = x * i.x + y * j.x;
				y = oldx * i.y + y * j.y;
			}

			void normalize() {
				float mag = getMagnitude();
				x /= mag;
				y /= mag;
			}

				// out-of-place modification:
			fvector2 transformed(const fvector2& i, const fvector2& j) const {
				fvector2 result = clone();
				result.transform(i, j);
				return result;
			}

			fvector2 normalized() {
				fvector2 result = clone();
				result.normalize();
				return result;
			}

			fvector2 clone() const { return *this; }

			fvector2 withX(const float& x) const { return fvector2(x, y); }
			fvector2 withY(const float& y) const { return fvector2(x, y); }
			fvector2 withX(const fvector2& other) const { return fvector2(other.x, y); }
			fvector2 withY(const fvector2& other) const { return fvector2(x, other.y); }

			fvector2 plusX(const float& x) const { return fvector2(this->x + x, y); }
			fvector2 plusY(const float& y) const { return fvector2(x, this->y + y); }
			fvector2 plusX(const fvector2& other) const { return fvector2(x + other.x, y); }
			fvector2 plusY(const fvector2& other) const { return fvector2(x, y + other.y); }

			fvector2 minusX(const float& x) const { return fvector2(this->x - x, y); }
			fvector2 minusY(const float& y) const { return fvector2(x, this->y - y); }
			fvector2 minusX(const fvector2& other) const { return fvector2(x - other.x, y); }
			fvector2 minusY(const fvector2& other) const { return fvector2(x, y - other.y); }

			fvector2 timesX(const float& x) const { return fvector2(this->x * x, y); }
			fvector2 timesY(const float& y) const { return fvector2(x, this->y * y); }
			fvector2 timesX(const fvector2& other) const { return fvector2(x * other.x, y); }
			fvector2 timesY(const fvector2& other) const { return fvector2(x, y * other.y); }

			fvector2 divbyX(const float& x) const { return fvector2(this->x / x, y); }
			fvector2 divbyY(const float& y) const { return fvector2(x, this->y / y); }
			fvector2 divbyX(const fvector2& other) const { return fvector2(x / other.x, y); }
			fvector2 divbyY(const fvector2& other) const { return fvector2(x, y / other.y); }

			fvector2 selectX() const { return { x, 0 }; }
			fvector2 selectY() const { return { 0, y }; }

			fvector2 abs() const { return { std::abs(x), std::abs(y) }; }
				//


			float getMagnitudeSquared() const { return x * x + y * y; }

			float getMagnitude() const { return std::sqrt(x * x + y * y); }

			float distanceSquared(const fvector2& other) const { return (*this - other).getMagnitudeSquared(); }
			float distance(const fvector2& other) const { return (*this - other).getMagnitude(); }

			std::string toString() const { return "[ " + std::to_string(x) + ", " + std::to_string(y) + " ]"; }

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
		};

		fvector2 operator*(const float& scaler, const fvector2& vector);
		fvector2 operator/(const float& scaler, const fvector2& vector);

		std::iostream& operator<< (std::iostream& ios, const fvector2& v);

		bool vectorRangeIntersection(const fvector2& a1, const fvector2& b1, const fvector2& a2, const fvector2 b2);
	}
}