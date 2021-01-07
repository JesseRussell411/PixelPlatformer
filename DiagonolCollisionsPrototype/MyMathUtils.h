#pragma once
#include<string>
#include<iostream>

namespace JesseRussell {
	namespace cmp {
		float min(const float& a, const float& b) { return a > b ? b : a; }
		float max(const float& a, const float& b) { return a < b ? b : a; }
		float closest(const float& a, const float& b, const float& from) {
			return std::abs(from - a) > std::abs(from - b) ? b : a;
		}
		float farthest(const float& a, const float& b, const float& from) {
			return std::abs(from - a) < std::abs(from - b) ? b : a;
		}

		int sign(const float& num) {
			return std::signbit(num) ? -1 : num == 0 ? 0 : 1;
		}

		int sign(const double& num) {
			return std::signbit(num) ? -1 : num == 0 ? 0 : 1;
		}

		int sign(const int& num) {
			return num == 0 ? 0 : num < 0 ? -1 : 1;
		}
	}
	namespace vectors {
		struct fvector2 {
			// fields:
			float x, y;

			// Constructors:
			fvector2() { x = 0, y = 0; }
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

				// out-of-place modification:
			fvector2 transformed(const fvector2& i, const fvector2& j) const {
				fvector2 result = clone();
				result.transform(i, j);
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
				//


			float getMagnitudeSquared() const {
				return x * x + y * y;
			}

			float getMagnitude() const {
				return std::sqrt(x * x + y * y);
			}

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

		fvector2 operator*(const float& scaler, const fvector2& vector) { return fvector2(scaler * vector.x, scaler * vector.y); }
		fvector2 operator/(const float& scaler, const fvector2& vector) { return fvector2(scaler / vector.x, scaler / vector.y); }

		std::iostream& operator<< (std::iostream& ios, const fvector2& v) {
			ios << "[ " << std::to_string(v.x) << ", " << std::to_string(v.y) << " ]";
			return ios;
		}
	}
}