#include "MyMathUtils.h"
namespace JesseRussell {
	namespace cmp {
		bool rangeIntersection(const float& a1, const float& b1, const float& a2, const float& b2) {
			if (a1 > b1) {
				if (a2 > b2)
					return b2 < a1&& b1 < a2;
				else
					return a2 < a1&& b1 < b2;
			}
			else {
				if (a2 > b2)
					return b2 < b1&& a1 < a2;
				else
					return a2 < b1&& a1 < b2;
			}
		}
		float min(const float& a, const float& b) { return a > b ? b : a; }
		float max(const float& a, const float& b) { return a < b ? b : a; }

		float& ref_min(float& a, float& b) { return a > b ? b : a; }
		float& ref_max(float& a, float& b) { return a < b ? b : a; }

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

	namespace math {
		float square(const float& value) { return value * value; }
	}

	namespace vectors {
		fvector2 operator*(const float& scaler, const fvector2& vector) { return fvector2(scaler * vector.x, scaler * vector.y); }
		fvector2 operator/(const float& scaler, const fvector2& vector) { return fvector2(scaler / vector.x, scaler / vector.y); }

		std::iostream& operator<< (std::iostream& ios, const fvector2& v) {
			ios << "[ " << std::to_string(v.x) << ", " << std::to_string(v.y) << " ]";
			return ios;
		}

		bool vectorRangeIntersection(const fvector2& a1, const fvector2& b1, const fvector2& a2, const fvector2 b2) {
			return cmp::rangeIntersection(a1.y, b1.y, a2.y, b2.y) &&
				cmp::rangeIntersection(a1.x, b1.x, a2.x, b2.x);
		}
	}
}