#include "PlatformPhysics.h"

namespace phy {
	// o------------o
	// | Constants: |
	// o------------o

	const float DynamicEntity::touching_threshold = 2.0f;



	// o=========================================================o
	// | Useful function for working with cardinals and vectors: |
	// o=========================================================o

	float& ref_vectorAxis(Cardinal axis, fvector2& v) {
		switch (axis.GetValue()) {
		case Cardinal::EAST: return v.x;
		case Cardinal::SOUTH: return v.y;
		default: throw std::invalid_argument("Expected exclusively east or south.");
		}
	}

	const float& vectorAxis(Cardinal axis, const fvector2& v) {
		switch (axis.GetValue()) {
		case Cardinal::EAST: return v.x;
		case Cardinal::SOUTH: return v.y;
		case Cardinal::SOUTH_EAST: return v.getMagnitude();
		default: throw std::invalid_argument("Expected exclusively east, south, or south-east.");
		}
	}

	fvector2 selectVectorAxis(Cardinal axis, const fvector2& v) {
		switch (axis.GetValue()) {
		case Cardinal::EAST: return v.selectX();
		case Cardinal::SOUTH: return v.selectY();
		case Cardinal::SOUTH_EAST: return v;
		default: throw std::invalid_argument("Expected exclusively east, south, or south-east.");
		}
	}

	Cardinal cardinalAxis(const Cardinal& c) {
		uint8_t result = 0;
		if (c.GetValue() & Cardinal::NORTH_SOUTH) result |= Cardinal::SOUTH;
		if (c.GetValue() & Cardinal::EAST_WEST)   result |= Cardinal::EAST;
		return result;
	}

	fvector2 cardinalAxisSign(const Cardinal& c) {
		fvector2 result;
		result.x = c.SelectEast() ? 1 : c.SelectWest() ? -1 : 0;
		result.y = c.SelectSouth() ? 1 : c.SelectNorth() ? -1 : 0;
		return result;
	}

	Cardinal toggleAxis(const Cardinal& c) {
		return
			(c.GetValue() & 0b1000) >> 3 |
			(c.GetValue() & 0b0100) >> 1 |
			(c.GetValue() & 0b0010) << 1 |
			(c.GetValue() & 0b0001) << 3;
	}
}