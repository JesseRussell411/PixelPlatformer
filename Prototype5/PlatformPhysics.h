#pragma once

#include "MyMathUtils.h"
#include<cmath>
#include<vector>
#include<set>
#include<map>
#include<iostream>

namespace phy {
	class CollisionBox;  // --- Stores size and position of a square with no rotation.
	class Entity;		 // --- extends CollisionBox. Stores collision groups.
	class MovableEntity; // --- extends Entity. Stores velocity and bounce.
	class DynamicEntity; // --- extends MovableEntity. Stores NetForce, drag, and effect of gravity.
	class Engine;		 // --- Stores a set of entities and updates their properties on the call of the Update method.

	using namespace JesseRussell::vectors;
	using namespace JesseRussell;
	
	class CollisionBox {
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


		float Position(const Cardinal& axis) const { return position.Axis(axis); }
		void  Position(const Cardinal& axis, const float& value) { position.ReplaceAxis(axis, value); }

		float Size(const Cardinal& axis) const { return size.Axis(axis); }
		void  Size(const Cardinal& axis, const float& value) { size.ReplaceAxis(axis, value); }

		float PointA(const Cardinal& axis) const { return Position(axis); }
		void  PointA(const Cardinal& axis, const float& value) { Position(axis, value); }

		float PointB(const Cardinal& axis) const { return Position(axis) + Size(axis); }
		void  PointB(const Cardinal& axis, const float& value) { Size(axis, value - Position(axis)); }


		float Volume() const { return std::abs(size.x * size.y); }

		const float& Width() const { return size.x; }
		void         Width(const float& value) { size.x = value; }

		const float& Height() const { return size.y; }
		void         Height(const float& value) { size.y = value; }

	public: // Methods:

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

		fvector2 GetCenter() { return position + size / 2; }

		bool Intersects(const CollisionBox& other) const {
			return vectorRangeIntersection(PointA(), PointB(), other.PointA(), other.PointB());
		}

		// o ------ o
		// | Smear: |
		// o ------ o

		// //===============================================================\\
		// ||	         |  S       M       E       A        R  |           ||
		// ||            ----------------------------- HOW TO: --           ||
		// ||                   											||
		// ||                   											||
		// ||                   											||
		// ||    Original Collision Box:			                        ||
		// ||    -----------------------                                    ||
		// ||                   											||
		// ||               T   o---------------o							||
		// ||               |   |               |              				||
		// ||               |   |               |							||
		// ||   Height----->|   | Collision Box |      ----------			||
		// ||               |   |               |         ^					||
		// ||               |   |               |         |					||
		// ||               L   o---------------o      Axis(horizontal)		||
		// ||                   											||
		// ||                   L_______________|							||
		// ||          Width--------^										||
		// ||                   											||
		// ||																||
		// ||																||
		// ||																||
		// ||	Result:                                                     ||
		// ||	-------								                        ||
		// ||                        |------Offset							||
		// ||                    ____V_____									||
		// ||                   T          T								||
		// ||               T   o----------+---------------o				||
		// ||               |   | s \\ s \\                |				||
		// ||               |   |\ m \\ m \                |				||
		// ||   Height----->|   |\\ e \\ e                 |				||
		// ||               |   | \\ a \\ a                |				||
		// ||               |   |r \\ r \\                 |				||
		// ||               L   o----------+---------------o				||
		// ||																||
		// ||					L__________________________|                ||
		// ||		Width--------------^                                    ||
		// ||																||
		// ||																||
		// \\===============================================================//

		CollisionBox Smear(const Cardinal& axis, const float& offset) const {
			return Smear(axis, { offset, offset });
		}

		CollisionBox Smear(const Cardinal& axis, const fvector2& offset) const {
			if (axis.isHorizontal()) {
				if (axis.isVertical()) {
					throw std::invalid_argument("axis was horizontal and vertical simultaneously.");
				}
				else{
					if (offset.x >= 0)
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).PlusX(offset.x));
					else
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).PlusX(offset.x), GetCorner(Cardinal::SOUTH_EAST));
				}
			}
			else {
				if (axis.isVertical()) {
					if (offset.y >= 0)
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST), GetCorner(Cardinal::SOUTH_EAST).PlusY(offset.y));
					else
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).PlusY(offset.y), GetCorner(Cardinal::SOUTH_EAST));
				}
				else {
					throw std::invalid_argument("axis was NONE.");
				}
			}
		}

		CollisionBox Offset(const Cardinal& axis, const float& offset) const {
			return Offset(axis, { offset, offset });
		}

		CollisionBox Offset(const Cardinal& axis, const fvector2& offset) const {
			if (axis.isHorizontal()) {
				if (axis.isVertical()) {
					throw std::invalid_argument("axis was horizontal and vertical simultaneously.");
				}
				else {
					if (offset.x >= 0)
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_EAST), GetCorner(Cardinal::SOUTH_EAST).PlusX(offset.x));
					else
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).PlusX(offset.x), GetCorner(Cardinal::SOUTH_WEST));
				}
			}
			else {
				if (axis.isVertical()) {
					if (offset.y >= 0)
						return CollisionBox::ByPoints(GetCorner(Cardinal::SOUTH_WEST), GetCorner(Cardinal::SOUTH_EAST).PlusY(offset.y));
					else
						return CollisionBox::ByPoints(GetCorner(Cardinal::NORTH_WEST).PlusY(offset.y), GetCorner(Cardinal::NORTH_EAST));
				}
				else {
					throw std::invalid_argument("axis was NONE.");
				}
			}
		}

	public:
		void Move(const fvector2& distance) { position += distance; }
		void Move(const Cardinal& axis, const float& distance) {
			position.AddAxis(axis, distance);
		}

		void SetPositionOnSide(const Cardinal& side, const float& value) {
			if (Size(side) >= 0) {
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

		void SetPositionOnCorner(const Cardinal& corner, const fvector2& value) {
			SetPositionOnSide(corner & Cardinal::EAST_WEST, value.x);
			SetPositionOnSide(corner & Cardinal::NORTH_SOUTH, value.y);
		}

		void SetPositionOnCenter(const fvector2& value) { position = value - size / 2; }

	private:
		fvector2
			position = { 0, 0 },
			size = { 0, 0 };
	};




	// o=========o
	// | Entity: |
	// o=========o

	class Entity : public CollisionBox {
		friend class Engine;
		friend class MovableEntity;
		friend class DynamicEntity;
	public: // Constructors:
		Entity(const fvector2& position, const fvector2& size, const size_t& collisionGroup = 0) : CollisionBox(position, size) {
			CollisionGroup(collisionGroup, true); // zero will be the default collision group.
		}
		
		Entity() = default;
	public: // Factories:
		static Entity ByPoints(const fvector2& pointA, const fvector2& pointB, const size_t& collisionGroup = 0) {
			Entity result({0, 0}, {0, 0}, collisionGroup);
			result.PointA(pointA);
			result.PointB(pointB);
			return result;
		}
	public: // collision group:
		bool CollsionGroup(const size_t& group) {
			auto block = collisionGroupBlocks.find(group / 8);
			// Check if block exists.
			if (block == collisionGroupBlocks.end()) return false;

			return block->second & (1 << (group % 8));
		}

		void CollisionGroup(const size_t& group, const bool& value) {
			auto block = collisionGroupBlocks.find(group / 8);
			// Check if block exists:
			if (block == collisionGroupBlocks.end())
				block = collisionGroupBlocks.insert(std::pair<size_t, uint8_t>(group / 8, 0)).first;
			//
			
			if (value)
				block->second |= (1 << (group % 8));
			else
				block->second &= ~(1 << (group % 8));
		}

		bool CompareCollisionGroups(const Entity& other) {
			auto iter = collisionGroupBlocks.cbegin();
			auto other_iter = other.collisionGroupBlocks.cbegin();

			// Main loop...
			while (iter != collisionGroupBlocks.cend() && other_iter != other.collisionGroupBlocks.cend()) {
				// make sure the same blocks are being compared:
				while (iter->first != other_iter->first) {
					while (other_iter->first < iter->first) {
						other_iter++;
						if (other_iter == other.collisionGroupBlocks.cend()) return false;
					}

					while (iter->first < other_iter->first) {
						iter++;
						if (iter == collisionGroupBlocks.cend()) return false;
					}
				}
				//

				if (iter->second & other_iter->second) return true;
				iter++;
				other_iter++;
			}
			
			return false;
		}
	public: // Methods:
		bool Touches(Cardinal side, const CollisionBox& other) {
			return Smear(side, touching_threshold).Intersects(other);
		}



	private: // Fields:
		std::map<size_t, uint8_t> collisionGroupBlocks;
		float friction_coef = .9;


		// o --------------- o
		// | identification: |
		// o --------------- o
	public:
		bool IsMovable() { return isMovable(); }
		bool IsDynamic() { return isDynamic(); }

	private:
		virtual bool isMovable() { return false; }
		virtual bool isDynamic() { return false; }

		// o --------- o
		// | constants |
		// o --------- o
	protected:
		static float touching_threshold;
		//

	protected:
		virtual void OnEngineUpdate(Engine& engine) {}
		virtual void OnIntersection(Engine& engine, Entity& other, const Cardinal& side, const float& spot) {}
	private:
		void onEngineUpdate(Engine& engine) { OnEngineUpdate(engine); }
		void onIntersection(Engine& engine, Entity& other, const Cardinal& side, const float& spot) { OnIntersection(engine, other, side, spot); }
	};



	// o================o
	// | MovableEntity: |
	// o================o

	class MovableEntity : public Entity {
		friend class Engine;
		friend class DynamicEntity;
		friend class Entity;
	public: // Constructors:
		MovableEntity(const fvector2& position, const fvector2& size, const size_t& collisionGroup = 0) : Entity(position, size, collisionGroup){}
		MovableEntity() = default;

	public: // Factories:
		static MovableEntity ByPoints(const fvector2& pointA, const fvector2& pointB, const size_t& collisionGroup = 0) {
			MovableEntity result({ 0, 0 }, { 0, 0 }, collisionGroup);
			result.PointA(pointA);
			result.PointB(pointB);
			return result;
		}

	public: // Properties:
		const fvector2& Velocity() { return velocity; }
		void            Velocity(const fvector2& value) { velocity = value; }

		const float& Bounce() { return bounce; }
		void         Bounce(const float& value) { bounce = value; }

	public: // Methods:
		// update:
		void ApplyVelocity(const Cardinal& axis, const float& timeScale = 1) {
			Move(velocity.SelectAxis(axis) * timeScale);
		}
		void ApplyVelocity(const float& timeScale = 1) {
			Move(velocity * timeScale);
		}
		//

		CollisionBox GetVelocitySmear(const Cardinal& axis, const float& timeScale) {
			return Smear(axis, velocity * timeScale);
		}

		CollisionBox GetVelocityOffset(const Cardinal& axis, const float& timeScale) {
			return Offset(axis, velocity * timeScale);
		}

		Cardinal GetHeading() {
			Cardinal result = 0;
			if (velocity.x >= 0) result |= Cardinal::EAST;
			else                 result |= Cardinal::WEST;

			if (velocity.y >= 0) result |= Cardinal::SOUTH;
			else                 result |= Cardinal::NORTH;
			return result;
		}

		bool Collides(Cardinal axis, const CollisionBox& other, float timeScale, float& out_collisionSpot, Cardinal& out_collisionSide) {
			out_collisionSide = (axis | axis.Flipped()) & GetHeading();
			out_collisionSpot = other.GetSide((out_collisionSide).Flipped());

			return Collides(axis, other, timeScale);
		}

		bool Collides(Cardinal axis, const CollisionBox& other, float timeScale) {
			return (GetVelocityOffset(axis, timeScale).Intersects(other));
		}

	private: // Fields:
		fvector2
			velocity = { 0, 0 };

		float
			bounce = 1;

		// o --------------- o
		// | identification: |
		// o --------------- o
	private:
		bool isMovable() override { return true; }
		//

	protected:
		virtual void BeforeMovementUpdate(Engine& engine) {}
		virtual void AfterMovementUpdate(Engine& engine) {}
		virtual void OnCollision(Engine& engine, Entity& other, const Cardinal& side, const float& spot) {}
	private:
		void beforeMovementUpdate(Engine& engine) { BeforeMovementUpdate(engine); }
		void afterMovementUpdate(Engine& engine) { AfterMovementUpdate(engine); }
		void onCollision(Engine& engine, Entity& other, const Cardinal& side, const float& spot) { OnCollision(engine, other, side, spot); }
	};



	// o===============o
	// | DynamicEntity |
	// o===============o

	class DynamicEntity : public MovableEntity {
		friend class Engine;
		friend class MovableEntity;
		friend class Entity;
	public: // Constructors:
		DynamicEntity(const fvector2& position, const fvector2& size, const float& mass, const size_t& collisionGroup = 0) : MovableEntity(position, size, collisionGroup) {
			this->mass = mass;
		}
		DynamicEntity() = default;

	public: // Factories:
		static DynamicEntity ByPoints(const fvector2 & pointA, const fvector2 & pointB, const float& mass, const size_t & collisionGroup = 0) {
			DynamicEntity result({ 0, 0 }, { 0, 0 }, mass, collisionGroup);
			result.PointA(pointA);
			result.PointB(pointB);
			return result;
		}

	public: // Properties:
		const fvector2& NetForce() const { return netForce; }
		void            NetForce(const fvector2& value) { netForce = value; }

		const fvector2& GravityEffect() const { return gravityEffect; }
		void            GravityEffect(const fvector2& value) { gravityEffect = value; }

		const float& Mass() const { return mass; }
		void         Mass(const float& value) { mass = value; }

		const float& Drag() const { return drag; }
		void         Drag(const float& value) { drag = value; }

	public:
		void AddForce(const fvector2& force) { netForce += force; }

		//update:
		void ApplyNetForce(const Cardinal& axis, const float& timeScale = 1) {
			velocity += (netForce.SelectAxis(axis) / mass) * timeScale;
		}
		void ApplyNetForce(const float& timeScale = 1) {
			velocity += (netForce / mass) * timeScale;
		}

		void ResetNetForce(const Cardinal& axis = Cardinal::SOUTH_EAST) {
			netForce.ReplaceAxis(axis, 0);
		}
	private:
		fvector2
			netForce = { 0, 0 },
			gravityEffect = { 1, 1 };

		float
			mass = 1,
			drag = 0.1;

	private: // masking fields:
		float bounce = .5;

		// o --------------- o
		// | identification: |
		// o --------------- o
	private:
		bool isDynamic() override { return true; }
		//


	protected:
		virtual void BeforePhysicsUpdate(Engine& engine) {}
		virtual void AfterPhysicsUpdate(Engine& engine) {}
	private:
		void beforePhysicsUpdate(Engine& engine) { BeforePhysicsUpdate(engine); }
		void afterPhysicsUpdate(Engine& engine) { AfterPhysicsUpdate(engine); }
	};



	// o=========o
	// | Engine: |
	// o=========o

	class Engine {
	public: // destructors:
		~Engine() {
			for (Entity* e : entities) delete e;
		}
	public: // entity management:
		void AddEntity(Entity* e) {
			entities.insert(e);
			if (e->IsMovable()) movableEntities.insert((MovableEntity*)e);
		}

		bool RemoveEntity(Entity* e) {
			std::set<Entity*>::const_iterator it = entities.find(e);
			if (it != entities.end()) {
				entities.erase(it);
				if (e->IsMovable()) movableEntities.erase((MovableEntity*)e);
			}
			else return false;
		}

		bool DeleteEntity(Entity* e) {
			if (RemoveEntity(e)) {
				delete e;
				return true;
			}
			else return false;
		}

		void RemoveEntities() {
			entities.clear();
			movableEntities.clear();
		}

		void DeleteEntities() {
			for (Entity* e : entities) delete e;
			RemoveEntities();
		}

		std::set<Entity*>::const_iterator Entities_cbegin() { return entities.cbegin(); }
		std::set<Entity*>::const_iterator Entities_cend() { return entities.cend(); }


	public:
		void Update(Cardinal axis, const float& timeScale) {
			// Update environment properties:
			this->axis = axis;
			this->timeScale = timeScale;
			// Main loop...
			for (Entity* e : movableEntities) {
				e->OnEngineUpdate(*this);

				if (e->IsMovable()) {
					bool e_isdyn = e->IsDynamic();

					((MovableEntity*)e)->beforeMovementUpdate(*this);
					if (e_isdyn) ((DynamicEntity*)e)->beforePhysicsUpdate(*this);

					UpdateSingleMovable((MovableEntity*)e);

					if (e_isdyn)((DynamicEntity*)e)->afterPhysicsUpdate(*this);
					((MovableEntity*)e)->afterMovementUpdate(*this);
				}
			}
		}

		void UpdateSingleMovable(MovableEntity* e) {
			float out_collisionSpot;
			Cardinal out_collisionSide;
			float closestSpot;
			Cardinal closestSide;
			Entity* closestEntity = nullptr;
			bool collided = false;


			// o ------------------- o
			// | Apply other forces: |
			// o ------------------- o
			// air resistance..
			if (e->IsDynamic()) {
				DynamicEntity* de = (DynamicEntity*)e;
				fvector2 force = { 0, 0 };
				float velocity = de->velocity.Axis(axis);
				force.ref_Axis(axis) = velocity * std::abs(velocity) * -0.5 * airDensity * de->Drag() * de->Size(axis.AxisSwapped());

				de->AddForce(force);
			}

			// gravity..
			if (e->IsDynamic()) {
				DynamicEntity* de = (DynamicEntity*)e;
				de->AddForce(gravity_acc * de->mass);
			}

			// continues in | Apply collision: |

			// o ---------------- o
			// | Apply net force: |
			// o ---------------- o
			if (e->IsDynamic()) {
				((DynamicEntity*)e)->ApplyNetForce(axis, timeScale);
				((DynamicEntity*)e)->ResetNetForce(axis);
			}


			// o --------------------- o
			// | Check for collisions: |
			// o --------------------- o
			for (Entity* other : entities) {
				if (e == other) continue;
				

				// Check for touching:
				if (e->Collides(axis, *other, timeScale, out_collisionSpot, out_collisionSide)) {
					if (e->CompareCollisionGroups(*other)) {
						// one of many collisions has occurred
						if (collided) {
							if (cmp::closer(out_collisionSpot, closestSpot, e->GetSide(out_collisionSide))) {
								closestSpot = out_collisionSpot;
								closestSide = out_collisionSide;
								closestEntity = other;
							}
						}
						else
						{
							closestSpot = out_collisionSpot;
							closestSide = out_collisionSide;
							closestEntity = other;
							collided = true;
						}
					}
					else {
						// intersection has occurred
						// Run special intersection code.
						e->onIntersection(*this, *other, out_collisionSide, out_collisionSpot);
					}
				}
			}

			// Run special collision code...
			if (collided) {
				e->onCollision(*this, *closestEntity, closestSide, closestSpot);
			}

			// Check validity of collision...
			bool trueCollision = false;
			if (collided) {
				if (closestEntity->IsMovable()) {
					MovableEntity other = *(MovableEntity*)closestEntity;
					switch (closestSide.GetValue()) {
					case Cardinal::NORTH:
						trueCollision = e->velocity.y - other.velocity.y < 0; break;
					case Cardinal::SOUTH:
						trueCollision = e->velocity.y - other.velocity.y > 0; break;
					case Cardinal::WEST:
						trueCollision = e->velocity.x - other.velocity.x < 0; break;
					case Cardinal::EAST:
						trueCollision = e->velocity.x - other.velocity.x > 0; break;
					}
				}
				else
					trueCollision = true;
			}


			// o ---------------- o
			// | Apply collision: |
			// o ---------------- o
			if (collided && trueCollision) {
				float spot = closestSpot;
				Cardinal side = out_collisionSide;
				e->SetPositionOnSide(side, spot);

				if (e->IsDynamic()) {
					DynamicEntity& entity = *(DynamicEntity*)e;

					// dynamic-dynamic collision:
					// --------------------------
					if (closestEntity->IsDynamic()) {
						DynamicEntity& other = *(DynamicEntity*)closestEntity;
						// get relative force of collision, and call it normal force even though you're not sure if that's technically right...
						float normalForce =
							entity.mass * (entity.velocity.Axis(axis) / timeScale) +
							other.mass * (other.velocity.Axis(axis) / timeScale);
						// effect of collision:
						// -------------------

						// Get new velocity:
						float other_vf =
							(2 * entity.mass / (entity.mass + other.mass)) * entity.velocity.Axis(axis) -
							((entity.mass - other.mass) / (entity.mass + other.mass)) * other.velocity.Axis(axis);

						float entity_vf =
							((entity.mass - other.mass) / (entity.mass + other.mass)) * entity.velocity.Axis(axis) +
							(2 * other.mass / (entity.mass + other.mass)) * other.velocity.Axis(axis);

						// average bounce.
						float averageBounce = (entity.bounce + other.bounce) / 2;

						// Set new velocity:
						entity.velocity.ref_Axis(axis) = entity_vf * averageBounce;
						other.velocity.ref_Axis(axis) = other_vf * averageBounce;
						//

						// effect of friction (continues from | Apply other forces: |):
						// -------------------
						// omg friction is complicated

						// static friction:
						float relativeVeloctiy = entity.velocity.Axis(axis.AxisSwapped()) - other.velocity.Axis(axis.AxisSwapped()); // *relative to entity not other.
						float averageFriction = (entity.friction_coef * other.friction_coef) / 2;

						fvector2 force = { 0, 0 };
						force.ref_Axis(axis.AxisSwapped()) = averageFriction * std::abs(normalForce) * -cmp::sign(relativeVeloctiy);

						entity.AddForce(force);
						other.AddForce(force * -1);
					}
					else {
						// effect of collision:
						entity.velocity.ref_Axis(axis) *= -entity.bounce;

						// get relative force of collision, and call it normal force even though you're not sure if that's technically right...
						float normalForce = 0;
						float relativeVeloctiy = 0; // *relative to entity not other.

						// if other is movable
						if (closestEntity->IsMovable()) {
							MovableEntity& other_me = *((MovableEntity*)closestEntity);
							normalForce =
								entity.mass * (entity.velocity.Axis(axis)   / timeScale) +
								entity.mass * (other_me.velocity.Axis(axis) / timeScale);

							relativeVeloctiy =
								entity.velocity.Axis(axis.AxisSwapped()) - other_me.velocity.Axis(axis.AxisSwapped());
						}
						// if other is not movable
						else {
							normalForce =
								entity.mass * (entity.velocity.Axis(axis) / timeScale);

							relativeVeloctiy = entity.velocity.Axis(axis.AxisSwapped());
						}

						// effect of friction (continues from | Apply other forces: |):
						// -------------------
						// omg friction is complicated

						// static friction:
						float averageFriction = (entity.friction_coef * closestEntity->friction_coef) / 2;

						fvector2 force = { 0, 0 };
						force.ref_Axis(axis.AxisSwapped()) = averageFriction * std::abs(normalForce) * -cmp::sign(relativeVeloctiy);

						entity.AddForce(force);
					}
				}
				else {
					MovableEntity& entity = *e;
					if (closestEntity->IsDynamic()) {
						DynamicEntity& other = *(DynamicEntity*)closestEntity;
						other.velocity.ref_Axis(axis) = entity.velocity.Axis(axis);
					}
					else {
						Entity& other = *closestEntity;
						entity.velocity.ref_Axis(axis) *= -entity.bounce;
					}
				}
			}

			// o --------------- o
			// | Apply velocity: |
			// o --------------- o
			if (!collided) {
				e->ApplyVelocity(axis, timeScale);
			}
		}

	private:
		std::set<Entity*> entities;
		std::set<MovableEntity*> movableEntities;
		Cardinal axis = Cardinal::NONE;
		float timeScale = 1;

		float airDensity = 0;
		fvector2 gravity_acc = { 0, 98 };
	};
}