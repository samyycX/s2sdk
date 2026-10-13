#ifndef IPHYSICSCHARACTERPROXY_H
#define IPHYSICSCHARACTERPROXY_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/utlvector.h"
#include "gametrace.h"
#include "mathlib/ray.h"
#include "vphysics2/physicscast.h"

class IVPhysics2World;

struct VPhys2CharacterProxyParams_t
{
	// AMNOTE: Capsule centers are local to m_vPosition; initialize the capsule before creation.
	Ray_t::Capsule_t m_Capsule = { { Vector( 0.0f, 0.0f, 0.0f ), Vector( 0.0f, 0.0f, 0.0f ) }, 0.0f };
	RnQueryAttr_t m_Query;
	Vector m_vPosition = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_vVelocity = Vector( 0.0f, 0.0f, 0.0f );
};

struct VPhys2CharacterProxyCast_t
{
	// AMNOTE: Inputs and result of a sweep performed while correcting character motion.
	Vector m_vStart;
	Vector m_vDisplacement;
	VPhys2CastResult_t m_Result;
	Vector m_vVelocity;
};

struct VPhys2CharacterProxyMove_t
{
	// AMNOTE: One iteration's starting position, corrected position and sweep fraction.
	Vector m_vStart;
	Vector m_vEnd;
	float m_flFraction;
};

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
abstract_class IPhysicsCharacterProxy
{
protected:
	virtual ~IPhysicsCharacterProxy() = default;

public:
	// AMNOTE: Returns the capsule's local centers and radius.
	virtual Ray_t::Capsule_t GetCapsule() const = 0;
	// AMNOTE: Replaces the capsule geometry without changing position or velocity.
	virtual void SetCapsule( const Ray_t::Capsule_t &capsule ) = 0;
	// AMNOTE: Returns the collision-query attributes used by movement.
	virtual RnQueryAttr_t GetQueryAttributes() const = 0;
	// AMNOTE: Replaces the collision-query attributes used by movement.
	virtual void SetQueryAttributes( const RnQueryAttr_t &query ) = 0;
	// AMNOTE: Returns the capsule's world position.
	virtual Vector GetPosition() const = 0;
	// AMNOTE: Replaces the capsule's world position without a collision query.
	virtual void SetPosition( const Vector &vPosition ) = 0;
	// AMNOTE: Returns the velocity used to advance the capsule.
	virtual Vector GetVelocity() const = 0;
	// AMNOTE: Replaces the velocity used to advance the capsule.
	virtual void SetVelocity( const Vector &vVelocity ) = 0;
	// AMNOTE: Returns the owning physics world, or null when unregistered.
	virtual IVPhysics2World *GetWorld() const = 0;
	// AMNOTE: Returns the index in the world's character-proxy list, or minus one when absent.
	virtual int GetIndexInWorld() const = 0;
	// AMNOTE: Advances position by velocity times deltaTime, resolves collisions and appends sweeps to casts; clear casts before a new collection. Returns whether correction planes constrained motion.
	virtual bool Move( CUtlVector<VPhys2CharacterProxyCast_t> &casts, float deltaTime ) = 0;
	// AMNOTE: Returns a movement iteration; nIndex must be below GetMoveIterationCount(). The next Move invalidates this reference.
	virtual const VPhys2CharacterProxyMove_t &GetMoveIteration( int nIndex ) const = 0;
	// AMNOTE: Returns the number of iterations recorded by the most recent Move.
	virtual int GetMoveIterationCount() const = 0;

private:
	// AMNOTE: Returns a cleared record; its complete result type and parameters remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Does no work in this build; its parameters remain unresolved.
	virtual void unk002() = 0;

public:
	// AMNOTE: Returns null in this engine build.
	virtual const char *GetDebugName() const = 0;

private:
	// AMNOTE: Debug-name setter; does no work in this build. Its parameters remain unresolved; this reservation is not callable.
	virtual void SetDebugNameV() = 0;
};

// AMNOTE: Engine-owned adapter layout; remaining native override definitions are unrecovered. Use the inherited interface on engine-created objects.
class CPhysicsCharacterProxy : public IPhysicsCharacterProxy
{
public:
	const char *GetDebugName() const override { return nullptr; }

	// AMNOTE: Borrows the backing implementation; destroying the character proxy invalidates this pointer.
	void *GetImplementation() { return m_unk001; }
	const void *GetImplementation() const { return m_unk001; }

private:
	CPhysicsCharacterProxy() = delete;
	CPhysicsCharacterProxy( const CPhysicsCharacterProxy & ) = delete;
	CPhysicsCharacterProxy &operator=( const CPhysicsCharacterProxy & ) = delete;
	~CPhysicsCharacterProxy() override = default;

	// AMNOTE: Backing implementation object; its concrete type remains unresolved.
	void *m_unk001;
};

#endif // IPHYSICSCHARACTERPROXY_H
