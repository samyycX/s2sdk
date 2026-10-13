#ifndef IPHYSICSROPE_H
#define IPHYSICSROPE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/utlvector.h"
#include "vphysics2/iphysicsropelink.h"

class CRnRope;
class CRnWorld;

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
abstract_class IPhysicsRope
{
protected:
	virtual ~IPhysicsRope() = default;

public:
	// AMNOTE: Adds a unique body from the same world, up to 128 links. Use a null parent for the first link and an existing parent afterward. Returns null on rejection. Zero nParentCollisionEnabled disables collisions with the parent body; nonzero leaves the remaining collision filters in effect.
	virtual IPhysicsRopeLink *AddLink( IPhysicsRopeLink *pParent, IPhysicsBody *pBody, uint8 nParentCollisionEnabled ) = 0;
	// AMNOTE: Removes only the last link, destroys its adapter and clears pLink on success. Other links and null are ignored.
	virtual void RemoveLink( IPhysicsRopeLink *&pLink ) = 0;
	// AMNOTE: Counts the links owned by this rope.
	virtual int GetLinkCount() const = 0;
	// AMNOTE: Returns the indexed link, or null for an invalid index.
	virtual IPhysicsRopeLink *GetLink( int nIndex ) = 0;
	// AMNOTE: Returns the indexed link, or null for an invalid index.
	virtual const IPhysicsRopeLink *GetLink( int nIndex ) const = 0;
	// AMNOTE: Angular spring frequency in hertz, default four; nonpositive values disable its bias and softness.
	virtual float GetAngularSpringFrequency() const = 0;
	// AMNOTE: Sets the angular spring frequency without clamping.
	virtual void SetAngularSpringFrequency( float flAngularSpringFrequency ) = 0;
	// AMNOTE: Angular spring damping ratio, default one (critical damping).
	virtual float GetAngularSpringDampingRatio() const = 0;
	// AMNOTE: Sets the angular spring damping ratio without clamping.
	virtual void SetAngularSpringDampingRatio( float flAngularSpringDampingRatio ) = 0;
};

// AMNOTE: Engine-owned linkage. Destruction and unresolved virtual operations remain supplied by the engine.
class CRnLinkage
{
public:
	bool IsEnabled() const { return m_bEnabled; }
	int GetLinkCount() const { return m_Links.Count(); }
	CRnLink *GetLink( int nIndex ) { return m_Links.IsValidIndex( nIndex ) ? m_Links[nIndex] : nullptr; }
	const CRnLink *GetLink( int nIndex ) const { return m_Links.IsValidIndex( nIndex ) ? m_Links[nIndex] : nullptr; }

protected:
	virtual ~CRnLinkage();

private:
	// AMNOTE: Clears the island association and link count. The full signature remains unresolved.
	virtual void ClearIslandAndLinks();
	// AMNOTE: Ordinary linkages reject link creation; derived linkages provide their own implementation.
	virtual CRnLink *AddLink( CRnLink *pParent, CRnBody *pBody, uint8 nParentCollisionEnabled ) { return nullptr; }
	// AMNOTE: Ordinary linkages reject link removal; derived linkages provide their own implementation.
	virtual bool RemoveLink( CRnLink *pLink ) { return false; }
	// AMNOTE: Pure solver operation; its parameters remain unresolved and this reservation is not callable.
	virtual void unk001() = 0;

	CRnLinkage() = delete;
	CRnLinkage( const CRnLinkage & ) = delete;
	CRnLinkage &operator=( const CRnLinkage & ) = delete;

	// AMNOTE: Borrowed solver-island association; its concrete type is unresolved.
	void *m_unk001;
	// AMNOTE: Index in an island's solver group; minus one when detached.
	int m_nIndexInSolverGroup;
	// AMNOTE: Index in the island's linkages; minus one when detached.
	int m_nIndexInIsland;
	// AMNOTE: Solver-group selector; 255 when detached and 16 for the fallback group.
	uint8 m_nSolverGroup;
	// AMNOTE: Set to one while participating in an island, and zero when removed.
	uint8 m_nIslandMembership;
	int m_nIndexInWorld;
	// AMNOTE: Contributes the linkage's work estimate to its island and solver group.
	int m_nSolverWorkEstimate;
	// AMNOTE: Disabled linkages do not participate in solver islands.
	bool m_bEnabled;
	// AMNOTE: Linkage-kind value supplied by construction; rope construction supplies zero.
	int m_unk101;
	CRnWorld *m_World;
	CUtlVector<CRnLink *> m_Links;
	// AMNOTE: Borrowed implementation data; ropes store their IPhysicsRope adapter here.
	void *m_unk201;
};

// AMNOTE: Engine-owned rope. Link creation/removal, solving and destruction remain defined by the engine.
class CRnRope : public CRnLinkage
{
	friend class CPhysicsRope;

private:
	CRnRope() = delete;
	CRnRope( const CRnRope & ) = delete;
	CRnRope &operator=( const CRnRope & ) = delete;
	~CRnRope() override;

	// AMNOTE: Creates a rope link and updates body/island membership.
	CRnLink *AddLink( CRnLink *pParent, CRnBody *pBody, uint8 nParentCollisionEnabled ) override;
	// AMNOTE: Removes only the last owned link and updates body/island membership.
	bool RemoveLink( CRnLink *pLink ) override;
	// AMNOTE: Rope solver operation; its full signature remains unresolved.
	void unk001() override;

	// AMNOTE: Angular spring frequency and damping ratio, initialized to four hertz and one.
	float m_flAngularSpringFrequency;
	float m_flAngularSpringDampingRatio;
	// AMNOTE: Per-segment world-space impulses used for warm starting, initialized to zero.
	CUtlVector<Vector> m_AngularImpulses;
	CUtlVector<Vector> m_LinearImpulses;
};

// AMNOTE: Engine-owned adapter. Link creation still requires the engine's implementation.
class CPhysicsRope : public IPhysicsRope
{
public:
	void RemoveLink( IPhysicsRopeLink *&pLink ) override;
	int GetLinkCount() const override { return m_Rope->GetLinkCount(); }
	IPhysicsRopeLink *GetLink( int nIndex ) override
	{
		CRnLink *pLink = m_Rope->GetLink( nIndex );
		return pLink ? static_cast<IPhysicsRopeLink *>( pLink->m_unk101 ) : nullptr;
	}
	const IPhysicsRopeLink *GetLink( int nIndex ) const override
	{
		const CRnRope *pRope = m_Rope;
		const CRnLink *pLink = pRope->GetLink( nIndex );
		return pLink ? static_cast<const IPhysicsRopeLink *>( pLink->m_unk101 ) : nullptr;
	}
	float GetAngularSpringFrequency() const override { return m_Rope->m_flAngularSpringFrequency; }
	void SetAngularSpringFrequency( float flAngularSpringFrequency ) override { m_Rope->m_flAngularSpringFrequency = flAngularSpringFrequency; }
	float GetAngularSpringDampingRatio() const override { return m_Rope->m_flAngularSpringDampingRatio; }
	void SetAngularSpringDampingRatio( float flAngularSpringDampingRatio ) override { m_Rope->m_flAngularSpringDampingRatio = flAngularSpringDampingRatio; }

	// AMNOTE: Borrows the backing rope; it remains valid only while this adapter is alive.
	CRnRope *GetRnRope() { return m_Rope; }
	const CRnRope *GetRnRope() const { return m_Rope; }

private:
	CPhysicsRope() = delete;
	CPhysicsRope( const CPhysicsRope & ) = delete;
	CPhysicsRope &operator=( const CPhysicsRope & ) = delete;
	~CPhysicsRope() override = default;

	CRnRope *m_Rope;
};

inline void CPhysicsRope::RemoveLink( IPhysicsRopeLink *&pLink )
{
	CRnRopeLink *pRnLink = pLink ? static_cast<CPhysicsRopeLink *>( pLink )->GetRnRopeLink() : nullptr;
	if ( m_Rope->RemoveLink( pRnLink ) )
	{
		delete pLink;
		pLink = nullptr;
	}
}

#endif // IPHYSICSROPE_H
