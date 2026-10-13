#ifndef INTERSECTION_CONTROLLER_INTERFACE_H
#define INTERSECTION_CONTROLLER_INTERFACE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "gametrace.h"
#include "rubikon/intersection.h"

class CIntersectionController;

// AMNOTE: Engine-owned; obtain through IVPhysics2. Do not implement or delete it. Collision-group indices must be in [0, 64).
abstract_class IPhysIntersectionController
{
public:
	virtual void SetCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) = 0;
	virtual void EnableCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) = 0;
	virtual void DisableCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) = 0;
	// AMNOTE: Copies the source row to the destination row and column, preserving symmetry.
	virtual void CopyCollisionGroupFlagsAllPairs( CollisionGroupIndex nDestGroup, CollisionGroupIndex nSourceGroup ) = 0;
	virtual IntersectionLayerPairFlags TestSimulation( const RnCollisionAttr_t &attributes1, const RnCollisionAttr_t &attributes2 ) const = 0;

private:
	// AMNOTE: The query also requires a world reference whose complete contract remains unresolved.
	virtual void TestQuery() = 0;

public:
	virtual IntersectionLayerPairFlags GetCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2 ) = 0;

private:
	virtual void unk001() = 0;

protected:
	virtual ~IPhysIntersectionController() = default;

private:
	IPhysIntersectionController() = default;
	IPhysIntersectionController( const IPhysIntersectionController & ) = delete;
	IPhysIntersectionController &operator=( const IPhysIntersectionController & ) = delete;
	friend class CIntersectionController;
};

// AMNOTE: Complete engine-owned view; do not construct, copy or delete it.
abstract_class CIntersectionController : public IPhysIntersectionController
{
public:
	void SetCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) override
	{
		m_GroupPairFlags[nGroup1][nGroup2] = nFlags;
		m_GroupPairFlags[nGroup2][nGroup1] = nFlags;
	}
	void EnableCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) override
	{
		m_GroupPairFlags[nGroup1][nGroup2] |= nFlags;
		m_GroupPairFlags[nGroup2][nGroup1] |= nFlags;
	}
	void DisableCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2, IntersectionLayerPairFlags nFlags ) override
	{
		m_GroupPairFlags[nGroup1][nGroup2] &= ~nFlags;
		m_GroupPairFlags[nGroup2][nGroup1] &= ~nFlags;
	}
	void CopyCollisionGroupFlagsAllPairs( CollisionGroupIndex nDestGroup, CollisionGroupIndex nSourceGroup ) override;
	IntersectionLayerPairFlags TestSimulation( const RnCollisionAttr_t &attributes1, const RnCollisionAttr_t &attributes2 ) const override;
	IntersectionLayerPairFlags GetCollisionGroupPairFlags( CollisionGroupIndex nGroup1, CollisionGroupIndex nGroup2 ) override { return m_GroupPairFlags[nGroup1][nGroup2]; }

protected:
	~CIntersectionController() override = default;

private:
	CIntersectionController() = delete;
	CIntersectionController( const CIntersectionController & ) = delete;
	CIntersectionController &operator=( const CIntersectionController & ) = delete;

	IntersectionLayerPairFlags m_GroupPairFlags[64][64];
};

inline void CIntersectionController::CopyCollisionGroupFlagsAllPairs( CollisionGroupIndex nDestGroup, CollisionGroupIndex nSourceGroup )
{
	for ( CollisionGroupIndex nGroup = 0; nGroup < 64; ++nGroup )
	{
		const IntersectionLayerPairFlags nFlags = m_GroupPairFlags[nSourceGroup][nGroup];
		m_GroupPairFlags[nDestGroup][nGroup] = nFlags;
		m_GroupPairFlags[nGroup][nDestGroup] = nFlags;
	}
}

inline IntersectionLayerPairFlags CIntersectionController::TestSimulation( const RnCollisionAttr_t &attributes1, const RnCollisionAttr_t &attributes2 ) const
{
	if ( ( attributes1.m_nInteractsAs & attributes2.m_nInteractsExclude ) |
		( attributes2.m_nInteractsAs & attributes1.m_nInteractsExclude ) )
		return 0;

	if ( attributes1.m_nCollisionGroup != 0 && attributes2.m_nCollisionGroup != 0 )
	{
		if ( ( ( attributes1.m_nInteractsWith & 2 ) && attributes2.ShouldIgnoreForHitboxTest() ) ||
			( ( attributes2.m_nInteractsWith & 2 ) && attributes1.ShouldIgnoreForHitboxTest() ) )
			return 0;
		if ( !( ( attributes1.m_nInteractsWith & attributes2.m_nInteractsAs ) |
			( attributes2.m_nInteractsWith & attributes1.m_nInteractsAs ) ) )
			return 0;
	}

	const uint8 nFunctions1 = attributes1.m_nCollisionFunctionMask;
	const uint8 nFunctions2 = attributes2.m_nCollisionFunctionMask;
	if ( uint16( attributes1.m_nHierarchyId - 1 ) <= 0xFFFD )
	{
		if ( attributes1.m_nHierarchyId == attributes2.m_nHierarchyId )
		{
			// AMNOTE: Bit 0x40 bypasses the equal-hierarchy gate; bit 0x80 rejects distinct nonzero hierarchies.
			if ( !( ( nFunctions1 | nFunctions2 ) & 0x40 ) &&
				( !( nFunctions1 & 0x48 ) || !( nFunctions2 & 0x48 ) ||
				!attributes1.IsSolidContactEnabled() || !attributes2.IsSolidContactEnabled() ) )
				return 0;
		}
		else if ( uint16( attributes2.m_nHierarchyId - 1 ) <= 0xFFFD && ( ( nFunctions1 | nFunctions2 ) & 0x80 ) )
			return 0;
	}

	if ( ( attributes2.m_nEntityId != uint32( -1 ) && attributes1.m_nOwnerId == attributes2.m_nEntityId ) ||
		( attributes1.m_nEntityId != uint32( -1 ) && attributes2.m_nOwnerId == attributes1.m_nEntityId ) )
		return 0;

	const uint16 nDetailMask1 = uint16( ~( attributes1.m_nDetailLayerMask ^ ( attributes1.m_nDetailLayerMaskType == 1 ? 0xFFFF : 0 ) ) );
	const uint16 nDetailMask2 = uint16( ~( attributes2.m_nDetailLayerMask ^ ( attributes2.m_nDetailLayerMaskType == 1 ? 0xFFFF : 0 ) ) );
	if ( !( nDetailMask1 & ( uint32( 1 ) << ( attributes2.m_nTargetDetailLayer & 31 ) ) ) ||
		!( nDetailMask2 & ( uint32( 1 ) << ( attributes1.m_nTargetDetailLayer & 31 ) ) ) )
		return 0;

	IntersectionLayerPairFlags nFlags = m_GroupPairFlags[attributes1.m_nCollisionGroup][attributes2.m_nCollisionGroup];
	if ( ( nFlags & 1 ) && ( !attributes1.IsSolidContactEnabled() || !attributes2.IsSolidContactEnabled() ) )
		return 0;
	if ( !attributes1.IsTouchEventEnabled() || !attributes2.IsTouchEventEnabled() || attributes1.m_nEntityId == attributes2.m_nEntityId )
		return nFlags & 0xFFE3;
	if ( ( nFlags & 0x1C ) && ( attributes1.IsTouchPersistsEnabled() || attributes2.IsTouchPersistsEnabled() ) )
		nFlags |= 8;
	return nFlags;
}

#endif // INTERSECTION_CONTROLLER_INTERFACE_H
