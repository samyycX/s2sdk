#ifndef IPHYSICSPARTICLEROPE_H
#define IPHYSICSPARTICLEROPE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "schemasystem/minschema.h"
#include "tier0/utlvector.h"
#include "tier0/utldelegate.h"
#include "gametrace.h"
#include "mathlib/camera.h"
#include "mathlib/vectoraligned.h"
#include "mathlib/transform.h"
#include "mathlib/ssemath.h"
#include "vphysics2/iphysicsrope.h"
#include <functional>

class IPhysicsBody;
class CRnParticleRope;
class ISceneViewDebugOverlays;

typedef CUtlDelegate<void()> FnVPhys2ParticleRopeCallback_t;

typedef std::function<void( IPhysicsBody *, float )> FnVPhys2AppliedConstraintImpulse_t;

// AMNOTE: Initialize every field before updating a section; body pointers are borrowed.
struct VPhys2TautRopeSectionParams_t
{
	IPhysicsBody *m_Body[2];
	Vector m_vLocalAnchor[2];
	int m_nContact[2];
	float m_flFrequency;
	float m_flDampingRatio;
	// AMNOTE: A nonpositive value leaves the force unrestricted.
	float m_flMaxForce;
	float m_flRestLengthScale;
};

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
// AMNOTE: Node-flag operations whose meaning remains unresolved are private.
abstract_class IPhysicsParticleRope
{
public:
	// Returns the engine-owned schema class binding for this interface.
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

protected:
	virtual ~IPhysicsParticleRope() = default;

public:
	// AMNOTE: Sets the entity ID used by world entity-pair collision suppression; initially all ones.
	virtual void SetEntityId( uint32 nEntityId ) = 0;
	virtual uint32 GetEntityId() const = 0;

	// AMNOTE: Borrows the collision query attributes.
	virtual const RnQueryAttr_t &GetQueryAttributes() const = 0;
	// AMNOTE: Replacing the query attributes wakes the rope.
	virtual void SetQueryAttributes( const RnQueryAttr_t &query ) = 0;
	// AMNOTE: Changing the enabled state wakes the rope.
	virtual void Enable( bool bEnable ) = 0;
	virtual bool IsEnabled() const = 0;
	virtual void EnableCollision( bool bEnable ) = 0;
	virtual bool IsCollisionEnabled() const = 0;
	virtual void EnableAsyncSimulation( bool bEnable ) = 0;
	virtual bool IsAsyncSimulationEnabled() const = 0;
	// AMNOTE: Returns accumulated asynchronous simulation time; submitting completed results clears it.
	virtual float GetAccumulatedAsyncSimulationTime() const = 0;
	// AMNOTE: Replaces accumulated asynchronous simulation time without clamping or waking the rope.
	virtual void SetAccumulatedAsyncSimulationTime( float flTime ) = 0;
	virtual void SetSimulateOncePerFrame( bool bEnable ) = 0;
	// AMNOTE: Returns true only when both this option and asynchronous simulation are enabled.
	virtual bool ShouldSimulateOncePerFrame() const = 0;
	virtual void EnableDirectDistanceConstraintSolver( bool bEnable ) = 0;
	virtual void EnableTautLinePositionCache( bool bEnable ) = 0;
	virtual void EnableTouchingEntityTracking( bool bEnable ) = 0;
	virtual bool IsTouchingEntityTrackingEnabled() const = 0;
	virtual bool IsAsleep() const = 0;
	virtual void Sleep() = 0;
	virtual void Wake() = 0;
	virtual int GetSubstepCount() const = 0;
	// AMNOTE: Clamps to at least one; a changed value wakes the rope.
	virtual void SetSubstepCount( int nSubsteps ) = 0;
	virtual int GetIterationCount() const = 0;
	// AMNOTE: Clamps to at least one; a changed value wakes the rope.
	virtual void SetIterationCount( int nIterations ) = 0;
	virtual float GetDamping() const = 0;
	// AMNOTE: A changed value wakes the rope.
	virtual void SetDamping( float flValue ) = 0;
	virtual float GetFriction() const = 0;
	// AMNOTE: A changed value wakes the rope.
	virtual void SetFriction( float flValue ) = 0;
	virtual float GetBendingStiffness() const = 0;
	// AMNOTE: Clamps to zero through one; a changed input wakes the rope.
	virtual void SetBendingStiffness( float flValue ) = 0;
	virtual float GetEndpointAlignmentStiffness() const = 0;
	// AMNOTE: Clamps to zero through one; a changed clamped value wakes the rope.
	virtual void SetEndpointAlignmentStiffness( float flValue ) = 0;
	virtual float GetEndpointAlignmentFalloff() const = 0;
	virtual void SetEndpointAlignmentFalloff( float flValue ) = 0;
	virtual float GetLinkStretchToDisableCollision() const = 0;
	// AMNOTE: Clamps to at least one; a changed clamped value wakes the rope.
	virtual void SetLinkStretchToDisableCollision( float flValue ) = 0;
	virtual float GetGravityScale() const = 0;
	// AMNOTE: A changed value wakes the rope.
	virtual void SetGravityScale( float flValue ) = 0;
	virtual float GetMaxNodeVelocity() const = 0;
	virtual void SetMaxNodeVelocity( float flValue ) = 0;
	virtual float GetNodeMass() const = 0;
	// AMNOTE: Changes the default node mass and updates nonzero inverse masses of ordinary nodes; pinned nodes keep zero inverse mass.
	virtual void SetNodeMass( float flValue ) = 0;
	// AMNOTE: nEndpoint must be zero or one. A newly positive radius initializes the extra plug node.
	virtual void SetEndpointPlug( int nEndpoint, float flLength, float flRadius, float flMass ) = 0;
	// AMNOTE: nEndpoint must be zero or one.
	virtual float GetEndpointPlugRadius( int nEndpoint ) const = 0;
	// AMNOTE: nEndpoint must be zero or one.
	virtual float GetEndpointPlugMass( int nEndpoint ) const = 0;
	// AMNOTE: nEndpoint must be zero or one; uses the plug node when its radius is positive. A zero or NaN direction length returns the positive x axis.
	virtual Vector GetEndpointDirection( int nEndpoint ) const = 0;
	// AMNOTE: Sums current distances between consecutive ordinary nodes, excluding extra plug nodes.
	virtual float GetLength() const = 0;
	// AMNOTE: Combines the interior node separation and the two endpoint link lengths.
	virtual float GetRestLength() const = 0;
	// AMNOTE: Returns distance along the unstretched ordinary-node chain; nNode must select an ordinary node.
	virtual float GetNodeRestDistance( int nNode ) const = 0;
	// AMNOTE: Selects the closest ordinary node; ties select the second bracketing node.
	virtual int FindNodeAtRestDistance( float flDistance ) const = 0;
	// AMNOTE: Returns the bracketing ordinary nodes and a fraction clamped to zero through one.
	virtual void GetNodesAtRestDistance( float flDistance, int &nFirstNode, int &nSecondNode, float &flFraction ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCount(); compares the adjacent links with their rest lengths.
	virtual float GetNodeStretch( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nFirstNode <= nLastNode < GetNodeCount(); maps sampled stretch from one through flMaxStretch to zero through one.
	virtual float GetNodeRangeStretchBlend( int nFirstNode, int nLastNode, float flMaxStretch ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCount(); maps node stretch from one through flMaxStretch to zero through one.
	virtual float GetNodeStretchBlend( int nNode, float flMaxStretch ) const = 0;
	// AMNOTE: Returns the retained world-space bounds.
	virtual AABB_t GetBounds() const = 0;
	// AMNOTE: Transforms node and target positions, replaces all node velocities, clears forces and contacts, removes retained taut sections and wakes the rope.
	virtual void TransformNodes( const CTransform &transform, const Vector &vVelocity ) = 0;
	// AMNOTE: Accepts 0 <= nNode <= GetNodeCount(); inserts an ordinary node and wakes the rope.
	virtual void InsertNode( const Vector &vPosition, const Vector &vVelocity, int nNode ) = 0;
	// AMNOTE: Removes a valid ordinary node when more than three remain; preserves pinning at a removed endpoint and wakes the rope.
	virtual void RemoveNode( int nNode ) = 0;
	// AMNOTE: Extends the first endpoint link, inserting nodes as needed; returns the number inserted.
	virtual int ExtendStart( float flDistance ) = 0;
	// AMNOTE: Shortens the first endpoint link, removing nodes as needed while retaining at least three; returns the number removed.
	virtual int RetractStart( float flDistance ) = 0;
	// AMNOTE: Extends the last endpoint link, inserting nodes as needed; returns the number inserted.
	virtual int ExtendEnd( float flDistance ) = 0;
	// AMNOTE: Shortens the last endpoint link, removing nodes as needed while retaining at least three; returns the number removed.
	virtual int RetractEnd( float flDistance ) = 0;
	virtual int GetNodeCount() const = 0;
	// AMNOTE: Includes the two extra plug nodes, even when their radii are zero.
	virtual int GetNodeCountIncludingPlugs() const = 0;
	virtual int GetSegmentCount() const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); extra nodes use the corresponding plug radius.
	virtual float GetNodeRadius( int nNode ) const = 0;
	virtual float GetNodeSeparation() const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(). Zero inverse mass denotes a pinned node.
	virtual float GetNodeInverseMass( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs().
	virtual Vector GetNodePosition( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); returns the node target when present, otherwise its current position.
	virtual Vector GetNodeTargetPosition( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs().
	virtual Vector GetNodeVelocity( int nNode ) const = 0;
	// AMNOTE: Resizes and fills the ordinary-node positions; excludes extra plug nodes.
	virtual void GetNodePositions( CUtlVector<VectorAligned> &positions ) const = 0;
	// AMNOTE: Resizes and fills the ordinary-node velocities; excludes extra plug nodes.
	virtual void GetNodeVelocities( CUtlVector<VectorAligned> &velocities ) const = 0;
	// AMNOTE: Replaces ordinary positions only when the input count equals GetNodeCount(); optionally wakes the rope.
	virtual void SetNodePositions( const CUtlVector<VectorAligned> &positions, bool bWake ) = 0;
	// AMNOTE: Replaces ordinary velocities only when the input count equals GetNodeCount(); optionally wakes the rope.
	virtual void SetNodeVelocities( const CUtlVector<VectorAligned> &velocities, bool bWake ) = 0;
	// AMNOTE: Fills ordinary-node positions adjusted for movement of the retained reference body, then applies node targets.
	virtual void GetNodeTargetPositions( CUtlVector<Vector> &positions ) const = 0;
	// AMNOTE: Disabling taut-line simulation clears its contact records.
	virtual void EnableTautLine( bool bEnable ) = 0;
	virtual bool IsTautLineEnabled() const = 0;
	virtual int GetTautLineContactCount() const = 0;
	// AMNOTE: Requires 0 <= nContact < GetTautLineContactCount(); writes the world position and rest distance along the rope.
	virtual void GetTautLineContact( int nContact, Vector &vPosition, float &flRestDistance ) const = 0;
	// AMNOTE: Requires 0 <= nContact < GetTautLineContactCount(); returns the borrowed contact body, or null.
	virtual IPhysicsBody *GetTautLineContactBody( int nContact ) const = 0;
	// AMNOTE: Requires 0 <= nContact < GetTautLineContactCount(); identifies the contact generated by the grab node.
	virtual bool IsTautLineContactGrabbed( int nContact ) const = 0;
	// AMNOTE: Retains a borrowed body used to adjust target positions for body motion; accepts null.
	virtual void SetTautLineReferenceBody( IPhysicsBody *pBody ) = 0;
	// AMNOTE: Creates or refreshes a section between two distinct ordered contact indices. Keep its bodies alive while the section is retained.
	virtual void UpdateTautRopeSection( const VPhys2TautRopeSectionParams_t &params ) = 0;
	// AMNOTE: Clears every retained section's refresh mark before a batch of UpdateTautRopeSection calls.
	virtual void ClearTautRopeSectionMarks() = 0;
	// AMNOTE: Destroys sections that have not been refreshed since their marks were cleared.
	virtual void RemoveUnmarkedTautRopeSections() = 0;
	// AMNOTE: Sums the retained sections' scalar constraint impulses.
	virtual float GetTautRopeConstraintImpulse() const = 0;
	// AMNOTE: Invokes the nonempty callback for each nonzero applied scalar constraint impulse and its body.
	virtual void ForEachAppliedConstraintImpulse( const FnVPhys2AppliedConstraintImpulse_t &callback ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); this operation does not wake the rope.
	virtual void AddNodeVelocity( int nNode, const Vector &vVelocity ) = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); this operation does not wake the rope.
	virtual void AddNodeForce( int nNode, const Vector &vForce ) = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); this operation does not wake the rope.
	virtual void SetNodeForce( int nNode, const Vector &vForce ) = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs().
	virtual Vector GetNodeForce( int nNode ) const = 0;
	// AMNOTE: Rejects nonfinite positions and invalid ordinary-node indices. A changed position clears velocity and wakes the rope.
	virtual bool SetNodePosition( int nNode, const Vector &vPosition ) = 0;
	// AMNOTE: Rejects nonfinite targets. Requires 0 <= nNode < GetNodeCountIncludingPlugs(); this operation does not wake the rope.
	virtual bool SetNodeTargetPosition( int nNode, const Vector &vPosition ) = 0;
	// AMNOTE: Rejects nonfinite velocities and invalid node indices; optionally blends the velocity into up to nine ordinary nodes on either side without waking the rope.
	virtual bool SetNodeVelocity( int nNode, const Vector &vVelocity, bool bDistribute ) = 0;
	// AMNOTE: nNode must be nonnegative; indices at or above GetNodeCount() return false. Returns whether the pin state changed; unpinning restores the default inverse mass.
	virtual bool SetNodePinned( int nNode, bool bPinned ) = 0;
	// AMNOTE: nNode must be nonnegative; indices at or above GetNodeCount() return false.
	virtual bool IsNodePinned( int nNode ) const = 0;

private:
	// AMNOTE: Changes bit zero of a node flag byte when the signed node index is below the ordinary-plus-plug count. Its meaning and complete source signature remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Reads bit zero of the same node flag byte, returning false above the ordinary-plus-plug limit. Its meaning and complete source signature remain unresolved.
	virtual void unk002() = 0;

public:
	// AMNOTE: nEndpoint must be zero or one. Normalizes the axis; a sufficiently small length becomes zero.
	virtual void SetEndpointAlignmentAxis( int nEndpoint, const Vector &vAxis ) = 0;
	// AMNOTE: Stores the grab-node index; minus one disables the grab contact.
	virtual void SetGrabNode( int nNode ) = 0;
	// AMNOTE: Requires at least one position; transforms the input into at least three ordinary nodes and two plug nodes, repeating the last position as needed. Clears motion, contacts and retained taut sections, then wakes the rope.
	virtual void ResetNodes( const CTransform &transform, const CUtlVector<Vector> &positions ) = 0;
	// AMNOTE: Retains a unique nonempty delegate; its bound target must remain valid until removed. Invoked with cleared node forces.
	virtual void AddMotionControlCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Removes the matching delegate; missing delegates are ignored.
	virtual void RemoveMotionControlCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Retains a unique nonempty delegate; its bound target must remain valid until removed. Invoked before the first substep when enabled simulation time is positive.
	virtual void AddPreSimulationCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Removes the matching delegate; missing delegates are ignored.
	virtual void RemovePreSimulationCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Retains a unique nonempty delegate; its bound target must remain valid until removed. Invoked after committing results when enabled simulation time is positive.
	virtual void AddPostSimulationCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Removes the matching delegate; missing delegates are ignored.
	virtual void RemovePostSimulationCallback( const FnVPhys2ParticleRopeCallback_t &callback ) = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); uses simulation storage while asynchronous work is active, otherwise retained node storage.
	virtual Vector GetSimulationNodePosition( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); uses simulation storage while asynchronous work is active, otherwise retained node storage.
	virtual Vector GetSimulationNodeVelocity( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); uses simulation storage while asynchronous work is active, otherwise retained node storage.
	virtual float GetSimulationNodeInverseMass( int nNode ) const = 0;
	// AMNOTE: Requires 0 <= nNode < GetNodeCountIncludingPlugs(); writes simulation storage while asynchronous work is active, otherwise retained node storage.
	virtual void SetSimulationNodeVelocity( int nNode, const Vector &vVelocity ) = 0;
	// AMNOTE: Returns simulation bounds while asynchronous work is active, otherwise retained bounds.
	virtual AABB_t GetSimulationBounds() const = 0;

	// AMNOTE: Draws node stretch and optional collision contacts; borrows pOverlays for the call. Obtain overlays through ISource2Server::GetDebugOverlays.
	virtual void DebugDraw( ISceneViewDebugOverlays *pOverlays, float flMaxStretch, bool bDrawContacts, float flDuration ) const = 0;
};

// AMNOTE: Engine-owned link belonging to a retained taut-rope section.
class CRnTautRopeSectionLink : public CRnLink
{
public:
	float GetAppliedConstraintImpulse() const { return m_flAppliedConstraintImpulse; }

private:
	CRnTautRopeSectionLink() = delete;
	CRnTautRopeSectionLink( const CRnTautRopeSectionLink & ) = delete;
	CRnTautRopeSectionLink &operator=( const CRnTautRopeSectionLink & ) = delete;
	~CRnTautRopeSectionLink() override;

	// AMNOTE: Supports linear/angular queries. All-axis queries and bit 0x80 return false; rigid-only queries require nonpositive spring frequency.
	bool HasConstraints( uint32 nQueryMask ) override;

	float m_flAppliedConstraintImpulse;
};

// AMNOTE: Engine-owned section retained by a particle rope. Bodies are borrowed; links are owned.
// AMNOTE: Native removal, solving and destruction remain supplied by the engine.
class CRnTautRopeSection : public CRnLinkage
{
public:
	CRnParticleRope *GetRnParticleRope() { return m_Rope; }
	const CRnParticleRope *GetRnParticleRope() const { return m_Rope; }
	float GetConstraintImpulse() const { return m_flConstraintImpulse; }

private:
	CRnTautRopeSection() = delete;
	CRnTautRopeSection( const CRnTautRopeSection & ) = delete;
	CRnTautRopeSection &operator=( const CRnTautRopeSection & ) = delete;
	~CRnTautRopeSection() override;

	bool RemoveLink( CRnLink *pLink ) override;
	// AMNOTE: Section solver operation; its full signature remains unresolved.
	void unk001() override;

	CRnParticleRope *m_Rope;
	CRnBody *m_Body[2];
	Vector m_vLocalAnchor[2];
	int m_nContact[2];
	float m_flFrequency;
	float m_flDampingRatio;
	float m_flMaxForce;
	float m_flRestLengthScale;
	float m_flConstraintImpulse;
	float m_flContactRestDistance[2];
	bool m_bMarked;
};

// AMNOTE: Engine-owned adapter layout; the remaining native overrides stay supplied by the engine.
class CPhysicsParticleRope : public IPhysicsParticleRope
{
public:
	// AMNOTE: Unused native SIMD lanes can set additional floating-point status flags.
	Vector GetEndpointDirection( int nEndpoint ) const override;
	float GetLength() const override;
	float GetRestLength() const override { return GetNodeRestDistance( GetNodeCount() - 1 ); }
	int FindNodeAtRestDistance( float flDistance ) const override;
	float GetNodeStretchBlend( int nNode, float flMaxStretch ) const override;
	int GetNodeCountIncludingPlugs() const override { return GetNodeCount() + 2; }
	int GetSegmentCount() const override { return GetNodeCount() - 1; }
	bool IsNodePinned( int nNode ) const override { return nNode < GetNodeCount() && GetNodeInverseMass( nNode ) == 0.0f; }

	// AMNOTE: Borrows the backing rope; destroying the rope invalidates this pointer.
	CRnParticleRope *GetRnParticleRope() { return m_Rope; }
	const CRnParticleRope *GetRnParticleRope() const { return m_Rope; }

private:
	CPhysicsParticleRope() = delete;
	CPhysicsParticleRope( const CPhysicsParticleRope & ) = delete;
	CPhysicsParticleRope &operator=( const CPhysicsParticleRope & ) = delete;
	~CPhysicsParticleRope() override = default;

	CRnParticleRope *m_Rope;
};

inline Vector CPhysicsParticleRope::GetEndpointDirection( int nEndpoint ) const
{
	const int nNodeCount = GetNodeCount();
	const int nEndpointNode = nEndpoint ? nNodeCount - 1 : 0;
	const Vector vEndpoint = GetNodePosition( nEndpointNode );
	const float flPlugRadius = GetEndpointPlugRadius( nEndpoint );
	const bool bUsePlug = flPlugRadius > 0.0f;
	const Vector vOther = GetNodePosition( bUsePlug ? nNodeCount + nEndpoint : ( nEndpoint ? nNodeCount - 2 : 1 ) );
	const fltx4 endpoint = _mm_set_ps( 0.0f, vEndpoint.z, vEndpoint.y, vEndpoint.x );
	const fltx4 other = _mm_set_ps( 0.0f, vOther.z, vOther.y, vOther.x );
	const fltx4 delta = bUsePlug ? SubSIMD( endpoint, other ) : SubSIMD( other, endpoint );
	const fltx4 length = SqrtSIMD( Dot3SIMD( delta, delta ) );
	const fltx4 direction = MaskedAssign( CmpGtSIMD( length, ReplicateX4( 0.0f ) ), DivSIMD( delta, length ), _mm_set_ps( 0.0f, 0.0f, 0.0f, 1.0f ) );
	Vector result;
	StoreUnaligned3SIMD( result.Base(), direction );
	return result;
}

inline float CPhysicsParticleRope::GetLength() const
{
	const int nNodeCount = GetNodeCount();
	float flLength = 0.0f;
	if ( nNodeCount > 1 )
	{
		Vector vPrevious = GetNodePosition( 0 );
		for ( int nNode = 1; nNode < nNodeCount; ++nNode )
		{
			const Vector vPosition = GetNodePosition( nNode );
			const float flX = vPrevious.x - vPosition.x;
			const float flY = vPrevious.y - vPosition.y;
			const float flZ = vPrevious.z - vPosition.z;
			flLength += FastSqrt( flX * flX + flY * flY + flZ * flZ );
			vPrevious = vPosition;
		}
	}
	return flLength;
}

inline int CPhysicsParticleRope::FindNodeAtRestDistance( float flDistance ) const
{
	int nFirstNode, nSecondNode;
	float flFraction;
	GetNodesAtRestDistance( flDistance, nFirstNode, nSecondNode, flFraction );
	return flFraction < 0.5f ? nFirstNode : nSecondNode;
}

inline float CPhysicsParticleRope::GetNodeStretchBlend( int nNode, float flMaxStretch ) const
{
	const float flBlend = ( GetNodeStretch( nNode ) - 1.0f ) / ( flMaxStretch - 1.0f );
	return SubFloat( MinSIMD( MaxSIMD( ReplicateX4( flBlend ), ReplicateX4( 0.0f ) ), ReplicateX4( 1.0f ) ), 0 );
}

#endif // IPHYSICSPARTICLEROPE_H
