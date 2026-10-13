#ifndef IPHYSICSROPELINK_H
#define IPHYSICSROPELINK_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "mathlib/vector.h"
#include "mathlib/ssemath.h"
#include "vphysics2/iphysicsbody.h"

class CRnBody;
class CRnLink;
class CRnLinkage;
class CRnRopeLink;

// AMNOTE: Engine-owned; create and remove through IPhysicsRope. Do not implement or delete it.
// AMNOTE: Anchor positions use each body's local frame. A root link's parent frame is world space.
// AMNOTE: Rotations are SIMD quaternions in xyzw order.
abstract_class IPhysicsRopeLink
{
	friend class CPhysicsRope;

protected:
	virtual ~IPhysicsRopeLink() = default;

public:
	// AMNOTE: Returns the parent link, or null for the root.
	virtual IPhysicsRopeLink *GetParentLink() = 0;
	// AMNOTE: Returns the parent link, or null for the root.
	virtual const IPhysicsRopeLink *GetParentLink() const = 0;
	// AMNOTE: Returns the body attached to this link.
	virtual IPhysicsBody *GetBody() const = 0;
	// AMNOTE: Returns the anchor in the parent body's local frame.
	virtual Vector GetParentLocalAnchor() const = 0;
	// AMNOTE: Replaces the anchor in the parent body's local frame.
	virtual void SetParentLocalAnchor( const Vector &vAnchor ) = 0;
	// AMNOTE: Returns the anchor in this link's body-local frame.
	virtual Vector GetBodyLocalAnchor() const = 0;
	// AMNOTE: Replaces the anchor in this link's body-local frame.
	virtual void SetBodyLocalAnchor( const Vector &vAnchor ) = 0;
	// AMNOTE: Converts a world-space anchor into both local frames and stores both positions.
	virtual void SetWorldAnchor( const Vector &vAnchor ) = 0;
	// AMNOTE: Returns the parent-local rotation as a SIMD quaternion.
	virtual fltx4 GetParentLocalRotation() const = 0;
	// AMNOTE: Replaces the parent-local SIMD quaternion.
	virtual void SetParentLocalRotation( const fltx4 &qRotation ) = 0;
	// AMNOTE: Returns this link's body-local rotation as a SIMD quaternion.
	virtual fltx4 GetBodyLocalRotation() const = 0;
	// AMNOTE: Replaces this link's body-local SIMD quaternion.
	virtual void SetBodyLocalRotation( const fltx4 &qRotation ) = 0;
	// AMNOTE: Converts a world-space quaternion into both local frames and stores both rotations.
	virtual void SetWorldRotation( const fltx4 &qRotation ) = 0;
};

// AMNOTE: Engine-owned, empty marker base. Do not construct, copy or derive from it.
class CRnLinkBase
{
	friend class CRnLink;

protected:
	~CRnLinkBase() = default;

private:
	CRnLinkBase() = default;
	CRnLinkBase( const CRnLinkBase & ) = delete;
	CRnLinkBase &operator=( const CRnLinkBase & ) = delete;
};

// AMNOTE: Intrusive node connecting a link and its linkage to a body's list of links.
struct RnLinkBodyNode_t
{
	CRnLink *m_Link;
	CRnLinkage *m_Linkage;
	RnLinkBodyNode_t *m_Previous;
	RnLinkBodyNode_t *m_Next;
};

// AMNOTE: Engine-owned link. Its destructor removes the body-list node and remains defined by the engine.
class CRnLink : public CRnLinkBase
{
	friend class CPhysicsRopeLink;
	friend class CPhysicsRope;
	friend class CRnRope;

public:
	bool IsParentCollisionEnabled() const { return m_nParentCollisionEnabled != 0; }

protected:
	virtual ~CRnLink();

private:
	// AMNOTE: Clears the backing body pointer; the complete native signature remains unresolved.
	virtual void ClearBody();
	// AMNOTE: Returns false for ordinary links; derived links evaluate the constraint query mask.
	virtual bool HasConstraints( uint32 nQueryMask ) { (void)nQueryMask; return false; }

	CRnLink() = delete;
	CRnLink( const CRnLink & ) = delete;
	CRnLink &operator=( const CRnLink & ) = delete;

	CRnLinkage *m_Linkage;
	CRnLink *m_Parent;
	CRnBody *m_Body;
	RnLinkBodyNode_t m_BodyNode;
	// AMNOTE: Initialized to minus one; its remaining meaning is unresolved.
	int m_unk001;
	// AMNOTE: Zero disables collisions between this body and its parent link's body.
	uint8 m_nParentCollisionEnabled;
	// AMNOTE: Borrowed implementation data; a rope link stores its IPhysicsRopeLink adapter here.
	void *m_unk101;
};

// AMNOTE: Engine-owned rope link; local anchors and rotations use the frames described by IPhysicsRopeLink.
class CRnRopeLink : public CRnLink
{
	friend class CPhysicsRopeLink;

private:
	CRnRopeLink() = delete;
	CRnRopeLink( const CRnRopeLink & ) = delete;
	CRnRopeLink &operator=( const CRnRopeLink & ) = delete;
	~CRnRopeLink() override;

	Vector m_vParentLocalAnchor;
	fltx4 m_qParentLocalRotation;
	Vector m_vBodyLocalAnchor;
	fltx4 m_qBodyLocalRotation;
};

// AMNOTE: Engine-owned adapter. Do not construct, copy, derive from or delete it.
// AMNOTE: GetBody and the core link destructors still require the engine's implementation.
class CPhysicsRopeLink : public IPhysicsRopeLink
{
public:
	IPhysicsRopeLink *GetParentLink() override { return m_RopeLink->m_Parent ? static_cast<IPhysicsRopeLink *>( m_RopeLink->m_Parent->m_unk101 ) : nullptr; }
	const IPhysicsRopeLink *GetParentLink() const override { return m_RopeLink->m_Parent ? static_cast<const IPhysicsRopeLink *>( m_RopeLink->m_Parent->m_unk101 ) : nullptr; }
	Vector GetParentLocalAnchor() const override { return m_RopeLink->m_vParentLocalAnchor; }
	void SetParentLocalAnchor( const Vector &vAnchor ) override { m_RopeLink->m_vParentLocalAnchor = vAnchor; }
	Vector GetBodyLocalAnchor() const override { return m_RopeLink->m_vBodyLocalAnchor; }
	void SetBodyLocalAnchor( const Vector &vAnchor ) override { m_RopeLink->m_vBodyLocalAnchor = vAnchor; }
	void SetWorldAnchor( const Vector &vAnchor ) override;
	fltx4 GetParentLocalRotation() const override { return m_RopeLink->m_qParentLocalRotation; }
	void SetParentLocalRotation( const fltx4 &qRotation ) override { m_RopeLink->m_qParentLocalRotation = qRotation; }
	fltx4 GetBodyLocalRotation() const override { return m_RopeLink->m_qBodyLocalRotation; }
	void SetBodyLocalRotation( const fltx4 &qRotation ) override { m_RopeLink->m_qBodyLocalRotation = qRotation; }
	void SetWorldRotation( const fltx4 &qRotation ) override;

	// AMNOTE: Borrows the backing link; removing the link invalidates this pointer and the adapter.
	CRnRopeLink *GetRnRopeLink() { return m_RopeLink; }
	const CRnRopeLink *GetRnRopeLink() const { return m_RopeLink; }

private:
	static Vector WorldToLocalAnchor( const Vector &vAnchor, const IPhysicsBody *pBody );
	static fltx4 WorldToLocalRotation( const fltx4 &qRotation, const IPhysicsBody *pBody );

	CPhysicsRopeLink() = delete;
	CPhysicsRopeLink( const CPhysicsRopeLink & ) = delete;
	CPhysicsRopeLink &operator=( const CPhysicsRopeLink & ) = delete;
	~CPhysicsRopeLink() override = default;

	CRnRopeLink *m_RopeLink;
};

inline Vector CPhysicsRopeLink::WorldToLocalAnchor( const Vector &vAnchor, const IPhysicsBody *pBody )
{
	const Vector vDelta = vAnchor - pBody->GetPosition();
	const fltx4 qBody = pBody->GetOrientation();
	const Vector vImaginary( SubFloat( qBody, 0 ), SubFloat( qBody, 1 ), SubFloat( qBody, 2 ) );
	const Vector vProduct = vDelta * SubFloat( qBody, 3 ) - CrossProduct( vImaginary, vDelta );
	const Vector vCross = CrossProduct( vImaginary, vProduct );
	return vDelta - ( vCross + vCross );
}

inline fltx4 CPhysicsRopeLink::WorldToLocalRotation( const fltx4 &qRotation, const IPhysicsBody *pBody )
{
	// AMNOTE: Uses the conjugate without normalization or quaternion hemisphere alignment.
	const fltx4 qBody = pBody->GetOrientation();
	const float x = -SubFloat( qBody, 0 );
	const float y = -SubFloat( qBody, 1 );
	const float z = -SubFloat( qBody, 2 );
	const float w = SubFloat( qBody, 3 );
	fltx4 qLocal;
	SubFloat( qLocal, 0 ) = ( ( w * SubFloat( qRotation, 0 ) + x * SubFloat( qRotation, 3 ) ) + y * SubFloat( qRotation, 2 ) ) - z * SubFloat( qRotation, 1 );
	SubFloat( qLocal, 1 ) = ( ( w * SubFloat( qRotation, 1 ) - x * SubFloat( qRotation, 2 ) ) + y * SubFloat( qRotation, 3 ) ) + z * SubFloat( qRotation, 0 );
	SubFloat( qLocal, 2 ) = ( ( w * SubFloat( qRotation, 2 ) + x * SubFloat( qRotation, 1 ) ) - y * SubFloat( qRotation, 0 ) ) + z * SubFloat( qRotation, 3 );
	SubFloat( qLocal, 3 ) = ( ( w * SubFloat( qRotation, 3 ) - x * SubFloat( qRotation, 0 ) ) - y * SubFloat( qRotation, 1 ) ) - z * SubFloat( qRotation, 2 );
	return qLocal;
}

inline void CPhysicsRopeLink::SetWorldAnchor( const Vector &vAnchor )
{
	const IPhysicsRopeLink *pParent = GetParentLink();
	m_RopeLink->m_vParentLocalAnchor = pParent ? WorldToLocalAnchor( vAnchor, pParent->GetBody() ) : vAnchor;
	m_RopeLink->m_vBodyLocalAnchor = WorldToLocalAnchor( vAnchor, GetBody() );
}

inline void CPhysicsRopeLink::SetWorldRotation( const fltx4 &qRotation )
{
	const IPhysicsRopeLink *pParent = GetParentLink();
	m_RopeLink->m_qParentLocalRotation = pParent ? WorldToLocalRotation( qRotation, pParent->GetBody() ) : qRotation;
	m_RopeLink->m_qBodyLocalRotation = WorldToLocalRotation( qRotation, GetBody() );
}

#endif // IPHYSICSROPELINK_H
