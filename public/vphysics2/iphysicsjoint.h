#ifndef IPHYSICSJOINT_H
#define IPHYSICSJOINT_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/utldelegate.h"
#include "entityhandle.h"
#include "tier0/utlstring.h"
#include "tier0/utlvector.h"
#include "mathlib/vector.h"
#include "mathlib/ssemath.h"
#include "mathlib/transform.h"
#include "schemasystem/minschema.h"
#include "vphysics2/iphysicsbody.h"
#include <float.h>

// AMNOTE: Identified numeric joint type codes; other values remain unresolved.
#define VPHYS2_JOINT_TYPE_NULL 0u
#define VPHYS2_JOINT_TYPE_SPHERICAL 1u
// Includes the block variant.
#define VPHYS2_JOINT_TYPE_PRISMATIC 2u
// Includes the block variant.
#define VPHYS2_JOINT_TYPE_REVOLUTE 3u
// Includes the block variant.
#define VPHYS2_JOINT_TYPE_WELD 6u
#define VPHYS2_JOINT_TYPE_SPRING 7u
#define VPHYS2_JOINT_TYPE_PULLEY 8u
#define VPHYS2_JOINT_TYPE_CONICAL 10u
#define VPHYS2_JOINT_TYPE_GENERIC 13u
#define VPHYS2_JOINT_TYPE_SPLINE 15u
#define VPHYS2_JOINT_TYPE_WHEEL 16u

// AMNOTE: Motor modes are stored as 32-bit values; the nominal enum type remains unresolved.
#define JOINT_MOTOR_POSITION_MODE 0u
#define JOINT_MOTOR_VELOCITY_MODE 1u
#define JOINT_MOTOR_POSITION_AND_VELOCITY_MODE 2u
#define JOINT_MOTOR_DISABLED 3u

enum JointMotion_t
{
	JOINT_MOTION_FREE = 0,
	JOINT_MOTION_LOCKED = 1,
	JOINT_MOTION_COUNT = 2,
};

class IVPhysics2World;
class IPhysicsBody;
class IPhysicsJoint;
class CRnJoint;

typedef CUtlDelegate<void( IPhysicsJoint *pJoint )> FnVPhys2JointBreakCallback_t;

struct PhysicsJointQuaternion_t
{
	float x = 0;
	float y = 0;
	float z = 0;
	float w = 1;

	PhysicsJointQuaternion_t &operator=( const Quaternion &q ) { x = q.x; y = q.y; z = q.z; w = q.w; return *this; }
	Quaternion ToQuaternion() const { return Quaternion( x, y, z, w ); }
};

// AMNOTE: Angles and angular velocities in these descriptors are in degrees.
struct PhysicsJointDesc_t
{
	explicit PhysicsJointDesc_t( uint32 nType = VPHYS2_JOINT_TYPE_NULL ) : m_nType( nType ) {}

	uint32 m_nType;
	IPhysicsBody *m_pBody1 = nullptr;
	IPhysicsBody *m_pBody2 = nullptr;
	Vector m_vLocalOrigin1 = Vector( 0, 0, 0 );
	PhysicsJointQuaternion_t m_qLocalBasis1 = {};
	Vector m_vLocalOrigin2 = Vector( 0, 0, 0 );
	PhysicsJointQuaternion_t m_qLocalBasis2 = {};
	bool m_bIsEnabled = true;
	bool m_bIsLinearConstraintEnabled = true;
	bool m_bIsAngularConstraintEnabled = true;
	bool m_bEnableCollision = false;
	// Removes endpoint body scale from the supplied local origins.
	bool m_bLocalOriginsIncludeBodyScale = true;
	float m_flFriction = 0;
	float m_flElasticity = 0;
	float m_flPlasticity = 0;
	float m_flMaxLinearConstraintImpulse = 0;
	float m_flMaxAngularConstraintImpulse = 0;
	FnVPhys2JointBreakCallback_t m_BreakCallback;
	CUtlString m_Tag;
	// Allows one endpoint to be null. Two identical endpoints are always rejected.
	bool m_bAllowNullBodies = false;
};

struct PhysicsSphericalJointDesc_t : PhysicsJointDesc_t
{
	PhysicsSphericalJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_SPHERICAL ) {}
	uint32 m_nMotorMode = JOINT_MOTOR_DISABLED;
	PhysicsJointQuaternion_t m_qMotorTargetOrientation = {};
	float m_flMotorFrequency = 0;
	float m_flMotorDampingRatio = 0;
	Vector m_vMotorTargetVelocity = Vector( 0, 0, 0 );
	float m_flMotorMaxTorque = 0;
};

struct PhysicsRevoluteJointDesc_t : PhysicsJointDesc_t
{
	PhysicsRevoluteJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_REVOLUTE ) {}
	bool m_bUseBlockSolver = false;
	uint32 m_nMotorMode = JOINT_MOTOR_DISABLED;
	float m_flMotorTargetAngle = 0;
	float m_flMotorFrequency = 0;
	float m_flMotorDampingRatio = 0;
	float m_flMotorTargetVelocity = 0;
	float m_flMotorMaxTorque = 0;
	bool m_bEnableLimit = false;
	float m_flMinAngle = 0;
	float m_flMaxAngle = 0;
};

struct PhysicsPrismaticJointDesc_t : PhysicsJointDesc_t
{
	PhysicsPrismaticJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_PRISMATIC ) {}
	bool m_bUseBlockSolver = false;
	uint32 m_nMotorMode = JOINT_MOTOR_DISABLED;
	float m_flMotorTargetOffset = 0;
	float m_flMotorFrequency = 0;
	float m_flMotorDampingRatio = 0;
	float m_flMotorTargetVelocity = 0;
	float m_flMotorMaxForce = 0;
	bool m_bEnableLimit = false;
	float m_flMinOffset = 0;
	float m_flMaxOffset = 0;
};

struct PhysicsConicalJointDesc_t : PhysicsJointDesc_t
{
	PhysicsConicalJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_CONICAL ) {}
	uint32 m_nMotorMode = JOINT_MOTOR_DISABLED;
	PhysicsJointQuaternion_t m_qMotorTargetOrientation = {};
	float m_flMotorFrequency = 0;
	float m_flMotorDampingRatio = 0;
	Vector m_vMotorTargetVelocity = Vector( 0, 0, 0 );
	float m_flMotorMaxTorque = 0;
	bool m_bEnableSwingLimit = false;
	float m_flSwingLimit = 0;
	bool m_bEnableTwistLimit = false;
	float m_flMinTwistAngle = 0;
	float m_flMaxTwistAngle = 0;
};

struct PhysicsWeldJointDesc_t : PhysicsJointDesc_t
{
	PhysicsWeldJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_WELD ) {}
	bool m_bUseBlockSolver = false;
	float m_flLinearFrequency = 0;
	float m_flLinearDampingRatio = 0;
	float m_flMaxForce = 0;
	float m_flAngularFrequency = 0;
	float m_flAngularDampingRatio = 0;
	float m_flMaxTorque = 0;
};

struct PhysicsGenericJointDesc_t : PhysicsJointDesc_t
{
	PhysicsGenericJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_GENERIC ) {}
	JointMotion_t m_nLinearMotions[3] = {};
	float m_flLinearFrequencies[3] = {};
	float m_flLinearDampingRatios[3] = {};
	float m_flMaxLinearImpulses[3] = {};
	JointMotion_t m_nAngularMotions[3] = {};
	float m_flAngularFrequencies[3] = {};
	float m_flAngularDampingRatios[3] = {};
	float m_flMaxAngularImpulses[3] = {};
};

struct PhysicsSpringJointDesc_t : PhysicsJointDesc_t
{
	PhysicsSpringJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_SPRING ) {}
	float m_flFrequency = 5;
	float m_flDampingRatio = 0.7f;
	float m_flReferenceMass = 0;
	float m_flMinRestLength = 0;
	float m_flMaxRestLength = 0;
};

struct PhysicsPulleyJointDesc_t : PhysicsJointDesc_t
{
	PhysicsPulleyJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_PULLEY ) {}
	Vector m_vGround1 = Vector( 0, 0, 0 );
	Vector m_vGround2 = Vector( 0, 0, 0 );
	float m_flRestLength1 = 0;
	float m_flRestLength2 = 0;
	float m_flRatio = 1;
	float m_flFrequency = 0;
	float m_flDampingRatio = 0;
	bool m_bIsRigid = true;
};

struct PhysicsNullJointDesc_t : PhysicsJointDesc_t
{
	PhysicsNullJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_NULL ) {}
};

struct PhysicsSplineJointDesc_t : PhysicsJointDesc_t
{
	PhysicsSplineJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_SPLINE ) {}
	// Equally sized arrays with at least two knots. Control points are offsets from their knots.
	CUtlVector<Vector> m_Positions;
	CUtlVector<Vector> m_IncomingControlPointOffsets;
	CUtlVector<Vector> m_OutgoingControlPointOffsets;
	float m_flFrequency = 0;
	float m_flDampingRatio = 0;
	bool m_bEnableMotor = false;
	float m_flMotorTargetVelocity = 0;
	float m_flMotorMaxForce = 0;
	bool m_bLimitToSplineEndpoints = false;
};

struct PhysicsWheelJointDesc_t : PhysicsJointDesc_t
{
	PhysicsWheelJointDesc_t() : PhysicsJointDesc_t( VPHYS2_JOINT_TYPE_WHEEL ) {}
	float m_flSuspensionFrequency = 0;
	float m_flSuspensionDampingRatio = 0;
	float m_flSuspensionRestLoad = 0;
	float m_flSuspensionHeightOffset = 0;
	bool m_bEnableSuspensionLimit = false;
	float m_flMinSuspensionOffset = 0;
	float m_flMaxSuspensionOffset = 0;
	uint32 m_nSteeringMotorMode = JOINT_MOTOR_DISABLED;
	float m_flSteeringMotorFrequency = 0;
	float m_flSteeringMotorDampingRatio = 0;
	float m_flSteeringMotorTargetAngle = 0;
	float m_flSteeringMotorTargetVelocity = 0;
	float m_flSteeringMotorMaxTorque = 0;
	float m_flSteeringMotorFriction = 0;
	bool m_bEnableSteeringLimit = false;
	float m_flMinSteeringAngle = 0;
	float m_flMaxSteeringAngle = 0;
	bool m_bEnableSpinMotor = false;
	float m_flSpinMotorTargetVelocity = 0;
	float m_flSpinMotorMaxTorque = 0;
};

// AMNOTE: Engine-owned; obtain and destroy through IVPhysics2World. Do not implement or delete it.
// AMNOTE: Unresolved virtual signatures are private placeholders.
abstract_class IPhysicsJoint
{
public:
	// Returns the engine-owned schema class binding for this interface.
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

	// AMNOTE: Returns the owning world, or null when no native world is assigned.
	virtual IVPhysics2World *GetWorld() = 0;
	// AMNOTE: Returns the owning world with const access.
	virtual const IVPhysics2World *GetWorld() const = 0;
	// AMNOTE: Changes enabled state, wakes dynamic endpoints and updates solver registration. An absent endpoint leaves the state unchanged.
	virtual void SetEnabled( bool bEnabled ) = 0;

	// AMNOTE: Returns whether this joint is enabled.
	virtual bool IsEnabled() const = 0;
	// AMNOTE: Returns the numeric joint type code.
	virtual uint32 GetJointType() const = 0;

public:
	// AMNOTE: Returns this family's current state with creation-only options at their defaults; a different family returns its default descriptor.
	virtual PhysicsSphericalJointDesc_t GetSphericalJointDesc() const = 0;
	virtual PhysicsRevoluteJointDesc_t GetRevoluteJointDesc() const = 0;
	virtual PhysicsPrismaticJointDesc_t GetPrismaticJointDesc() const = 0;
	virtual PhysicsConicalJointDesc_t GetConicalJointDesc() const = 0;
	virtual PhysicsWeldJointDesc_t GetWeldJointDesc() const = 0;
	virtual PhysicsGenericJointDesc_t GetGenericJointDesc() const = 0;
	virtual PhysicsSpringJointDesc_t GetSpringJointDesc() const = 0;
	virtual PhysicsPulleyJointDesc_t GetPulleyJointDesc() const = 0;
	virtual PhysicsNullJointDesc_t GetNullJointDesc() const = 0;
	// AMNOTE: Retains common state; spline-specific parameters are returned at their defaults.
	virtual PhysicsSplineJointDesc_t GetSplineJointDesc() const = 0;
	virtual PhysicsWheelJointDesc_t GetWheelJointDesc() const = 0;

public:
	// AMNOTE: nBodyIndex must be zero or one. An absent endpoint returns null.
	virtual IPhysicsBody *GetBody( int nBodyIndex ) const = 0;

	// AMNOTE: nBodyIndex selects zero or one; other indices leave both endpoints unchanged.
	virtual void SetBody( int nBodyIndex, IPhysicsBody *pBody ) = 0;

	// AMNOTE: Returns the first body, or null for an absent endpoint.
	virtual IPhysicsBody *GetBodyA() const = 0;
	// AMNOTE: Returns the second body, or null for an absent endpoint.
	virtual IPhysicsBody *GetBodyB() const = 0;
	// AMNOTE: Matches the backing body and returns the opposite endpoint, or null. A null input can match an absent endpoint.
	virtual IPhysicsBody *GetOtherBody( const IPhysicsBody *pBody ) const = 0;

	// AMNOTE: Changes the first endpoint and updates joint registration; null removes the endpoint.
	virtual void SetBodyA( IPhysicsBody *pBody ) = 0;
	// AMNOTE: Changes the second endpoint and updates joint registration; null removes the endpoint.
	virtual void SetBodyB( IPhysicsBody *pBody ) = 0;

	// AMNOTE: Wakes dynamic endpoints while this joint is enabled.
	virtual void WakeBodies() = 0;

	// AMNOTE: Returns the first endpoint origin in world space, or zero for an absent endpoint.
	virtual Vector GetWorldOriginA() const = 0;
	// AMNOTE: Returns the first endpoint local origin with body scale applied; an absent endpoint returns the retained origin.
	virtual Vector GetLocalOriginA() const = 0;

	// AMNOTE: Removes endpoint body scale before retaining the local origin.
	virtual void SetLocalOriginA( const Vector &vOrigin ) = 0;

	// AMNOTE: Returns the second endpoint origin in world space, or zero for an absent endpoint.
	virtual Vector GetWorldOriginB() const = 0;
	// AMNOTE: Returns the second endpoint local origin with body scale applied; an absent endpoint returns the retained origin.
	virtual Vector GetLocalOriginB() const = 0;

	// AMNOTE: Removes endpoint body scale before retaining the local origin.
	virtual void SetLocalOriginB( const Vector &vOrigin ) = 0;

	// AMNOTE: Returns body orientation multiplied by the first retained local basis; an absent endpoint returns the identity quaternion.
	virtual fltx4 GetWorldBasisA() const = 0;
	// AMNOTE: Returns the first retained local basis without normalization.
	virtual fltx4 GetLocalBasisA() const = 0;

	// AMNOTE: Copies the local basis without normalization.
	virtual void SetLocalBasisA( const fltx4 &qBasis ) = 0;

	// AMNOTE: Returns body orientation multiplied by the second retained local basis; an absent endpoint returns the identity quaternion.
	virtual fltx4 GetWorldBasisB() const = 0;
	// AMNOTE: Returns the second retained local basis without normalization.
	virtual fltx4 GetLocalBasisB() const = 0;

	// AMNOTE: Copies the local basis without normalization.
	virtual void SetLocalBasisB( const fltx4 &qBasis ) = 0;

	// AMNOTE: nBodyIndex selects zero or one; other values return FLT_MAX in all three components.
	virtual Vector GetWorldOrigin( int nBodyIndex ) const = 0;
	// AMNOTE: nBodyIndex selects zero or one; other values return FLT_MAX in all three components.
	virtual Vector GetLocalOrigin( int nBodyIndex ) const = 0;

	// AMNOTE: nBodyIndex selects zero or one; other indices leave the local origins unchanged.
	virtual void SetLocalOrigin( int nBodyIndex, const Vector &vOrigin ) = 0;

	// AMNOTE: nBodyIndex selects zero or one; other values return the identity quaternion.
	virtual fltx4 GetWorldBasis( int nBodyIndex ) const = 0;
	// AMNOTE: nBodyIndex selects zero or one; other values return the identity quaternion.
	virtual fltx4 GetLocalBasis( int nBodyIndex ) const = 0;

	// AMNOTE: nBodyIndex selects zero or one; other indices leave the local bases unchanged.
	virtual void SetLocalBasis( int nBodyIndex, const fltx4 &qBasis ) = 0;

	// AMNOTE: Query mask: 0x1 linear, 0x4 angular, 0x8 all axes, 0x10 rigid only,
	// 0x20 exclude motors, 0x40 skip limit and unresolved freedom masks. The nominal flag type remains unresolved.
	// Disabled joints or missing endpoints return false. Linear and angular selections are combined with OR.
	virtual bool HasConstraints( uint32 nQueryMask ) const = 0;
	// AMNOTE: Additionally requires the selected endpoint to have zero effective inverse mass/inertia and the opposite endpoint to have nonzero values, after mass-priority adjustment.
	// Returns false for an absent or unrelated backing body.
	virtual bool HasConstraints( uint32 nQueryMask, const IPhysicsBody *pAnchorBody ) const = 0;

	// AMNOTE: Changes the linear constraint state without waking the endpoints.
	virtual void SetLinearConstraintEnabled( bool bEnabled ) = 0;
	virtual bool IsLinearConstraintEnabled() const = 0;
	// AMNOTE: Changes the angular constraint state without waking the endpoints.
	virtual void SetAngularConstraintEnabled( bool bEnabled ) = 0;
	virtual bool IsAngularConstraintEnabled() const = 0;

	// AMNOTE: nAxisIndex must be zero, one or two. Generic joints return the signed local-axis impulse component; other identified families return zero.
	virtual float GetLinearConstraintImpulse( int nAxisIndex ) const = 0;
	// AMNOTE: Returns the linear constraint impulse magnitude; generic joints return zero.
	virtual float GetLinearConstraintImpulse() const = 0;
	// AMNOTE: Returns the angular constraint impulse magnitude; families without a retained angular impulse return zero.
	virtual float GetAngularConstraintImpulse() const = 0;

	// AMNOTE: Retains the maximum impulse without clamping; a nonpositive value disables the linear break threshold.
	virtual void SetMaxLinearConstraintImpulse( float flImpulse ) = 0;
	virtual float GetMaxLinearConstraintImpulse() const = 0;
	// AMNOTE: Retains the maximum impulse without clamping; a nonpositive value disables the angular break threshold.
	virtual void SetMaxAngularConstraintImpulse( float flImpulse ) = 0;
	virtual float GetMaxAngularConstraintImpulse() const = 0;
	// AMNOTE: Adjusts the difference between the endpoint mass priorities; the stored adjustment is signed eight-bit.
	virtual void SetMassPriorityAdjustment( int8 nPriorityAdjustment ) = 0;
	virtual int GetMassPriorityAdjustment() const = 0;
	// AMNOTE: Borrows caller data without taking ownership.
	// AMNOTE: Ragdoll joints retain an engine descriptor here; replacing it invalidates subsequent joint-friction updates.
	virtual void SetUserData( void *pUserData ) = 0;
	virtual void *GetUserData() const = 0;

private:
	// AMNOTE: Does no work in this build; the complete formatted-name parameters remain unresolved.
	virtual void SetDebugNameV() = 0;

public:
	// AMNOTE: Returns null in this build.
	virtual const char *GetDebugName() const = 0;

	// AMNOTE: Borrows the retained tag; changing the tag or destroying the joint invalidates the borrowed contents.
	virtual const CUtlString &GetTag() const = 0;
	// AMNOTE: Copies the tag; null clears it.
	virtual void SetTag( const char *pTag ) = 0;

	// AMNOTE: Retains the entity handle associated with this joint.
	virtual void SetEntityHandle( CEntityHandle hEntity ) = 0;
	virtual uint32 GetEntityHandleValue() const = 0;

	CEntityHandle GetEntityHandle() const { return CEntityHandle( GetEntityHandleValue() ); }

	// AMNOTE: Returns the owning world index, or -1 while unregistered.
	virtual int GetWorldIndex() const = 0;

	// AMNOTE: Changes collisions between the endpoints; an enabled joint refreshes pair registration and wakes dynamic endpoints on a change.
	virtual void SetCollisionEnabled( bool bEnabled ) = 0;
	virtual bool IsCollisionEnabled() const = 0;

public:
	// Returns the supplied body transform unchanged for a disabled joint or an unrelated endpoint.
	// Projection acts on the relative joint frames; angular projection can also move the body origin.
	virtual CTransform ProjectBodyTransform( const IPhysicsBody *pBody, const CTransform &otherBodyTransform, const CTransform &bodyTransform, bool bProjectLinearConstraint, bool bProjectAngularConstraint, bool bApplyLimits ) const = 0;

	// AMNOTE: Invokes the break callback before disabling this joint.
	virtual void Break() = 0;
	// AMNOTE: Retains a copy of the delegate; its bound target must remain valid until replaced or the joint is destroyed.
	virtual void SetBreakCallback( const FnVPhys2JointBreakCallback_t &callback ) = 0;

	// AMNOTE: Borrows the backing joint; destroying this joint invalidates the pointer.
	virtual CRnJoint *GetNativeJoint() const = 0;

private:
	// AMNOTE: Does no work in this build; its parameters remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Returns a zero-initialized record; its complete nominal type remains unresolved.
	virtual void unk002() = 0;

protected:
	virtual ~IPhysicsJoint() = default;
};

// AMNOTE: Engine-owned adapter layout; remaining native override definitions are unrecovered. Use the inherited interface on engine-created objects.
class CPhysicsJoint : public IPhysicsJoint
{
public:
	const char *GetDebugName() const override { return nullptr; }
	IPhysicsBody *GetBody( int nBodyIndex ) const override { return nBodyIndex == 0 ? GetBodyA() : GetBodyB(); }
	void SetBody( int nBodyIndex, IPhysicsBody *pBody ) override;
	void SetLocalOrigin( int nBodyIndex, const Vector &vOrigin ) override;
	void SetLocalBasis( int nBodyIndex, const fltx4 &qBasis ) override;

	// AMNOTE: NaN payload propagation follows the body's coordinate conversion.
	Vector GetWorldOriginA() const override;
	Vector GetWorldOriginB() const override;
	fltx4 GetWorldBasisA() const override;
	fltx4 GetWorldBasisB() const override;

	Vector GetWorldOrigin( int nBodyIndex ) const override;
	Vector GetLocalOrigin( int nBodyIndex ) const override;
	fltx4 GetWorldBasis( int nBodyIndex ) const override;
	fltx4 GetLocalBasis( int nBodyIndex ) const override;
	void SetUserData( void *pUserData ) override { m_pUserData = pUserData; }
	void *GetUserData() const override { return m_pUserData; }

	CRnJoint *GetNativeJoint() const override { return m_Joint; }

private:
	CPhysicsJoint() = delete;
	CPhysicsJoint( const CPhysicsJoint & ) = delete;
	CPhysicsJoint &operator=( const CPhysicsJoint & ) = delete;
	~CPhysicsJoint() override = default;

	CRnJoint *m_Joint;
	void *m_pUserData;
};

inline Vector CPhysicsJoint::GetWorldOriginA() const
{
	IPhysicsBody *pBody = GetBodyA();
	return pBody ? pBody->LocalToWorld( GetLocalOriginA() ) : Vector( 0, 0, 0 );
}

inline Vector CPhysicsJoint::GetWorldOriginB() const
{
	IPhysicsBody *pBody = GetBodyB();
	return pBody ? pBody->LocalToWorld( GetLocalOriginB() ) : Vector( 0, 0, 0 );
}

inline fltx4 CPhysicsJoint::GetWorldBasisA() const
{
	IPhysicsBody *pBody = GetBodyA();
	if ( pBody )
		return pBody->LocalToWorldOrientation( GetLocalBasisA() );
	const Quaternion identity( 0.0f, 0.0f, 0.0f, 1.0f );
	return LoadAlignedSIMD( identity.Base() );
}

inline fltx4 CPhysicsJoint::GetWorldBasisB() const
{
	IPhysicsBody *pBody = GetBodyB();
	if ( pBody )
		return pBody->LocalToWorldOrientation( GetLocalBasisB() );
	const Quaternion identity( 0.0f, 0.0f, 0.0f, 1.0f );
	return LoadAlignedSIMD( identity.Base() );
}

inline void CPhysicsJoint::SetBody( int nBodyIndex, IPhysicsBody *pBody )
{
	if ( nBodyIndex == 0 )
		SetBodyA( pBody );
	else if ( nBodyIndex == 1 )
		SetBodyB( pBody );
}

inline void CPhysicsJoint::SetLocalOrigin( int nBodyIndex, const Vector &vOrigin )
{
	if ( nBodyIndex == 0 )
		SetLocalOriginA( vOrigin );
	else if ( nBodyIndex == 1 )
		SetLocalOriginB( vOrigin );
}

inline void CPhysicsJoint::SetLocalBasis( int nBodyIndex, const fltx4 &qBasis )
{
	if ( nBodyIndex == 0 )
		SetLocalBasisA( qBasis );
	else if ( nBodyIndex == 1 )
		SetLocalBasisB( qBasis );
}

inline Vector CPhysicsJoint::GetWorldOrigin( int nBodyIndex ) const
{
	if ( nBodyIndex == 0 )
		return GetWorldOriginA();
	if ( nBodyIndex == 1 )
		return GetWorldOriginB();
	return Vector( FLT_MAX, FLT_MAX, FLT_MAX );
}

inline Vector CPhysicsJoint::GetLocalOrigin( int nBodyIndex ) const
{
	if ( nBodyIndex == 0 )
		return GetLocalOriginA();
	if ( nBodyIndex == 1 )
		return GetLocalOriginB();
	return Vector( FLT_MAX, FLT_MAX, FLT_MAX );
}

inline fltx4 CPhysicsJoint::GetWorldBasis( int nBodyIndex ) const
{
	if ( nBodyIndex == 0 )
		return GetWorldBasisA();
	if ( nBodyIndex == 1 )
		return GetWorldBasisB();
	const Quaternion identity( 0.0f, 0.0f, 0.0f, 1.0f );
	return LoadAlignedSIMD( identity.Base() );
}

inline fltx4 CPhysicsJoint::GetLocalBasis( int nBodyIndex ) const
{
	if ( nBodyIndex == 0 )
		return GetLocalBasisA();
	if ( nBodyIndex == 1 )
		return GetLocalBasisB();
	const Quaternion identity( 0.0f, 0.0f, 0.0f, 1.0f );
	return LoadAlignedSIMD( identity.Base() );
}

#endif // IPHYSICSJOINT_H
