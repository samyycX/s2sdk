#ifndef IPHYSICSBODY_H
#define IPHYSICSBODY_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/tier0_stdlib.h"
#include "mathlib/vector.h"
#include "mathlib/ssemath.h"
#include "mathlib/mathlib.h"
#include "mathlib/transform.h"
#include "mathlib/camera.h"
#include "mathlib/vectorws.h"
#include "tier0/utlstring.h"
#include "tier0/utlvector.h"
#include "tier0/utlsortvector.h"
#include "tier0/utlleanvector.h"
#include "vphysics2/iphysicsshape.h"
#include "schemasystem/minschema.h"

class IVPhysics2World;
class IPhysicsShape;
class IPhysicsMotionController;
class IPhysicsJoint;
class CRnBody;
class IPhysicsBody;
struct RnCollisionAttr_t;

struct QuaternionStorage
{
	float x;
	float y;
	float z;
	float w;
};

enum DynamicContinuousContactBehavior_t : uint8
{
	DYNAMIC_CONTINUOUS_ALLOW_IF_REQUESTED_BY_OTHER_BODY = 0,
	DYNAMIC_CONTINUOUS_ALWAYS = 1,
	DYNAMIC_CONTINUOUS_NEVER = 2,
};

// AMNOTE: Engine-owned descriptor; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnBodyDesc_t
{
	CUtlString m_sDebugName;
	VectorWS m_vPosition;
	QuaternionStorage m_qOrientation;
	Vector m_vLinearVelocity;
	Vector m_vAngularVelocity;
	Vector m_vLocalMassCenter;
	Vector m_LocalInertiaInv[3];
	float m_flMassInv;
	float m_flGameMass;
	float m_flMassScaleInv;
	float m_flInertiaScaleInv;
	float m_flLinearDamping;
	float m_flAngularDamping;
	float m_flLinearDragScale;
	float m_flAngularDragScale;
	float m_flLinearFluidDragScale;
	float m_flAngularFluidDragScale;
	Vector m_vLastAwakeForceAccum;
	Vector m_vLastAwakeTorqueAccum;
	float m_flBuoyancyScale;
	float m_flGravityScale;
	float m_flTimeScale;
	int32 m_nBodyType;
	uint32 m_nGameIndex;
	uint32 m_nGameFlags;
	int8 m_nMinVelocityIterations;
	int8 m_nMinPositionIterations;
	int8 m_nMassPriority;
	bool m_bEnabled;
	bool m_bSleeping;
	bool m_bIsContinuousEnabled;
	bool m_bDragEnabled;
	Vector m_vGravity;
	bool m_bSpeculativeEnabled;
	bool m_bHasShadowController;
	DynamicContinuousContactBehavior_t m_nDynamicContinuousContactBehavior;

private:
	RnBodyDesc_t() = delete;
	RnBodyDesc_t( const RnBodyDesc_t & ) = delete;
	RnBodyDesc_t &operator=( const RnBodyDesc_t & ) = delete;
	~RnBodyDesc_t() = delete;
} ALIGN8_POST;

struct RnSphere_t
{
	Vector m_vCenter;
	float m_flRadius;
};

struct RnCapsule_t
{
	Vector m_vCenter[2];
	float m_flRadius;
};

// AMNOTE: XYZW SIMD orientations and centers of mass; only m_vLocalCenterOfMass is in body-local space.
struct RnSweep_t
{
	fltx4 m_qStart;
	fltx4 m_qEnd;
	float m_flStartFraction;
	Vector m_vLocalCenterOfMass;
	Vector m_vStartCenterOfMass;
	Vector m_vEndCenterOfMass;
};

struct RnHalfEdge_t
{
	uint8 m_nNext;
	uint8 m_nTwin;
	uint8 m_nOrigin;
	uint8 m_nFace;
};

struct RnFace_t
{
	uint8 m_nEdge;
};

struct RnVertex_t
{
	uint8 m_nEdge;
};

struct RnPlane_t
{
	Vector m_vNormal;
	float m_flOffset;
};

// AMNOTE: Engine-owned hull region tree; creation, cloning and destruction belong to the engine.
struct CRegionSVM
{
public:
	// AMNOTE: Returns the encoded leaf region for a hull-local point. Requires a nonempty valid tree.
	uint32 ClassifyPoint( const Vector &point ) const;

	CUtlVector<RnPlane_t> m_Planes;
	CUtlVector<uint32> m_Nodes;

private:
	CRegionSVM() = delete;
	CRegionSVM( const CRegionSVM & ) = delete;
	CRegionSVM &operator=( const CRegionSVM & ) = delete;
	~CRegionSVM();
};

// AMNOTE: Engine-owned geometry; keep it alive while a shape borrows it. Creation, cloning and destruction belong to the engine.
struct RnHull_t
{
	Vector m_vCentroid;
	float m_flMaxAngularRadius;
	float m_flMinCentroidRadius;
	AABB_t m_Bounds;
	Vector m_vOrthographicAreas;
	matrix3x4_t m_MassProperties;
	float m_flVolume;
	float m_flSurfaceArea;
	CUtlVector<Vector> m_VertexPositions;
	CUtlVector<RnPlane_t> m_FacePlanes;
	uint32 m_nFlags;
	CRegionSVM *m_pRegionSVM;
	CUtlVector<RnVertex_t> m_Vertices;
	CUtlVector<RnHalfEdge_t> m_Edges;
	CUtlVector<RnFace_t> m_Faces;

private:
	RnHull_t() = delete;
	RnHull_t( const RnHull_t & ) = delete;
	RnHull_t &operator=( const RnHull_t & ) = delete;
	~RnHull_t();
};

// AMNOTE: Engine-owned storage; the engine keeps an extra Vector beyond Count(). Do not construct or delete it.
class CUtlVectorSIMDPaddedVector
{
public:
	CUtlVectorSIMDPaddedVector() = delete;
	CUtlVectorSIMDPaddedVector( const CUtlVectorSIMDPaddedVector & ) = delete;
	CUtlVectorSIMDPaddedVector &operator=( const CUtlVectorSIMDPaddedVector & ) = delete;

	int Count() const { return m_Size; }
	bool IsEmpty() const { return m_Size == 0; }
	bool IsValidIndex( int i ) const { return i >= 0 && i < m_Size; }
	int NumAllocated() const { return m_Memory.NumAllocated(); }
	const Vector *Base() const { return m_Memory.Base(); }
	const Vector &Element( int i ) const { Assert( IsValidIndex( i ) ); return m_Memory[ i ]; }
	const Vector &operator[]( int i ) const { return Element( i ); }
	const Vector *begin() const { return Base(); }
	const Vector *end() const { const Vector *p = Base(); return p ? p + Count() : nullptr; }

private:
	~CUtlVectorSIMDPaddedVector();
	int m_Size;
	CUtlVectorMemory_Growable< Vector > m_Memory;
};

struct ALIGN16 RnNode_t
{
	Vector m_vMin;
	uint32 m_nChildren;
	Vector m_vMax;
	uint32 m_nTriangleOffset;
} ALIGN16_POST;

struct RnTriangle_t
{
	int32 m_nIndex[3];
};

struct RnWing_t
{
	int32 m_nIndex[3];
};

struct RnCompoundTreeNode_t
{
	Vector m_vMin;
	Vector m_vMax;
	uint32 m_nType : 3;
	uint32 m_nSubtreeEndOrCompoundId : 29;
};

struct RnCompoundTree_t
{
	RnCompoundTree_t() = delete;
	RnCompoundTree_t( const RnCompoundTree_t & ) = delete;
	RnCompoundTree_t &operator=( const RnCompoundTree_t & ) = delete;

	CUtlLeanVector< RnCompoundTreeNode_t > m_Nodes;
	uint32 m_nStartIterationIndex;

private:
	~RnCompoundTree_t();
};

// AMNOTE: Engine-owned geometry; keep it alive while a shape borrows it. Creation, cloning and destruction belong to the engine.
struct RnMesh_t
{
	RnMesh_t() = delete;
	RnMesh_t( const RnMesh_t & ) = delete;
	RnMesh_t &operator=( const RnMesh_t & ) = delete;

	Vector m_vMin;
	Vector m_vMax;
	CUtlVector< RnNode_t > m_Nodes;
	CUtlVectorSIMDPaddedVector m_Vertices;
	CUtlVector< RnTriangle_t > m_Triangles;
	CUtlVector< RnWing_t > m_Wings;
	CUtlVector< uint8 > m_TriangleEdgeFlags;
	CUtlVector< uint8 > m_Materials;
	Vector m_vOrthographicAreas;
	uint32 m_nFlags;
	uint32 m_nDebugFlags;
	float m_flSurfaceArea;

private:
	~RnMesh_t();
};

// AMNOTE: Engine-owned inline mesh storage; the native capacity selector ignores the external marker.
template<>
class CUtlLeanVectorFixedGrowable< RnMesh_t, 1, int, CMemAllocAllocator >
{
public:
	CUtlLeanVectorFixedGrowable() = delete;
	CUtlLeanVectorFixedGrowable( const CUtlLeanVectorFixedGrowable & ) = delete;
	CUtlLeanVectorFixedGrowable &operator=( const CUtlLeanVectorFixedGrowable & ) = delete;

	int Count() const { return m_Header.m_nCount; }
	bool IsEmpty() const { return Count() == 0; }
	int NumAllocated() const { return m_Header.m_nAllocated & 0x7FFFFFFF; }
	bool IsValidIndex( int i ) const { return i >= 0 && i < Count(); }
	const RnMesh_t *Base() const
	{
		const int nAllocated = NumAllocated();
		return nAllocated > 1 ? m_Allocated.m_pElements : nAllocated ? m_Fixed.m_Elements : nullptr;
	}
	const RnMesh_t &Element( int i ) const { Assert( IsValidIndex( i ) ); return Base()[ i ]; }
	const RnMesh_t &operator[]( int i ) const { return Element( i ); }
	const RnMesh_t *begin() const { return Base(); }
	const RnMesh_t *end() const { const RnMesh_t *p = Base(); return p ? p + Count() : nullptr; }

private:
	~CUtlLeanVectorFixedGrowable();

	struct Header_t
	{
		int m_nCount;
		int m_nAllocated;
	};
	struct FixedLayout_t
	{
		int m_nCount;
		int m_nAllocated;
		RnMesh_t m_Elements[1];
	private:
		~FixedLayout_t();
	};
	struct AllocatedLayout_t
	{
		int m_nCount;
		int m_nAllocated;
		RnMesh_t *m_pElements;
	};
	union
	{
		Header_t m_Header;
		FixedLayout_t m_Fixed;
		AllocatedLayout_t m_Allocated;
	};
};

// AMNOTE: Engine-owned geometry; keep it alive while a shape borrows it. Creation, cloning and destruction belong to the engine.
struct RnCompound_t
{
	RnCompound_t() = delete;
	RnCompound_t( const RnCompound_t & ) = delete;
	RnCompound_t &operator=( const RnCompound_t & ) = delete;

	RnCompoundTree_t m_Tree;
	int32 m_nHullBaseIndex;
	int32 m_nMeshBaseIndex;
	int32 m_nShapeCount;
	CUtlLeanVectorFixedGrowable< RnMesh_t, 1 > m_Meshes;
	CUtlLeanVector< RnHull_t > m_Hulls;
	CUtlLeanVector< RnCapsule_t > m_Capsules;
	CUtlLeanVector< RnSphere_t > m_Spheres;
	CUtlLeanVector< uint8 > m_CompoundMaterialIndices;
	AABB_t m_Bounds;
	Vector m_vOrthographicAreas;
	float m_flSurfaceArea;
	float m_flVolume;

private:
	~RnCompound_t();
};

// AMNOTE: Engine-owned; obtain through IPhysicsBody::GetShadowController. Do not implement or delete it.
// AMNOTE: Unresolved virtual signatures are private placeholders.
abstract_class IPhysicsShadowController
{
protected:
	virtual ~IPhysicsShadowController() = default;

public:
	// AMNOTE: Writes the last target position and xyzw SIMD quaternion. Either output may be null.
	virtual void GetTargetPosition( Vector *pPositionOut, fltx4 *pOrientationOut ) = 0;
	// AMNOTE: Writes the last target position and Euler angles in degrees. Either output may be null.
	virtual void GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) = 0;
	// AMNOTE: Replaces the target position, xyzw SIMD quaternion and time offset; negative timeOffset becomes zero, while NaN and negative zero are preserved. Wakes a sleeping body when position error exceeds 1/32 unit or rotation error exceeds two degrees.
	virtual void Update( const Vector &position, const fltx4 &qOrientation, float timeOffset ) = 0;
	// AMNOTE: Converts Euler angles in degrees to a quaternion and updates the shadow target using the same wake thresholds.
	virtual void Update( const Vector &position, const QAngle &angles, float timeOffset ) = 0;

private:
	// AMNOTE: Does no work in this build; its parameters remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Does no work in this build; its parameters remain unresolved.
	virtual void unk002() = 0;

public:
	// AMNOTE: Writes a zero impulse when pOut is nonnull.
	virtual void GetLastImpulse( Vector *pOut ) = 0;

private:
	// AMNOTE: Returns zero in a floating-point register; its complete result type and purpose remain unresolved.
	virtual void unk101() = 0;
	// AMNOTE: Does no work in this build; its parameters remain unresolved.
	virtual void unk102() = 0;
	// AMNOTE: Reads whether the backing body's motion type is zero; this entry's distinct role remains unresolved.
	virtual bool unk103() const = 0;
	// AMNOTE: Reads the same motion-type predicate; this entry's distinct role remains unresolved.
	virtual bool unk104() const = 0;
};

// AMNOTE: Engine-owned shadow target state; do not construct or delete it.
class VPhys2ShadowControllerState
{
public:
	Vector GetTargetPosition() const { return m_TargetPosition; }
	fltx4 GetTargetOrientation() const { return m_TargetOrientation; }
	CRnBody *GetBody() { return m_Body; }
	const CRnBody *GetBody() const { return m_Body; }
	float GetTimeOffset() const { return m_TimeOffset; }

private:
	VPhys2ShadowControllerState() = delete;
	VPhys2ShadowControllerState( const VPhys2ShadowControllerState & ) = delete;
	VPhys2ShadowControllerState &operator=( const VPhys2ShadowControllerState & ) = delete;
	~VPhys2ShadowControllerState() = default;

	fltx4 m_TargetOrientation;
	CRnBody *m_Body;
	Vector m_TargetPosition;
	float m_TimeOffset;
};

// AMNOTE: Deprecated, use VPhys2ShadowControllerState.
using unkVPhys2ShadowControllerState = VPhys2ShadowControllerState;

// AMNOTE: Engine-owned adapter layout; remaining native override definitions are unrecovered. Use the inherited interface on engine-created objects.
class CPhysicsShadowController : public IPhysicsShadowController
{
public:
	void GetTargetPosition( Vector *pPositionOut, fltx4 *pOrientationOut ) override;
	// AMNOTE: NaN payloads for nonfinite orientations can vary with the compiler.
	void GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;
	using IPhysicsShadowController::Update;
	void Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void GetLastImpulse( Vector *pOut ) override { if ( pOut ) pOut->Init( 0.0f, 0.0f, 0.0f ); }

	// AMNOTE: Borrows the backing shadow state; removing the controller invalidates this pointer.
	VPhys2ShadowControllerState *GetImplementation() { return m_Implementation; }
	const VPhys2ShadowControllerState *GetImplementation() const { return m_Implementation; }

private:
	CPhysicsShadowController() = delete;
	CPhysicsShadowController( const CPhysicsShadowController & ) = delete;
	CPhysicsShadowController &operator=( const CPhysicsShadowController & ) = delete;
	~CPhysicsShadowController() override = default;

	VPhys2ShadowControllerState *m_Implementation;
};
struct VPhys2MassProperties_t
{
	VPhys2MassProperties_t() :
		m_InertiaTensor( 1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f ),
		m_vMassCenter( 0.0f, 0.0f, 0.0f ),
		m_flMass( 0.0f ) {}

	// AMNOTE: Local inertia about m_vMassCenter; the fourth column is unused.
	matrix3x4_t m_InertiaTensor;
	Vector m_vMassCenter;
	float m_flMass;
};

// AMNOTE: Engine-owned contact iterator; create and destroy through IPhysicsBody.
// Contact topology and simulation changes invalidate its borrowed data. Do not implement or delete it.
abstract_class IPhysicsFrictionSnapshot
{
protected:
	virtual ~IPhysicsFrictionSnapshot() = default;

public:
	// AMNOTE: Returns whether the iterator has a current contact point.
	virtual bool IsValid() = 0;
	// AMNOTE: Zero selects the source body; any other index selects the opposing body and requires a valid iterator. The returned pointer is borrowed.
	virtual IPhysicsBody *GetBody( int index ) = 0;
	// AMNOTE: Writes the current point in world space; an invalid iterator writes (0,0,1).
	virtual void GetContactPoint( Vector &out ) = 0;
	// AMNOTE: Writes the normal facing away from the source body; an invalid iterator writes (0,0,1).
	virtual void GetSurfaceNormal( Vector &out ) = 0;
	// AMNOTE: Returns the current stored contact value, with the sign selected by the contact side; zero when invalid.
	virtual float GetNormalForce() = 0;
	// AMNOTE: This build forwards to GetNormalForce and returns the same value.
	virtual float GetEnergyAbsorbed() = 0;
	// AMNOTE: Advances to the next point, then manifold, contact and shape; exhausting the iterator makes IsValid false.
	virtual void NextFrictionData() = 0;
	// AMNOTE: Returns the geometric mean of the contacting shapes' friction coefficients, or zero when invalid.
	virtual float GetFrictionCoefficient() = 0;

private:
	friend class CPhysicsBody;
	// AMNOTE: Does no work in this build; the complete signature remains unresolved.
	virtual void unk001() = 0;
};

// AMNOTE: Complete engine-owned iterator layout; other native overrides remain inherited.
class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot
{
public:
	bool IsValid() override { return m_TaggedContact != 0; }
	float GetEnergyAbsorbed() override { return GetNormalForce(); }

private:
	CPhysicsFrictionSnapshot() = delete;
	CPhysicsFrictionSnapshot( const CPhysicsFrictionSnapshot & ) = delete;
	CPhysicsFrictionSnapshot &operator=( const CPhysicsFrictionSnapshot & ) = delete;
	~CPhysicsFrictionSnapshot() override = default;

	CRnBody *m_Body;
	int m_ShapeIndex;
	// AMNOTE: Borrowed contact address with its source-side bit in the low bit.
	uintp m_TaggedContact;
	int m_ManifoldIndex;
	int m_PointIndex;
};

// AMNOTE: Numeric motion types used by GetMotionType.
#define BODY_STATIC 0
#define BODY_KEYFRAMED 1
#define BODY_DYNAMIC 2

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
// AMNOTE: Unresolved virtual signatures are private placeholders.
abstract_class IPhysicsBody
{
public:
	// Returns the engine-owned schema class binding for this interface.
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

	// AMNOTE: Returns this body's world, or null when detached.
	virtual IVPhysics2World *GetWorld() = 0;
	// AMNOTE: Returns this body's world with const access, or null when detached.
	virtual const IVPhysics2World *GetWorld() const = 0;
	// AMNOTE: Changes the body's motion type and refreshes collision, velocity and shadow-controller state. motionType must be BODY_STATIC, BODY_KEYFRAMED or BODY_DYNAMIC.
	virtual void SetMotionType( int motionType ) = 0;
	// AMNOTE: Returns the motion type: BODY_STATIC, BODY_KEYFRAMED or BODY_DYNAMIC.
	virtual int GetMotionType() const = 0;
	// AMNOTE: Enables or disables the body's simulation and collision participation. nEnabled must be zero or one; changes are queued while the world defers body updates.
	virtual void SetEnabled( uint8 nEnabled ) = 0;
	// AMNOTE: Returns whether the body is enabled.
	virtual bool IsEnabled() const = 0;
	// AMNOTE: Counts the body's attached collision shapes.
	virtual int GetShapeCount() const = 0;
	// AMNOTE: Returns the indexed shape, or null when nIndex reaches the shape count. nIndex must be nonnegative.
	virtual IPhysicsShape *GetShape( int nIndex ) = 0;
	// AMNOTE: Returns the indexed shape with const access. nIndex must be nonnegative; indices at or beyond the count return null.
	virtual const IPhysicsShape *GetShape( int nIndex ) const = 0;
	// AMNOTE: Returns the first matching attached shape adapter's index, or -1. A null pointer matches an attached shape without an adapter.
	virtual int FindShape( const IPhysicsShape *pShape ) const = 0;
	// AMNOTE: Copies sphere geometry, using unit scale for nonpositive or NaN scale. Nonzero nRecalculateMass requests a mass-properties refresh.
	virtual IPhysicsShape *CreateSphereShape( const RnSphere_t &sphere, float flScale, uint8 nRecalculateMass ) = 0;
	// AMNOTE: Copies capsule geometry, using unit scale for nonpositive or NaN scale. Nonzero nRecalculateMass requests a mass-properties refresh.
	virtual IPhysicsShape *CreateCapsuleShape( const RnCapsule_t &capsule, float flScale, uint8 nRecalculateMass ) = 0;
	// AMNOTE: Borrows hull geometry. Null or invalid hull returns null; nonpositive or NaN scale selects one. Nonzero nRecalculateMass requests a mass-properties refresh.
	virtual IPhysicsShape *CreateHullShape( const RnHull_t *pHull, float flScale, uint8 nRecalculateMass ) = 0;
	// AMNOTE: Borrows mesh geometry and copies optional material bindings. Null or invalid mesh returns null. All three scale components must be positive to retain the supplied scale.
	virtual IPhysicsShape *CreateMeshShape( const RnMesh_t *pMesh, const Vector &scale, uint8 nRecalculateMass, const CUtlVector<VPhys2ShapeMaterialBinding> *pMaterialBindings ) = 0;
	// AMNOTE: Borrows compound geometry and copies optional material bindings. Null or empty compound returns null. The supplied scale is stored unchanged.
	virtual IPhysicsShape *CreateCompoundShape( const RnCompound_t *pCompound, float flScale, uint8 nRecalculateMass, const CUtlVector<VPhys2ShapeMaterialBinding> *pMaterialBindings ) = 0;
	// AMNOTE: Copies collision geometry and material into this body. Returns null for a null source, a source attached to this body or invalid geometry. An existing first shape supplies the collision-filter attributes.
	virtual IPhysicsShape *CloneShape( const IPhysicsShape *pShape ) = 0;
	// AMNOTE: Detaches a registered shape and queues its adapter for deletion. pShape must be nonnull and attached to this body; do not use it after removal.
	virtual void DestroyShape( IPhysicsShape *pShape ) = 0;
	// AMNOTE: Detaches all collision shapes and queues their adapters for deletion.
	virtual void DestroyAllShapes() = 0;
	// AMNOTE: Enables or disables touch events on every attached shape and wakes an eligible body. Nonzero nEnable enables the flag.
	virtual void EnableTouchEvents( uint8 nEnable ) = 0;
	// AMNOTE: Replaces collision attributes on every attached shape. The supplied record is borrowed for this call.
	virtual void SetCollisionAttributes( const RnCollisionAttr_t &attributes ) = 0;
	// AMNOTE: Applies resolved surface properties to all shapes without updating mass. Failed resolution leaves materials unchanged; a missing name may use the default properties.
	virtual void SetSurfaceProperties( uint32 surfacePropertyHash ) = 0;

	// AMNOTE: Sums attached shapes' surface areas, including shape scale but before body scale. A body with no shapes returns zero.
	virtual float GetSurfaceArea() const = 0;

	// AMNOTE: Returns the directional support point in world space. unitDirection must have unit length.
	virtual VPhys2SupportPoint_t GetSupportPoint( const Vector &unitDirection ) const = 0;
	// AMNOTE: Uses bodyToQuery, including its scale, instead of the body's current transform. The transformed support direction is not normalized.
	virtual VPhys2SupportPoint_t GetSupportPoint( const CTransform &bodyToQuery, const Vector &unitDirection ) const = 0;
	// AMNOTE: Tests a sphere with a world-space center. The center is converted through body scale; the radius is used unchanged for both the world-bounds rejection and shape-local tests.
	virtual bool Overlap( const Vector &vCenter, float flRadius ) const = 0;
	// AMNOTE: Tests bounds in query space using the supplied query-to-world transform.
	virtual bool Overlap( const CTransform &queryToWorld, const AABB_t &bounds ) const = 0;
	// AMNOTE: Tests a world-space capsule against attached shapes.
	virtual bool Overlap( const RnCapsule_t &capsule ) const = 0;
	// AMNOTE: Returns the closest eligible shape point in world space. Writes its shape index when pShapeIndex is nonnull; no eligible result returns FLT_MAX in each component and index -1.
	virtual Vector GetClosestPoint( const Vector &worldPosition, int *pShapeIndex ) const = 0;
	// AMNOTE: Returns the eligible shape point closest to the world-space segment. Writes its shape index when pShapeIndex is nonnull; no eligible result returns FLT_MAX in each component and index -1.
	virtual Vector GetClosestPointToSegment( const Vector &vStart, const Vector &vEnd, int *pShapeIndex ) const = 0;
	// AMNOTE: Returns the union of attached shape bounds in body-local coordinates. A body without shapes returns invalid bounds with FLT_MAX minima and -FLT_MAX maxima.
	virtual AABB_t GetLocalBounds() const = 0;
	// AMNOTE: Returns the union of attached shape bounds transformed by the body's current origin and orientation. A body without shapes returns invalid bounds with FLT_MAX minima and -FLT_MAX maxima.
	virtual AABB_t GetWorldBounds() const = 0;
	// AMNOTE: Sets the requested mass, using 500 for nonpositive values and limiting it to 1000000. nControlValue is forwarded to the mass-property update; its meaning remains unresolved.
	virtual void SetMass( float mass, uint8 nControlValue ) = 0;
	// AMNOTE: Replaces the unscaled local center of mass while preserving the body's origin.
	virtual void SetMassCenterLocalSpace( const Vector &vMassCenter ) = 0;
	// AMNOTE: Overrides mass, local inertia and local center of mass while preserving the body's origin. The tensor's fourth column is ignored. nControlValue is forwarded to the mass-property update; its meaning remains unresolved.
	virtual void SetMassProperties( const VPhys2MassProperties_t &massProperties, uint8 nControlValue ) = 0;
	// AMNOTE: Returns whether the requested mass overrides the mass computed from shapes.
	virtual bool IsMassOverridden() const = 0;
	// AMNOTE: Returns whether the local center of mass overrides the center computed from shapes.
	virtual bool IsMassCenterOverridden() const = 0;
	// AMNOTE: Returns the requested body mass.
	virtual float GetMass() const = 0;
	// AMNOTE: Returns the cached inverse mass used when applying a linear impulse.
	virtual float GetInvMass() const = 0;
	// AMNOTE: Multiplies the inverse-mass scale cumulatively without changing the requested mass. Does not clamp or reject nonfinite values.
	virtual void ScaleInvMass( float flScale ) = 0;
	// AMNOTE: Resets the inverse-mass scale to one and updates the body's effective mass properties.
	virtual void ResetInvMassScale() = 0;
	// AMNOTE: Returns the world-space center of mass.
	virtual Vector GetMassCenterWorldSpace() const = 0;
	// AMNOTE: Returns the unscaled local center of mass.
	virtual Vector GetMassCenterLocalSpace() const = 0;
	// AMNOTE: Returns requested local mass properties, before body scale and inverse-mass/inertia multipliers. A singular inertia calculation produces NaN tensor entries.
	virtual VPhys2MassProperties_t GetMassProperties() const = 0;
	// AMNOTE: Returns whether local inertia overrides the inertia computed from shapes.
	virtual bool IsInertiaOverridden() const = 0;
	// AMNOTE: Returns the requested local inertia tensor with a zero fourth column, before body scale and inverse-inertia multipliers. A singular calculation produces NaN entries.
	virtual matrix3x4_t GetInertiaTensorLocalSpace() const = 0;
	// AMNOTE: Returns principal moments derived from the effective world inverse-inertia tensor. Components are not the body's xyz tensor diagonal; a zero eigenvalue produces infinity.
	virtual Vector GetInertia() const = 0;
	// AMNOTE: Multiplies the inverse-inertia scale cumulatively without changing requested local inertia. Does not clamp or reject nonfinite values.
	virtual void ScaleInvInertia( float flScale ) = 0;
	// AMNOTE: Resets the inverse-inertia scale to one and updates the body's effective mass properties.
	virtual void ResetInvInertiaScale() = 0;
	// AMNOTE: Moves the body's origin, accounting for its rotated and scaled local center of mass.
	virtual void SetPosition( const Vector &worldPosition ) = 0;
	// AMNOTE: Returns the body's world-space origin, removing the rotated and scaled local center-of-mass offset.
	virtual Vector GetPosition() const = 0;
	// AMNOTE: Replaces the body's xyzw SIMD quaternion when it differs from the current rotation.
	virtual void SetOrientation( const fltx4 &qOrientation ) = 0;
	// AMNOTE: Returns the body's xyzw SIMD quaternion.
	virtual fltx4 GetOrientation() const = 0;
	// AMNOTE: Updates the world-space origin and rotation without changing the body's scale.
	virtual void SetTransform( const CTransform &transform ) = 0;
	// AMNOTE: Returns the world-space origin and a normalized rotation with scale one.
	virtual CTransform GetTransform() const = 0;
	// AMNOTE: Updates the world-space origin, rotation and scale from transform.
	virtual void SetScaledTransform( const CTransform &transform ) = 0;
	// AMNOTE: Returns the world-space origin, stored rotation and body scale.
	virtual CTransform GetScaledTransform() const = 0;
	// AMNOTE: Returns an enabled, nonstatic body's shadow target at scale one; otherwise returns GetTransform().
	virtual CTransform GetShadowTransform() const = 0;
	// AMNOTE: Rotates a world vector into body-local space using the stored orientation, without applying translation or scale.
	virtual Vector WorldToLocalVector( const Vector &worldVector ) const = 0;
	// AMNOTE: Rotates a local vector into world space using the stored orientation, without applying translation or scale.
	virtual Vector LocalToWorldVector( const Vector &localVector ) const = 0;
	// AMNOTE: Converts a world-space point to body-local coordinates using the stored quaternion and body origin, without dividing by body scale.
	virtual Vector WorldToLocal( const Vector &worldPosition ) const = 0;
	// AMNOTE: Converts a body-local point to world space using the stored quaternion and body origin without scaling the point.
	virtual Vector LocalToWorld( const Vector &localPosition ) const = 0;
	// AMNOTE: Multiplies a world orientation by the conjugated stored quaternion without normalization.
	virtual fltx4 WorldToLocalOrientation( const fltx4 &qWorldOrientation ) const = 0;
	// AMNOTE: Multiplies a local orientation by the stored quaternion without normalization.
	virtual fltx4 LocalToWorldOrientation( const fltx4 &qLocalOrientation ) const = 0;
	// AMNOTE: Sets world-space linear velocity for a nonstatic body. Ignores nonfinite values and wakes the body when the resulting speed exceeds the wake threshold.
	virtual void SetLinearVelocity( const Vector &velocity ) = 0;
	// AMNOTE: Adds world-space linear velocity for a nonstatic body, rejecting a nonfinite result and waking above the speed threshold.
	virtual void AddLinearVelocity( const Vector &velocity ) = 0;
	// AMNOTE: Returns world-space linear velocity.
	virtual Vector GetLinearVelocity() const = 0;
	// AMNOTE: Returns linear velocity plus angular velocity crossed with the point's offset from the world-space center of mass.
	virtual Vector GetVelocityAtPoint( const Vector &worldPosition ) const = 0;
	// AMNOTE: Sets a nonstatic body's world-space angular velocity in degrees per second, converting to radians internally. Ignores nonfinite values and wakes above the angular-speed threshold.
	virtual void SetAngularVelocity( const AngularImpulse &angularVelocity ) = 0;
	// AMNOTE: Adds world-space angular velocity in degrees per second, rejecting a nonfinite result and waking above the angular-speed threshold.
	virtual void AddAngularVelocity( const AngularImpulse &angularVelocity ) = 0;
	// AMNOTE: Returns world-space angular velocity in degrees per second.
	virtual AngularImpulse GetAngularVelocity() const = 0;
	// AMNOTE: Sets a nonstatic body's local angular velocity in degrees per second, rotating into world space and converting to radians. Rejects a nonfinite result and wakes above the angular-speed threshold.
	virtual void SetLocalAngularVelocity( const AngularImpulse &angularVelocity ) = 0;
	// AMNOTE: Adds local angular velocity in degrees per second after rotating into world space. Rejects a nonfinite result and wakes a nonstatic body above the angular-speed threshold.
	virtual void AddLocalAngularVelocity( const AngularImpulse &angularVelocity ) = 0;
	// AMNOTE: Returns angular velocity rotated into body-local space, in degrees per second, without applying body scale.
	virtual AngularImpulse GetLocalAngularVelocity() const = 0;
	// AMNOTE: Applies a finite linear impulse to a dynamic body and wakes it; the velocity change is forceVector times cached inverse mass.
	virtual void ApplyForceCenter( const Vector &forceVector ) = 0;
	// AMNOTE: Applies a finite impulse at a world-space point on a dynamic body, updating linear and angular velocity and waking it.
	virtual void ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition ) = 0;
	// AMNOTE: Applies an angular impulse in degree-based units using inverse inertia and wakes a dynamic body. Ignores a nonfinite converted impulse.
	virtual void ApplyTorqueCenter( const AngularImpulse &torque ) = 0;

	// AMNOTE: Updates a nonstatic body's velocities toward target when deltaTime is positive. Wakes an eligible body when either speed exceeds its threshold; the angular threshold is in radians per second.
	virtual void SetVelocitiesFromTransform( const CTransform &target, float deltaTime, float flLinearWakeThreshold, float flAngularWakeThreshold ) = 0;
	// AMNOTE: Calculates world-space linear/angular velocity changes without applying the impulse. Either output may be null; angular output is in degrees per second. Linear output is written first; aliasing an input changes the angular result.
	virtual void CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const = 0;
	// AMNOTE: Accumulates world-space acceleration through effective mass; zero inverse mass is ignored. Uses the force update path, where a nonzero nWakeBody permits waking.
	virtual void AddLinearAcceleration( const Vector &acceleration, uint8 nWakeBody ) = 0;
	// AMNOTE: Accumulates world-space force for integration on a dynamic body without a shadow controller. Does not reject nonfinite input; a nonzero nWakeBody permits waking.
	virtual void AddForceCenter( const Vector &forceVector, uint8 nWakeBody ) = 0;
	// AMNOTE: Accumulates world-space force and its moment about the world mass center for integration. Rejects a nonfinite moment independently of the linear force; a nonzero nWakeBody permits waking.
	virtual void AddForceOffset( const Vector &forceVector, const Vector &worldPosition, uint8 nWakeBody ) = 0;
	// AMNOTE: Accumulates world-space angular acceleration in degrees per second squared through effective inertia. Ignores nonfinite converted torque and bodies with a shadow controller; a nonzero nWakeBody permits waking.
	virtual void AddAngularAcceleration( const AngularImpulse &angularAcceleration, uint8 nWakeBody ) = 0;
	// AMNOTE: Accumulates finite world-space torque for integration on a dynamic body without a shadow controller. Uses radian-based torque units; a nonzero nWakeBody permits waking.
	virtual void AddTorque( const Vector &torque, uint8 nWakeBody ) = 0;
	// AMNOTE: Clears current accumulated linear force and pending linear velocity changes, and resets the pending velocity multiplier to one.
	virtual void ClearLinearForces() = 0;
	// AMNOTE: Clears current accumulated torque and pending angular velocity changes, and resets the pending angular velocity multiplier to one.
	virtual void ClearAngularForces() = 0;
	// AMNOTE: Enables or disables linear and angular drag.
	virtual void EnableDrag( bool enable ) = 0;
	// AMNOTE: Returns whether linear and angular drag are enabled.
	virtual bool IsDragEnabled() const = 0;
	// AMNOTE: Sets the linear drag coefficient without clamping and enables drag.
	virtual void SetLinearDragCoefficient( float flCoefficient ) = 0;
	// AMNOTE: Returns the linear drag coefficient.
	virtual float GetLinearDragCoefficient() const = 0;
	// AMNOTE: Sets the angular drag coefficient without clamping and enables drag.
	virtual void SetAngularDragCoefficient( float flCoefficient ) = 0;
	// AMNOTE: Returns the angular drag coefficient.
	virtual float GetAngularDragCoefficient() const = 0;
	// AMNOTE: Calculates the directional angular drag factor in object space without normalizing the input or checking whether drag is enabled.
	virtual float CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const = 0;
	// AMNOTE: Calculates the directional linear drag factor in object space without normalizing the input or checking whether drag is enabled.
	virtual float CalculateLinearDrag( const Vector &unitDirection ) const = 0;
	// AMNOTE: Sets linear damping for velocity integration; negative values and NaN are ignored.
	virtual void SetLinearDamping( float flDamping ) = 0;
	// AMNOTE: Returns linear damping for velocity integration.
	virtual float GetLinearDamping() const = 0;
	// AMNOTE: Sets angular damping for velocity integration; negative values and NaN are ignored.
	virtual void SetAngularDamping( float flDamping ) = 0;
	// AMNOTE: Returns angular damping for velocity integration.
	virtual float GetAngularDamping() const = 0;
	// AMNOTE: Sets the body's contact-friction multiplier without clamping or waking it.
	virtual void SetFrictionScale( float flScale ) = 0;
	// AMNOTE: Returns the body's contact-friction multiplier.
	virtual float GetFrictionScale() const = 0;
	// AMNOTE: Sets the body's linear fluid-drag scale without clamping.
	virtual void SetLinearFluidDragScale( float flScale ) = 0;
	// AMNOTE: Sets the body's angular fluid-drag scale without clamping.
	virtual void SetAngularFluidDragScale( float flScale ) = 0;
	// AMNOTE: Sets the body's buoyancy scale without clamping and wakes an eligible body when the value changes.
	virtual void SetBuoyancyScale( float flScale ) = 0;
	// AMNOTE: Returns the body's buoyancy scale.
	virtual float GetBuoyancyScale() const = 0;

private:
	// AMNOTE: Accumulates buoyancy and fluid drag using supplied controller data. The complete source signature and controller parameter type remain unresolved.
	virtual void ApplyFluidForces() = 0;

public:
	virtual bool HasJoints() const = 0;
	virtual bool HasRopeLinks() const = 0;
	// AMNOTE: Returns the first borrowed connected joint, or null.
	virtual IPhysicsJoint *GetFirstJoint() const = 0;
	// AMNOTE: Returns the next borrowed connected joint, or null. pJoint must be nonnull and attached to this body.
	virtual IPhysicsJoint *GetNextJoint( const IPhysicsJoint *pJoint ) const = 0;
	// AMNOTE: Returns a borrowed joint connecting this body to pBody, or null. pBody must be nonnull.
	virtual IPhysicsJoint *FindJoint( const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Returns whether any attached shape has a contact node; the node need not contain touching points.
	virtual bool HasContacts() const = 0;
	// AMNOTE: Finds the first touching contact with a manifold. Writes the world-space point and borrowed opposing body when their outputs are nonnull; failure leaves outputs unchanged.
	virtual bool GetContactPoint( Vector *contactPoint, IPhysicsBody **contactObject ) const = 0;
	// AMNOTE: Finds the same contact and optionally writes its world-space point; failure leaves the output unchanged.
	virtual bool GetContactPoint( Vector *contactPoint ) const = 0;
	// AMNOTE: Appends unique backing-body IDs from all attached contact nodes without clearing the sorted output. Nodes need not contain touching points; bodyIDs must not need sorting.
	virtual void GetContactingBodyIDs( CUtlSortVector<uint32> &bodyIDs ) const = 0;
	// AMNOTE: Creates a temporary, engine-owned friction iterator over this body's current touching contact points. Body and contact topology must remain valid while using it.
	virtual IPhysicsFrictionSnapshot *CreateFrictionSnapshot() = 0;
	// AMNOTE: Destroys the iterator through its native virtual destructor; null is ignored.
	virtual void DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) = 0;
	// AMNOTE: Returns the cached controller, creating one when needed with the current body position and orientation as its target. The body owns the returned pointer.
	virtual IPhysicsShadowController *CreateShadowController() = 0;
	// AMNOTE: Destroys the cached shadow controller and its backing state. Any pointer returned by GetShadowController becomes invalid.
	virtual void RemoveShadowController() = 0;
	// AMNOTE: Enables the shadow controller for a dynamic body when nEnabled is nonzero, creating or destroying it as needed. Keyframed bodies retain a controller regardless of this setting; static bodies remove it.
	virtual void EnableDynamicShadowController( uint8 nEnabled ) = 0;
	// AMNOTE: Returns the cached shadow controller, or null when none is attached. The body owns the returned pointer.
	virtual IPhysicsShadowController *GetShadowController() const = 0;

private:
	// AMNOTE: Forwards to an empty shadow-controller operation when attached; its parameters remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Updates an attached shadow target using a position, SIMD quaternion and time offset. An ignored intervening argument leaves the complete source signature unresolved.
	virtual void UpdateShadowTarget() = 0;
	// AMNOTE: Returns zero in a floating-point register; the complete result type and purpose remain unresolved.
	virtual void unk101() = 0;

public:
	// AMNOTE: Returns one and writes the shadow target when a controller is attached; otherwise returns zero without changing outputs. Either output may be null.
	virtual int GetShadowPosition( Vector *position, QAngle *angles ) const = 0;
	// AMNOTE: Returns the borrowed list of attached motion controllers. Its membership changes when controllers attach or detach.
	virtual const CUtlVector<IPhysicsMotionController *> &GetMotionControllers() const = 0;
	// AMNOTE: Enables automatic sleeping after sustained low motion. Disabling it wakes an eligible dynamic body; explicit Sleep remains available.
	virtual void EnableAutoSleep( bool bEnable ) = 0;
	// AMNOTE: Returns whether automatic sleeping is enabled.
	virtual bool IsAutoSleepEnabled() const = 0;
	// AMNOTE: Sleeps an enabled body, zeroing its velocities and updating its collision state.
	virtual void Sleep() = 0;
	// AMNOTE: Wakes an enabled nonstatic body unless its current state suppresses waking.
	virtual void Wake() = 0;
	// AMNOTE: Returns the body's sleeping flag.
	virtual bool IsAsleep() const = 0;
	// AMNOTE: Sets the factor multiplying the body's effective gravity and wakes an eligible body when it changes.
	virtual void SetGravityScale( float flScale ) = 0;
	// AMNOTE: Returns the body's gravity multiplier.
	virtual float GetGravityScale() const = 0;
	// AMNOTE: Replaces custom gravity when any component changes by more than 0.001. A nonzero nWakeBody requests waking an eligible body; zero custom gravity selects world gravity.
	virtual void SetGravity( const Vector &gravityVector, uint8 nWakeBody ) = 0;
	// AMNOTE: Returns custom gravity when nonzero, otherwise world gravity, multiplied by this body's gravity scale.
	virtual Vector GetGravity() const = 0;
	// AMNOTE: Returns the body's time scale.
	virtual float GetTimeScale() const = 0;
	// AMNOTE: Updates the body's time scale, effective mass properties and an eligible body's awake state. Values below 0.01 are compared as zero but stored unchanged when an update occurs.
	virtual void SetTimeScale( float flTimeScale ) = 0;
	// AMNOTE: Stores the signed mass priority used by the solver without waking the body.
	virtual void SetMassPriority( int8 nPriority ) = 0;
	// AMNOTE: Returns the body's signed solver mass priority.
	virtual int8 GetMassPriority() const = 0;

private:
	// AMNOTE: Stores a pointer-sized value on the body adapter; its type and meaning remain unresolved.
	virtual void unk201() = 0;
	// AMNOTE: Reads the preceding pointer-sized value.
	virtual void unk202() = 0;

public:
	// AMNOTE: Returns the body's stored 32-bit game flags.
	virtual uint32 GetGameFlags() const = 0;
	// AMNOTE: Stores the body's 32-bit game flags without changing other body state.
	virtual void SetGameFlags( uint32 userFlags ) = 0;
	// AMNOTE: Sets the filtering entity ID, propagates it into attached shapes and refreshes selected existing contacts.
	virtual void SetEntityId( uint32 nEntityId ) = 0;
	// AMNOTE: Returns the filtering entity ID, initially all ones.
	virtual uint32 GetEntityId() const = 0;
	// AMNOTE: Stores the body's 16-bit game index.
	virtual void SetGameIndex( uint16 gameIndex ) = 0;
	// AMNOTE: Returns the body's stored 16-bit game index.
	virtual uint16 GetGameIndex() const = 0;

private:
	// AMNOTE: Stores a pointer-sized value on the backing body; its type and meaning remain unresolved.
	virtual void unk301() = 0;
	// AMNOTE: Reads the preceding pointer-sized value.
	virtual void unk302() = 0;

public:
	// AMNOTE: Returns the body's borrowed debug name. This build returns an empty string.
	virtual const char *GetDebugName() const = 0;
	// AMNOTE: Does no work in this build; ignores the supplied debug name.
	virtual void SetDebugName( const char *pName ) = 0;

protected:
	virtual ~IPhysicsBody() = default;
};

// AMNOTE: Engine-owned adapter layout; remaining native override definitions are unrecovered. Use the inherited interface on engine-created objects.
class CPhysicsBody : public IPhysicsBody
{
public:
	// AMNOTE: Borrows the backing body; destroying this adapter invalidates the pointer.
	CRnBody *GetNativeBody() const { return m_Body; }
	IPhysicsShadowController *GetShadowController() const override { return m_ShadowController; }
	const char *GetDebugName() const override { return ""; }
	int GetShadowPosition( Vector *position, QAngle *angles ) const override;
	CTransform GetShadowTransform() const override;
	// AMNOTE: Nonfinite orientations can produce compiler-dependent NaN payloads.
	CTransform GetTransform() const override;
	VPhys2MassProperties_t GetMassProperties() const override;
	int FindShape( const IPhysicsShape *pShape ) const override;
	void AddLinearVelocity( const Vector &velocity ) override;
	void SetLocalAngularVelocity( const AngularImpulse &angularVelocity ) override;
	void AddLinearAcceleration( const Vector &acceleration, uint8 nWakeBody ) override;
	void AddForceOffset( const Vector &forceVector, const Vector &worldPosition, uint8 nWakeBody ) override;
	Vector WorldToLocalVector( const Vector &worldVector ) const override;
	Vector LocalToWorldVector( const Vector &localVector ) const override;
	Vector WorldToLocal( const Vector &worldPosition ) const override;
	Vector LocalToWorld( const Vector &localPosition ) const override;
	fltx4 WorldToLocalOrientation( const fltx4 &qWorldOrientation ) const override;
	fltx4 LocalToWorldOrientation( const fltx4 &qLocalOrientation ) const override;
	const CUtlVector<IPhysicsMotionController *> &GetMotionControllers() const override { return m_MotionControllers; }
	void SetDebugName( const char * ) override {}
	using IPhysicsBody::GetContactPoint;
	bool GetContactPoint( Vector *contactPoint ) const override { return GetContactPoint( contactPoint, nullptr ); }
	void DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) override { delete pSnapshot; }

private:
	friend class CPhysicsMotionController;

	CPhysicsBody() = delete;
	CPhysicsBody( const CPhysicsBody & ) = delete;
	CPhysicsBody &operator=( const CPhysicsBody & ) = delete;
	~CPhysicsBody() override = default;

	// AMNOTE: Caller storage; its native member type remains unresolved.
	class unkCallerStorage_t
	{
		uint8 m_unk001[8];
	};

	CRnBody *m_Body;
	unkCallerStorage_t m_unk001;
	CPhysicsShadowController *m_ShadowController;
	CUtlVector<IPhysicsMotionController *> m_MotionControllers;
};

inline VPhys2MassProperties_t CPhysicsBody::GetMassProperties() const
{
	VPhys2MassProperties_t properties;
	properties.m_InertiaTensor = GetInertiaTensorLocalSpace();
	properties.m_vMassCenter = GetMassCenterLocalSpace();
	properties.m_flMass = GetMass();
	return properties;
}

inline int CPhysicsBody::FindShape( const IPhysicsShape *pShape ) const
{
	const int nShapeCount = GetShapeCount();
	for ( int i = 0; i < nShapeCount; ++i )
	{
		if ( GetShape( i ) == pShape )
			return i;
	}
	return -1;
}

inline void CPhysicsBody::AddLinearVelocity( const Vector &velocity )
{
	const Vector currentVelocity = GetLinearVelocity();
	Vector result;
#if defined( COMPILER_GCC )
	result.x = velocity.x + currentVelocity.x;
	if ( !IsFinite( result.x ) )
		return;
	result.y = velocity.y + currentVelocity.y;
	if ( !IsFinite( result.y ) )
		return;
	result.z = velocity.z + currentVelocity.z;
#else
	result.x = currentVelocity.x + velocity.x;
	result.y = currentVelocity.y + velocity.y;
	result.z = currentVelocity.z + velocity.z;
#endif
	SetLinearVelocity( result );
}

inline void CPhysicsBody::SetLocalAngularVelocity( const AngularImpulse &angularVelocity )
{
	const fltx4 qOrientation = GetOrientation();
	const float qX = SubFloat( qOrientation, 0 );
	const float qY = SubFloat( qOrientation, 1 );
	const float qZ = SubFloat( qOrientation, 2 );
	const float qW = SubFloat( qOrientation, 3 );
	const float intermediateY = ( angularVelocity.x * qZ - angularVelocity.z * qX ) + angularVelocity.y * qW;
	const float intermediateZ = ( angularVelocity.y * qX - angularVelocity.x * qY ) + angularVelocity.z * qW;
	const float deltaX = qY * intermediateZ - qZ * intermediateY;
	AngularImpulse worldVelocity;
	worldVelocity.x = ( deltaX + deltaX ) + angularVelocity.x;
#if defined( COMPILER_GCC )
	if ( !IsFinite( worldVelocity.x * 0.017453292f ) )
		return;
#endif
	const float intermediateX = ( angularVelocity.z * qY - angularVelocity.y * qZ ) + angularVelocity.x * qW;
	const float deltaY = qZ * intermediateX - intermediateZ * qX;
	worldVelocity.y = ( deltaY + deltaY ) + angularVelocity.y;
#if defined( COMPILER_GCC )
	if ( !IsFinite( worldVelocity.y * 0.017453292f ) )
		return;
#endif
	const float deltaZ = qX * intermediateY - qY * intermediateX;
	worldVelocity.z = ( deltaZ + deltaZ ) + angularVelocity.z;
	SetAngularVelocity( worldVelocity );
}

inline void CPhysicsBody::AddLinearAcceleration( const Vector &acceleration, uint8 nWakeBody )
{
	const float flInvMass = GetInvMass();
	if ( flInvMass == 0.0f )
		return;

	const float flMass = 1.0f / flInvMass;
	Vector force;
	force.x = flMass * acceleration.x;
	force.y = flMass * acceleration.y;
	force.z = flMass * acceleration.z;
	AddForceCenter( force, nWakeBody );
}

inline void CPhysicsBody::AddForceOffset( const Vector &forceVector, const Vector &worldPosition, uint8 nWakeBody )
{
#if defined( COMPILER_GCC )
	const Vector center = GetMassCenterWorldSpace();
	const float flPositionX = worldPosition.x;
	const float flOffsetY = worldPosition.y - center.y;
	const float flOffsetZ = worldPosition.z - center.z;
	AddForceCenter( forceVector, nWakeBody );
	Vector torque;
	// AMNOTE: Stop before later components when a moment is nonfinite.
	torque.x = flOffsetY * forceVector.z - flOffsetZ * forceVector.y;
	if ( !IsFinite( torque.x ) )
		return;
	const float flOffsetX = flPositionX - center.x;
	torque.y = flOffsetZ * forceVector.x - forceVector.z * flOffsetX;
	if ( !IsFinite( torque.y ) )
		return;
	torque.z = flOffsetX * forceVector.y - flOffsetY * forceVector.x;
#else
	const Vector center = GetMassCenterWorldSpace();
	const float flOffsetX = worldPosition.x - center.x;
	const float flOffsetY = worldPosition.y - center.y;
	const float flOffsetZ = worldPosition.z - center.z;
	AddForceCenter( forceVector, nWakeBody );
	Vector torque;
	torque.x = flOffsetY * forceVector.z - flOffsetZ * forceVector.y;
	torque.y = flOffsetZ * forceVector.x - flOffsetX * forceVector.z;
	torque.z = flOffsetX * forceVector.y - flOffsetY * forceVector.x;
#endif
	AddTorque( torque, nWakeBody );
}

inline void CPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, fltx4 *pOrientationOut )
{
	if ( pPositionOut )
		*pPositionOut = m_Implementation->GetTargetPosition();
	if ( pOrientationOut )
		*pOrientationOut = m_Implementation->GetTargetOrientation();
}

inline void CPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	float sp, cp, sy, cy, sr, cr;
	const float flHalfRadians = 0.0087266462f;
	V_sincosf( angles.x * flHalfRadians, &sp, &cp );
	V_sincosf( angles.y * flHalfRadians, &sy, &cy );
	V_sincosf( angles.z * flHalfRadians, &sr, &cr );
	const float srCp = sr * cp;
	const float crSp = cr * sp;
	const float srSp = sr * sp;
	const float crCp = cr * cp;
	const fltx4 qOrientation = _mm_setr_ps( cy * srCp - sy * crSp,
		cy * crSp + sy * srCp, sy * crCp - cy * srSp, cy * crCp + sy * srSp );
	Update( position, qOrientation, timeOffset );
}

inline int CPhysicsBody::GetShadowPosition( Vector *position, QAngle *angles ) const
{
	if ( !m_ShadowController )
		return 0;

	m_ShadowController->GetTargetPosition( position, angles );
	return 1;
}

inline void CPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = m_Implementation->GetTargetPosition();
	if ( !pAnglesOut )
		return;

	const fltx4 qOrientation = m_Implementation->GetTargetOrientation();
	const fltx4 doubled = AddSIMD( qOrientation, qOrientation );
	const fltx4 squared = MulSIMD( qOrientation, doubled );
	// AMNOTE: Operand order affects NaN payload propagation.
	const fltx4 diagonal = SubSIMD( SubSIMD( ReplicateX4( 1.0f ), _mm_shuffle_ps( squared, squared, MM_SHUFFLE_REV( 1, 0, 0, 3 ) ) ), _mm_shuffle_ps( squared, squared, MM_SHUFFLE_REV( 2, 2, 1, 3 ) ) );
	const fltx4 products = MulSIMD( _mm_shuffle_ps( doubled, doubled, MM_SHUFFLE_REV( 1, 2, 2, 3 ) ), _mm_shuffle_ps( qOrientation, qOrientation, MM_SHUFFLE_REV( 0, 1, 0, 3 ) ) );
	const fltx4 wProducts = MulSIMD( _mm_shuffle_ps( doubled, doubled, MM_SHUFFLE_REV( 2, 0, 1, 3 ) ), SplatWSIMD( qOrientation ) );
	const fltx4 sums = AddSIMD( products, wProducts );
	const fltx4 differences = SubSIMD( products, wProducts );
	const float forwardX = SubFloat( diagonal, 0 );
	const float forwardY = SubFloat( sums, 0 );
	const float forwardZ = SubFloat( differences, 2 );
	const float xyDist = FastSqrt( forwardX * forwardX + forwardY * forwardY );

	QAngle angles;
	if ( xyDist > 0.001f )
	{
		angles.y = RAD2DEG( V_atan2f( forwardY, forwardX ) );
		angles.x = RAD2DEG( V_atan2f( -forwardZ, xyDist ) );
		angles.z = RAD2DEG( V_atan2f( SubFloat( sums, 1 ), SubFloat( diagonal, 2 ) ) );
	}
	else
	{
		angles.y = RAD2DEG( V_atan2f( -SubFloat( differences, 0 ), SubFloat( diagonal, 1 ) ) );
		angles.x = RAD2DEG( V_atan2f( -forwardZ, xyDist ) );
		angles.z = 0.0f;
	}
	*pAnglesOut = angles;
}

inline CTransform CPhysicsBody::GetTransform() const
{
	const CTransform scaledTransform = GetScaledTransform();
	const fltx4 qOrientation = LoadAlignedSIMD( scaledTransform.m_orientation.Base() );
	const float x = SubFloat( qOrientation, 0 );
	const float y = SubFloat( qOrientation, 1 );
	const float z = SubFloat( qOrientation, 2 );
	const float w = SubFloat( qOrientation, 3 );
	// AMNOTE: Keep the scalar squares from evaluating the unused quaternion w lane.
	const volatile float xx = x * x;
	const volatile float yy = y * y;
	const volatile float zz = z * z;
#if defined( COMPILER_GCC )
	const float xy = x * y;
	const float xz = x * z;
	const float yz = y * z;
	const float wx = x * w;
	const float wy = y * w;
	const float wz = z * w;
	const float sumYZ = yy + zz;
	const float sumZX = zz + xx;
	const float sumXY = xx + yy;
	const float sumXZ_WY = xz + wy;
	const float sumXY_WZ = xy + wz;
	const float sumYZ_WX = wx + yz;
#else
	const float xy = y * x;
	const float xz = z * x;
	const float yz = z * y;
	const float wx = w * x;
	const float wy = w * y;
	const float wz = w * z;
	const float sumYZ = zz + yy;
	const float sumZX = zz + xx;
	const float sumXY = yy + xx;
	const float sumXZ_WY = wy + xz;
	const float sumXY_WZ = wz + xy;
	const float sumYZ_WX = yz + wx;
#endif
	const float m00 = 1.0f - ( sumYZ + sumYZ );
	const float m11 = 1.0f - ( sumZX + sumZX );
	const float m22 = 1.0f - ( sumXY + sumXY );
	const float m02 = sumXZ_WY + sumXZ_WY;
	const float m10 = sumXY_WZ + sumXY_WZ;
	const float m21 = sumYZ_WX + sumYZ_WX;
	const float diffXZ_WY = xz - wy;
	const float diffXY_WZ = xy - wz;
	const float diffYZ_WX = yz - wx;
	const float m20 = diffXZ_WY + diffXZ_WY;
	const float m01 = diffXY_WZ + diffXY_WZ;
	const float m12 = diffYZ_WX + diffYZ_WX;
	const float trace = ( m00 + m11 ) + m22;
	fltx4 basis;
	if ( _mm_comigt_ss( _mm_setzero_ps(), _mm_set_ss( trace ) ) != 0 )
	{
		if ( m00 > m11 && m00 > m22 )
			basis = _mm_setr_ps( ( ( m00 - m11 ) - m22 ) + 1.0f, m01 + m10, m02 + m20, m21 - m12 );
		else if ( m11 > m22 )
			basis = _mm_setr_ps( m01 + m10, ( ( m11 - m22 ) - m00 ) + 1.0f, m12 + m21, m02 - m20 );
		else
			basis = _mm_setr_ps( m02 + m20, m12 + m21, ( ( m22 - m00 ) - m11 ) + 1.0f, m10 - m01 );
	}
	else
	{
		basis = _mm_setr_ps( m21 - m12, m02 - m20, m10 - m01, trace + 1.0f );
	}
	const fltx4 squared = MulSIMD( basis, basis );
	const fltx4 pairs = AddSIMD( squared, _mm_shuffle_ps( squared, squared, MM_SHUFFLE_REV( 1, 0, 3, 2 ) ) );
	const fltx4 length = SqrtSIMD( AddSIMD( pairs, RotateLeft2( pairs ) ) );
	const fltx4 identity = _mm_setr_ps( 0.0f, 0.0f, 0.0f, 1.0f );
	const fltx4 normalized = MaskedAssign( _mm_cmpeq_ps( _mm_setzero_ps(), length ), identity, DivSIMD( basis, length ) );
	const Vector &position = scaledTransform.m_vPosition;
	CTransform transform;
	transform.m_vPosition.x = position.x;
	transform.m_vPosition.y = position.y;
	transform.m_vPosition.z = position.z;
	transform.m_vPosition.w = 1.0f;
	StoreAlignedSIMD( transform.m_orientation.Base(), normalized );
	return transform;
}

inline CTransform CPhysicsBody::GetShadowTransform() const
{
	IPhysicsShadowController *pShadow = m_ShadowController;
	if ( pShadow && IsEnabled() && GetMotionType() != BODY_STATIC )
	{
		CTransform transform;
		Vector position;
		fltx4 qOrientation;
		pShadow->GetTargetPosition( &position, &qOrientation );
		transform.m_vPosition = position;
		transform.m_vPosition.w = 1.0f;
		StoreAlignedSIMD( transform.m_orientation.Base(), qOrientation );
		return transform;
	}

	return GetTransform();
}

inline Vector CPhysicsBody::LocalToWorldVector( const Vector &localVector ) const
{
	const fltx4 qOrientation = GetOrientation();
	const fltx4 vector = _mm_set_ps( 0.0f, localVector.z, localVector.y, localVector.x );
	const fltx4 qXYZ = _mm_movelh_ps( qOrientation, _mm_unpackhi_ps( qOrientation, _mm_setzero_ps() ) );
	const fltx4 qW = SplatWSIMD( qOrientation );
	const fltx4 qWXYZ = _mm_movelh_ps( qW, _mm_unpackhi_ps( qW, _mm_setzero_ps() ) );
	const fltx4 qYZX = _mm_shuffle_ps( qXYZ, qXYZ, MM_SHUFFLE_REV( 1, 2, 0, 3 ) );
	const fltx4 qZXY = _mm_shuffle_ps( qXYZ, qXYZ, MM_SHUFFLE_REV( 2, 0, 1, 3 ) );
	const fltx4 vectorYZX = _mm_shuffle_ps( vector, vector, MM_SHUFFLE_REV( 1, 2, 0, 3 ) );
	const fltx4 vectorZXY = _mm_shuffle_ps( vector, vector, MM_SHUFFLE_REV( 2, 0, 1, 3 ) );
	// AMNOTE: Preserve this operation grouping; compilers may propagate different NaN payloads.
	const fltx4 intermediate = AddSIMD( SubSIMD( MulSIMD( vectorZXY, qYZX ), MulSIMD( vectorYZX, qZXY ) ), MulSIMD( vector, qWXYZ ) );
	const fltx4 intermediateYZX = _mm_shuffle_ps( intermediate, intermediate, MM_SHUFFLE_REV( 1, 2, 0, 3 ) );
	const fltx4 intermediateZXY = _mm_shuffle_ps( intermediate, intermediate, MM_SHUFFLE_REV( 2, 0, 1, 3 ) );
	const fltx4 delta = SubSIMD( MulSIMD( intermediateZXY, qYZX ), MulSIMD( intermediateYZX, qZXY ) );
	Vector result;
	StoreUnaligned3SIMD( result.Base(), AddSIMD( AddSIMD( delta, delta ), vector ) );
	return result;
}

inline Vector CPhysicsBody::WorldToLocal( const Vector &worldPosition ) const
{
	const Vector origin = GetPosition();
	const fltx4 qOrientation = GetOrientation();
	const float qX = SubFloat( qOrientation, 0 );
	const float qY = SubFloat( qOrientation, 1 );
	const float qZ = SubFloat( qOrientation, 2 );
	const float qW = SubFloat( qOrientation, 3 );
	Vector relativePosition;
	relativePosition.x = worldPosition.x - origin.x;
	relativePosition.y = worldPosition.y - origin.y;
	relativePosition.z = worldPosition.z - origin.z;
	// AMNOTE: Preserve this operation grouping; compilers may propagate different NaN payloads.
	const float intermediateX = relativePosition.x * qW - ( relativePosition.z * qY - relativePosition.y * qZ );
	const float intermediateY = relativePosition.y * qW - ( relativePosition.x * qZ - relativePosition.z * qX );
	const float intermediateZ = relativePosition.z * qW - ( relativePosition.y * qX - relativePosition.x * qY );
	const float deltaX = qY * intermediateZ - qZ * intermediateY;
	const float deltaY = qZ * intermediateX - qX * intermediateZ;
	const float deltaZ = qX * intermediateY - intermediateX * qY;
	Vector result;
	result.x = relativePosition.x - ( deltaX + deltaX );
	result.y = relativePosition.y - ( deltaY + deltaY );
	result.z = relativePosition.z - ( deltaZ + deltaZ );
	return result;
}

inline fltx4 CPhysicsBody::WorldToLocalOrientation( const fltx4 &qWorldOrientation ) const
{
	const int32 signBit = static_cast<int32>( 0x80000000u );
	const fltx4 conjugateMask = _mm_castsi128_ps( _mm_setr_epi32( signBit, signBit, signBit, 0 ) );
	const fltx4 xMask = _mm_castsi128_ps( _mm_setr_epi32( 0, signBit, 0, signBit ) );
	const fltx4 yMask = _mm_castsi128_ps( _mm_setr_epi32( 0, 0, signBit, signBit ) );
	const fltx4 zMask = _mm_castsi128_ps( _mm_setr_epi32( signBit, signBit, 0, 0 ) );
	const fltx4 qOrientation = XorSIMD( GetOrientation(), conjugateMask );
	const fltx4 input = qWorldOrientation;
	const fltx4 xInput = XorSIMD( _mm_shuffle_ps( input, input, 0x1b ), xMask );
	const fltx4 yInput = XorSIMD( _mm_shuffle_ps( input, input, 0x4e ), yMask );
	const fltx4 zInput = XorSIMD( _mm_shuffle_ps( xInput, xInput, 0x4e ), zMask );
	const fltx4 wTerm = MulSIMD( SplatWSIMD( qOrientation ), input );
	const fltx4 xTerm = MulSIMD( SplatXSIMD( qOrientation ), xInput );
	const fltx4 yTerm = MulSIMD( SplatYSIMD( qOrientation ), yInput );
	const fltx4 zTerm = MulSIMD( SplatZSIMD( qOrientation ), zInput );
	// AMNOTE: Preserve this operation grouping; compilers may propagate different NaN payloads.
	return AddSIMD( AddSIMD( AddSIMD( wTerm, xTerm ), yTerm ), zTerm );
}


inline Vector CPhysicsBody::WorldToLocalVector( const Vector &worldVector ) const
{
	const fltx4 qOrientation = GetOrientation();
	const float qX = SubFloat( qOrientation, 0 );
	const float qY = SubFloat( qOrientation, 1 );
	const float qZ = SubFloat( qOrientation, 2 );
	const float qW = SubFloat( qOrientation, 3 );
	// AMNOTE: Preserve this operation grouping; compilers may propagate different NaN payloads.
	const float intermediateX = worldVector.x * qW - ( worldVector.z * qY - worldVector.y * qZ );
	const float intermediateY = worldVector.y * qW - ( worldVector.x * qZ - worldVector.z * qX );
	const float intermediateZ = worldVector.z * qW - ( worldVector.y * qX - worldVector.x * qY );
	const float deltaX = qY * intermediateZ - qZ * intermediateY;
	const float deltaY = qZ * intermediateX - qX * intermediateZ;
	const float deltaZ = qX * intermediateY - intermediateX * qY;
	Vector result;
	result.x = worldVector.x - ( deltaX + deltaX );
	result.y = worldVector.y - ( deltaY + deltaY );
	result.z = worldVector.z - ( deltaZ + deltaZ );
	return result;
}

inline Vector CPhysicsBody::LocalToWorld( const Vector &localPosition ) const
{
	const Vector origin = GetPosition();
	const Vector rotatedPosition = LocalToWorldVector( localPosition );
	Vector result;
	result.x = rotatedPosition.x + origin.x;
	result.y = rotatedPosition.y + origin.y;
	result.z = rotatedPosition.z + origin.z;
	return result;
}

inline fltx4 CPhysicsBody::LocalToWorldOrientation( const fltx4 &qLocalOrientation ) const
{
	const int32 signBit = static_cast<int32>( 0x80000000u );
	const fltx4 xMask = _mm_castsi128_ps( _mm_setr_epi32( 0, signBit, 0, signBit ) );
	const fltx4 yMask = _mm_castsi128_ps( _mm_setr_epi32( 0, 0, signBit, signBit ) );
	const fltx4 zMask = _mm_castsi128_ps( _mm_setr_epi32( signBit, signBit, 0, 0 ) );
	const fltx4 qOrientation = GetOrientation();
	const fltx4 input = qLocalOrientation;
	const fltx4 xInput = XorSIMD( _mm_shuffle_ps( input, input, 0x1b ), xMask );
	const fltx4 yInput = XorSIMD( _mm_shuffle_ps( input, input, 0x4e ), yMask );
	const fltx4 zInput = XorSIMD( _mm_shuffle_ps( xInput, xInput, 0x4e ), zMask );
	const fltx4 wTerm = MulSIMD( SplatWSIMD( qOrientation ), input );
	const fltx4 xTerm = MulSIMD( SplatXSIMD( qOrientation ), xInput );
	const fltx4 yTerm = MulSIMD( SplatYSIMD( qOrientation ), yInput );
	const fltx4 zTerm = MulSIMD( SplatZSIMD( qOrientation ), zInput );
	// AMNOTE: Preserve this operation grouping; compilers may propagate different NaN payloads.
	return AddSIMD( AddSIMD( AddSIMD( wTerm, xTerm ), yTerm ), zTerm );
}

inline uint32 CRegionSVM::ClassifyPoint( const Vector &point ) const
{
	const fltx4 position = _mm_set_ps( -1.0f, point.z, point.y, point.x );
	const uint32 *pNodes = m_Nodes.Base();
	int nNode = 0;
	uint32 node = pNodes[nNode];
	while ( node & 0x80000000u )
	{
		const RnPlane_t &plane = m_Planes[( node >> 16 ) & 0x7FFFu];
		const fltx4 products = MulSIMD( LoadUnalignedSIMD( &plane ), position );
		const fltx4 pairs = AddSIMD( products, _mm_shuffle_ps( products, products, MM_SHUFFLE_REV( 1, 0, 3, 2 ) ) );
		const float distance = SubFloat( AddSIMD( pairs, RotateLeft2( pairs ) ), 0 );
		nNode += int( node & 0xFFFFu ) + 1 + ( distance < 0.0f );
		node = pNodes[nNode];
	}
	return node;
}

#endif // IPHYSICSBODY_H
