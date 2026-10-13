#ifndef IPHYSICSSHAPE_H
#define IPHYSICSSHAPE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "mathlib/vector.h"
#include "mathlib/mathlib.h"
#include "mathlib/camera.h"
#include "mathlib/transform.h"
#include "tier0/utlvector.h"
#include "mathlib/ray.h"
#include "gametrace.h"
#include "color.h"
#include "rubikon/param_types.h"

class IPhysicsBody;
class ISceneViewDebugOverlays;
class CRnContact;
class CRnShape;
struct RnHull_t;
struct VPhys2ContactSnapshot_t;

// AMNOTE: Geometry types returned by GetGeometryType.
#define VPHYS2_GEOMETRY_SPHERE 0u
#define VPHYS2_GEOMETRY_CAPSULE 1u
#define VPHYS2_GEOMETRY_HULL 2u
#define VPHYS2_GEOMETRY_MESH 3u
#define VPHYS2_GEOMETRY_COMPOUND 4u

// AMNOTE: Known geometry debug-draw flags; other flags remain unresolved.
#define VPHYS2_DEBUG_DRAW_FACE_LABELS 0x800ull
#define VPHYS2_DEBUG_DRAW_VERTEX_LABELS 0x1000ull
#define VPHYS2_DEBUG_DRAW_HULL_EDGE_LABELS 0x2000ull
#define VPHYS2_DEBUG_DRAW_SPHERE_CAPSULE_OUTLINES 0x400000000ull

// AMNOTE: Contact endpoint entity identifier; these are not world body-list indices.
class VPhys2ContactEntityId_t
{
public:
	constexpr VPhys2ContactEntityId_t() = default;
	explicit constexpr VPhys2ContactEntityId_t( uint32 nEntityId ) : m_nEntityId( nEntityId ) {}
	constexpr uint32 GetEntityId() const { return m_nEntityId; }
	constexpr bool IsValid() const { return m_nEntityId != ~uint32( 0 ); }
	constexpr bool operator==( const VPhys2ContactEntityId_t &other ) const { return m_nEntityId == other.m_nEntityId; }
	constexpr bool operator!=( const VPhys2ContactEntityId_t &other ) const { return m_nEntityId != other.m_nEntityId; }

private:
	uint32 m_nEntityId = ~uint32( 0 );
};

// AMNOTE: Deprecated, use VPhys2ContactEntityId_t.
using unkVPhys2ContactEntityId_t = VPhys2ContactEntityId_t;

// AMNOTE: The native value type and meaning remain unresolved; its storage is opaque.
class unkVPhys2ShapeMaterialValue_t
{
public:
	constexpr unkVPhys2ShapeMaterialValue_t() = default;

private:
	uint32 m_unk001 = 0;
};

// AMNOTE: Surface property name hash.
class VPhys2SurfacePropertyNameHash_t
{
public:
	constexpr VPhys2SurfacePropertyNameHash_t() = default;
	explicit constexpr VPhys2SurfacePropertyNameHash_t( uint32 nHash ) : m_nHash( nHash ) {}
	constexpr uint32 GetHash() const { return m_nHash; }

private:
	uint32 m_nHash = 0;
};

// AMNOTE: Deprecated, use VPhys2SurfacePropertyNameHash_t.
using unkVPhys2SurfacePropertyNameHash_t = VPhys2SurfacePropertyNameHash_t;

// AMNOTE: Shape material binding.
class VPhys2ShapeMaterialBinding
{
public:
	// AMNOTE: Surface-property density is converted to cubic-inch volume units.
	float m_flDensity = 0.015625f;
	float m_flFriction = 0.2f;
	float m_flElasticity = 0.0f;
	float m_flThickness = 0.0f;
	float m_flSoftContactFrequency = 0.0f;
	float m_flSoftContactDampingRatio = 0.0f;
	float m_flWheelDrag = 0.0f;
	float m_flWheelFrictionScale = 1.0f;
	unkVPhys2ShapeMaterialValue_t m_unk001;
	uint32 m_nSurfacePropertyNameHash = 0;
	// AMNOTE: Borrowed surface properties; the source must outlive this binding.
	const CPhysSurfaceProperties *m_pSurfaceProperties = nullptr;

	void Reset() { *this = VPhys2ShapeMaterialBinding(); }

	// AMNOTE: The surface hash must be initialized; the borrowed surface must outlive this material binding.
	void SetSurfaceProperties( const CPhysSurfaceProperties &properties );
};

// AMNOTE: Deprecated, use VPhys2ShapeMaterialBinding.
using unkVPhys2ShapeMaterialBinding = VPhys2ShapeMaterialBinding;

struct RnShapeConvexId_t
{
	// AMNOTE: Compound convex selection; all-one bits select the shape's default material.
	uint32 m_nConvexIndex = ~uint32( 0 );
	// AMNOTE: Triangle selection within a mesh convex; all-one bits mean no triangle selection.
	uint32 m_nTriangleIndex = ~uint32( 0 );
};

// AMNOTE: Deprecated, use RnShapeConvexId_t.
using unkVPhys2ShapeCastId_t = RnShapeConvexId_t;

// AMNOTE: Use a fresh or reset result for each cast. Both successful and failed casts can leave some fields unchanged.
struct VPhys2ShapeCastResult_t
{
	float m_flFraction = 1.0f;
	// AMNOTE: Some hull cast paths leave this at its reset value.
	Vector m_vPoint = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_vNormal = Vector( 0.0f, 0.0f, 0.0f );
	RnShapeConvexId_t m_GeometryId;
	// AMNOTE: Borrowed engine material binding; changes to the shape or its materials can invalidate it.
	const VPhys2ShapeMaterialBinding *m_pMaterial = nullptr;
	// AMNOTE: The cast starts overlapping collision geometry, including the cast's collision tolerance.
	bool m_bStartInSolid = false;

	void Reset() { *this = VPhys2ShapeCastResult_t(); }
};

// AMNOTE: Deprecated, use RnInertiaProperties_t.
using VPhys2ShapeMassProperties_t = RnInertiaProperties_t;

struct VPhys2SupportPoint_t
{
	Vector m_vPoint;
	// AMNOTE: Directional support projection for a unit direction in shape space.
	float m_flProjection;
};

// AMNOTE: Native solver parameter identities and meanings remain unresolved; storage is opaque.
class unkVPhys2ConvexSupportParameters_t
{
public:
	constexpr unkVPhys2ConvexSupportParameters_t() = default;

private:
	uint32 m_unk001[2] = { 0x3d800000, 0 };
};

// AMNOTE: Convex support proxy used by time-of-impact sweeps.
class CTOIProxy
{
public:
	int m_nVertexCount = 0;
	// AMNOTE: Borrowed geometry; shape removal or geometry changes can invalidate it.
	const Vector *m_pVertices = nullptr;
	float m_flScale = 1.0f;
	// AMNOTE: When true, use the cached corners instead of the vertex array.
	bool m_bUseBoundsCorners = false;
	// AMNOTE: Scaled X/Y/Z coordinate pairs, maximum followed by minimum.
	float m_CornerCoordinates[3][2] = {};
	// AMNOTE: Borrowed hull backing; do not free it.
	const RnHull_t *m_pHull = nullptr;
	unkVPhys2ConvexSupportParameters_t m_unk001;

	void Reset() { *this = CTOIProxy(); }
};

// AMNOTE: Deprecated, use CTOIProxy.
using VPhys2ConvexSupportProxy_t = CTOIProxy;
// AMNOTE: Deprecated, use CTOIProxy.
using unkVPhys2ConvexSupportProxy_t = CTOIProxy;

struct VPhys2ShapeContactStatistics_t
{
	int m_nContacts;
	// AMNOTE: Contacts with accepted points whose opposite body is static.
	int m_nAcceptedStaticContacts;
	int m_nPoints;
	// AMNOTE: Points accepted by the native transformed separation test.
	int m_nAcceptedPoints;
	int m_nManifolds;
};

struct ALIGN16 VPhys2SubmergedVolume_t
{
	// AMNOTE: The submerged centroid multiplied by the submerged volume; its coordinate space varies by geometry kind.
	Vector m_vVolumeMoment;
	float m_flSubmergedVolume;
	// AMNOTE: Can be zero for unsupported geometry or an unsubmerged capsule.
	float m_flTotalVolume;
} ALIGN16_POST;

// AMNOTE: Engine-owned; obtain through IPhysicsBody and destroy through its shape removal methods. Do not implement or delete it.
// AMNOTE: Unresolved virtual signatures and incomplete result types remain private.
abstract_class IPhysicsShape
{

public:
	// AMNOTE: Returns whether the shape participates in collision detection.
	virtual bool IsCollisionEnabled() const = 0;
	// AMNOTE: Enables or disables collision participation. nEnable must be zero or one; updates broadphase registration when the owning body is enabled.
	virtual void EnableCollisions( uint8 nEnable ) = 0;
	// AMNOTE: Returns the owning body adapter, or null when no body is attached.
	virtual IPhysicsBody *GetBody() const = 0;
	// AMNOTE: Returns a VPHYS2_GEOMETRY_* value. The native enum's identity remains unresolved.
	virtual uint32 GetGeometryType() const = 0;
	// AMNOTE: Sphere/capsule output includes shape scale. Hull bounds and borrowed vertices are unscaled. Mesh/compound output is point geometry.
	virtual void GetGeometry( Ray_t &geometry ) const = 0;
	// AMNOTE: Copies the material. Nonzero nUpdateMass requires an owning body and recomputes its mass when density changes.
	virtual void SetMaterial( const VPhys2ShapeMaterialBinding &material, uint8 nUpdateMass ) = 0;
	// AMNOTE: Returns borrowed material storage; shape removal or material changes can invalidate it.
	virtual const VPhys2ShapeMaterialBinding &GetMaterial() const = 0;
	// AMNOTE: Assigns material from a surface name hash, possibly using the default surface. Success does not imply an exact hash match; body mass is not recalculated.
	virtual bool SetMaterialFromSurfaceNameHash( uint32 nSurfacePropertyNameHash ) = 0;
	// AMNOTE: Returns the current material's surface property name hash.
	virtual VPhys2SurfacePropertyNameHash_t GetSurfacePropertyNameHash() const = 0;
	// AMNOTE: Preserves entity/detail-layer controls and applies stored overrides. A registered shape or nonzero nUpdateBroadphase requires an owning body/world.
	virtual void SetCollisionAttributes( const RnCollisionAttr_t &attributes, uint8 nUpdateBroadphase ) = 0;
	// AMNOTE: Returns borrowed collision attributes; shape removal or attribute changes can invalidate them.
	virtual const RnCollisionAttr_t &GetCollisionAttributes() const = 0;
	// AMNOTE: Returns the collision attributes' group byte.
	virtual uint8 GetCollisionGroup() const = 0;
	// AMNOTE: Stores interaction/group overrides and detail-layer controls; does not apply the remaining collision attributes or update world registration.
	virtual void SetCollisionAttributeOverrides( const RnCollisionAttr_t &attributes ) = 0;
	// AMNOTE: Casts a line segment in shape-local coordinates.
	virtual bool CastLine( VPhys2ShapeCastResult_t &result, const Vector &vStart, const Vector &vDisplacement, float flMaxFraction ) const = 0;
	// AMNOTE: Casts a sphere in shape-local coordinates; a sufficiently small radius selects the line cast.
	virtual bool CastSphere( VPhys2ShapeCastResult_t &result, const Vector &vStart, const Vector &vDisplacement, float flRadius, float flMaxFraction ) const = 0;
	// AMNOTE: Casts an axis-aligned box against the transformed shape.
	virtual bool CastBox( VPhys2ShapeCastResult_t &result, const matrix3x4_t &shapeToQuery, const Vector &vStart, const Vector &vDisplacement, const Vector &vHalfExtents, float flMaxFraction ) const = 0;
	// AMNOTE: Casts the supplied ray geometry against the transformed shape.
	virtual bool CastShape( VPhys2ShapeCastResult_t &result, const matrix3x4_t &shapeToQuery, const Vector &vStart, const Vector &vDisplacement, const Ray_t &ray, float flMaxFraction ) const = 0;
	// AMNOTE: Tests overlap with a sphere in shape-local coordinates.
	virtual bool Overlap( const Vector &vCenter, float flRadius ) const = 0;
	// AMNOTE: Transforms this shape into the bounds' coordinate space and tests overlap.
	virtual bool Overlap( const matrix3x4_t &shapeToQuery, const AABB_t &bounds ) const = 0;
	// AMNOTE: Tests overlap using the transform's position and orientation. Its scale is ignored.
	virtual bool Overlap( const CTransform &shapeToQuery, const AABB_t &bounds ) const = 0;
	// AMNOTE: Tests overlap with ray geometry using the transform's position and orientation. Its scale is ignored.
	virtual bool Overlap( const CTransform &shapeToQuery, const Ray_t &ray ) const = 0;
	// AMNOTE: Returns the closest point in shape-local coordinates. Sphere and capsule geometry retain interior points.
	virtual Vector GetClosestPoint( const Vector &vPoint ) const = 0;
	// AMNOTE: Returns geometry mass properties, including shape scale, before the owning body's mass and inertia overrides.
	virtual RnInertiaProperties_t GetInertiaProperties() const = 0;
	// AMNOTE: Returns cached world-space broadphase bounds. The shape must belong to a body and be registered in its world.
	virtual AABB_t GetWorldBounds() const = 0;
	// AMNOTE: Returns local geometry bounds.
	virtual AABB_t GetLocalBounds() const = 0;
	// AMNOTE: Computes world-space geometry bounds using the owning body's current origin and orientation. The shape must belong to a body.
	virtual AABB_t GetTransformedBounds() const = 0;
	// AMNOTE: Returns the shape-local support point and directional projection. Does not normalize unitDirection.
	virtual VPhys2SupportPoint_t GetSupportPoint( const Vector &unitDirection ) const = 0;
	// AMNOTE: Transforms the support query and result, consuming the transform's scale. The projection's support-distance interpretation requires a unit direction in shape space.
	virtual VPhys2SupportPoint_t GetSupportPoint( const CTransform &shapeToQuery, const Vector &unitDirection ) const = 0;
	// AMNOTE: Returns borrowed convex geometry and solver state. Mesh/compound return an empty proxy; copied proxies do not extend geometry lifetime.
	virtual CTOIProxy GetConvexSupportProxy() const = 0;
	// AMNOTE: Returns collision geometry volume, including shape scale. Mesh geometry returns zero.
	virtual float GetVolume() const = 0;
	// AMNOTE: Returns normalized projected areas along the local coordinate axes; uniform shape scale does not change them.
	virtual Vector GetOrthographicAreas() const = 0;

private:
	// AMNOTE: Returns the opposite shape from a contact snapshot, or null when this shape is absent. Snapshot state and ownership remain unresolved.
	virtual IPhysicsShape *GetOtherShape( const VPhys2ContactSnapshot_t *pContact ) const = 0;
	// AMNOTE: Replaces the vector with retained contact snapshots. Their state type and ownership wrapper remain unresolved.
	virtual void GetContacts( CUtlVector<VPhys2ContactSnapshot_t> &contacts ) const = 0;

public:
	// AMNOTE: Counts current contact nodes, stored points and manifolds, and points accepted by the native separation test.
	virtual VPhys2ShapeContactStatistics_t GetContactStatistics() const = 0;
	// AMNOTE: Returns the opposite shape from a non-null contact obtained through this shape's GetContacts.
	virtual IPhysicsShape *GetOtherShape( const CRnContact *pContact ) const = 0;
	// AMNOTE: Replaces the vector with borrowed contacts. The engine owns the contacts; simulation or contact removal can invalidate them.
	virtual void GetContacts( CUtlVector<CRnContact *> &contacts ) const = 0;
	// AMNOTE: Appends the opposite bodies' entity IDs from contacts, then sorts and deduplicates the entire vector. Newly encountered invalid IDs are skipped; existing entries are retained.
	virtual void AppendContactEntityIds( CUtlVector<VPhys2ContactEntityId_t> &entityIds ) const = 0;
	// AMNOTE: A nonzero value permits the world's registered contact callback for contacts involving this shape. Stores the value without normalization.
	virtual void SetContactCallbackEnabled( uint8 nEnabled ) = 0;
	virtual uint8 GetContactCallbackEnabled() const = 0;
	// AMNOTE: Returns the native geometry-storage byte count, excluding the adapter and owning body.
	virtual uint64 GetGeometryMemoryUsage() const = 0;
	// AMNOTE: Computes the portion below the plane frame's z=0 using a rigid plane-to-shape transform. Mesh and compound shapes return zero; an unsubmerged capsule also reports zero total volume.
	virtual void ComputeSubmergedVolume( const matrix3x4a_t &planeToShape, VPhys2SubmergedVolume_t &result ) const = 0;

private:
	// AMNOTE: Stores an opaque caller value on the shape adapter; its actual type remains unresolved. Inaccessible placeholder; not a recovered prototype.
	virtual void unk001() = 0;
	// AMNOTE: Reads the preceding opaque caller value; its actual type remains unresolved. Inaccessible placeholder; not a recovered prototype.
	virtual void unk002() = 0;
	// AMNOTE: Does no work in this build; the complete formatted-name parameters remain unresolved.
	virtual void SetDebugNameV() = 0;

public:
	// AMNOTE: Returns null in this build.
	virtual const char *GetDebugName() const = 0;

	// AMNOTE: Returns the owning body's borrowed debug name. The shape must be attached to a body; this build returns an empty string.
	virtual const char *GetBodyDebugName() const = 0;
	// AMNOTE: Borrows the name without copying; keep its storage valid until replaced or shape removal. Null is stored unchanged.
	virtual void SetUserFriendlyName( const char *pszName ) = 0;
	// AMNOTE: Returns borrowed name storage, which can be null after a null assignment.
	virtual const char *GetUserFriendlyName() const = 0;
	// AMNOTE: Draws collision geometry using the supplied position/orientation; transform scale is ignored. pOverlays must be non-null.
	// AMNOTE: Known draw flags are defined by VPHYS2_DEBUG_DRAW_* macros.
	virtual void DebugDraw( ISceneViewDebugOverlays *pOverlays, const CTransform &shapeToOverlay, const Color &color, uint64 nDrawFlags, float flDuration ) const = 0;
	// AMNOTE: Stores an unclamped opacity multiplier for debug scene geometry; its initial value is one. Applied on the next debug scene update.
	virtual void SetDebugSceneOpacity( float flOpacity ) = 0;
	virtual float GetDebugSceneOpacity() const = 0;

protected:
	virtual ~IPhysicsShape() = default;
};


// AMNOTE: Engine-owned adapter layout; remaining native override definitions are unrecovered. Use the inherited interface on engine-created objects.
class CPhysicsShape : public IPhysicsShape
{
public:
	CRnShape *GetNativeShape() const { return m_Shape; }

	using IPhysicsShape::Overlap;
	bool Overlap( const CTransform &shapeToQuery, const AABB_t &bounds ) const override;
	bool Overlap( const CTransform &shapeToQuery, const Ray_t &ray ) const override;

	using IPhysicsShape::GetSupportPoint;
	// AMNOTE: Nonfinite transforms can produce compiler-dependent NaN payloads.
	VPhys2SupportPoint_t GetSupportPoint( const CTransform &shapeToQuery, const Vector &unitDirection ) const override;

	VPhys2SurfacePropertyNameHash_t GetSurfacePropertyNameHash() const override { return VPhys2SurfacePropertyNameHash_t( GetMaterial().m_nSurfacePropertyNameHash ); }
	uint8 GetCollisionGroup() const override { return GetCollisionAttributes().m_nCollisionGroup; }
	const char *GetDebugName() const override { return nullptr; }
	// AMNOTE: Body debug names are disabled in this build; the returned empty string has static storage.
	const char *GetBodyDebugName() const override { return ""; }
	void SetUserFriendlyName( const char *pszName ) override { m_pszUserFriendlyName = pszName; }
	const char *GetUserFriendlyName() const override { return m_pszUserFriendlyName; }

private:
	CPhysicsShape() = delete;
	CPhysicsShape( const CPhysicsShape & ) = delete;
	CPhysicsShape &operator=( const CPhysicsShape & ) = delete;
	~CPhysicsShape() override = default;

	static void ComputeOverlapMatrix( const CTransform &transform, matrix3x4_t &matrix );

	// AMNOTE: Caller storage; its native member type remains unresolved.
	class unkCallerStorage_t
	{
		uint8 m_unk001[8];
	};

	CRnShape *m_Shape;
	unkCallerStorage_t m_unk001;
	const char *m_pszUserFriendlyName;
};

inline VPhys2SupportPoint_t CPhysicsShape::GetSupportPoint( const CTransform &shapeToQuery, const Vector &unitDirection ) const
{
	// Separate scalar SIMD operations preserve the matrix and query direction on fused multiply-add targets.
	const Quaternion &q = shapeToQuery.m_orientation;
	const fltx4 qX = _mm_set_ss( q.x );
	const fltx4 qY = _mm_set_ss( q.y );
	const fltx4 qZ = _mm_set_ss( q.z );
	const fltx4 qW = _mm_set_ss( q.w );
	const fltx4 qX2 = _mm_add_ss( qX, qX );
	const fltx4 qY2 = _mm_add_ss( qY, qY );
	const fltx4 qZ2 = _mm_add_ss( qZ, qZ );
	const fltx4 qXX2 = _mm_mul_ss( qX, qX2 );
	const fltx4 qYY2 = _mm_mul_ss( qY, qY2 );
	const fltx4 qZZ2 = _mm_mul_ss( qZ, qZ2 );
	const fltx4 qXY2 = _mm_mul_ss( qY2, qX );
	const fltx4 qYZ2 = _mm_mul_ss( qZ2, qY );
	const fltx4 qXZ2 = _mm_mul_ss( qZ2, qX );
	const fltx4 qZW2 = _mm_mul_ss( qZ2, qW );
	const fltx4 qXW2 = _mm_mul_ss( qX2, qW );
	const fltx4 qYW2 = _mm_mul_ss( qY2, qW );
	const fltx4 one = _mm_set_ss( 1.0f );
	matrix3x4a_t matrix;
	matrix[0][0] = SubFloat( _mm_sub_ss( _mm_sub_ss( one, qYY2 ), qZZ2 ), 0 );
	matrix[0][1] = SubFloat( _mm_sub_ss( qXY2, qZW2 ), 0 );
	matrix[0][2] = SubFloat( _mm_add_ss( qXZ2, qYW2 ), 0 );
	matrix[1][0] = SubFloat( _mm_add_ss( qXY2, qZW2 ), 0 );
	matrix[1][1] = SubFloat( _mm_sub_ss( _mm_sub_ss( one, qXX2 ), qZZ2 ), 0 );
	matrix[1][2] = SubFloat( _mm_sub_ss( qYZ2, qXW2 ), 0 );
	matrix[2][0] = SubFloat( _mm_sub_ss( qXZ2, qYW2 ), 0 );
	matrix[2][1] = SubFloat( _mm_add_ss( qYZ2, qXW2 ), 0 );
	matrix[2][2] = SubFloat( _mm_sub_ss( _mm_sub_ss( one, qXX2 ), qYY2 ), 0 );
	matrix[0][3] = shapeToQuery.m_vPosition.x;
	matrix[1][3] = shapeToQuery.m_vPosition.y;
	matrix[2][3] = shapeToQuery.m_vPosition.z;
	if ( shapeToQuery.m_vPosition.w != 1.0f )
	{
		for ( int row = 0; row < 3; ++row )
		{
			for ( int column = 0; column < 3; ++column )
				matrix[row][column] *= shapeToQuery.m_vPosition.w;
		}
	}

	Vector localDirection;
	const fltx4 directionX = _mm_set_ss( unitDirection.x );
	const fltx4 directionY = _mm_set_ss( unitDirection.y );
	const fltx4 directionZ = _mm_set_ss( unitDirection.z );
	for ( int column = 0; column < 3; ++column )
	{
		const fltx4 firstProduct = _mm_mul_ss( directionX, _mm_set_ss( matrix[0][column] ) );
		const fltx4 secondProduct = _mm_mul_ss( directionY, _mm_set_ss( matrix[1][column] ) );
		const fltx4 thirdProduct = _mm_mul_ss( directionZ, _mm_set_ss( matrix[2][column] ) );
		localDirection[column] = SubFloat( _mm_add_ss( _mm_add_ss( firstProduct, secondProduct ), thirdProduct ), 0 );
	}
	VPhys2SupportPoint_t result = GetSupportPoint( localDirection );
	const fltx4 point = _mm_set_ps( 1.0f, result.m_vPoint.z, result.m_vPoint.y, result.m_vPoint.x );
	const fltx4 firstRow = MulSIMD( LoadAlignedSIMD( matrix[0] ), point );
	const fltx4 secondRow = MulSIMD( LoadAlignedSIMD( matrix[1] ), point );
	const fltx4 thirdRow = MulSIMD( LoadAlignedSIMD( matrix[2] ), point );
	// AMNOTE: Preserve these pairwise sums and the projection's operation grouping.
	const fltx4 firstPairs = AddSIMD( _mm_shuffle_ps( firstRow, secondRow, 238 ), _mm_shuffle_ps( firstRow, secondRow, 68 ) );
	const fltx4 thirdPairs = AddSIMD( _mm_shuffle_ps( thirdRow, thirdRow, 14 ), _mm_shuffle_ps( thirdRow, thirdRow, 4 ) );
#if defined( COMPILER_GCC )
	const fltx4 transformed = AddSIMD( _mm_shuffle_ps( firstPairs, thirdPairs, 200 ), _mm_shuffle_ps( firstPairs, thirdPairs, 221 ) );
#else
	const fltx4 transformed = AddSIMD( _mm_shuffle_ps( firstPairs, thirdPairs, 221 ), _mm_shuffle_ps( firstPairs, thirdPairs, 200 ) );
#endif
	StoreUnaligned3SIMD( result.m_vPoint.Base(), transformed );
	const fltx4 projectionZ = _mm_mul_ss( _mm_set_ss( matrix[2][3] ), directionZ );
	const fltx4 projectionY = _mm_mul_ss( _mm_set_ss( matrix[1][3] ), directionY );
	const fltx4 projectionX = _mm_mul_ss( _mm_set_ss( matrix[0][3] ), directionX );
	result.m_flProjection = SubFloat( _mm_add_ss( _mm_add_ss( _mm_add_ss( projectionZ, projectionY ), projectionX ), _mm_set_ss( result.m_flProjection ) ), 0 );
	return result;
}

inline bool CPhysicsShape::Overlap( const CTransform &shapeToQuery, const AABB_t &bounds ) const
{
	matrix3x4_t matrix;
	ComputeOverlapMatrix( shapeToQuery, matrix );
	return Overlap( matrix, bounds );
}

inline bool CPhysicsShape::Overlap( const CTransform &shapeToQuery, const Ray_t &ray ) const
{
	matrix3x4_t matrix;
	ComputeOverlapMatrix( shapeToQuery, matrix );
	VPhys2ShapeCastResult_t result;
	const Vector vZero( 0.0f, 0.0f, 0.0f );
	return CastShape( result, matrix, vZero, vZero, ray, 1.0f );
}

inline void CPhysicsShape::ComputeOverlapMatrix( const CTransform &transform, matrix3x4_t &matrix )
{
	const Quaternion &q = transform.m_orientation;
	const fltx4 x = _mm_set_ss( q.x );
	const fltx4 y = _mm_set_ss( q.y );
	const fltx4 z = _mm_set_ss( q.z );
	const fltx4 w = _mm_set_ss( q.w );
	const fltx4 xx = _mm_mul_ss( x, x );
	const fltx4 yy = _mm_mul_ss( y, y );
	const fltx4 zz = _mm_mul_ss( z, z );
	const fltx4 xy = _mm_mul_ss( x, y );
	const fltx4 xz = _mm_mul_ss( x, z );
	const fltx4 yz = _mm_mul_ss( y, z );
	const fltx4 xw = _mm_mul_ss( x, w );
	const fltx4 yw = _mm_mul_ss( y, w );
	const fltx4 zw = _mm_mul_ss( z, w );
	// Overlap sums products before doubling, unlike the transformed support query.
	const fltx4 yyzz = _mm_add_ss( yy, zz );
	const fltx4 xxzz = _mm_add_ss( xx, zz );
	const fltx4 xxyy = _mm_add_ss( xx, yy );
	const fltx4 xyPlusZW = _mm_add_ss( xy, zw );
	const fltx4 xyMinusZW = _mm_sub_ss( xy, zw );
	const fltx4 xzPlusYW = _mm_add_ss( xz, yw );
	const fltx4 xzMinusYW = _mm_sub_ss( xz, yw );
	const fltx4 yzPlusXW = _mm_add_ss( xw, yz );
	const fltx4 yzMinusXW = _mm_sub_ss( yz, xw );
	const fltx4 one = _mm_set_ss( 1.0f );
	matrix[0][0] = SubFloat( _mm_sub_ss( one, _mm_add_ss( yyzz, yyzz ) ), 0 );
	matrix[1][0] = SubFloat( _mm_add_ss( xyPlusZW, xyPlusZW ), 0 );
	matrix[2][0] = SubFloat( _mm_add_ss( xzMinusYW, xzMinusYW ), 0 );
	matrix[0][1] = SubFloat( _mm_add_ss( xyMinusZW, xyMinusZW ), 0 );
	matrix[1][1] = SubFloat( _mm_sub_ss( one, _mm_add_ss( xxzz, xxzz ) ), 0 );
	matrix[2][1] = SubFloat( _mm_add_ss( yzPlusXW, yzPlusXW ), 0 );
	matrix[0][2] = SubFloat( _mm_add_ss( xzPlusYW, xzPlusYW ), 0 );
	matrix[1][2] = SubFloat( _mm_add_ss( yzMinusXW, yzMinusXW ), 0 );
	matrix[2][2] = SubFloat( _mm_sub_ss( one, _mm_add_ss( xxyy, xxyy ) ), 0 );
	matrix[0][3] = transform.m_vPosition.x;
	matrix[1][3] = transform.m_vPosition.y;
	matrix[2][3] = transform.m_vPosition.z;
}

inline void VPhys2ShapeMaterialBinding::SetSurfaceProperties( const CPhysSurfaceProperties &properties )
{
	m_flDensity = properties.m_physics.m_density * 0.000016387063f;
	m_flFriction = properties.m_physics.m_friction;
	m_flElasticity = properties.m_physics.m_elasticity;
	m_flThickness = properties.m_physics.m_thickness;
	m_flSoftContactFrequency = properties.m_physics.m_softContactFrequency;
	m_flSoftContactDampingRatio = properties.m_physics.m_softContactDampingRatio;
	m_flWheelDrag = properties.m_vehicleParams.m_wheelDrag;
	m_flWheelFrictionScale = properties.m_vehicleParams.m_wheelFrictionScale;
	m_nSurfacePropertyNameHash = properties.m_nameHash;
	m_pSurfaceProperties = &properties;
}

#endif // IPHYSICSSHAPE_H
