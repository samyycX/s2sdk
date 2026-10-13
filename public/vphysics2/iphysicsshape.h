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

struct VPhys2ShapeMassProperties_t
{
	// AMNOTE: Inertia about the local mass center; the fourth column stores that center.
	matrix3x4_t m_InertiaTensor;
	float m_flMass;
};

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
	virtual VPhys2ShapeMassProperties_t GetMassProperties() const = 0;
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
	const fltx4 q = LoadAlignedSIMD( shapeToQuery.m_orientation.Base() );
	const fltx4 qDoubled = AddSIMD( q, q );
	const fltx4 qSquaredDoubled = MulSIMD( q, qDoubled );
	const fltx4 diagonal = SubSIMD( SubSIMD( ReplicateX4( 1.0f ), _mm_shuffle_ps( qSquaredDoubled, qSquaredDoubled, 193 ) ), _mm_shuffle_ps( qSquaredDoubled, qSquaredDoubled, 218 ) );
	const fltx4 products = MulSIMD( _mm_shuffle_ps( qDoubled, qDoubled, 233 ), _mm_shuffle_ps( q, q, 196 ) );
	const fltx4 wProducts = MulSIMD( _mm_shuffle_ps( qDoubled, qDoubled, 210 ), SplatWSIMD( q ) );
	const fltx4 positive = AddSIMD( products, wProducts );
	const fltx4 negative = SubSIMD( products, wProducts );
	matrix3x4a_t matrix;
	StoreAlignedSIMD( matrix[0], _mm_shuffle_ps( _mm_unpacklo_ps( diagonal, negative ), positive, 36 ) );
	StoreAlignedSIMD( matrix[1], _mm_shuffle_ps( _mm_unpacklo_ps( positive, diagonal ), negative, 28 ) );
	StoreAlignedSIMD( matrix[2], _mm_shuffle_ps( _mm_shuffle_ps( negative, positive, 90 ), diagonal, 40 ) );
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
	localDirection.x = ( unitDirection.x * matrix[0][0] + unitDirection.y * matrix[1][0] ) + unitDirection.z * matrix[2][0];
	localDirection.y = ( unitDirection.x * matrix[0][1] + unitDirection.y * matrix[1][1] ) + unitDirection.z * matrix[2][1];
	localDirection.z = ( unitDirection.x * matrix[0][2] + unitDirection.y * matrix[1][2] ) + unitDirection.z * matrix[2][2];
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
	result.m_flProjection = ( ( matrix[2][3] * unitDirection.z + matrix[1][3] * unitDirection.y ) + matrix[0][3] * unitDirection.x ) + result.m_flProjection;
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
	const float xx = q.x * q.x;
	const float yy = q.y * q.y;
	const float zz = q.z * q.z;
	const float xy = q.x * q.y;
	const float xz = q.x * q.z;
	const float yz = q.y * q.z;
	const float xw = q.x * q.w;
	const float yw = q.y * q.w;
	const float zw = q.z * q.w;
	matrix[0][0] = 1.0f - ( ( yy + zz ) + ( yy + zz ) );
	matrix[1][0] = ( xy + zw ) + ( xy + zw );
	matrix[2][0] = ( xz - yw ) + ( xz - yw );
	matrix[0][1] = ( xy - zw ) + ( xy - zw );
	matrix[1][1] = 1.0f - ( ( xx + zz ) + ( xx + zz ) );
	matrix[2][1] = ( xw + yz ) + ( xw + yz );
	matrix[0][2] = ( xz + yw ) + ( xz + yw );
	matrix[1][2] = ( yz - xw ) + ( yz - xw );
	matrix[2][2] = 1.0f - ( ( xx + yy ) + ( xx + yy ) );
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
