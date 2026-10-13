#ifndef IPHYSICSQUERYCACHE_H
#define IPHYSICSQUERYCACHE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/utlbuffer.h"
#include "tier0/utlvector.h"
#include "vphysics2/iphysicsbody.h"
#include "mathlib/vectoraligned.h"
#include "mathlib/ssemath.h"

class CUtlBuffer;
class ISceneViewDebugOverlays;

abstract_class ITraceObject
{
public:
	virtual ~ITraceObject() = default;
	virtual Vector GetSupportPoint( const Vector &vDirection ) const = 0;
};

class CTracePoint : public ITraceObject
{
public:
	explicit CTracePoint( const Vector &vPoint ) : m_vPoint( vPoint ) {}
	Vector GetSupportPoint( const Vector & ) const override { return m_vPoint; }

private:
	Vector m_vPoint;
};

// AMNOTE: Support of the two endpoints; the capsule radius is handled separately by the query.
class CTraceCapsule : public ITraceObject
{
public:
	CTraceCapsule( const Vector &vStart, const Vector &vEnd ) : m_vEndpoints{ vStart, vEnd } {}
	Vector GetSupportPoint( const Vector &vDirection ) const override;

private:
	Vector m_vEndpoints[2];
};

class CTraceAABB : public ITraceObject
{
public:
	CTraceAABB( const Vector &vMin, const Vector &vMax ) : m_flBounds{ { vMax.x, vMin.x }, { vMax.y, vMin.y }, { vMax.z, vMin.z } } {}
	Vector GetSupportPoint( const Vector &vDirection ) const override
	{
		return Vector( m_flBounds[0][FloatBits( vDirection.x ) >> 31], m_flBounds[1][FloatBits( vDirection.y ) >> 31], m_flBounds[2][FloatBits( vDirection.z ) >> 31] );
	}

private:
	float m_flBounds[3][2];
};

// AMNOTE: Borrows at least one world-space point; keep the point array alive while using the trace object.
class CTraceWorldspaceCloud : public ITraceObject
{
public:
	CTraceWorldspaceCloud( int nPointCount, const Vector *pPoints ) : m_nPointCount( nPointCount ), m_pPoints( pPoints ) { Assert( nPointCount > 0 && pPoints ); }
	Vector GetSupportPoint( const Vector &vDirection ) const override;

private:
	int m_nPointCount;
	const Vector *m_pPoints;
};

class CTraceOBB : public CTraceWorldspaceCloud
{
public:
	// AMNOTE: transform.m_vPosition.w contains the uniform scale.
	CTraceOBB( const Vector &vMin, const Vector &vMax, const CTransform &transform );
	CTraceOBB( const CTraceOBB & ) = delete;
	CTraceOBB &operator=( const CTraceOBB & ) = delete;

private:
	Vector m_vCorners[8];
};

class CTraceTriangle : public ITraceObject
{
public:
	CTraceTriangle( const Vector &vPoint0, const Vector &vPoint1, const Vector &vPoint2 );
	Vector GetSupportPoint( const Vector &vDirection ) const override;

private:
	FourVectors m_Points;
	Vector m_vPoints[3];
};

struct VPhys2RaySegment_t
{
	Vector m_vStart;
	Vector m_vEnd;
};

struct VPhys2QueryCacheCastResult_t
{
	// AMNOTE: xyz contains the hit normal and w contains the segment fraction; one means no hit. Triangle normals are not normalized.
	fltx4 m_HitNormalAndFraction;

	Vector GetHitNormal() const { return Vector( SubFloat( m_HitNormalAndFraction, 0 ), SubFloat( m_HitNormalAndFraction, 1 ), SubFloat( m_HitNormalAndFraction, 2 ) ); }
	float GetFraction() const { return SubFloat( m_HitNormalAndFraction, 3 ); }
};

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
// AMNOTE: The cache captures geometry at creation. Keep referenced convex shapes alive while using it.
abstract_class IPhysicsQueryCache
{
protected:
	virtual ~IPhysicsQueryCache() = default;

public:
	// AMNOTE: Casts each start/end segment and writes nRayCount results; both arrays must have that many elements.
	virtual void CastRays( const VPhys2RaySegment_t *pRays, int nRayCount, VPhys2QueryCacheCastResult_t *pResults ) const = 0;
	// AMNOTE: Casts from vStart to vEnd against cached geometry. The inputs require their declared alignment.
	virtual VPhys2QueryCacheCastResult_t CastRay( const VectorAligned &vStart, const VectorAligned &vEnd ) const = 0;

	// AMNOTE: Draws cached triangles; a non-empty cache requires pOverlays.
	virtual void DebugDraw( ISceneViewDebugOverlays *pOverlays ) const = 0;

	// AMNOTE: Appends the cache's stored geometry and bounds to buffer. Live convex-shape references are not serialized.
	virtual void Serialize( CUtlBuffer &buffer ) const = 0;
	// AMNOTE: Returns the number of cached triangles; other cached geometry is not included.
	virtual int CountTriangles() const = 0;
};

struct VPhys2QueryCacheTriangle_t
{
	VectorAligned m_vPoint[3];
};

struct ALIGN16 VPhys2QueryCacheNode_t
{
	Vector m_vHalfExtent;
	uint32 m_nCountOrRightChild : 29;
	// AMNOTE: 0 branch, 1 triangles, 2 convex, 3 capsule, 4 sphere.
	uint32 m_nType : 3;
	Vector m_vCenter;
	int m_nLeftChildOrElementOffset;
} ALIGN16_POST;

struct VPhys2QueryCacheConvex_t
{
	// AMNOTE: Borrowed hull; keep its geometry alive while using the cache.
	const RnHull_t *m_pHull;
	Vector m_vXAxis;
	Vector m_vYAxis;
	Vector m_vZAxis;
	Vector m_vOrigin;
	float m_flScale;
};

// AMNOTE: Borrows a hull with at least one vertex; keep the hull and its vertex positions alive.
class CTraceHull : public ITraceObject
{
public:
	explicit CTraceHull( const VPhys2QueryCacheConvex_t &convex ) : m_vXAxis( convex.m_vXAxis ), m_vYAxis( convex.m_vYAxis ), m_vZAxis( convex.m_vZAxis ), m_vOrigin( convex.m_vOrigin ), m_flScale( convex.m_flScale ), m_pHull( convex.m_pHull ) { Assert( m_pHull && m_pHull->m_VertexPositions.Count() > 0 ); }
	Vector GetSupportPoint( const Vector &vDirection ) const override;

private:
	Vector m_vXAxis;
	Vector m_vYAxis;
	Vector m_vZAxis;
	Vector m_vOrigin;
	float m_flScale;
	const RnHull_t *m_pHull;
};

// AMNOTE: Borrows at least one local-space point; keep the point array alive while using the trace object.
class CTraceCloud : public ITraceObject
{
public:
	CTraceCloud( const Vector &vXAxis, const Vector &vYAxis, const Vector &vZAxis, const Vector &vOrigin, float flScale, int nPointCount, const Vector *pPoints ) : m_vXAxis( vXAxis ), m_vYAxis( vYAxis ), m_vZAxis( vZAxis ), m_vOrigin( vOrigin ), m_nPointCount( nPointCount ), m_flScale( flScale ), m_pPoints( pPoints ) { Assert( nPointCount > 0 && pPoints ); }
	Vector GetSupportPoint( const Vector &vDirection ) const override;

private:
	Vector m_vXAxis;
	Vector m_vYAxis;
	Vector m_vZAxis;
	Vector m_vOrigin;
	int m_nPointCount;
	float m_flScale;
	const Vector *m_pPoints;
};

// AMNOTE: Engine-owned cache layout; create and destroy through IVPhysics2World.
class CRnQueryCache : public IPhysicsQueryCache
{
public:
	void CastRays( const VPhys2RaySegment_t *pRays, int nRayCount, VPhys2QueryCacheCastResult_t *pResults ) const override;
	VPhys2QueryCacheCastResult_t CastRay( const VectorAligned &vStart, const VectorAligned &vEnd ) const override = 0;
	void DebugDraw( ISceneViewDebugOverlays *pOverlays ) const override = 0;
	void Serialize( CUtlBuffer &buffer ) const override;
	int CountTriangles() const override { return m_Triangles.Count(); }

	const CUtlVectorFixedGrowable<VPhys2QueryCacheTriangle_t, 64> &GetTriangles() const { return m_Triangles; }
	const CUtlVectorFixedGrowable<VPhys2QueryCacheNode_t, 32> &GetNodes() const { return m_Nodes; }
	const CUtlVectorFixedGrowable<RnSphere_t, 8> &GetSpheres() const { return m_Spheres; }
	const CUtlVectorFixedGrowable<RnCapsule_t, 8> &GetCapsules() const { return m_Capsules; }
	const CUtlVectorFixedGrowable<VPhys2QueryCacheConvex_t, 16> &GetConvexes() const { return m_Convexes; }

private:
	CRnQueryCache() = delete;
	CRnQueryCache( const CRnQueryCache & ) = delete;
	CRnQueryCache &operator=( const CRnQueryCache & ) = delete;
	~CRnQueryCache() override = default;

	CUtlVectorFixedGrowable<VPhys2QueryCacheTriangle_t, 64> m_Triangles;
	CUtlVectorFixedGrowable<VPhys2QueryCacheNode_t, 32> m_Nodes;
	CUtlVectorFixedGrowable<RnSphere_t, 8> m_Spheres;
	CUtlVectorFixedGrowable<RnCapsule_t, 8> m_Capsules;
	CUtlVectorFixedGrowable<VPhys2QueryCacheConvex_t, 16> m_Convexes;
};

inline Vector CTraceCapsule::GetSupportPoint( const Vector &vDirection ) const
{
	fltx4 direction = _mm_set_ps( 0.0f, vDirection.z, vDirection.y, vDirection.x );
	float flProjection[2];
	for ( int i = 0; i < 2; ++i )
	{
		fltx4 products = MulSIMD( direction, _mm_set_ps( 0.0f, m_vEndpoints[i].z, m_vEndpoints[i].y, m_vEndpoints[i].x ) );
		flProjection[i] = ( SubFloat( products, 2 ) + SubFloat( products, 1 ) ) + SubFloat( products, 0 );
	}
	return flProjection[0] > flProjection[1] ? m_vEndpoints[0] : m_vEndpoints[1];
}

inline Vector CTraceWorldspaceCloud::GetSupportPoint( const Vector &vDirection ) const
{
	fltx4 direction = _mm_set_ps( 0.0f, vDirection.z, vDirection.y, vDirection.x );
	fltx4 products = MulSIMD( direction, _mm_set_ps( 0.0f, m_pPoints[0].z, m_pPoints[0].y, m_pPoints[0].x ) );
	float flBestProjection = ( SubFloat( products, 2 ) + SubFloat( products, 1 ) ) + SubFloat( products, 0 );
	int nBestPoint = 0;
	for ( int i = 1; i < m_nPointCount; ++i )
	{
		products = MulSIMD( direction, _mm_set_ps( 0.0f, m_pPoints[i].z, m_pPoints[i].y, m_pPoints[i].x ) );
		float flProjection = ( SubFloat( products, 2 ) + SubFloat( products, 1 ) ) + SubFloat( products, 0 );
		if ( flProjection > flBestProjection )
		{
			flBestProjection = flProjection;
			nBestPoint = i;
		}
	}
	return m_pPoints[nBestPoint];
}

inline CTraceOBB::CTraceOBB( const Vector &vMin, const Vector &vMax, const CTransform &transform ) : CTraceWorldspaceCloud( 8, m_vCorners )
{
	fltx4 rotation = LoadAlignedSIMD( transform.m_orientation.Base() );
	fltx4 position = LoadAlignedSIMD( transform.m_vPosition.Base() );
	fltx4 rotationYZX = _mm_shuffle_ps( rotation, rotation, 9 );
	fltx4 rotationZXY = _mm_shuffle_ps( rotation, rotation, 18 );
	for ( int i = 0; i < 8; ++i )
	{
		fltx4 point = _mm_set_ps( 0.0f, ( i & 4 ) ? vMax.z : vMin.z, ( i & 2 ) ? vMax.y : vMin.y, ( i & 1 ) ? vMax.x : vMin.x );
		fltx4 cross = SubSIMD( MulSIMD( _mm_shuffle_ps( point, point, 18 ), rotationYZX ), MulSIMD( _mm_shuffle_ps( point, point, 9 ), rotationZXY ) );
		cross = AddSIMD( cross, cross );
		fltx4 rotated = AddSIMD( SubSIMD( MulSIMD( _mm_shuffle_ps( cross, cross, 18 ), rotationYZX ), MulSIMD( _mm_shuffle_ps( cross, cross, 9 ), rotationZXY ) ), AddSIMD( MulSIMD( ReplicateX4( transform.m_orientation.w ), cross ), point ) );
		fltx4 world = AddSIMD( MulSIMD( rotated, ReplicateX4( transform.m_vPosition.w ) ), position );
		m_vCorners[i] = Vector( SubFloat( world, 0 ), SubFloat( world, 1 ), SubFloat( world, 2 ) );
	}
}

inline CTraceTriangle::CTraceTriangle( const Vector &vPoint0, const Vector &vPoint1, const Vector &vPoint2 ) : m_vPoints{ vPoint0, vPoint1, vPoint2 }
{
	m_Points.x = _mm_set_ps( vPoint2.x, vPoint2.x, vPoint1.x, vPoint0.x );
	m_Points.y = _mm_set_ps( vPoint2.y, vPoint2.y, vPoint1.y, vPoint0.y );
	m_Points.z = _mm_set_ps( vPoint2.z, vPoint2.z, vPoint1.z, vPoint0.z );
}

inline Vector CTraceTriangle::GetSupportPoint( const Vector &vDirection ) const
{
	fltx4 projections = AddSIMD( AddSIMD( MulSIMD( ReplicateX4( vDirection.z ), m_Points.z ), MulSIMD( ReplicateX4( vDirection.y ), m_Points.y ) ), MulSIMD( ReplicateX4( vDirection.x ), m_Points.x ) );
	fltx4 indices = _mm_set_ps( 2.0f, 2.0f, 1.0f, 0.0f );
	fltx4 oppositeProjections = _mm_shuffle_ps( projections, projections, 78 );
	indices = MaskedAssign( CmpLtSIMD( projections, oppositeProjections ), _mm_shuffle_ps( indices, indices, 78 ), indices );
	projections = MaxSIMD( oppositeProjections, projections );
	indices = MaskedAssign( CmpLtSIMD( projections, _mm_shuffle_ps( projections, projections, 57 ) ), _mm_shuffle_ps( indices, indices, 57 ), indices );
	return m_vPoints[int( SubFloat( indices, 0 ) )];
}

namespace VPhysics2TraceDetail
{
inline Vector GetTransformedSupportPoint( const Vector &vDirection, const Vector &vXAxis, const Vector &vYAxis, const Vector &vZAxis, const Vector &vOrigin, float flScale, int nPointCount, const Vector *pPoints )
{
	fltx4 direction = _mm_set_ps( 0.0f, vDirection.z, vDirection.y, vDirection.x );
	fltx4 xAxis = _mm_set_ps( 0.0f, vXAxis.z, vXAxis.y, vXAxis.x );
	fltx4 yAxis = _mm_set_ps( 0.0f, vYAxis.z, vYAxis.y, vYAxis.x );
	fltx4 zAxis = _mm_set_ps( 0.0f, vZAxis.z, vZAxis.y, vZAxis.x );
	fltx4 productsX = MulSIMD( direction, xAxis );
	fltx4 productsY = MulSIMD( direction, yAxis );
	fltx4 productsZ = MulSIMD( direction, zAxis );
	float flLocalX = ( SubFloat( productsX, 0 ) + SubFloat( productsX, 1 ) ) + SubFloat( productsX, 2 );
	float flLocalY = ( SubFloat( productsY, 0 ) + SubFloat( productsY, 1 ) ) + SubFloat( productsY, 2 );
	float flLocalZ = ( SubFloat( productsZ, 0 ) + SubFloat( productsZ, 1 ) ) + SubFloat( productsZ, 2 );
	direction = _mm_set_ps( 0.0f, flLocalZ, flLocalY, flLocalX );
	fltx4 products = MulSIMD( direction, _mm_set_ps( 0.0f, pPoints[0].z, pPoints[0].y, pPoints[0].x ) );
#if defined( PLATFORM_LINUX )
	float flBestProjection = SubFloat( products, 0 ) + ( SubFloat( products, 1 ) + SubFloat( products, 2 ) );
#else
	float flBestProjection = ( SubFloat( products, 2 ) + SubFloat( products, 1 ) ) + SubFloat( products, 0 );
#endif
	int nBestPoint = 0;
	for ( int i = 1; i < nPointCount; ++i )
	{
		products = MulSIMD( direction, _mm_set_ps( 0.0f, pPoints[i].z, pPoints[i].y, pPoints[i].x ) );
		float flProjection = ( SubFloat( products, 1 ) + SubFloat( products, 2 ) ) + SubFloat( products, 0 );
		if ( flProjection > flBestProjection )
		{
			flBestProjection = flProjection;
			nBestPoint = i;
		}
	}
	fltx4 point = MulSIMD( _mm_set_ps( 0.0f, pPoints[nBestPoint].z, pPoints[nBestPoint].y, pPoints[nBestPoint].x ), ReplicateX4( flScale ) );
	productsX = MulSIMD( ReplicateX4( SubFloat( point, 0 ) ), xAxis );
	productsY = MulSIMD( ReplicateX4( SubFloat( point, 1 ) ), yAxis );
	productsZ = MulSIMD( ReplicateX4( SubFloat( point, 2 ) ), zAxis );
#if defined( PLATFORM_LINUX )
	fltx4 world = AddSIMD( AddSIMD( productsY, productsX ), productsZ );
	float flWorldZ = SubFloat( AddSIMD( AddSIMD( productsX, productsY ), productsZ ), 2 );
	world = AddSIMD( world, _mm_set_ps( 0.0f, vOrigin.z, vOrigin.y, vOrigin.x ) );
	flWorldZ = vOrigin.z + flWorldZ;
	return Vector( SubFloat( world, 0 ), SubFloat( world, 1 ), flWorldZ );
#else
	fltx4 world = AddSIMD( AddSIMD( productsX, productsY ), productsZ );
	world = AddSIMD( world, _mm_set_ps( 0.0f, vOrigin.z, vOrigin.y, vOrigin.x ) );
	return Vector( SubFloat( world, 0 ), SubFloat( world, 1 ), SubFloat( world, 2 ) );
#endif
}
}

inline Vector CTraceHull::GetSupportPoint( const Vector &vDirection ) const
{
	return VPhysics2TraceDetail::GetTransformedSupportPoint( vDirection, m_vXAxis, m_vYAxis, m_vZAxis, m_vOrigin, m_flScale, m_pHull->m_VertexPositions.Count(), m_pHull->m_VertexPositions.Base() );
}

inline Vector CTraceCloud::GetSupportPoint( const Vector &vDirection ) const
{
	return VPhysics2TraceDetail::GetTransformedSupportPoint( vDirection, m_vXAxis, m_vYAxis, m_vZAxis, m_vOrigin, m_flScale, m_nPointCount, m_pPoints );
}

inline void CRnQueryCache::CastRays( const VPhys2RaySegment_t *pRays, int nRayCount, VPhys2QueryCacheCastResult_t *pResults ) const
{
	for ( int i = 0; i < nRayCount; ++i )
	{
		VectorAligned vStart( pRays[i].m_vStart.x, pRays[i].m_vStart.y, pRays[i].m_vStart.z );
		VectorAligned vEnd( pRays[i].m_vEnd.x, pRays[i].m_vEnd.y, pRays[i].m_vEnd.z );
		vStart.w = 0.0f;
		vEnd.w = 0.0f;
		pResults[i] = CastRay( vStart, vEnd );
	}
}

inline void CRnQueryCache::Serialize( CUtlBuffer &buffer ) const
{
	buffer.PutInt( 0x01230000 );
	buffer.PutInt( m_Triangles.Count() );
	buffer.Put( m_Triangles.Base(), int( sizeof( VPhys2QueryCacheTriangle_t ) * m_Triangles.Count() ) );
	buffer.PutInt( m_Nodes.Count() );
	buffer.Put( m_Nodes.Base(), int( sizeof( VPhys2QueryCacheNode_t ) * m_Nodes.Count() ) );
	buffer.PutInt( m_Spheres.Count() );
	buffer.Put( m_Spheres.Base(), int( sizeof( RnSphere_t ) * m_Spheres.Count() ) );
	buffer.PutInt( m_Capsules.Count() );
	buffer.Put( m_Capsules.Base(), int( sizeof( RnCapsule_t ) * m_Capsules.Count() ) );
}

#endif // IPHYSICSQUERYCACHE_H
