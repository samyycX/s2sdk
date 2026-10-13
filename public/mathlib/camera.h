//====== Copyright © 1996-2005, Valve Corporation, All rights reserved. =======//
//
// Purpose: 
//
// $NoKeywords: $
//
//=============================================================================//

#ifndef CAMERA_H
#define CAMERA_H

#ifdef _WIN32
#pragma once
#endif

#include <math.h>
#include <float.h>

// For vec_t, put this somewhere else?
#include "tier0/basetypes.h"
#include "mathlib/vector.h"
#include "mathlib/vectoraligned.h"

#include "tier0/dbg.h"
#include "mathlib/vector2d.h"
#include "mathlib/math_pfns.h"
#include "mathlib/vmatrix.h"
#include "mathlib/ssemath.h"
#include "datamap.h"
#include "mathlib/aabb.h"

#include "tier0/memalloc.h"
#include "tier0/tier0_stdlib.h"
// declarations for camera and frustum

struct ALIGN16 Camera_t
{
	Vector m_origin;
	QAngle m_angles;
	float m_flFOVX;		// FOV for X/width; minus one selects orthographic projection.
	float m_flAspect;
	float m_flZNear;
	float m_flZFar;
	float m_flWidth;
	float m_flHeight;
} ALIGN16_POST;


//-----------------------------------------------------------------------------
// accessors for generated matrices
//-----------------------------------------------------------------------------
inline void ComputeViewMatrix( VMatrix *pWorldToView, const Camera_t& camera );
inline void ComputeViewMatrix( matrix3x4_t *pWorldToView, const Camera_t& camera );
inline void ComputeViewMatrix( matrix3x4_t *pWorldToView, matrix3x4_t *pCameraToWorld, const Camera_t &camera );
inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, const Camera_t& camera, int width, int height );
inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, float flZNear, float flZFar, float flFOVX, float flAspectRatio );
inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, float flZNear, float flZFar, float flFOVX, float flAspectRatio, float flLeft, float flBottom, float flRight, float flTop );


//-----------------------------------------------------------------------------
// Computes the screen space position given a screen size
//-----------------------------------------------------------------------------
inline void ComputeScreenSpacePosition( Vector2D *pScreenPosition, const Vector &vecWorldPosition, 
	const Camera_t &camera, int width, int height );


class ALIGN16 CFrustum
{
public:
	CFrustum();

	~CFrustum() = default;
	
	bool BoundingVolumeIntersectsFrustum( AABB_t const &box ) const
	{
		Vector vMins = box.m_vMinBounds - m_camera.m_origin;
		Vector vMaxs = box.m_vMaxBounds - m_camera.m_origin;
		return !m_frustum.CullBox( vMins, vMaxs );
	}

	bool BoundingVolumeIntersectsFrustum( Vector const &mins, Vector const &maxes ) const
	{
		Vector vMins = mins - m_camera.m_origin;
		Vector vMaxs = maxes - m_camera.m_origin;
		return !m_frustum.CullBox( vMins, vMaxs );
	}

	bool BoundingVolumeIntersectsFrustum( AABB_t const &box, Vector &vOriginShift ) const
	{
		Vector vMins = box.m_vMinBounds - m_camera.m_origin - vOriginShift;
		Vector vMaxs = box.m_vMaxBounds - m_camera.m_origin - vOriginShift;
		return !m_frustum.CullBox( vMins, vMaxs );
	}

	FORCEINLINE bool CheckBoxInline( const VectorAligned &mins, const VectorAligned &maxs )
	{
		fltx4 fl4Origin = LoadAlignedSIMD( &m_camera.m_origin );
		fltx4 mins4 = SubSIMD( LoadAlignedSIMD( &mins.x ), fl4Origin );
		fltx4 maxs4 = SubSIMD( LoadAlignedSIMD( &maxs.x ), fl4Origin );
	
		fltx4 minx = SplatXSIMD(mins4);
		fltx4 miny = SplatYSIMD(mins4);
		fltx4 minz = SplatZSIMD(mins4);
		fltx4 maxx = SplatXSIMD(maxs4);
		fltx4 maxy = SplatYSIMD(maxs4);
		fltx4 maxz = SplatZSIMD(maxs4);

		// compute the dot product of the normal and the farthest corner
		// dotBack0 = DotProduct( normal, normals.x < 0 ? mins.x : maxs.x );
		for ( int i = 0; i < 2; i++ )
		{
			fltx4 xTotalBack = MulSIMD( m_frustum.planes[i].nX, MaskedAssign( m_frustum.planes[i].xSign, minx, maxx ) );
			fltx4 yTotalBack = MulSIMD( m_frustum.planes[i].nY, MaskedAssign( m_frustum.planes[i].ySign, miny, maxy ) );
			fltx4 zTotalBack = MulSIMD( m_frustum.planes[i].nZ, MaskedAssign( m_frustum.planes[i].zSign, minz, maxz ) );
			fltx4 dotBack = AddSIMD( xTotalBack, AddSIMD(yTotalBack, zTotalBack) );
			// if plane of the farthest corner is behind the plane, then the box is completely outside this plane
#if _X360
			if  ( !XMVector3GreaterOrEqual( dotBack, m_frustum.planes[i].dist ) )
				return false;
#else
			fltx4 isOut = CmpLtSIMD( dotBack, m_frustum.planes[i].dist );
			if ( IsAnyNegative(isOut) )
				return false;
#endif
		}
		return true;
	}

	void GetNearAndFarPlanesAroundBox( float *pNear, float *pFar, AABB_t const &inBox, Vector &vOriginShift ) const
	{
		AABB_t box = inBox;
		box.m_vMinBounds -= vOriginShift;
		box.m_vMaxBounds -= vOriginShift;

		Vector vCorners[8];
		vCorners[0] = box.m_vMinBounds;
		vCorners[1] = Vector( box.m_vMinBounds.x, box.m_vMinBounds.y, box.m_vMaxBounds.z );
		vCorners[2] = Vector( box.m_vMinBounds.x, box.m_vMaxBounds.y, box.m_vMinBounds.z );
		vCorners[3] = Vector( box.m_vMinBounds.x, box.m_vMaxBounds.y, box.m_vMaxBounds.z );

		vCorners[4] = Vector( box.m_vMaxBounds.x, box.m_vMinBounds.y, box.m_vMinBounds.z );
		vCorners[5] = Vector( box.m_vMaxBounds.x, box.m_vMinBounds.y, box.m_vMaxBounds.z );
		vCorners[6] = Vector( box.m_vMaxBounds.x, box.m_vMaxBounds.y, box.m_vMinBounds.z );
		vCorners[7] = box.m_vMaxBounds;

		float flNear = FLT_MAX;//m_camera.m_flZNear;
		float flFar = -FLT_MAX;//m_camera.m_flZFar;
		for ( int i=0; i<8; ++i )
		{
			Vector vDelta = vCorners[i] - m_camera.m_origin;
			float flDist = DotProduct( m_forward, vDelta );
			flNear = MIN( flNear, flDist );
			flFar = MAX( flFar, flDist );
		}

		*pNear = flNear;
		*pFar = flFar;
	}

	void UpdateFrustumFromCamera();



	// Just cram this here since until I figure out how to convert from direction to angles
	static VMatrix ViewMatrixRH( Vector &vEye, Vector &vAt, Vector &vUp )
	{
		Vector xAxis, yAxis;
		Vector zAxis = vEye - vAt;
		xAxis = CrossProduct( vUp, zAxis );
		yAxis = CrossProduct( zAxis, xAxis );
		xAxis.NormalizeInPlace();
		yAxis.NormalizeInPlace();
		zAxis.NormalizeInPlace();
		float flDotX = -DotProduct( xAxis, vEye );
		float flDotY = -DotProduct( yAxis, vEye );
		float flDotZ = -DotProduct( zAxis, vEye );
		VMatrix mRet( 
			xAxis.x, yAxis.x, zAxis.x, 0,
			xAxis.y, yAxis.y, zAxis.y, 0,
			xAxis.z, yAxis.z, zAxis.z, 0,
			flDotX, flDotY, flDotZ, 1 );
		return mRet.Transpose();
	}

	VMatrix OrthoMatrixRH( float flWidth, float flHeight, float flNear, float flFar )
	{
		float flDelta = flNear - flFar;

		VMatrix mRet( 
			2.0f / flWidth, 0, 0, 0,
			0, 2.0f / flHeight, 0, 0,
			0, 0, 1.0f / flDelta, 0,
			0, 0, flNear / flDelta, 1 );
		return mRet.Transpose();
	}


	const Vector &Forward() const { return m_forward; }
	const Vector &Left() const { return m_left; }
	const Vector &Up() const { return m_up; }

	void SetView( VMatrix &mView ) { m_worldToView = mView.As3x4(); CalcViewProj(); UpdatePlanes(); }
	void SetProj( VMatrix &mProj );
	void CalcViewProj();
	void SetFrustum( const Frustum_t &frustum ) { m_frustum = frustum; }
	void SetForward( Vector &forward ) { m_forward = forward; }
	void SetNearFarPlanes( float flNear, float flFar ) { m_camera.m_flZNear = flNear; m_camera.m_flZFar = flFar; UpdateProjection(); }
	void SetCameraPosition( const Vector &origin ) { m_camera.m_origin = origin; UpdateFrustumFromCamera(); }
	void SetCameraAngles( const QAngle &angles ) { m_camera.m_angles = angles; UpdateFrustumFromCamera(); }
	void SetAspect( float flAspect ) { m_camera.m_flAspect = flAspect; UpdateProjection(); }
	void SetFOV( float flFOV ) { m_camera.m_flFOVX = flFOV; UpdateProjection(); }

	const Vector &GetCameraPosition() const { return m_camera.m_origin; }
	const QAngle &GetCameraAngles() const { return m_camera.m_angles; }
	const matrix3x4_t &GetView() const { return m_worldToView; }
	const VMatrix &GetProj() const { return m_projection; }
	const VMatrix &GetViewProj() const { return m_viewProj; }
	float GetAspect() const { return m_camera.m_flAspect; }
	float GetNearPlane() const { return m_camera.m_flZNear; }
	float GetFarPlane() const { return m_camera.m_flZFar; }
	float GetFOV() const { return m_camera.m_flFOVX; }

	void InitCamera( const Vector &origin, const QAngle &angles, float flNear, float flFar, float flFOV, float flAspect );

	void InitOrthoCamera( const Vector &origin, const QAngle &angles, float flNear, float flFar, float flWidth, float flHeight );
	void SetCameraWidthHeight( float flWidth, float flHeight ) { m_camera.m_flWidth = flWidth; m_camera.m_flHeight = flHeight; m_camera.m_flAspect = flWidth / flHeight; UpdateProjection(); }
	bool IsOrthographic() const { return m_camera.m_flFOVX == -1.0f; }
	const VMatrix &GetInvProj() const { return m_invProjection; }
	const VMatrix &GetInvViewProj() const { return m_invViewProj; }
	const VMatrix &GetReverseZProj() const { return m_reverseZProjection; }
	const VMatrix &GetInvReverseZProj() const { return m_invReverseZProjection; }
	const VMatrix &GetReverseZViewProj() const { return m_reverseZViewProj; }
	const VMatrix &GetInvReverseZViewProj() const { return m_invReverseZViewProj; }

protected:
	Camera_t m_camera;
	Frustum_t m_frustum;
	float m_flClipSpaceBottomLeftX;
	float m_flClipSpaceBottomLeftY;
	float m_flClipSpaceTopRightX;
	float m_flClipSpaceTopRightY;
	bool m_bD3DClippingRange;
	bool m_bInfiniteReverseZProjection;
	Vector m_forward;
	Vector m_left;
	Vector m_up;
	matrix3x4_t m_cameraToWorld;
	matrix3x4_t m_worldToView;
	VMatrix m_projection;
	VMatrix m_invProjection;
	VMatrix m_viewProj;
	VMatrix m_invViewProj;
	VMatrix m_reverseZProjection;
	VMatrix m_invReverseZProjection;
	VMatrix m_reverseZViewProj;
	VMatrix m_invReverseZViewProj;
	// AMNOTE: Scale saved when transforming a camera between coordinate frames.
	float m_flTransformScale;
	float m_flTanHalfFOVX;
	float m_flInvTanHalfFOVX;
	float m_flClipSpaceOffsetX;
	float m_flClipSpaceOffsetY;

private:
	friend void ComputeViewMatrix( matrix3x4_t *pWorldToView, matrix3x4_t *pCameraToWorld, const Camera_t &camera );
	static void BuildCameraToWorld( const Camera_t &camera, matrix3x4_t &cameraToWorld );
	static void BuildViewFromCameraToWorld( const matrix3x4_t &cameraToWorld, matrix3x4_t &worldToView );
	static VMatrix MultiplyMatrices( const VMatrix &left, const VMatrix &right );
	static bool InvertMatrix( const VMatrix &matrix, VMatrix &inverse );
	void UpdateProjection();
	void UpdatePlanes();
	void UpdateReverseZProjection();
} ALIGN16_POST;

inline void ComputeViewMatrix( matrix3x4_t *pWorldToView, matrix3x4_t *pCameraToWorld, const Camera_t &camera )
{
	CFrustum::BuildCameraToWorld( camera, *pCameraToWorld );
	CFrustum::BuildViewFromCameraToWorld( *pCameraToWorld, *pWorldToView );
}

inline void ComputeViewMatrix( matrix3x4_t *pWorldToView, const Camera_t &camera )
{
	matrix3x4_t cameraToWorld;
	ComputeViewMatrix( pWorldToView, &cameraToWorld, camera );
}

inline void ComputeViewMatrix( VMatrix *pWorldToView, const Camera_t &camera )
{
	matrix3x4_t worldToView;
	ComputeViewMatrix( &worldToView, camera );
	pWorldToView->Init( worldToView );
}

inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, float flZNear, float flZFar, float flFOVX, float flAspectRatio )
{
	ComputeProjectionMatrix( pCameraToProjection, flZNear, flZFar, flFOVX, flAspectRatio, -1, -1, 1, 1 );
}

inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, float flZNear, float flZFar, float flFOVX, float flAspectRatio, float flLeft, float flBottom, float flRight, float flTop )
{
	float flHalfWidth = 1.0f, flHalfHeight = 1.0f;
	if ( flFOVX != -1.0f )
	{
		const float flTangent = V_tanf( ( flFOVX * 0.5f ) * 0.017453292f );
		flHalfWidth = flTangent * flZNear;
		flHalfHeight = ( flTangent / MAX( 0.0009765625f, flAspectRatio ) ) * flZNear;
	}
	const float left = flLeft * flHalfWidth, right = flRight * flHalfWidth;
	const float bottom = flBottom * flHalfHeight, top = flTop * flHalfHeight;
	const float flDepth = flZFar == INFINITY ? -1.0f : flZFar / ( flZNear - flZFar );
	const float flDepthOffset = flZFar == INFINITY ? -flZNear : ( flZNear * flZFar ) / ( flZNear - flZFar );
	pCameraToProjection->Init( ( flZNear + flZNear ) / ( right - left ), 0, ( right + left ) / ( right - left ), 0,
		0, ( flZNear + flZNear ) / ( top - bottom ), ( top + bottom ) / ( top - bottom ), 0,
		0, 0, flDepth, flDepthOffset, 0, 0, -1, 0 );
}

inline void ComputeProjectionMatrix( VMatrix *pCameraToProjection, const Camera_t &camera, int width, int height )
{
	if ( camera.m_flFOVX != -1.0f )
	{
		ComputeProjectionMatrix( pCameraToProjection, camera.m_flZNear, camera.m_flZFar, camera.m_flFOVX, static_cast<float>( width ) / height );
		return;
	}
	pCameraToProjection->Init( 2.0f / camera.m_flWidth, 0, 0, 0,
		0, 2.0f / camera.m_flHeight, 0, 0,
		0, 0, 1.0f / ( camera.m_flZNear - camera.m_flZFar ), camera.m_flZNear / ( camera.m_flZNear - camera.m_flZFar ),
		0, 0, 0, 1 );
}

inline void ComputeScreenSpacePosition( Vector2D *pScreenPosition, const Vector &vecWorldPosition, const Camera_t &camera, int width, int height )
{
	VMatrix view, projection;
	ComputeViewMatrix( &view, camera );
	ComputeProjectionMatrix( &projection, camera, width, height );
	const Vector vView = view.VMul4x3( vecWorldPosition );
	Vector vProjected;
	projection.V3Mul( vView, vProjected );
	pScreenPosition->x = ( vProjected.x + 1.0f ) * 0.5f * width;
	pScreenPosition->y = ( 1.0f - vProjected.y ) * 0.5f * height;
}

inline void CFrustum::BuildCameraToWorld( const Camera_t &camera, matrix3x4_t &cameraToWorld )
{
	float sp, cp, sy, cy, sr, cr;
	V_sincosf( camera.m_angles.x * 0.017453292f, &sp, &cp );
	V_sincosf( camera.m_angles.y * 0.017453292f, &sy, &cy );
	V_sincosf( camera.m_angles.z * 0.017453292f, &sr, &cr );
	const float spSr = sp * sr, spCr = sp * cr;
	cameraToWorld[0][0] = cp * cy;
	cameraToWorld[1][0] = cp * sy;
	cameraToWorld[2][0] = -sp;
	cameraToWorld[0][1] = cy * spSr - sy * cr;
	cameraToWorld[1][1] = spSr * sy + cy * cr;
	cameraToWorld[2][1] = cp * sr;
	cameraToWorld[0][2] = cy * spCr + sy * sr;
	cameraToWorld[1][2] = sy * spCr - sr * cy;
	cameraToWorld[2][2] = cp * cr;
	MatrixSetColumn( camera.m_origin, 3, cameraToWorld );
}

inline void CFrustum::BuildViewFromCameraToWorld( const matrix3x4_t &cameraToWorld, matrix3x4_t &worldToView )
{
	const fltx4 zero = ReplicateX4( 0.0f ), minusOne = ReplicateX4( -1.0f );
	const fltx4 axes[3] = { SetZSIMD( zero, minusOne ), SetXSIMD( zero, minusOne ), SetYSIMD( zero, ReplicateX4( 1.0f ) ) };
	matrix3x4_t viewToWorld;
	for ( int row = 0; row < 3; ++row )
	{
		const fltx4 values = LoadUnalignedSIMD( cameraToWorld[row] );
		const fltx4 yz = AddSIMD( MulSIMD( SplatZSIMD( values ), axes[2] ), MulSIMD( SplatYSIMD( values ), axes[1] ) );
		const fltx4 translation = AndSIMD( values, LoadAlignedSIMD( g_SIMD_ComponentMask[3] ) );
		StoreUnalignedSIMD( viewToWorld[row], AddSIMD( AddSIMD( MulSIMD( SplatXSIMD( values ), axes[0] ), yz ), translation ) );
	}
	for ( int row = 0; row < 3; ++row )
	{
		for ( int column = 0; column < 3; ++column ) worldToView[row][column] = viewToWorld[column][row];
		worldToView[row][3] = -( ( viewToWorld[2][3] * worldToView[row][2] + viewToWorld[1][3] * worldToView[row][1] ) + viewToWorld[0][3] * worldToView[row][0] );
	}
	const float lengthSquared = ( viewToWorld[0][0] * viewToWorld[0][0] + viewToWorld[1][0] * viewToWorld[1][0] ) + viewToWorld[2][0] * viewToWorld[2][0];
	if ( !( FloatMakePositive( lengthSquared - 1.0f ) < 0.001f ) && lengthSquared != 0.0f )
	{
		const float inverseLengthSquared = 1.0f / lengthSquared;
		for ( int row = 0; row < 3; ++row )
			for ( int column = 0; column < 4; ++column ) worldToView[row][column] *= inverseLengthSquared;
	}
}

inline VMatrix CFrustum::MultiplyMatrices( const VMatrix &left, const VMatrix &right )
{
	fltx4 columns[4] = { LoadUnalignedSIMD( right[0] ), LoadUnalignedSIMD( right[1] ), LoadUnalignedSIMD( right[2] ), LoadUnalignedSIMD( right[3] ) };
	_MM_TRANSPOSE4_PS( columns[0], columns[1], columns[2], columns[3] );
	VMatrix product;
	for ( int row = 0; row < 4; ++row )
	{
		const fltx4 values = LoadUnalignedSIMD( left[row] );
		for ( int column = 0; column < 4; ++column )
		{
			const fltx4 terms = MulSIMD( values, columns[column] );
			const fltx4 pairs = AddSIMD( terms, RotateLeft( terms ) );
			product[row][column] = SubFloat( AddSIMD( pairs, RotateLeft2( pairs ) ), 0 );
		}
	}
	return product;
}

inline bool CFrustum::InvertMatrix( const VMatrix &matrix, VMatrix &inverse )
{
	float rows[4][8];
	int order[4] = { 0, 1, 2, 3 };
	for ( int row = 0; row < 4; ++row )
		for ( int column = 0; column < 4; ++column )
		{
			rows[row][column] = matrix[row][column];
			rows[row][column + 4] = row == column ? 1.0f : 0.0f;
		}
	for ( int column = 0; column < 4; ++column )
	{
		float largest = 0.000001f;
		int pivot = -1;
		for ( int row = column; row < 4; ++row )
		{
			const float value = FloatMakePositive( rows[order[row]][column] );
			if ( value > largest ) { largest = value; pivot = row; }
		}
		if ( pivot == -1 )
			return false;
		const int selected = order[pivot];
		order[pivot] = order[column];
		order[column] = selected;
		float *pPivot = rows[selected];
		const float inversePivot = 1.0f / pPivot[column];
		for ( int entry = 0; entry < 8; ++entry ) pPivot[entry] *= inversePivot;
		pPivot[column] = 1.0f;
		for ( int row = 0; row < 4; ++row )
		{
			if ( row == column ) continue;
			float *pRow = rows[order[row]];
			const float factor = -pRow[column];
			for ( int entry = 0; entry < 8; ++entry ) pRow[entry] += pPivot[entry] * factor;
			pRow[column] = 0.0f;
		}
	}
	for ( int row = 0; row < 4; ++row )
		for ( int column = 0; column < 4; ++column ) inverse[row][column] = rows[order[row]][column + 4];
	return true;
}

inline CFrustum::CFrustum()
{
	m_camera = { Vector( 0, 0, 0 ), QAngle( 0, 0, 0 ), 90.0f, 1.0f, 10.0f, 100.0f, -1.0f, -1.0f };
	V_memset( &m_frustum, 0, sizeof( m_frustum ) );
	m_flClipSpaceBottomLeftX = m_flClipSpaceBottomLeftY = -1.0f;
	m_flClipSpaceTopRightX = m_flClipSpaceTopRightY = 1.0f;
	m_bD3DClippingRange = false;
	m_bInfiniteReverseZProjection = true;
	m_forward.Init( 1, 0, 0 );
	m_left.Init( 0, 1, 0 );
	m_up.Init( 0, 0, 1 );
	m_cameraToWorld.Init( Vector( 1, 0, 0 ), Vector( 0, 1, 0 ), Vector( 0, 0, 1 ), Vector( 0, 0, 0 ) );
	m_worldToView.Init( Vector( 0, 0, -1 ), Vector( -1, 0, 0 ), Vector( 0, 1, 0 ), Vector( 0, 0, 0 ) );
	const float depth = BitsToFloat( 0xBF8E38E4u );
	const float depthOffset = BitsToFloat( 0xC131C71Cu );
	const float inverseDepthOffset = BitsToFloat( 0xBDB851ECu );
	const float inverseDepth = BitsToFloat( 0x3DCCCCCEu );
	m_projection.Init( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, depth, depthOffset, 0, 0, -1, 0 );
	m_invProjection.Init( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, inverseDepthOffset, inverseDepth );
	m_viewProj.Init( 0, -1, 0, 0, 0, 0, 1, 0, -depth, 0, 0, depthOffset, 1, 0, 0, 0 );
	m_invViewProj.Init( 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, inverseDepthOffset, inverseDepth );
	const float scale = 1.0f / V_tanf( 0.78539819f );
	m_reverseZProjection.Init( scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 0, 10, 0, 0, -1, 0 );
	InvertMatrix( m_reverseZProjection, m_invReverseZProjection );
	m_reverseZViewProj = MultiplyMatrices( m_reverseZProjection, VMatrix( m_worldToView ) );
	InvertMatrix( m_reverseZViewProj, m_invReverseZViewProj );
	m_flTransformScale = m_flTanHalfFOVX = m_flInvTanHalfFOVX = 1.0f;
	m_flClipSpaceOffsetX = m_flClipSpaceOffsetY = 0.0f;
}

inline void CFrustum::CalcViewProj()
{
	const VMatrix view( m_worldToView );
	m_viewProj = MultiplyMatrices( m_projection, view );
	InvertMatrix( m_viewProj, m_invViewProj );
	InvertMatrix( m_projection, m_invProjection );
	m_reverseZViewProj = MultiplyMatrices( m_reverseZProjection, view );
	InvertMatrix( m_reverseZViewProj, m_invReverseZViewProj );
	InvertMatrix( m_reverseZProjection, m_invReverseZProjection );
}

inline void CFrustum::UpdateReverseZProjection()
{
	m_reverseZProjection = m_projection;
	for ( int column = 0; column < 4; ++column ) m_reverseZProjection[2][column] = m_projection[3][column] - m_projection[2][column];
}

inline void CFrustum::UpdateFrustumFromCamera()
{
	ComputeViewMatrix( &m_worldToView, &m_cameraToWorld, m_camera );
	MatrixGetColumn( m_cameraToWorld, 0, m_forward );
	MatrixGetColumn( m_cameraToWorld, 1, m_left );
	MatrixGetColumn( m_cameraToWorld, 2, m_up );
	UpdateProjection();
}

inline void CFrustum::InitOrthoCamera( const Vector &origin, const QAngle &angles, float flNear, float flFar, float flWidth, float flHeight )
{
	m_camera = { origin, angles, -1.0f, flWidth / flHeight, flNear, flFar, flWidth, flHeight };
	m_flClipSpaceBottomLeftX = m_flClipSpaceBottomLeftY = -1.0f;
	m_flClipSpaceTopRightX = m_flClipSpaceTopRightY = 1.0f;
	UpdateFrustumFromCamera();
}

inline void CFrustum::UpdateProjection()
{
	m_bInfiniteReverseZProjection = !IsOrthographic();
	if ( IsOrthographic() )
	{
		const float left = ( m_flClipSpaceBottomLeftX * m_camera.m_flWidth ) * 0.5f;
		const float right = ( m_flClipSpaceTopRightX * m_camera.m_flWidth ) * 0.5f;
		const float bottom = ( m_flClipSpaceBottomLeftY * m_camera.m_flHeight ) * 0.5f;
		const float top = ( m_flClipSpaceTopRightY * m_camera.m_flHeight ) * 0.5f;
		const float x = 2.0f / ( right - left ), y = 2.0f / ( top - bottom );
		m_projection.Init( x, 0, 0, -left * x - 1.0f, 0, y, 0, -bottom * y - 1.0f,
			0, 0, 1.0f / ( m_camera.m_flZNear - m_camera.m_flZFar ), m_camera.m_flZNear / ( m_camera.m_flZNear - m_camera.m_flZFar ),
			0, 0, 0, 1 );
		UpdateReverseZProjection();
	}
	else
	{
		ComputeProjectionMatrix( &m_projection, m_camera.m_flZNear, m_camera.m_flZFar, m_camera.m_flFOVX, m_camera.m_flAspect, m_flClipSpaceBottomLeftX, m_flClipSpaceBottomLeftY, m_flClipSpaceTopRightX, m_flClipSpaceTopRightY );
		VMatrix infiniteProjection;
		ComputeProjectionMatrix( &infiniteProjection, m_camera.m_flZNear, INFINITY, m_camera.m_flFOVX, m_camera.m_flAspect, m_flClipSpaceBottomLeftX, m_flClipSpaceBottomLeftY, m_flClipSpaceTopRightX, m_flClipSpaceTopRightY );
		m_reverseZProjection = infiniteProjection;
		for ( int column = 0; column < 4; ++column ) m_reverseZProjection[2][column] = infiniteProjection[3][column] - infiniteProjection[2][column];
	}
	VMatrix translation;
	translation.Identity();
	translation[0][3] = m_flClipSpaceOffsetX;
	translation[1][3] = m_flClipSpaceOffsetY;
	m_projection = MultiplyMatrices( translation, m_projection );
	m_reverseZProjection = MultiplyMatrices( translation, m_reverseZProjection );
	m_flTanHalfFOVX = V_tanf( ( m_camera.m_flFOVX * 0.5f ) * 0.017453292f );
	m_flInvTanHalfFOVX = 1.0f / m_flTanHalfFOVX;
	CalcViewProj();
	UpdatePlanes();
}

inline void CFrustum::UpdatePlanes()
{
	VPlane planes[6];
	for ( int index = 0; index < 6; ++index )
	{
		const int row = index / 2;
		const float sign = index >= FRUSTUM_NEARZ ? ( index == FRUSTUM_NEARZ ? 1.0f : -1.0f ) : ( index % 2 == 0 ? -1.0f : 1.0f );
		Vector normal;
		float distance;
		if ( index == FRUSTUM_NEARZ && m_bD3DClippingRange )
		{
			normal.Init( m_viewProj[2][0], m_viewProj[2][1], m_viewProj[2][2] );
			distance = -m_viewProj[2][3];
		}
		else
		{
			normal.Init( m_viewProj[3][0] + sign * m_viewProj[row][0], m_viewProj[3][1] + sign * m_viewProj[row][1], m_viewProj[3][2] + sign * m_viewProj[row][2] );
			distance = -( m_viewProj[3][3] + sign * m_viewProj[row][3] );
		}
		const float lengthSquared = ( normal.x * normal.x + normal.y * normal.y ) + normal.z * normal.z;
		if ( index == FRUSTUM_FARZ && lengthSquared <= 1.0e-20f )
		{
			planes[index] = planes[FRUSTUM_NEARZ];
			continue;
		}
		if ( lengthSquared >= 1.0e-20f )
		{
			const float inverseLength = 1.0f / FastSqrt( lengthSquared );
			normal *= inverseLength;
			distance *= inverseLength;
		}
		distance = ( ( normal.z * -m_camera.m_origin.z + normal.y * -m_camera.m_origin.y ) + normal.x * -m_camera.m_origin.x ) + distance;
		planes[index].Init( normal, distance );
	}
	m_frustum.SetPlanes( planes );
}

inline void CFrustum::SetProj( VMatrix &mProj )
{
	m_projection = mProj;
	UpdateReverseZProjection();
	m_bInfiniteReverseZProjection = false;
	CalcViewProj();
	UpdatePlanes();
}

inline void CFrustum::InitCamera( const Vector &origin, const QAngle &angles, float flNear, float flFar, float flFOV, float flAspect )
{
	m_camera.m_origin = origin;
	m_camera.m_angles = angles;
	m_camera.m_flZNear = flNear;
	m_camera.m_flZFar = flFar;
	m_camera.m_flFOVX = flFOV;
	m_camera.m_flAspect = flAspect;
	m_camera.m_flWidth = -1.0f;
	m_camera.m_flHeight = -1.0f;
	m_flClipSpaceBottomLeftX = m_flClipSpaceBottomLeftY = -1.0f;
	m_flClipSpaceTopRightX = m_flClipSpaceTopRightY = 1.0f;
	UpdateFrustumFromCamera();
}

#endif // CAMERA_H

