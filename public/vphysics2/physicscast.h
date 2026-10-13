#ifndef PHYSICSCAST_H
#define PHYSICSCAST_H

#ifdef _WIN32
#pragma once
#endif

#include "mathlib/vector.h"
#include "mathlib/vplane.h"
#include <functional>

class IPhysicsBody;
class IPhysicsShape;
class CPhysSurfaceProperties;

typedef std::function<bool( IPhysicsShape * )> FnVPhys2ShapeFilter_t;

class CRnPlaneResult
{
public:
	// AMNOTE: World-space plane; m_Dist is the normal's dot product with the selected plane point.
	VPlane m_Plane;
	IPhysicsShape *m_pShape = nullptr;
	// AMNOTE: Additional result bits, written as zero by the checked collision paths; meaning unresolved.
	uint32 m_unk001 = 0;
};

struct VPhys2ClosestPointResult_t
{
	Vector m_vPoint{ 0.0f, 0.0f, 0.0f };
	IPhysicsShape *m_pShape = nullptr;
	// AMNOTE: Surface properties selected by the closest subshape; the pointer is engine-owned.
	const CPhysSurfaceProperties *m_pSurfaceProperties = nullptr;
};

struct VPhys2CastResult_t
{
	IPhysicsBody *m_pBody = nullptr;
	IPhysicsShape *m_pShape = nullptr;
	// AMNOTE: The reported point depends on the cast shape; generic line and hull casts report the swept reference point.
	Vector m_vHitPoint{ 0.0f, 0.0f, 0.0f };
	Vector m_vHitNormal{ 0.0f, 0.0f, 0.0f };
	// AMNOTE: Surface properties selected by the hit shape or subshape; the pointer is engine-owned.
	const CPhysSurfaceProperties *m_pSurfaceProperties = nullptr;
	float m_flHitOffset = 0.0f;
	// AMNOTE: Fraction of the full sweep; one means no hit. Reset the result before a new query.
	float m_flFraction = 1.0f;
	// AMNOTE: Compound convex selection. Casts against a body can leave both geometry indices unchanged; use a fresh result.
	int m_nConvexIndex = -1;
	int m_nTriangle = -1;
	bool m_bStartInSolid = false;
};

#endif // PHYSICSCAST_H
