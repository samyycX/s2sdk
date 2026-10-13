#ifndef RUBIKON_PARAM_TYPES_H
#define RUBIKON_PARAM_TYPES_H

#ifdef _WIN32
#pragma once
#endif

#include "mathlib/vector.h"
#include "mathlib/mathlib.h"

struct RnDistanceQueryResult_t
{
	float m_flDistance;
	Vector m_vPoint1;
	Vector m_vPoint2;
};

struct RnInertiaProperties_t
{
	// AMNOTE: Inertia about the local mass center; the fourth column stores that center.
	matrix3x4_t m_Inertia;
	float m_flMass;
};

#endif // RUBIKON_PARAM_TYPES_H
