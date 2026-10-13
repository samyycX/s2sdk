#ifndef VPHYSICS_CONSTRAINTS_H
#define VPHYSICS_CONSTRAINTS_H

#ifdef _WIN32
#pragma once
#endif

#include "mathlib/vectorws.h"
#include "vphysics2/iphysicsbody.h"

struct constraint_breakableparams_t
{
	float strength = 1.0f;
	float forceLimit = 0.0f;
	float torqueLimit = 0.0f;
	float bodyMassScale[2] = { 1.0f, 1.0f };
	bool isActive = true;
};

struct constraint_axislimit_t
{
	float flMinRotation = 0.0f;
	float flMaxRotation = 0.0f;
	float flMotorTargetAngSpeed = 0.0f;
	float flMotorMaxTorque = 0.0f;
};

struct constraint_hingeparams_t
{
	VectorWS worldPosition = VectorWS( 0.0f, 0.0f, 0.0f );
	Vector worldAxisDirection = Vector( 0.0f, 0.0f, 0.0f );
	constraint_axislimit_t hingeAxis;
	constraint_breakableparams_t constraint;
};

struct constraint_ballsocketparams_t
{
	// AMNOTE: Local coordinates include body scale; an absent endpoint uses world coordinates.
	Vector constraintPosition[2];
	constraint_breakableparams_t constraint;

	void Defaults()
	{
		constraintPosition[0].Init();
		constraintPosition[1].Init();
		constraint = constraint_breakableparams_t();
	}
};

struct constraint_lengthparams_t
{
	constraint_breakableparams_t constraint;
	// AMNOTE: Local coordinates include body scale; an absent endpoint uses world coordinates.
	Vector objectPosition[2];
	float totalLength;
	float minLength;

	void Defaults()
	{
		constraint = constraint_breakableparams_t();
		objectPosition[0].Init();
		objectPosition[1].Init();
		totalLength = 1.0f;
		minLength = 0.0f;
	}
	void InitWorldspace( const IPhysicsBody *pBodyA, const IPhysicsBody *pBodyB, const Vector &vWorldA, const Vector &vWorldB, bool bFixedLength );
};

inline void constraint_lengthparams_t::InitWorldspace( const IPhysicsBody *pBodyA, const IPhysicsBody *pBodyB, const Vector &vWorldA, const Vector &vWorldB, bool bFixedLength )
{
	objectPosition[0] = pBodyA ? pBodyA->WorldToLocal( vWorldA ) : vWorldA;
	objectPosition[1] = pBodyB ? pBodyB->WorldToLocal( vWorldB ) : vWorldB;
	const Vector delta = vWorldA - vWorldB;
	const fltx4 vX = _mm_set_ss( delta.x );
	const fltx4 vY = _mm_set_ss( delta.y );
	const fltx4 vZ = _mm_set_ss( delta.z );
	const fltx4 vLengthSquared = _mm_add_ss( _mm_add_ss( _mm_mul_ss( vZ, vZ ), _mm_mul_ss( vY, vY ) ), _mm_mul_ss( vX, vX ) );
	totalLength = FastSqrt( SubFloat( vLengthSquared, 0 ) );
	minLength = bFixedLength ? totalLength : 0.0f;
}

#endif // VPHYSICS_CONSTRAINTS_H
