#ifndef ANIMGRAPH_TYPES_H
#define ANIMGRAPH_TYPES_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/globalsymbol.h"
#include "mathlib/transform.h"

enum class NmGraphValueType_t : uint8
{
	Unknown = 0,
	Bool = 1,
	ID = 2,
	Float = 3,
	Vector = 4,
	Target = 5,
	BoneMask = 6,
	Pose = 7,
	Special = 8,
};

class ALIGN16 CNmTarget
{
public:
	CNmTarget();
	explicit CNmTarget( const CTransform &transform );

	bool IsSet() const { return m_bIsSet; }
	bool IsBoneTarget() const { return m_bIsBoneTarget; }
	void Reset() { m_bIsSet = false; }

public:
	CTransform m_transform;
	CGlobalSymbol m_boneID;
	bool m_bIsBoneTarget;
	bool m_bIsUsingBoneSpaceOffsets;
	bool m_bHasOffsets;
	bool m_bIsSet;
} ALIGN16_POST;

inline CNmTarget::CNmTarget()
	: m_bIsBoneTarget( false ), m_bIsUsingBoneSpaceOffsets( true ),
	  m_bHasOffsets( false ), m_bIsSet( false )
{
	m_transform.m_vPosition.Invalidate();
	m_transform.m_vPosition.w = VEC_T_NAN;
	m_transform.m_orientation.Invalidate();
}

inline CNmTarget::CNmTarget( const CTransform &transform )
	: m_transform( transform ), m_bIsBoneTarget( false ),
	  m_bIsUsingBoneSpaceOffsets( true ), m_bHasOffsets( false ), m_bIsSet( true )
{
}

#endif
