#ifndef PHYSAGGREGATEDATA_H
#define PHYSAGGREGATEDATA_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"

struct VPhysXConstraintParams_t;

// AMNOTE: The parameter pointer is borrowed from the owning aggregate resource.
struct CPhysConstraintData
{
	uint32 m_nFlags;
	uint16 m_nParent;
	uint16 m_nChild;
	const VPhysXConstraintParams_t *m_pParams;
};

#endif // PHYSAGGREGATEDATA_H
