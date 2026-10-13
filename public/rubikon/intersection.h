#ifndef RUBIKON_INTERSECTION_H
#define RUBIKON_INTERSECTION_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"

typedef int32 CollisionGroupIndex;
typedef uint16 IntersectionLayerPairFlags;

enum OverlapTest_t : int
{
	OVERLAP_TEST_PROXY = 0,
	OVERLAP_TEST_SHAPE = 1,
};

#endif // RUBIKON_INTERSECTION_H
