#ifndef MODELSKELETONDATA_H
#define MODELSKELETONDATA_H

#ifdef _WIN32
#pragma once
#endif

#include "vphysics2/iphysicsbody.h"

// AMNOTE: Engine-owned model data; do not construct, copy or destroy it.
struct ModelSkeletonData_t
{
	enum BoneFlags_t : uint32
	{
		FLAG_NO_BONE_FLAGS = 0x0,
		FLAG_BONEFLEXDRIVER = 0x4,
		FLAG_CLOTH = 0x8,
		FLAG_PHYSICS = 0x10,
		FLAG_ATTACHMENT = 0x20,
		FLAG_ANIMATION = 0x40,
		FLAG_MESH = 0x80,
		FLAG_HITBOX = 0x100,
		FLAG_BONE_USED_BY_VERTEX_LOD0 = 0x400,
		FLAG_BONE_USED_BY_VERTEX_LOD1 = 0x800,
		FLAG_BONE_USED_BY_VERTEX_LOD2 = 0x1000,
		FLAG_BONE_USED_BY_VERTEX_LOD3 = 0x2000,
		FLAG_BONE_USED_BY_VERTEX_LOD4 = 0x4000,
		FLAG_BONE_USED_BY_VERTEX_LOD5 = 0x8000,
		FLAG_BONE_USED_BY_VERTEX_LOD6 = 0x10000,
		FLAG_BONE_USED_BY_VERTEX_LOD7 = 0x20000,
		FLAG_BONE_MERGE_READ = 0x40000,
		FLAG_BONE_MERGE_WRITE = 0x80000,
		FLAG_ALL_BONE_FLAGS = 0xfffff,
		BLEND_PREALIGNED = 0x100000,
		FLAG_RIGIDLENGTH = 0x200000,
		FLAG_PROCEDURAL = 0x400000,
	};

	CUtlVector< CUtlString > m_boneName;
	CUtlVector< int16 > m_nParent;
	CUtlVector< float > m_boneSphere;
	CUtlVector< uint32 > m_nFlag;
	CUtlVector< Vector > m_bonePosParent;
	CUtlVector< QuaternionStorage > m_boneRotParent;
	CUtlVector< float > m_boneScaleParent;

private:
	ModelSkeletonData_t() = delete;
	ModelSkeletonData_t( const ModelSkeletonData_t & ) = delete;
	ModelSkeletonData_t &operator=( const ModelSkeletonData_t & ) = delete;
	~ModelSkeletonData_t() = delete;
};

#endif // MODELSKELETONDATA_H
