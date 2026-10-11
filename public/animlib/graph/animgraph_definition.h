#ifndef ANIMGRAPH_DEFINITION_H
#define ANIMGRAPH_DEFINITION_H

#ifdef _WIN32
#pragma once
#endif

#include "animlib/graph/animgraph_types.h"
#include "tier0/utlhashtable.h"
#include "tier0/utlvector.h"
#include "tier0/utlstring.h"

class CNmGraphVariationUserData;

// AMNOTE: Resource-handle members and remaining runtime metadata are opaque.
class CNmGraphDefinition
{
public:
	struct ReferencedGraphSlot_t
	{
		int16 m_nNodeIdx;
		int16 m_dataSlotIdx;
	};

	struct ExternalGraphSlot_t
	{
		int16 m_nNodeIdx;
		CGlobalSymbol m_slotID;
	};

	struct ExternalPoseSlot_t
	{
		int16 m_nNodeIdx;
		CGlobalSymbol m_slotID;
	};

	const CUtlVector< CGlobalSymbol > &GetControlParameterIDs() const { return m_controlParameterIDs; }
	int16 FindControlParameterIndex( const CGlobalSymbol &id ) const;

private:
	CNmGraphDefinition() = delete;
	CNmGraphDefinition( const CNmGraphDefinition & ) = delete;
	CNmGraphDefinition &operator=( const CNmGraphDefinition & ) = delete;
	~CNmGraphDefinition() = delete;

	CGlobalSymbol m_variationID;
	// AMNOTE: Skeleton resource handles are not exposed.
	uint8 m_unk101[32];
	CNmGraphVariationUserData *m_pUserData;
	CUtlVector< int16 > m_persistentNodeIndices;
	int16 m_nRootNodeIdx;
	CUtlVector< CGlobalSymbol > m_controlParameterIDs;
	CUtlVector< CGlobalSymbol > m_virtualParameterIDs;
	CUtlVector< int16 > m_virtualParameterNodeIndices;
	CUtlVector< CNmGraphDefinition::ReferencedGraphSlot_t > m_referencedGraphSlots;
	CUtlVector< CNmGraphDefinition::ExternalGraphSlot_t > m_externalGraphSlots;
	CUtlVector< CNmGraphDefinition::ExternalPoseSlot_t > m_externalPoseSlots;
	CUtlHashtable< CGlobalSymbol, int16 > m_controlParameterLookup;
	// AMNOTE: Other lookup tables and node-definition storage are not exposed.
	uint8 m_unk201[80];
	CUtlVector< CUtlString > m_nodePaths;
	// AMNOTE: Resource storage and remaining runtime metadata are not exposed.
	uint8 m_unk301[80];
};

inline int16 CNmGraphDefinition::FindControlParameterIndex( const CGlobalSymbol &id ) const
{
	const int16 *pIndex = m_controlParameterLookup.GetPtr( id );
	return pIndex ? *pIndex : -1;
}

#endif
