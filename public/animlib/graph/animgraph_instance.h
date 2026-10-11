#ifndef ANIMGRAPH_INSTANCE_H
#define ANIMGRAPH_INSTANCE_H

#ifdef _WIN32
#pragma once
#endif

#include "animlib/graph/animgraph_definition.h"
#include "animlib/graph/nodes/animgraphnode_controlparameter.h"
#include "resourcefile/resourcetype.h"
#include "tier0/utlbinaryblock.h"

// AMNOTE: Evaluation and debug members remain opaque.
class ALIGN16 CNmGraphInstance
{
public:
	const CNmGraphDefinition *GetDefinition() const { return m_hGraphDefinition ? static_cast< const CNmGraphDefinition * >( m_hGraphDefinition->m_pData ) : NULL; }
	int GetNodeCount() const { return m_nodes.Count(); }
	CNmGraphNode *GetNode( int16 nIndex ) const { return m_nodes.IsValidIndex( nIndex ) ? m_nodes[nIndex] : NULL; }
	CNmGraphNode *GetRootNode() const { return m_pRootNode; }
	CNmGraphContext &GetContext() { return *reinterpret_cast< CNmGraphContext * >( m_unk203 ); }
	const CNmGraphContext &GetContext() const { return *reinterpret_cast< const CNmGraphContext * >( m_unk203 ); }
	int16 FindControlParameterIndex( const CGlobalSymbol &id ) const;

private:
	CNmGraphInstance() = delete;
	CNmGraphInstance( const CNmGraphInstance & ) = delete;
	CNmGraphInstance &operator=( const CNmGraphInstance & ) = delete;
	~CNmGraphInstance() = delete;

	ResourceHandle_t m_hGraphDefinition;
	CUtlVector< CNmGraphNode * > m_nodes;
	CUtlBinaryBlock m_nodeMemory;
	CNmGraphNode *m_pRootNode;
	uint32 m_unk101;
	bool m_bIsPrimaryGraph;
	bool m_unk201;
	bool m_unk202;
	// AMNOTE: Context layout, task storage and debug state are not exposed.
	ALIGN16 uint8 m_unk203[880] ALIGN16_POST;
} ALIGN16_POST;

inline int16 CNmGraphInstance::FindControlParameterIndex( const CGlobalSymbol &id ) const
{
	const CNmGraphDefinition *pDefinition = GetDefinition();
	return pDefinition ? pDefinition->FindControlParameterIndex( id ) : -1;
}

#endif
