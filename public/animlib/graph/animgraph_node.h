#ifndef ANIMGRAPH_NODE_H
#define ANIMGRAPH_NODE_H

#ifdef _WIN32
#pragma once
#endif

#include "animlib/graph/animgraph_types.h"

class CNmGraphContext;

// Nodes are owned by the graph instance.
class CNmGraphNode
{
public:
	virtual ~CNmGraphNode() = default;

	class CDefinition;

	virtual bool IsValid() const = 0;
	virtual NmGraphValueType_t GetValueType() const = 0;
	virtual void Initialize( CNmGraphContext &context ) = 0;
	// AMNOTE: The recording-state type is not recovered.
	virtual void RecordGraphState( void *pState ) const = 0;
	virtual bool RestoreGraphState( const void *pState ) = 0;

	const CDefinition *GetDefinition() const { return m_pDefinition; }
	bool IsInitialized() const { return m_nInitializationCount > 0; }
	void Shutdown( CNmGraphContext &context );

protected:
	CNmGraphNode() = delete;
	CNmGraphNode( const CNmGraphNode & ) = delete;
	CNmGraphNode &operator=( const CNmGraphNode & ) = delete;
	virtual void InitializeInternal( CNmGraphContext &context ) = 0;
	virtual void ShutdownInternal( CNmGraphContext &context ) = 0;

	const CDefinition *m_pDefinition;
	int m_nLastUpdateID;
	int m_nInitializationCount;
};

class CNmValueNode : public CNmGraphNode
{
public:
	virtual void ResetValue() = 0;

protected:
	virtual void GetValueInternal( CNmGraphContext &context, void *pValue ) = 0;
	virtual void SetValueInternal( const void *pValue ) = 0;
};

#define DECLARE_NM_VALUE_NODE( name, type, valueType ) \
class name : public CNmValueNode \
{ \
public: \
	NmGraphValueType_t GetValueType() const final { return NmGraphValueType_t::valueType; } \
	type GetValue( CNmGraphContext &context ) { type value; GetValueInternal( context, &value ); return value; } \
	void SetValue( const type &value ) { SetValueInternal( &value ); } \
}

DECLARE_NM_VALUE_NODE( CNmBoolValueNode, bool, Bool );
DECLARE_NM_VALUE_NODE( CNmIDValueNode, CGlobalSymbol, ID );
DECLARE_NM_VALUE_NODE( CNmFloatValueNode, float, Float );
DECLARE_NM_VALUE_NODE( CNmVectorValueNode, Vector, Vector );
DECLARE_NM_VALUE_NODE( CNmTargetValueNode, CNmTarget, Target );

#undef DECLARE_NM_VALUE_NODE

inline void CNmGraphNode::Shutdown( CNmGraphContext &context )
{
	if ( m_nInitializationCount-- == 1 )
		ShutdownInternal( context );
}

#endif
