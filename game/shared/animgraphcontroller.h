#ifndef ANIMGRAPHCONTROLLER_H
#define ANIMGRAPHCONTROLLER_H

#ifdef _WIN32
#pragma once
#endif

#include "animlib/graph/animgraph_instance.h"

template< typename T > struct CAnimGraph2ParamType;

#define DECLARE_ANIMGRAPH2_PARAM_TYPE( type, nodeType, valueType ) \
template<> struct CAnimGraph2ParamType< type > \
{ \
	typedef nodeType Node_t; \
	static constexpr NmGraphValueType_t ValueType = NmGraphValueType_t::valueType; \
	static bool Read( const Node_t *pNode, type &value ) { value = pNode->GetCachedValue(); return true; } \
	static void Write( Node_t *pNode, const type &value ) { pNode->SetValue( value ); } \
}

DECLARE_ANIMGRAPH2_PARAM_TYPE( bool, CNmControlParameterBoolNode, Bool );
DECLARE_ANIMGRAPH2_PARAM_TYPE( CGlobalSymbol, CNmControlParameterIDNode, ID );
DECLARE_ANIMGRAPH2_PARAM_TYPE( float, CNmControlParameterFloatNode, Float );
DECLARE_ANIMGRAPH2_PARAM_TYPE( Vector, CNmControlParameterVectorNode, Vector );
DECLARE_ANIMGRAPH2_PARAM_TYPE( CNmTarget, CNmControlParameterTargetNode, Target );

#undef DECLARE_ANIMGRAPH2_PARAM_TYPE

template<> struct CAnimGraph2ParamType< CTransform >
{
	typedef CNmControlParameterTargetNode Node_t;
	static constexpr NmGraphValueType_t ValueType = NmGraphValueType_t::Target;
	static bool Read( const Node_t *pNode, CTransform &value );
	static void Write( Node_t *pNode, const CTransform &value ) { pNode->SetValue( CNmTarget( value ) ); }
};

template< typename T >
// Unbind before the graph instance is destroyed or replaced.
class CAnimGraph2ParamRefBase
{
public:
	const CGlobalSymbol &GetParameterID() const { return m_paramID; }
	int16 GetParameterIndex() const { return m_nParamIndex; }
	bool IsBound() const { return m_pGraphInstance != NULL && m_nParamIndex >= 0; }
	bool WasBindingAttempted() const { return m_bBindingAttempted; }
	bool Bind( CNmGraphInstance *pGraphInstance );
	void Unbind() { m_pGraphInstance = NULL; m_nParamIndex = -1; m_bBindingAttempted = false; }
	bool TryGetValue( T &value ) const;
	bool SetValue( const T &value );

protected:
	CAnimGraph2ParamRefBase( const CGlobalSymbol &id, bool bFlag )
		: m_paramID( id ), m_pGraphInstance( NULL ), m_nParamIndex( -1 ), m_unk101( bFlag ), m_bBindingAttempted( false ) {}

private:
	CGlobalSymbol m_paramID;
	CNmGraphInstance *m_pGraphInstance;
	int16 m_nParamIndex;
	// AMNOTE: Optional-reference constructors set this flag; its other uses are unknown.
	bool m_unk101;
	bool m_bBindingAttempted;
};

template< typename T >
class CAnimGraph2ParamOptionalRef : public CAnimGraph2ParamRefBase< T >
{
public:
	explicit CAnimGraph2ParamOptionalRef( const CGlobalSymbol &id ) : CAnimGraph2ParamRefBase< T >( id, true ) {}
	explicit CAnimGraph2ParamOptionalRef( const char *pName ) : CAnimGraph2ParamOptionalRef( MakeGlobalSymbol( pName ) ) {}
};

inline bool CAnimGraph2ParamType< CTransform >::Read( const Node_t *pNode, CTransform &value )
{
	const CNmTarget &target = pNode->GetCachedValue();
	if ( !target.IsSet() || target.IsBoneTarget() )
		return false;
	value = target.m_transform;
	return true;
}

template< typename T >
inline bool CAnimGraph2ParamRefBase< T >::Bind( CNmGraphInstance *pGraphInstance )
{
	Assert( m_nParamIndex < 0 );
	if ( m_nParamIndex >= 0 )
		return false;
	m_bBindingAttempted = true;
	m_pGraphInstance = pGraphInstance;
	if ( !pGraphInstance )
		return false;
	int16 nIndex = pGraphInstance->FindControlParameterIndex( m_paramID );
	CNmGraphNode *pNode = pGraphInstance->GetNode( nIndex );
	if ( !pNode || pNode->GetValueType() != CAnimGraph2ParamType< T >::ValueType )
		return false;
	m_nParamIndex = nIndex;
	return true;
}

template< typename T >
inline bool CAnimGraph2ParamRefBase< T >::TryGetValue( T &value ) const
{
	if ( !IsBound() )
		return false;
	CNmGraphNode *pNode = m_pGraphInstance->GetNode( m_nParamIndex );
	return pNode && CAnimGraph2ParamType< T >::Read( static_cast< typename CAnimGraph2ParamType< T >::Node_t * >( pNode ), value );
}

template< typename T >
inline bool CAnimGraph2ParamRefBase< T >::SetValue( const T &value )
{
	if ( !IsBound() )
		return false;
	CNmGraphNode *pNode = m_pGraphInstance->GetNode( m_nParamIndex );
	if ( !pNode )
		return false;
	CAnimGraph2ParamType< T >::Write( static_cast< typename CAnimGraph2ParamType< T >::Node_t * >( pNode ), value );
	return true;
}

#endif
