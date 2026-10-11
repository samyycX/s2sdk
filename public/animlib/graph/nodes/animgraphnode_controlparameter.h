#ifndef ANIMGRAPH_CONTROL_PARAMETER_NODES_H
#define ANIMGRAPH_CONTROL_PARAMETER_NODES_H

#ifdef _WIN32
#pragma once
#endif

#include "animlib/graph/animgraph_node.h"

#define DECLARE_NM_CONTROL_PARAMETER_NODE( name, base, type ) \
class name : public base \
{ \
public: \
	const type &GetCachedValue() const { return m_value; } \
private: \
	type m_value; \
}

DECLARE_NM_CONTROL_PARAMETER_NODE( CNmControlParameterBoolNode, CNmBoolValueNode, bool );
DECLARE_NM_CONTROL_PARAMETER_NODE( CNmControlParameterIDNode, CNmIDValueNode, CGlobalSymbol );
DECLARE_NM_CONTROL_PARAMETER_NODE( CNmControlParameterFloatNode, CNmFloatValueNode, float );
DECLARE_NM_CONTROL_PARAMETER_NODE( CNmControlParameterVectorNode, CNmVectorValueNode, Vector );
DECLARE_NM_CONTROL_PARAMETER_NODE( CNmControlParameterTargetNode, CNmTargetValueNode, CNmTarget );

#undef DECLARE_NM_CONTROL_PARAMETER_NODE

#endif
