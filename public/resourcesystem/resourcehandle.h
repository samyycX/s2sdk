#ifndef RESOURCEHANDLE_H
#define RESOURCEHANDLE_H

#ifdef COMPILER_MSVC
#pragma once
#endif

#include "resourcefile/resourcetype.h"
#include "resourcesystem/resourcesystemtypes.h"

template <class T> struct ResourceBinding_t;

class CWeakHandleBase
{
protected:
	ResourceHandle_t m_pBinding = nullptr;
};

// AMNOTE: Borrows the resource binding; keep its owner alive while using the handle. A weak handle does not keep resource data resident.
template <class T>
class CWeakHandle : public CWeakHandleBase
{
public:
	using ResourceHandleTyped_t = const ResourceBinding_t<T> *;

	CWeakHandle() = default;
	explicit CWeakHandle( ResourceHandleTyped_t hResource ) { Init( hResource ); }
	void Init( ResourceHandleTyped_t hResource ) { m_pBinding = reinterpret_cast<ResourceHandle_t>( hResource ); }
	void InitUntypedUnchecked( ResourceHandle_t hResource ) { m_pBinding = hResource; }
	static CWeakHandle FromUntypedHandleUnchecked( ResourceHandle_t hResource )
	{
		CWeakHandle handle;
		handle.InitUntypedUnchecked( hResource );
		return handle;
	}
	ResourceHandleTyped_t GetBinding() const { return reinterpret_cast<ResourceHandleTyped_t>( m_pBinding ); }
	ResourceHandle_t GetResourceBindingBase() const { return m_pBinding; }
	bool HasHandle() const { return m_pBinding != nullptr; }
	bool HasIdentity() const { return m_pBinding && m_pBinding->m_Name != nullptr; }
	bool HasData() const { return m_pBinding && m_pBinding->m_pData != nullptr; }
	bool IsError() const { return m_pBinding && ( m_pBinding->m_nFlags & RESOURCE_BINDING_ERROR ) != 0; }
	int GetRefCount() const { return m_pBinding ? m_pBinding->m_nRefCount() : 0; }
	int GetReloadCounter() const { return m_pBinding ? m_pBinding->m_nReloadCounter : 0; }
	operator ResourceHandle_t() const { return m_pBinding; }
	explicit operator bool() const { return HasData(); }
	bool operator==( const CWeakHandle &other ) const { return m_pBinding == other.m_pBinding; }
	bool operator!=( const CWeakHandle &other ) const { return m_pBinding != other.m_pBinding; }
};

struct ResourceSerialHandle_t
{
	ResourceSerialHandle_t() : m_Value( 0 ) {}
	ResourceSerialHandle_t( uint64 nValue ) : m_Value( nValue ) {}

	uint64 m_Value;
};

abstract_class IResourceHandleUtils
{
public:
	virtual ResourceHandle_t FindOrRegisterResourceByName( const CResourceName &name ) = 0;
	virtual ResourceHandle_t FindResourceById( ResourceId_t id, ResourceType_t nType ) = 0;
	virtual void DeleteResource( ResourceHandle_t hResource ) = 0;
	virtual void ResourceReferenceLeakTracking_AddRef( ResourceHandle_t hResource, ResourceLeakTrackingGroup_t eGroup, uintp nContext ) = 0;
	virtual void ResourceReferenceLeakTracking_Release( ResourceHandle_t hResource, ResourceLeakTrackingGroup_t eGroup, uintp nContext ) = 0;
	virtual void ResourceReferenceLeakTracking_ReportReferences( ResourceHandle_t hResource ) = 0;
	virtual ResourceStatus_t GetResourceStatus( ResourceHandle_t hResource ) = 0;
	virtual ResourceType_t GetResourceHandleType( ResourceHandle_t hResource ) = 0;
	// AMNOTE: Runs the handle allocator self-test
	virtual void unk101() = 0;
	// AMNOTE: Returns hResource as a serial handle
	virtual ResourceSerialHandle_t unk102( ResourceHandle_t hResource ) = 0;
	virtual ResourceHandle_t HandleUtils_Deref( ResourceSerialHandle_t hSerial ) = 0;
};

#endif // RESOURCEHANDLE_H
