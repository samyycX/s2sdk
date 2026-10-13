#ifndef IPHYSICSMOTIONCONTROLLER_H
#define IPHYSICSMOTIONCONTROLLER_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "schemasystem/minschema.h"
#include "vphysics2/imotionevent.h"
#include "vphysics2/iphysicsbody.h"
#include "tier0/dbg.h"
#include "tier0/utlvector.h"

class IPhysicsBody;
class IVPhysics2World;

// AMNOTE: Engine-owned; create and destroy through IVPhysics2World. Do not implement or delete it.
// AMNOTE: Finish asynchronous work before changing its handler or attached objects, or destroying the controller.
abstract_class IPhysicsMotionController
{
public:
	// Returns the engine-owned schema class binding for this interface.
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

protected:
	virtual ~IPhysicsMotionController() = default;

public:
	// AMNOTE: Replaces the handler and refreshes its asynchronous-simulation setting. Keep the handler alive until replacement or destruction.
	virtual void SetEventHandler( IMotionEvent *handler ) = 0;

private:
	// AMNOTE: Forwards to the handler's additional parameterless hook; its purpose remains unresolved.
	virtual void unk001() = 0;

public:
	// AMNOTE: Updates attached bodies immediately, or accumulates deltaTime for asynchronous processing and returns false.
	virtual bool Simulate( float deltaTime ) = 0;
	// AMNOTE: Clears the accumulated delta time, then updates attached bodies when asynchronous simulation is enabled.
	virtual void SimulateAsync() = 0;
	// AMNOTE: Null objects are ignored. With checkIfAlreadyAttached, an existing attachment is retained without adding a duplicate.
	virtual void AttachObject( IPhysicsBody *pObject, bool checkIfAlreadyAttached ) = 0;
	// AMNOTE: Removes the object from the controller and the controller from the object's attachment list.
	virtual void DetachObject( IPhysicsBody *pObject ) = 0;
	// AMNOTE: Checks whether the object is in this controller's attachment list.
	virtual bool IsObjectAttached( const IPhysicsBody *pObject ) const = 0;
	// AMNOTE: Returns the number of attached object pointers.
	virtual int CountObjects() const = 0;
	// AMNOTE: pObjectList must have room for at least CountObjects() pointers.
	virtual void GetObjects( IPhysicsBody **pObjectList ) const = 0;
	// AMNOTE: Detaches all objects while retaining the controller and its handler.
	virtual void ClearObjects() = 0;
	// AMNOTE: Wakes all attached bodies.
	virtual void WakeObjects() = 0;
};

// AMNOTE: Engine-owned adapter layout; the native schema-binding override remains supplied by the engine.
class CPhysicsMotionController : public IPhysicsMotionController
{
public:
	void SetEventHandler( IMotionEvent *handler ) override
	{
		m_EventHandler = handler;
		m_SimulateAsync = handler && handler->ShouldSimulateAsync();
	}

	bool Simulate( float deltaTime ) override;
	void SimulateAsync() override;
	void AttachObject( IPhysicsBody *pObject, bool checkIfAlreadyAttached ) override;
	void DetachObject( IPhysicsBody *pObject ) override;
	bool IsObjectAttached( const IPhysicsBody *pObject ) const override;
	int CountObjects() const override { return m_ObjectList.Count(); }
	void GetObjects( IPhysicsBody **pObjectList ) const override;
	void ClearObjects() override;
	void WakeObjects() override;

private:
	CPhysicsMotionController() = delete;
	CPhysicsMotionController( const CPhysicsMotionController & ) = delete;
	CPhysicsMotionController &operator=( const CPhysicsMotionController & ) = delete;
	~CPhysicsMotionController() override { ClearObjects(); }

	void unk001() override { if ( m_EventHandler ) m_EventHandler->unk001(); }
	void SimulateObjects( float deltaTime );

	IVPhysics2World *m_World;
	IMotionEvent *m_EventHandler;
	CUtlVector<IPhysicsBody *> m_ObjectList;
	bool m_SimulateAsync;
	float m_AccumulatedDeltaTime;
};

inline bool CPhysicsMotionController::Simulate( float deltaTime )
{
	if ( m_SimulateAsync )
	{
		m_AccumulatedDeltaTime = deltaTime + m_AccumulatedDeltaTime;
		return false;
	}

	SimulateObjects( deltaTime );
	return true;
}

inline void CPhysicsMotionController::SimulateAsync()
{
	const bool bSimulateAsync = m_SimulateAsync;
	const float deltaTime = m_AccumulatedDeltaTime;
	m_AccumulatedDeltaTime = 0.0f;
	if ( bSimulateAsync )
		SimulateObjects( deltaTime );
}

inline void CPhysicsMotionController::SimulateObjects( float deltaTime )
{
	IMotionEvent *pEventHandler = m_EventHandler;
	if ( !pEventHandler )
		return;

	pEventHandler->PreSimulate();
	for ( int i = m_ObjectList.Count() - 1; i >= 0; --i )
	{
		IPhysicsBody *pObject = m_ObjectList[i];
		if ( !pObject )
		{
			m_ObjectList.FastRemove( i );
			continue;
		}

		CPhysicsBody *pBody = static_cast<CPhysicsBody *>( pObject );
		if ( pBody->m_Body && !pObject->IsAsleep() && pObject->GetMotionType() == BODY_DYNAMIC )
			m_EventHandler->Simulate( this, pObject, deltaTime );
		else
			m_EventHandler->SimulateInactive( this, pObject, deltaTime );
	}
	m_EventHandler->PostSimulate( deltaTime );
}

inline void CPhysicsMotionController::AttachObject( IPhysicsBody *pObject, bool checkIfAlreadyAttached )
{
	if ( !pObject )
		return;

	if ( checkIfAlreadyAttached && IsObjectAttached( pObject ) )
	{
		DevMsg( 1, "AttachObject called on motion controller with this object already attached.\n" );
		return;
	}

	m_ObjectList.AddToTail( pObject );
	CPhysicsBody *pBody = static_cast<CPhysicsBody *>( pObject );
	if ( !pBody->m_MotionControllers.HasElement( this ) )
		pBody->m_MotionControllers.AddToHead( this );
}

inline void CPhysicsMotionController::DetachObject( IPhysicsBody *pObject )
{
	if ( !pObject )
		return;

	m_ObjectList.FindAndRemove( pObject );
	static_cast<CPhysicsBody *>( pObject )->m_MotionControllers.FindAndRemove( this );
}

inline bool CPhysicsMotionController::IsObjectAttached( const IPhysicsBody *pObject ) const
{
	for ( int i = 0; i < m_ObjectList.Count(); ++i )
	{
		if ( m_ObjectList[i] == pObject )
			return true;
	}
	return false;
}

inline void CPhysicsMotionController::GetObjects( IPhysicsBody **pObjectList ) const
{
	for ( int i = 0; i < m_ObjectList.Count(); ++i )
		pObjectList[i] = m_ObjectList[i];
}

inline void CPhysicsMotionController::ClearObjects()
{
	for ( IPhysicsBody *pObject : m_ObjectList )
		static_cast<CPhysicsBody *>( pObject )->m_MotionControllers.FindAndRemove( this );
	m_ObjectList.RemoveAll();
}

inline void CPhysicsMotionController::WakeObjects()
{
	for ( int i = 0; i < m_ObjectList.Count(); ++i )
		m_ObjectList[i]->Wake();
}

#endif // IPHYSICSMOTIONCONTROLLER_H
