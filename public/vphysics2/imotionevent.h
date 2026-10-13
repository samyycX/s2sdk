#ifndef IMOTIONEVENT_H
#define IMOTIONEVENT_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"

class IPhysicsBody;
class IPhysicsMotionController;

// AMNOTE: Register through IVPhysics2World::CreateMotionController; the handler must outlive its controller.
abstract_class IMotionEvent
{
public:
	// AMNOTE: Begins an update before the attached bodies are processed.
	virtual void PreSimulate() = 0;
	// AMNOTE: Completes an update; deltaTime may accumulate several steps when asynchronous simulation is enabled.
	virtual void PostSimulate( float deltaTime ) = 0;
	// AMNOTE: Updates an attached, awake dynamic body. Apply forces or impulses through the body interface.
	virtual void Simulate( IPhysicsMotionController *pController, IPhysicsBody *pBody, float deltaTime ) = 0;
	// AMNOTE: Updates an attached body that is sleeping or is not dynamic.
	virtual void SimulateInactive( IPhysicsMotionController *pController, IPhysicsBody *pBody, float deltaTime ) = 0;
	// AMNOTE: An additional parameterless handler hook; its purpose remains unresolved.
	virtual void unk001() = 0;
	// AMNOTE: True permits delayed updates on a worker thread; all simulation callbacks must support that execution context.
	virtual bool ShouldSimulateAsync() const = 0;

protected:
	~IMotionEvent() = default;
};

#endif // IMOTIONEVENT_H
