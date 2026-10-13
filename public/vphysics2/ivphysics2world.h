#ifndef IVPHYSICS2WORLD_H
#define IVPHYSICS2WORLD_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/threadtools.h"
#include "tier0/jobthread.h"
#include "tier0/utlvector.h"
#include "tier0/utlleanvector.h"
#include "schemasystem/minschema.h"
#include "tier0/utldelegate.h"
#include "mathlib/vector.h"
#include "mathlib/camera.h"
#include "mathlib/transform.h"
#include "vphysics2/physicscast.h"
#include "vphysics2/imotionevent.h"
#include "vphysics2/iphysicsmotioncontroller.h"
#include "vphysics2/iphysicsquerycache.h"
#include "vphysics2/iphysicscharacterproxy.h"
#include "vphysics2/iphysicsrope.h"
#include "vphysics2/iphysicsparticlerope.h"
#include "vphysics2/iphysicsbody.h"
#include "vphysics2/iphysicssoftbody.h"
#include "vphysics2/iphysicsjoint.h"
#include <float.h>
#include <functional>

// AMNOTE: Known debug layer bits; higher flag meanings remain unresolved.
#define VPHYS2_DEBUG_DRAW_LAYER_STATIC_BODIES 0x0000000000000001ull
#define VPHYS2_DEBUG_DRAW_LAYER_KEYFRAMED_BODIES 0x0000000000000002ull
#define VPHYS2_DEBUG_DRAW_LAYER_DYNAMIC_BODIES 0x0000000000000004ull
#define VPHYS2_DEBUG_DRAW_LAYER_CONTACT_POINTS 0x0000000000000008ull
#define VPHYS2_DEBUG_DRAW_LAYER_CONTACT_MANIFOLDS 0x0000000000000010ull
#define VPHYS2_DEBUG_DRAW_LAYER_CONTACT_INDICES 0x0000000000000020ull
#define VPHYS2_DEBUG_DRAW_LAYER_CONTACTS 0x0000000000000040ull
#define VPHYS2_DEBUG_DRAW_LAYER_JOINTS 0x0000000000000080ull
#define VPHYS2_DEBUG_DRAW_LAYER_PROXIES 0x0000000000000100ull
#define VPHYS2_DEBUG_DRAW_LAYER_RECORDED_RAYCASTS 0x0000000000000200ull
#define VPHYS2_DEBUG_DRAW_LAYER_RECORDED_FORCES 0x0000000000000400ull
#define VPHYS2_DEBUG_DRAW_LAYER_INERTIA_BOXES 0x0000000000000800ull
#define VPHYS2_DEBUG_DRAW_LAYER_MESH_FACE_INDICES 0x0000000000001000ull
#define VPHYS2_DEBUG_DRAW_LAYER_MESH_VERTEX_INDICES 0x0000000000002000ull
#define VPHYS2_DEBUG_DRAW_LAYER_MESH_EDGE_INDICES 0x0000000000004000ull
#define VPHYS2_DEBUG_DRAW_LAYER_AWAKE_BODIES 0x0000000000008000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODIES 0x0000000000010000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_INDICES 0x0000000000020000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_FIELDS 0x0000000000040000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_COLLISION 0x0000000000080000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_BASES 0x0000000000100000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_ANIM_BASES 0x0000000000200000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SOFTBODY_NODE_NAMES 0x0000000000400000ull
#define VPHYS2_DEBUG_DRAW_LAYER_SHAPE_EDGES 0x0000000000800000ull

class IPhysicsBody;
class IPhysicsSoftbody;
class IPhysicsRope;
class IPhysicsVehicle;
class IPhysicsCharacterProxy;
class IPhysicsMotionController;
class IPhysAggregateInstance;
class IPhysicsRagdoll;
class IPhysicsRagdollControl;
class IVPhysics2World;
class CBufferString;
class CCommand;
struct Ray_t;
struct RnQueryAttr_t;

struct CPhysicsRagdollWeight
{
	float m_unk001;
	float m_unk002;
	bool m_unk003;
};

// AMNOTE: Engine-owned; do not construct, implement or delete it. Finish pending update tasks before modifying or destroying its bodies and joints.
abstract_class IPhysicsRagdollControl
{
public:
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

protected:
	virtual ~IPhysicsRagdollControl() = default;

public:
	virtual IVPhysics2World *GetWorld() const = 0;
	virtual IPhysicsRagdoll *GetRagdoll() const = 0;
	virtual float GetMinSpringFrequency() const = 0;
	virtual void SetMinSpringFrequency( float flFrequency ) = 0;
	virtual float GetMaxSpringFrequency() const = 0;
	virtual void SetMaxSpringFrequency( float flFrequency ) = 0;
	// AMNOTE: Native construction does not initialize this value; set it before updating joints.
	virtual float GetMaxStretch() const = 0;
	// AMNOTE: Initialize this limit before enabling stretch correction.
	virtual void SetMaxStretch( float flStretch ) = 0;
	virtual void SetSolidCollisionAtZeroWeight( bool bEnable ) = 0;
	virtual void SetRequiresDynamicBodies( bool bRequire ) = 0;
	virtual void RemoveControlBody( int nIndex ) = 0;
	// AMNOTE: The task vector retains callbacks borrowing this controller and its bodies; execute and clear them before changing or destroying either.
	virtual void UpdateBodiesParallel( const CTransform *pSkeletonPose, float flDeltaTime, CUtlLeanVectorFixedGrowable<std::function<void()>, 100, int, CMemAllocAllocator> &tasks, CAtomicMutex &mutex ) = 0;
	// AMNOTE: The final boolean argument is unused.
	virtual void UpdateJointsParallel( int nBoneCount, CPhysicsRagdollWeight *pWeights, const int *pBoneIndices, const CTransform *pSkeletonPose, float flDeltaTime, bool bUnused, CUtlLeanVectorFixedGrowable<std::function<void()>, 100, int, CMemAllocAllocator> &tasks, CAtomicMutex &mutex ) = 0;
	virtual void ApplyTeleport( const CTransform &previousTransform, const CTransform &currentTransform ) = 0;
	virtual void SetIgnoreTeleport( bool bIgnore ) = 0;
	virtual void AddLinearVelocity( Vector velocity ) = 0;
	virtual void AddAngularVelocity( Vector velocity ) = 0;
	virtual void AddForce( Vector force ) = 0;
	virtual void ClearAccumulators() = 0;
	virtual const CUtlVector<IPhysicsBody *> &GetControlBodies() const = 0;
	virtual void SetStretchCorrectionEnabled( bool bEnable ) = 0;
};


typedef CUtlDelegate<void( float deltaTime )> FnVPhys2SimulationCallback_t;
typedef std::function<void()> FnVPhys2PostTickWork_t;

struct CSweepResult
{
	// A user-provided constructor preserves GNU base tail reuse with older compilers.
	constexpr CSweepResult() : m_pBody( nullptr ), m_pShape( nullptr ), m_flFraction( 1.0f ), m_nConvexIndex( -1 ), m_nTriangle( -1 ) {}

	IPhysicsBody *m_pBody;
	IPhysicsShape *m_pShape;
	float m_flFraction;
	// Compound geometry selection; includes spheres, capsules, hulls and meshes. Noncompound shapes use -1.
	int m_nConvexIndex;
	int m_nTriangle;
};

struct CLinearSweepResult : CSweepResult
{
	// Point and normal are not computed for an initial overlap.
	Vector m_vHitPoint{ 0.0f, 0.0f, 0.0f };
	Vector m_vHitNormal{ 0.0f, 0.0f, 0.0f };
};

struct VPhys2RayInterval_t
{
	Vector m_vNormal;
	float m_flFraction;
	IPhysicsBody *m_pBody;
	IPhysicsShape *m_pShape;
	int m_nSubShape;
	int m_nTriangle;
	// AMNOTE: Additional result index, initialized to -1; meaning unresolved.
	int m_unk001;
	int m_nBodyIndex;
	int16 m_nSurfacePropertyIndex;
	bool m_bExit;
	// AMNOTE: Additional result boolean, written as false by the collision paths; meaning unresolved.
	bool m_unk101;
};

class CRayIntervalVector : public CUtlVectorFixedGrowable<VPhys2RayInterval_t, 128>
{
public:
	// AMNOTE: Retained debug-recording flag; the query does not update it and its meaning remains unresolved.
	bool m_unk001 = false;
};

struct VPhys2EntityCollisionPair_t
{
	// AMNOTE: IDs are the entity/owner IDs used by collision attributes, not body indices.
	uint32 m_nEntityIdA;
	uint32 m_nEntityIdB;
};

struct VPhysXWorldStepResults
{
	// AMNOTE: Instances that acquired an awake body since the previous snapshot update.
	CUtlVector<IPhysAggregateInstance *> m_NewlyAwakeInstances;
	// AMNOTE: Instances that no longer have any awake bodies since the previous snapshot update.
	CUtlVector<IPhysAggregateInstance *> m_NewlySleepingInstances;
	// AMNOTE: All tracked instances with at least one awake body after the update.
	CUtlVector<IPhysAggregateInstance *> m_AwakeInstances;
};

class IVPhysics2World;
class CVPhys2World;
struct unkVPhys2ReflectionInfo_t;

// AMNOTE: Borrowed world callback; unregister before destroying the listener.
abstract_class IPhysicsWorldEventListener
{
public:
	// AMNOTE: Notified before replacing or clearing the world's debug draw filter. bUnk is false when clearing; its filter-specific meaning remains unresolved.
	virtual void OnDebugDrawFilterChange( IVPhysics2World *pWorld, bool bUnk ) = 0;

protected:
	~IPhysicsWorldEventListener() = default;
};

// AMNOTE: Engine-owned; obtain it through IVPhysics2World and keep it only while the world is alive.
abstract_class IPhysReflContext
{
public:
	// AMNOTE: Resolves a recorded reflection value into an opaque output record. World contexts return false and leave pInfo unchanged.
	virtual bool ResolveReflectionValue( uint64 nValue, unkVPhys2ReflectionInfo_t *pInfo ) const = 0;
	// AMNOTE: Returns the world that owns this context.
	virtual IVPhysics2World *GetWorld() const = 0;

private:
	IPhysReflContext() = default;
	IPhysReflContext( const IPhysReflContext & ) = delete;
	IPhysReflContext &operator=( const IPhysReflContext & ) = delete;
	~IPhysReflContext() = default;

	friend class CVPhys2World;
};

// AMNOTE: Unresolved virtual signatures are private placeholders.
// This is an engine-owned interface; do not implement or delete it.
abstract_class IVPhysics2World
{
public:
	// AMNOTE: Saves the calling thread identifier, advances the core world when deltaTime exceeds 1e-5, and updates the registered motion controllers. bNewStep refreshes the previous-time snapshot before advancing the core world.
	virtual void StepSimulation( float deltaTime, bool bNewStep ) = 0;
	// AMNOTE: Updates aggregate sleep/wake tracking and appends transitions and awake instances to pResults when nonnull; clear its vectors before a new collection.
	virtual void UpdateSerialStreamSnapshots( VPhysXWorldStepResults *pResults ) = 0;
	// AMNOTE: Simulates softbodies and their linked children, waiting for all scheduled jobs. deltaTime is the time per substep and nSubsteps must be positive.
	virtual void SimulateSoftbodies( float deltaTime, int nSubsteps ) = 0;
	// AMNOTE: Drains the four deferred deletion queues and the world cleanup queue.
	virtual void FlushQueuedDeletions() = 0;
	// AMNOTE: Returns the thread identifier saved by the step wrapper; uint32 on Windows, uintp on Linux.
	virtual ThreadId_t GetSimulationThreadId() const = 0;
	// AMNOTE: Drains the world's deferred deletion queues.
	virtual void CleanupDeleteList() = 0;
	// AMNOTE: Removes core physics objects, invalidates their adapters, drains deferred deletions, and resets the simulation state.
	virtual void ResetWorld() = 0;
	// AMNOTE: The callee updates registered per-step objects, advances the simulation clock, and increments its step counter.
	virtual void AdvanceSimulationTime( float deltaTime ) = 0;
	// AMNOTE: Runs collision detection and solving for a new simulation step.
	virtual void Simulate( float deltaTime ) = 0;

private:
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk001() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk002() = 0;
	// AMNOTE: Starts asynchronous motion-controller and rope updates; the unused leading argument's type remains unresolved.
	virtual void BeginAsyncPostTickJobs() = 0;

public:
	// AMNOTE: Waits for motion-controller and rope jobs, releases the job reference, and clears the asynchronous-work flag.
	virtual void FinishAsyncPostTickJobs() = 0;

public:
	// AMNOTE: Moves callback into the post-tick queue. Captured references must survive execution. The conditional queue is consumed only when the post-tick pass selects it.
	virtual void QueuePostTickWork( FnVPhys2PostTickWork_t &&callback, bool bConditionalQueue ) = 0;
	// AMNOTE: Copies pending vehicle simulation results and runs the registered vehicle update callbacks.
	virtual void UpdateVehicles() = 0;
	// AMNOTE: Returns the state set when post-tick jobs begin and cleared after those jobs finish.
	virtual bool IsAsyncPostTickWorkActive() const = 0;
	// AMNOTE: Updates broadphase overlaps and performs collision detection without running the complete solver step.
	virtual void Collide( float deltaTime ) = 0;

private:
	// AMNOTE: Processes broadphase objects against the configured world bounds; the complete signature remains unresolved.
	virtual void unk101() = 0;

public:
	// AMNOTE: Copies three floats into the world's gravity vector.
	virtual void SetGravity( const Vector &gravityVector ) = 0;
	// AMNOTE: Returns the world's gravity vector; the returned reference is owned by the world.
	virtual const Vector &GetGravity() const = 0;
	// AMNOTE: Sets the air density used by the bodies' aerodynamic drag calculation.
	virtual void SetAirDensity( float density ) = 0;
	// AMNOTE: Returns the air density used by the bodies' aerodynamic drag calculation.
	virtual float GetAirDensity() const = 0;
	// AMNOTE: Sets an unidentified integer world control, initially one.
	virtual void unk201( int nValue ) = 0;
	// AMNOTE: Reads the unidentified integer world control set by the preceding method.
	virtual int unk202() const = 0;
	// AMNOTE: Sets the world's position-solver iteration count without clamping; initially two. Each island uses the greater of this count and its own requested count.
	virtual void SetPositionSolverIterationCount( int nIterations ) = 0;
	// AMNOTE: Returns the world's position-solver iteration count.
	virtual int GetPositionSolverIterationCount() const = 0;
	// AMNOTE: Sets the world's velocity-solver iteration count without clamping; initially eight. Each island uses the greater of this count and its own requested count.
	virtual void SetVelocitySolverIterationCount( int nIterations ) = 0;
	// AMNOTE: Returns the world's velocity-solver iteration count.
	virtual int GetVelocitySolverIterationCount() const = 0;
	// AMNOTE: Restores the maximum number of time-of-impact body iterations to four.
	virtual void ResetTOISolverIterationLimit() = 0;
	// AMNOTE: Limits time-of-impact body iterations, clamping nIterations to zero through four. Zero skips that iteration loop.
	virtual void SetTOISolverIterationLimit( int nIterations ) = 0;
	// AMNOTE: Returns the maximum number of time-of-impact body iterations.
	virtual int GetTOISolverIterationLimit() const = 0;
	// AMNOTE: Scales cached contact and joint solver values.
	virtual void ScaleCachedConstraintData( float flScale ) = 0;
	// AMNOTE: Reads an unidentified broadphase tracking-mode flag.
	virtual bool unk301() const = 0;
	// AMNOTE: Changes that broadphase tracking mode and rebuilds its tracked-proxy list.
	virtual void unk302( bool bEnable ) = 0;
	// AMNOTE: Returns whether the configured thread limit permits more than one worker.
	virtual bool IsMultithreadingEnabled() const = 0;
	// AMNOTE: Enables or disables parallel work while retaining the magnitude of the configured thread limit.
	virtual void EnableMultithreading( bool bEnable ) = 0;
	// AMNOTE: Returns the stored thread limit; a negative value means parallel work is disabled.
	virtual int GetThreadCountLimit() const = 0;
	// AMNOTE: Clamps the limit to at least one and preserves the current enabled/disabled state.
	virtual void SetThreadCountLimit( int nThreadCount ) = 0;
	// AMNOTE: Sets the priority forwarded to the world's parallel jobs.
	virtual void SetJobPriority( JobPriority_t priority ) = 0;
	// AMNOTE: Returns the clock incremented by the simulation step.
	virtual float GetSimulationTime() const = 0;
	// AMNOTE: Returns the clock saved before a new step.
	virtual float GetPreviousSimulationTime() const = 0;
	// AMNOTE: Returns the same simulation clock as GetSimulationTime.
	virtual float GetCurrentSimulationTime() const = 0;
	// AMNOTE: Returns the counter incremented by both simulation-time advance and full simulation.
	virtual int GetSimulationTickCount() const = 0;
	// AMNOTE: Sets an unidentified integer world control, initially four.
	virtual void unk401( int nValue ) = 0;
	// AMNOTE: Reads the unidentified integer world control set by the preceding method.
	virtual int unk402() const = 0;
	// AMNOTE: Sets an unidentified integer world control, initially one.
	virtual void unk403( int nValue ) = 0;
	// AMNOTE: Reads the unidentified integer world control set by the preceding method.
	virtual int unk404() const = 0;
	// AMNOTE: Sets an unidentified integer world control, initially one.
	virtual void unk405( int nValue ) = 0;
	// AMNOTE: Reads the unidentified integer world control set by the preceding method.
	virtual int unk406() const = 0;

private:
	// AMNOTE: Stores an unidentified core-world pointer; its interface type is unresolved.
	virtual void unk407() = 0;

public:
	// AMNOTE: Changing this flag immediately sleeps eligible nonstatic bodies when enabled, or wakes them when disabled.
	virtual void EnableSleeping( bool bEnable ) = 0;
	// AMNOTE: Returns the global body-sleeping flag.
	virtual bool IsSleepingEnabled() const = 0;
	// AMNOTE: Counts world bodies whose sleeping flag is set.
	virtual int GetSleepingBodyCount() const = 0;
	// AMNOTE: Defers body wake and enable/disable changes into the world's update queue.
	virtual void BeginDeferredBodyUpdates() = 0;
	// AMNOTE: Applies queued body wake and enable/disable changes, then stops deferring them.
	virtual void EndDeferredBodyUpdates() = 0;
	// AMNOTE: Returns whether body wake and enable/disable changes are currently deferred.
	virtual bool AreBodyUpdatesDeferred() const = 0;
	// AMNOTE: Counts the body vector populated by both body creation methods.
	virtual int GetBodyCount() const = 0;
	// AMNOTE: nIndex must be within the corresponding object count.
	// AMNOTE: Returns the CPhysicsBody adapter from a body vector entry.
	virtual IPhysicsBody *GetBody( int nIndex ) = 0;
	// AMNOTE: Returns the indexed body with const access.
	virtual const IPhysicsBody *GetBody( int nIndex ) const = 0;
	// AMNOTE: pBody must be nonnull.
	// AMNOTE: Checks the body's index and identity against this world's body vector.
	virtual bool IsBodyInWorld( const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Creates a body at the origin with identity orientation and scale 1.
	virtual IPhysicsBody *CreateBody() = 0;
	// AMNOTE: Creates a body with the supplied position, orientation and scale.
	virtual IPhysicsBody *CreateBody( const Vector &vPosition, const Quaternion &qOrientation, float flScale ) = 0;
	// AMNOTE: Destruction is deferred; do not use pBody after successful removal.
	// AMNOTE: Unregisters the body and queues its adapter for deferred deletion.
	virtual void DestroyBody( IPhysicsBody *pBody ) = 0;
	// AMNOTE: Counts the awake, enabled nonstatic bodies in the world's active list.
	virtual int GetActiveBodyCount() const = 0;
	// AMNOTE: Appends active body adapters to bodies; existing entries are retained.
	virtual void GetActiveBodies( CUtlVector<IPhysicsBody *> &bodies ) const = 0;

private:
	// AMNOTE: Creates and registers a softbody adapter; the creation descriptor remains unresolved.
	virtual void CreateSoftbody() = 0;

public:
	// AMNOTE: Deletes the softbody adapter, removes its world-vector entry, and deletes its implementation.
	virtual void DestroySoftbody( IPhysicsSoftbody *pSoftbody ) = 0;
	// AMNOTE: Counts the softbody vector populated by the softbody creation method.
	virtual int GetSoftbodyCount() const = 0;
	// AMNOTE: nIndex must be within the corresponding object count.
	// AMNOTE: Returns the adapter from the indexed softbody implementation.
	virtual IPhysicsSoftbody *GetSoftbody( int nIndex ) = 0;
	// AMNOTE: Returns the indexed softbody with const access.
	virtual const IPhysicsSoftbody *GetSoftbody( int nIndex ) const = 0;
	// AMNOTE: Reads an unidentified softbody tuning value, initially 100.
	virtual float unk501() const = 0;
	// AMNOTE: Sets that tuning value for subsequently created softbodies.
	virtual void unk502( float flValue ) = 0;

private:
	// AMNOTE: Stores an unidentified pointer associated with softbody processing.
	virtual void unk503() = 0;
	// AMNOTE: Returns the unidentified pointer associated with softbody processing.
	virtual void unk504() = 0;

public:
	// AMNOTE: Counts the joint vector populated by the joint creation methods.
	virtual int GetJointCount() const = 0;
	// AMNOTE: nIndex must be within the corresponding object count.
	// AMNOTE: Returns the CPhysicsJoint adapter from the indexed joint.
	virtual IPhysicsJoint *GetJoint( int nIndex ) = 0;
	// AMNOTE: Returns the indexed joint with const access.
	virtual const IPhysicsJoint *GetJoint( int nIndex ) const = 0;
	// AMNOTE: pJoint must be nonnull.
	// AMNOTE: Checks the joint against both the regular and null-joint vectors.
	virtual bool IsJointInWorld( const IPhysicsJoint *pJoint ) const = 0;

	// AMNOTE: Created joints are owned by the world; remove them with DestroyJoint.
	virtual IPhysicsJoint *CreateSphericalJoint( const PhysicsSphericalJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateRevoluteJoint( const PhysicsRevoluteJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreatePrismaticJoint( const PhysicsPrismaticJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateConicalJoint( const PhysicsConicalJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateWeldJoint( const PhysicsWeldJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateGenericJoint( const PhysicsGenericJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateSpringJoint( const PhysicsSpringJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreatePulleyJoint( const PhysicsPulleyJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateNullJoint( const PhysicsNullJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateSplineJoint( const PhysicsSplineJointDesc_t &desc ) = 0;
	virtual IPhysicsJoint *CreateWheelJoint( const PhysicsWheelJointDesc_t &desc ) = 0;

private:
	// AMNOTE: Creates joints from aggregate constraint data; the resource descriptors and output parameters remain unresolved.
	virtual void unk601() = 0;

public:
	// AMNOTE: Destruction is deferred; do not use pJoint after successful removal.
	// AMNOTE: Returns false for a missing/unregistered joint, otherwise removes it and queues deferred deletion.
	virtual bool DestroyJoint( IPhysicsJoint *pJoint ) = 0;

private:
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk701() = 0;
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk702() = 0;
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk703() = 0;
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk704() = 0;
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk705() = 0;
	// AMNOTE: Creates a joint between body adapters; its constraint parameters remain unresolved.
	virtual void unk706() = 0;

public:
	// AMNOTE: Appends other bodies in the body's connection groups; duplicates are retained and pBody must be nonnull.
	virtual void GetConnectedBodies( const IPhysicsBody *pBody, CUtlVector<IPhysicsBody *> &bodies ) const = 0;
	// AMNOTE: Deletes or detaches the body's connection groups without deleting the body; pBody must be nonnull.
	virtual void RemoveBodyConnections( IPhysicsBody *pBody ) = 0;
	// AMNOTE: Creates and registers an initially empty rope.
	virtual IPhysicsRope *CreateRope() = 0;
	// AMNOTE: Deletes a removed CPhysicsRope adapter, clears the caller's pointer, and returns whether removal succeeded.
	virtual bool DestroyRope( IPhysicsRope *&pRope ) = 0;
	// AMNOTE: Creates and registers a particle rope using the query's shape-filter fields and copies localPositions through transform. Creates at least three particles, repeating the last point of a nonempty input when needed. Nonpositive mass, radius and segment length default to 0.1, 2 and 8; radius and segment length are clamped to at least 0.0625 and 0.125.
	virtual IPhysicsParticleRope *CreateParticleRope( const RnQueryAttr_t &query, float flMass, float flRadius, float flSegmentLength, const CTransform &transform, const CUtlVector<Vector> &localPositions ) = 0;
	// AMNOTE: Deletes a removed CPhysicsParticleRope adapter, clears the caller's pointer, and returns whether removal succeeded.
	virtual bool DestroyParticleRope( IPhysicsParticleRope *&pRope ) = 0;
	// AMNOTE: Counts the particle-rope vector populated by CPhysicsParticleRope creation.
	virtual int GetParticleRopeCount() const = 0;
	// AMNOTE: Returns the particle-rope adapter, or null for an invalid index.
	virtual IPhysicsParticleRope *GetParticleRope( int nIndex ) = 0;
	// AMNOTE: Returns the indexed particle rope with const access, or null for an invalid index.
	virtual const IPhysicsParticleRope *GetParticleRope( int nIndex ) const = 0;

private:
	// AMNOTE: Creates and registers a vehicle; the creation descriptor remains unresolved.
	virtual void CreateVehicle() = 0;

public:
	// AMNOTE: Deletes the removed vehicle adapter, clears the caller's pointer, and returns whether removal succeeded.
	virtual bool DestroyVehicle( IPhysicsVehicle *&pVehicle ) = 0;
	// AMNOTE: Counts the vector paired with vehicle creation/removal.
	virtual int GetVehicleCount() const = 0;
	// AMNOTE: Returns the vehicle adapter, or null for an invalid index.
	virtual IPhysicsVehicle *GetVehicle( int nIndex ) = 0;
	// AMNOTE: Returns the indexed vehicle with const access, or null for an invalid index.
	virtual const IPhysicsVehicle *GetVehicle( int nIndex ) const = 0;
	// AMNOTE: Creates and registers a capsule character proxy with the supplied query attributes, position and velocity.
	virtual IPhysicsCharacterProxy *CreateCharacterProxy( const VPhys2CharacterProxyParams_t &params ) = 0;
	// AMNOTE: Deletes the removed character-proxy adapter, clears the caller's pointer, and returns whether removal succeeded.
	virtual bool DestroyCharacterProxy( IPhysicsCharacterProxy *&pProxy ) = 0;
	// AMNOTE: Counts the vector populated by character-proxy creation.
	virtual int GetCharacterProxyCount() const = 0;
	// AMNOTE: Returns the character-proxy adapter, or null for an invalid index.
	virtual IPhysicsCharacterProxy *GetCharacterProxy( int nIndex ) = 0;
	// AMNOTE: Returns the indexed character proxy with const access, or null for an invalid index.
	virtual const IPhysicsCharacterProxy *GetCharacterProxy( int nIndex ) const = 0;
	// AMNOTE: Counts contacts in the world's solver-contact list.
	virtual int GetSolverContactCount() const = 0;

private:
	// AMNOTE: Copies a solver-contact record and retains its manifold snapshot; the owning result type is unresolved.
	virtual void GetSolverContact() = 0;

public:
	// AMNOTE: Counts contacts in the world's report-contact list.
	virtual int GetReportContactCount() const = 0;

private:
	// AMNOTE: Copies a report-contact record and retains its manifold snapshot; the owning result type is unresolved.
	virtual void GetReportContact() = 0;

public:
	// AMNOTE: Removes existing contact records between the two bodies; it does not disable future collisions.
	virtual void RemoveContactsBetweenBodies( IPhysicsBody *pBodyA, IPhysicsBody *pBodyB ) = 0;
	// AMNOTE: Checks for an existing contact record between the two bodies; both pointers must be nonnull.
	virtual bool AreBodiesInContact( const IPhysicsBody *pBodyA, const IPhysicsBody *pBodyB ) const = 0;

private:
	// AMNOTE: Collects contact and body-pair events, including retained snapshots; the owning event types are unresolved.
	virtual void CollectContactEvents() = 0;

public:
	// AMNOTE: Clears contact reports, body-pair events and pending listener notifications.
	virtual void ClearContactReports() = 0;

private:
	// AMNOTE: Transfers pending contact records into an output vector; the owning record type is unresolved.
	virtual void TakePendingContactRecords() = 0;

public:
	// AMNOTE: Creates and registers a controller for pHandler; the nonnull handler must outlive its controller.
	virtual IPhysicsMotionController *CreateMotionController( IMotionEvent *pHandler ) = 0;
	// AMNOTE: Removes the motion controller from its vector and invokes its deleting destructor.
	virtual void DestroyMotionController( IPhysicsMotionController *pController ) = 0;
	// AMNOTE: Counts the same motion-controller vector used by the step and asynchronous updater.
	virtual int GetMotionControllerCount() const = 0;
	// AMNOTE: Counts the vector populated by aggregate-instance creation.
	virtual int GetAggregateInstanceCount() const = 0;
	// AMNOTE: Appends each aggregate represented by an active body once per call; existing entries are retained.
	virtual void GetActiveAggregateInstances( CUtlVector<IPhysAggregateInstance *> &instances ) const = 0;
	// AMNOTE: nIndex must be within the corresponding object count.
	// AMNOTE: Returns the indexed adapter from the aggregate-instance vector.
	virtual IPhysAggregateInstance *GetAggregateInstance( int nIndex ) = 0;

private:
	// AMNOTE: Creates and registers an aggregate instance; its resource and transform parameters remain unresolved.
	virtual void CreateAggregateInstance() = 0;

public:
	// AMNOTE: Destruction is deferred; do not use pInstance after successful removal.
	// AMNOTE: Removes the indexed aggregate instance from its vector and maps, invalidates its world/index, and queues its adapter for deferred deletion.
	virtual void DestroyAggregateInstance( IPhysAggregateInstance *pInstance ) = 0;

private:
	// AMNOTE: Creates a ragdoll and its constraints; the creation descriptors remain unresolved.
	virtual void CreateRagdoll() = 0;

public:
	// AMNOTE: Deletes only a ragdoll whose world accessor returns this world.
	virtual void DestroyRagdoll( IPhysicsRagdoll *pRagdoll ) = 0;

private:
	// AMNOTE: Creates a ragdoll control; its initialization parameters remain unresolved.
	virtual void unk801() = 0;
	// AMNOTE: Creates a ragdoll control using another initialization path; its parameters remain unresolved.
	virtual void unk802() = 0;

public:
	// AMNOTE: Deletes only a ragdoll control whose world accessor returns this world.
	virtual void DestroyRagdollControl( IPhysicsRagdollControl *pControl ) = 0;
	// AMNOTE: Replaces the borrowed world event listener. Keep it alive while registered; pass null to unregister.
	virtual void SetWorldEventListener( IPhysicsWorldEventListener *pListener ) = 0;
	// AMNOTE: Registers a nonempty callback once, invoked with deltaTime before core simulation. Callbacks run in reverse registration order; keep their targets alive until removal and finish simulation work before modifying registrations.
	virtual void AddPreSimulationCallback( const FnVPhys2SimulationCallback_t &callback ) = 0;
	// AMNOTE: Removes a matching pre-simulation callback, preserving the remaining registration order.
	virtual void RemovePreSimulationCallback( const FnVPhys2SimulationCallback_t &callback ) = 0;
	// AMNOTE: Registers a nonempty callback once, invoked with deltaTime after core simulation. Callbacks run in reverse registration order; keep their targets alive until removal and finish simulation work before modifying registrations.
	virtual void AddPostSimulationCallback( const FnVPhys2SimulationCallback_t &callback ) = 0;
	// AMNOTE: Removes a matching post-simulation callback, preserving the remaining registration order.
	virtual void RemovePostSimulationCallback( const FnVPhys2SimulationCallback_t &callback ) = 0;

private:
	// AMNOTE: Stores a contact callback and its context. Invocations receive a retained contact record whose owning snapshot type remains unresolved.
	virtual void SetContactCallback() = 0;

public:
	// AMNOTE: Adds an unordered entity/owner ID pair to collision suppression; returns false when the pair already exists.
	virtual bool DisableCollisionForEntityPair( uint32 nEntityIdA, uint32 nEntityIdB ) = 0;
	// AMNOTE: Removes an unordered entity/owner ID pair from collision suppression.
	virtual void EnableCollisionForEntityPair( uint32 nEntityIdA, uint32 nEntityIdB ) = 0;
	// AMNOTE: Checks whether the unordered entity/owner ID pair is registered for collision suppression.
	virtual bool IsCollisionDisabledForEntityPair( uint32 nEntityIdA, uint32 nEntityIdB ) const = 0;
	// AMNOTE: Removes every collision-suppression pair containing this entity/owner ID.
	virtual void ClearDisabledCollisionsForEntity( uint32 nEntityId ) = 0;
	// AMNOTE: Checks whether any collision-suppression pair contains this entity/owner ID.
	virtual bool HasDisabledCollisionsForEntity( uint32 nEntityId ) const = 0;
	// AMNOTE: Appends collision-suppression pairs, with the smaller ID first; existing entries are retained.
	virtual void GetDisabledEntityCollisionPairs( CUtlVector<VPhys2EntityCollisionPair_t> &pairs ) const = 0;
	// AMNOTE: Tests the shape against the query's object-set mask and collision filters; pShape must be nonnull.
	virtual bool CanQueryShape( const IPhysicsShape *pShape, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Combines all body shapes' layer masks: zero selects interaction layers, one detail layers, and two target layers. Other values return zero.
	virtual uint64 GetCombinedShapeLayerMask( int nMaskType ) const = 0;

private:
	// AMNOTE: Returns a zero-valued short result; the original result type remains unresolved.
	virtual void unk901() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk902() = 0;

public:
	// AMNOTE: Returns the combined bounds of nonempty broadphase trees; an empty world returns invalid bounds.
	virtual AABB_t GetBroadphaseBounds() const = 0;
	// AMNOTE: Returns the symmetric bounds imposed by the configured maximum coordinate.
	virtual AABB_t GetWorldBounds() const = 0;

private:
	// AMNOTE: Returns a zero-initialized result; its complete result type remains unresolved.
	virtual void unk1001() = 0;
	// AMNOTE: Returns a zero-initialized result; its complete result type remains unresolved.
	virtual void unk1002() = 0;

public:
	// AMNOTE: Casts against the body's shapes without a world query filter; pBody must be nonnull and result must be reset before querying.
	virtual void CastRayAgainstBody( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Writes the closest ray hit. Reset result before querying; vDisplacement is the full sweep vector.
	virtual void CastRay( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Appends ray hits; clear results before querying. vDisplacement is the full sweep vector.
	virtual void CastRayMultiple( CUtlVector<VPhys2CastResult_t> &results, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Uses the world ray query without a query filter; reset result before querying.
	virtual void CastRayUnfiltered( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement ) const = 0;
	// AMNOTE: Appends world ray hits without a query filter; clear results before querying.
	virtual void CastRayMultipleUnfiltered( CUtlVector<VPhys2CastResult_t> &results, const Vector &vStart, const Vector &vDisplacement ) const = 0;

private:
	// AMNOTE: Collects geometry hit descriptors; the complete result type and trailing options remain unresolved.
	virtual void unk1101() = 0;

public:
	// AMNOTE: Appends entry/exit boundaries and sorts the unmerged result by fraction; clear intervals before a new query. Nonnegative flAdditionalDistance narrows subsequent broadphase traversal to that distance beyond the nearest entry encountered; negative values search the full displacement. bMerge combines overlapping intervals.
	virtual void CastRayIntervals( CRayIntervalVector &intervals, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query, bool bMerge, float flAdditionalDistance ) const = 0;

	// AMNOTE: Sweeps a sphere against the body's shapes; pBody must be nonnull. Resets the hit fraction to one before querying.
	virtual void CastSphereAgainstBody( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, float flRadius, const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Writes the closest swept-sphere hit; a negligible radius uses the ray query.
	virtual void CastSphere( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, float flRadius, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Appends swept-sphere hits; clear results before querying. A negligible radius uses the ray query.
	virtual void CastSphereMultiple( CUtlVector<VPhys2CastResult_t> &results, const Vector &vStart, const Vector &vDisplacement, float flRadius, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Sweeps a box against the body's shapes; pBody must be nonnull. Resets the hit fraction to one before querying.
	virtual void CastBoxAgainstBody( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, const Vector &vHalfExtents, const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Writes the closest swept-box hit. vStart is its center and vHalfExtents gives its half dimensions.
	virtual void CastBox( VPhys2CastResult_t &result, const Vector &vStart, const Vector &vDisplacement, const Vector &vHalfExtents, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Appends swept-box hits using a center and half dimensions; clear results before querying.
	virtual void CastBoxMultiple( CUtlVector<VPhys2CastResult_t> &results, const Vector &vStart, const Vector &vDisplacement, const Vector &vHalfExtents, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Sweeps the Ray_t shape and returns whether the closest hit fraction is below one.
	virtual bool CastShape( VPhys2CastResult_t &result, const Ray_t &shape, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Appends Ray_t sweep hits and returns whether results is nonempty; clear results before querying.
	virtual bool CastShapeMultiple( CUtlVector<VPhys2CastResult_t> &results, const Ray_t &shape, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;

public:
	// Sweeps the transformed capsule along vDisplacement. Initial overlaps are skipped when requested.
	virtual CLinearSweepResult LinearCast( const CTransform &transform, const RnCapsule_t &capsule, const Vector &vDisplacement, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	// Sweeps every shape of pBody between the two world transforms; pBody must be nonnull.
	virtual CSweepResult SweepBody( const CTransform &start, const CTransform &end, const IPhysicsBody *pBody, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	// Appends hits; clear results before a new query.
	virtual void SweepBodyMultiple( CUtlVector<CSweepResult> &results, const CTransform &start, const CTransform &end, const IPhysicsBody *pBody, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	// pShape must be nonnull. The transforms place the supplied geometry in world space.
	virtual CSweepResult SweepShape( const CTransform &start, const CTransform &end, const IPhysicsShape *pShape, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	// Appends hits; clear results before a new query.
	virtual void SweepShapeMultiple( CUtlVector<CSweepResult> &results, const CTransform &start, const CTransform &end, const IPhysicsShape *pShape, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	virtual CSweepResult SweepShape( const CTransform &start, const CTransform &end, const Ray_t &shape, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;
	virtual void SweepShapeMultiple( CUtlVector<CSweepResult> &results, const CTransform &start, const CTransform &end, const Ray_t &shape, const RnQueryAttr_t &query, bool bIgnoreInitialOverlap ) const = 0;

public:
	// AMNOTE: Appends shapes overlapping the sphere. Use zero for bounds testing or one for exact overlap testing.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Vector &vCenter, float flRadius, const RnQueryAttr_t &query, int nOverlapTest ) const = 0;
	// AMNOTE: Appends shapes overlapping bounds. Use zero for bounds testing or one for exact overlap testing.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const AABB_t &bounds, const RnQueryAttr_t &query, int nOverlapTest ) const = 0;
	// AMNOTE: Replaces shapes with the shapes hit by a box sweep; the same shape may appear more than once.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Vector &vStart, const Vector &vDisplacement, const Vector &vHalfExtents, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Replaces shapes with the shapes hit by a ray; the same shape may appear more than once.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Replaces shapes with the shapes hit by a sphere sweep; a negligible radius uses the ray query.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Vector &vStart, const Vector &vDisplacement, float flRadius, const RnQueryAttr_t &query ) const = 0;
	// AMNOTE: Appends overlapping shapes at vOrigin. Use zero for bounds testing or one for exact overlap testing.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Ray_t &shape, const Vector &vOrigin, const RnQueryAttr_t &query, int nOverlapTest ) const = 0;
	// AMNOTE: Replaces shapes with the shapes hit by the Ray_t sweep; the same shape may appear more than once.
	virtual void Query( CUtlVector<IPhysicsShape *> &shapes, const Ray_t &shape, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;

private:
	// AMNOTE: Appends shape and spatial hierarchy records for a custom broadphase query; its descriptor and result record types remain unresolved.
	virtual void Query() = 0;
	// AMNOTE: Visits shapes whose broadphase bounds overlap the supplied bounds; the std::function callback contract remains unresolved.
	virtual void VisitShapesInBounds() = 0;

public:
	// AMNOTE: Captures matching geometry within bounds; the cache retains references to convex shapes, which must outlive its use.
	virtual IPhysicsQueryCache *CreateQueryCache( const AABB_t &bounds, const RnQueryAttr_t &query ) = 0;
	// AMNOTE: Destroys the engine-owned query cache through its virtual destructor; null is ignored.
	virtual void DestroyQueryCache( IPhysicsQueryCache *pCache ) = 0;
	// AMNOTE: Returns the closest accepted shape distance below flMaxDistance and updates result; otherwise leaves result unchanged and returns flMaxDistance. filter must be nonempty and flMaxDistance nonnegative.
	virtual float GetClosestPoint( const Vector &vPoint, float flMaxDistance, const FnVPhys2ShapeFilter_t &filter, VPhys2ClosestPointResult_t &result ) const = 0;
	// AMNOTE: Returns the closest accepted shape distance below flMaxDistance and updates vClosestPoint; otherwise leaves it unchanged and returns flMaxDistance. filter must be nonempty and flMaxDistance nonnegative.
	virtual float GetClosestPoint( const Vector &vPoint, float flMaxDistance, const FnVPhys2ShapeFilter_t &filter, Vector &vClosestPoint ) const = 0;
	// AMNOTE: Appends collision planes for sphere, capsule and mesh shapes; clear planes before a new query. bUseQueryOrigin uses a corrected query origin instead of the world-geometry witness point. Nonnegative flExtraSeparation expands bounds and admits that extra gap; negative values select the default skin margins.
	virtual void CollideShape( CUtlVectorFixedGrowable<CRnPlaneResult, 32> &planes, const CTransform &transform, const Ray_t &shape, const RnQueryAttr_t &query, bool bUseQueryOrigin, float flExtraSeparation ) const = 0;

private:
	// AMNOTE: Stores a borrowed height-grid descriptor used by ground collision; its complete type remains unresolved.
	virtual void SetGroundHeightGrid() = 0;
	// AMNOTE: Returns the borrowed height-grid descriptor; its complete return type remains unresolved.
	virtual void GetGroundHeightGrid() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1201() = 0;

public:
	// AMNOTE: Returns the stored world creation flags, including any bits added by the engine.
	virtual uint32 GetCreationFlags() const = 0;
	// AMNOTE: Stores a borrowed world-geometry body; null clears the registration. Clear it before destroying the body.
	virtual void SetWorldBody( IPhysicsBody *pBody ) = 0;
	// AMNOTE: Returns the borrowed world-geometry body without validating its lifetime.
	virtual IPhysicsBody *GetWorldBody() const = 0;

private:
	// AMNOTE: Serializes a typed physics object; its serialization parameter structure remains unresolved.
	virtual void SavePhysicsObject() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1301() = 0;
	// AMNOTE: Restores or reuses a typed physics object; its restoration parameter structure remains unresolved.
	virtual void RestorePhysicsObject() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1401() = 0;
	// AMNOTE: Returns zero in this build; the complete signature remains unresolved.
	virtual void unk1402() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk1403() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk1404() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1405() = 0;
	// AMNOTE: Returns zero in this build; the complete signature remains unresolved.
	virtual void unk1406() = 0;
	// AMNOTE: Returns zero in this build; the complete signature remains unresolved.
	virtual void unk1407() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1408() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1409() = 0;
	// AMNOTE: Returns a zero-initialized result; its complete result type remains unresolved.
	virtual void unk14010() = 0;
	// AMNOTE: Returns a zero-initialized result; its complete result type remains unresolved.
	virtual void unk14011() = 0;
	// AMNOTE: Returns a zero-initialized result; its complete result type remains unresolved.
	virtual void unk14012() = 0;

public:
	// AMNOTE: Returns the world's engine-owned reflection context; keep it only while this world is alive.
	virtual IPhysReflContext *GetReflectionContext() const = 0;

private:
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1501() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1502() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk1503() = 0;
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk1504() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1505() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1506() = 0;
	// AMNOTE: Draws the world's debug overlays; dependent interface types and the complete signature remain unresolved.
	virtual void DebugDraw() = 0;
	// AMNOTE: Updates the world's scene objects; dependent interface types and the complete signature remain unresolved.
	virtual void UpdateSceneObjects() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1601() = 0;

public:
	// AMNOTE: Returns the stored debug draw flags.
	virtual uint64 GetDebugDrawFlags() const = 0;
	// AMNOTE: Stores debug draw flags and requests a scene-object update. Drawing may use command override options.
	virtual void SetDebugDrawFlags( uint64 flags ) = 0;

private:
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1701() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1702() = 0;
	// AMNOTE: Copies a CFrustum into the core world for debug drawing; the ignored leading parameter remains unresolved.
	virtual void SetDebugDrawFrustum() = 0;

public:
	// AMNOTE: Borrows the world's stored debug frustum.
	virtual const CFrustum *GetDebugDrawFrustum() const = 0;

private:
	// AMNOTE: Does no work in this build; the original parameters remain unresolved.
	virtual void unk1801() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1802() = 0;
	// AMNOTE: Replaces the world's debug draw filter and notifies its world event listener; the complete filter type remains unresolved.
	virtual void SetDebugDrawFilter() = 0;

public:
	// AMNOTE: Clears the world's debug draw filter and requests a scene-object update, notifying a registered world event listener first.
	virtual void ClearDebugDrawFilter() = 0;

private:
	// AMNOTE: Tests collision attributes against the world's debug draw filter; the original parameter contract remains unresolved.
	virtual bool ShouldDebugDraw() const = 0;

public:
	// AMNOTE: Tests the shape against the world's debug draw filter; pShape must be nonnull.
	virtual bool ShouldDebugDraw( const IPhysicsShape *pShape ) const = 0;
	// AMNOTE: Null bodies are rejected. When the filter is enabled, bCheckShapes additionally requires at least one shape accepted by the filter.
	virtual bool ShouldDebugDraw( const IPhysicsBody *pBody, bool bCheckShapes ) const = 0;
	// AMNOTE: Tests whether the softbody can be drawn and passes the world's debug draw filter; pSoftbody must be nonnull.
	virtual bool ShouldDebugDraw( const IPhysicsSoftbody *pSoftbody ) const = 0;
	// AMNOTE: Tests the joint layer and connected body acceptance when a debug draw filter is installed; pJoint must be nonnull.
	virtual bool ShouldDebugDraw( const IPhysicsJoint *pJoint ) const = 0;

private:
	// AMNOTE: Applies sorted internal-shape-ID/float records to debug scene shader parameters; unlisted shapes receive zero. The scene parameter's meaning remains unresolved.
	virtual void SetShapeSceneParameters() = 0;

public:
	// AMNOTE: Sends a nonempty world name to VConComm; debug world names are disabled in this build.
	virtual void SendDebugWorldName() = 0;
	// AMNOTE: Returns the world debug name; this build returns null.
	virtual const char *GetDebugName() const = 0;

public:
	// AMNOTE: Returns the engine's build description; the string is engine-owned.
	virtual const char *GetBuildDescription() const = 0;
	// AMNOTE: Processes physics debug options; softbody reports compare their time against flTime.
	virtual void ExecuteDebugCommand( const CCommand &args, float flTime ) = 0;
	// AMNOTE: Writes benchmark JSON, text and binary files below the physics-testbed benchmark directory.
	virtual void DumpWorld( const char *pName ) = 0;
	// AMNOTE: Writes the physics-testbed benchmark directory relative to the VGAME environment variable.
	virtual void GetBenchmarkDirectory( CBufferString &directory ) const = 0;

private:
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1901() = 0;
	// AMNOTE: Unresolved signature: inaccessible placeholder, not a callable declaration.
	virtual void unk1902() = 0;

public:
	// AMNOTE: Appends an HTML collision-filter report. Both shapes must be nonnull and attached to bodies.
	virtual void AppendShapeCollisionDescription( CBufferString &description, const IPhysicsShape *pShapeA, const IPhysicsShape *pShapeB ) const = 0;
	// AMNOTE: Appends an HTML query-filter report. pShape must be nonnull and attached to a body.
	virtual void AppendShapeQueryDescription( CBufferString &description, const IPhysicsShape *pShape, const RnQueryAttr_t &query ) const = 0;

protected:
	~IVPhysics2World() = default;
};

#include "vphysics2/iphysaggregateinstance.h"

// AMNOTE: Engine-owned adapter layout. The engine supplies its schema binding and parallel update implementations.
class CPhysicsAbsoluteRagdollControl : public IPhysicsRagdollControl
{
public:
	IVPhysics2World *GetWorld() const override { return m_World; }
	IPhysicsRagdoll *GetRagdoll() const override { return m_Ragdoll; }
	float GetMinSpringFrequency() const override { return m_flMinSpringFrequency; }
	void SetMinSpringFrequency( float flFrequency ) override { m_flMinSpringFrequency = flFrequency < 0.0f ? 0.0f : MIN( 30.0f, flFrequency ); }
	float GetMaxSpringFrequency() const override { return m_flMaxSpringFrequency; }
	void SetMaxSpringFrequency( float flFrequency ) override { m_flMaxSpringFrequency = flFrequency < 0.0f ? 0.0f : MIN( 30.0f, flFrequency ); }
	float GetMaxStretch() const override { return m_flMaxStretch; }
	void SetMaxStretch( float flStretch ) override { m_flMaxStretch = flStretch; }
	void SetSolidCollisionAtZeroWeight( bool bEnable ) override { m_bSolidCollisionAtZeroWeight = bEnable; }
	void SetRequiresDynamicBodies( bool bRequire ) override { m_bRequiresDynamicBodies = bRequire; }
	inline void RemoveControlBody( int nIndex ) override;
	void UpdateBodiesParallel( const CTransform *pSkeletonPose, float flDeltaTime, CUtlLeanVectorFixedGrowable<std::function<void()>, 100, int, CMemAllocAllocator> &tasks, CAtomicMutex &mutex ) override;
	void UpdateJointsParallel( int nBoneCount, CPhysicsRagdollWeight *pWeights, const int *pBoneIndices, const CTransform *pSkeletonPose, float flDeltaTime, bool bUnused, CUtlLeanVectorFixedGrowable<std::function<void()>, 100, int, CMemAllocAllocator> &tasks, CAtomicMutex &mutex ) override;
	// AMNOTE: NaN payloads and floating-point exception flags can vary with the compiler.
	inline void ApplyTeleport( const CTransform &previousTransform, const CTransform &currentTransform ) override;
	void SetIgnoreTeleport( bool bIgnore ) override { m_bIgnoreTeleport = bIgnore; }
	void AddLinearVelocity( Vector velocity ) override
	{
		m_vLinearVelocityAccumulator.x += velocity.x;
		m_vLinearVelocityAccumulator.y += velocity.y;
		m_vLinearVelocityAccumulator.z += velocity.z;
	}
	void AddAngularVelocity( Vector velocity ) override
	{
		m_vAngularVelocityAccumulator.x += velocity.x;
		m_vAngularVelocityAccumulator.y += velocity.y;
		m_vAngularVelocityAccumulator.z += velocity.z;
	}
	void AddForce( Vector force ) override
	{
		m_vForceAccumulator.x += force.x;
		m_vForceAccumulator.y += force.y;
		m_vForceAccumulator.z += force.z;
	}
	void ClearAccumulators() override { m_vLinearVelocityAccumulator.Init(); m_vAngularVelocityAccumulator.Init(); m_vForceAccumulator.Init(); }
	const CUtlVector<IPhysicsBody *> &GetControlBodies() const override { return m_ControlBodies; }
	void SetStretchCorrectionEnabled( bool bEnable ) override { m_bStretchCorrectionEnabled = bEnable; }

private:
	CPhysicsAbsoluteRagdollControl() = delete;
	CPhysicsAbsoluteRagdollControl( const CPhysicsAbsoluteRagdollControl & ) = delete;
	CPhysicsAbsoluteRagdollControl &operator=( const CPhysicsAbsoluteRagdollControl & ) = delete;
	inline ~CPhysicsAbsoluteRagdollControl() override;

	IVPhysics2World *m_World;
	IPhysicsRagdoll *m_Ragdoll;
	float m_flMinSpringFrequency;
	float m_flMaxSpringFrequency;
	float m_flMaxStretch;
	bool m_bSolidCollisionAtZeroWeight;
	bool m_bRequiresDynamicBodies;
	CUtlVector<IPhysicsBody *> m_ControlBodies;
	CUtlVector<IPhysicsJoint *> m_ControlJoints;
	CUtlVector<int16> m_BoneToControlBodyMap;
	bool m_bIgnoreTeleport;
	Vector m_vLinearVelocityAccumulator;
	Vector m_vAngularVelocityAccumulator;
	Vector m_vForceAccumulator;
	bool m_bStretchCorrectionEnabled;
};

inline CPhysicsAbsoluteRagdollControl::~CPhysicsAbsoluteRagdollControl()
{
	for ( int i = m_ControlJoints.Count() - 1; i >= 0; --i )
		m_World->DestroyJoint( m_ControlJoints[i] );
	m_ControlJoints.RemoveAll();
	for ( int i = m_ControlBodies.Count() - 1; i >= 0; --i )
		m_World->DestroyBody( m_ControlBodies[i] );
	m_ControlBodies.RemoveAll();
	m_BoneToControlBodyMap.RemoveAll();
}

inline void CPhysicsAbsoluteRagdollControl::RemoveControlBody( int nIndex )
{
	if ( nIndex < 0 || nIndex >= m_ControlBodies.Count() )
		return;
	if ( m_ControlJoints[nIndex] )
		m_World->DestroyJoint( m_ControlJoints[nIndex] );
	if ( m_ControlBodies[nIndex] )
		m_World->DestroyBody( m_ControlBodies[nIndex] );
	m_ControlJoints.Remove( nIndex );
	m_ControlBodies.Remove( nIndex );
	for ( int i = 0; i < m_BoneToControlBodyMap.Count(); ++i )
	{
		int16 &nBodyIndex = m_BoneToControlBodyMap[i];
		if ( nBodyIndex == nIndex )
			nBodyIndex = -1;
		else if ( nBodyIndex > nIndex )
			--nBodyIndex;
	}
}

inline void CPhysicsAbsoluteRagdollControl::ApplyTeleport( const CTransform &previousTransform, const CTransform &currentTransform )
{
	const int nBodyCount = m_Ragdoll->GetBodyCount();
	if ( nBodyCount != m_ControlBodies.Count() || m_bIgnoreTeleport )
		return;

	const auto quaternionLength = []( const fltx4 &q )
	{
		const fltx4 squared = MulSIMD( q, q );
		const fltx4 xy = _mm_add_ss( squared, _mm_shuffle_ps( squared, squared, 0x55 ) );
		const fltx4 zw = _mm_add_ss( _mm_shuffle_ps( squared, squared, 0xAA ), _mm_shuffle_ps( squared, squared, 0xFF ) );
		return _mm_sqrt_ps( SplatXSIMD( _mm_add_ss( xy, zw ) ) );
	};
	const auto rotate = []( const fltx4 &position, const fltx4 &q )
	{
		const fltx4 qYZX = _mm_shuffle_ps( q, q, 0x09 );
		const fltx4 qZXY = _mm_shuffle_ps( q, q, 0x12 );
		const fltx4 cross = SubSIMD( MulSIMD( _mm_shuffle_ps( position, position, 0x12 ), qYZX ), MulSIMD( _mm_shuffle_ps( position, position, 0x09 ), qZXY ) );
		const fltx4 twiceCross = AddSIMD( cross, cross );
		const fltx4 secondCross = SubSIMD( MulSIMD( _mm_shuffle_ps( twiceCross, twiceCross, 0x12 ), qYZX ), MulSIMD( _mm_shuffle_ps( twiceCross, twiceCross, 0x09 ), qZXY ) );
		return AddSIMD( AddSIMD( MulSIMD( SplatWSIMD( q ), twiceCross ), position ), secondCross );
	};
	const auto concatenate = [&quaternionLength, &rotate]( const CTransform &left, const CTransform &right )
	{
		const fltx4 leftPosition = LoadAlignedSIMD( left.m_vPosition.Base() );
		const fltx4 rightPosition = LoadAlignedSIMD( right.m_vPosition.Base() );
		const fltx4 leftRotation = LoadAlignedSIMD( left.m_orientation.Base() );
		const fltx4 rightRotation = LoadAlignedSIMD( right.m_orientation.Base() );
		const fltx4 scale = SplatWSIMD( leftPosition );
		CTransform result;
		StoreAlignedSIMD( result.m_vPosition.Base(), SetWSIMD( AddSIMD( MulSIMD( rotate( rightPosition, leftRotation ), scale ), leftPosition ), MulSIMD( SplatWSIMD( rightPosition ), scale ) ) );
		const fltx4 reversed = _mm_shuffle_ps( rightRotation, rightRotation, 0x1B );
		const fltx4 xTerm = MulSIMD( SplatXSIMD( leftRotation ), _mm_xor_ps( reversed, _mm_setr_ps( 0.0f, -0.0f, 0.0f, -0.0f ) ) );
		const fltx4 wTerm = MulSIMD( rightRotation, SplatWSIMD( leftRotation ) );
		const fltx4 yTerm = MulSIMD( SplatYSIMD( leftRotation ), _mm_xor_ps( _mm_shuffle_ps( rightRotation, rightRotation, 0x4E ), _mm_setr_ps( 0.0f, 0.0f, -0.0f, -0.0f ) ) );
		const fltx4 zTerm = MulSIMD( SplatZSIMD( leftRotation ), _mm_xor_ps( _mm_shuffle_ps( reversed, reversed, 0x4E ), _mm_setr_ps( -0.0f, 0.0f, 0.0f, -0.0f ) ) );
		const fltx4 product = AddSIMD( AddSIMD( AddSIMD( xTerm, wTerm ), yTerm ), zTerm );
		const fltx4 length = quaternionLength( product );
		const fltx4 identity = _mm_setr_ps( 0.0f, 0.0f, 0.0f, 1.0f );
		StoreAlignedSIMD( result.m_orientation.Base(), MaskedAssign( CmpEqSIMD( _mm_setzero_ps(), length ), identity, _mm_div_ps( product, length ) ) );
		return result;
	};

	const fltx4 position = LoadAlignedSIMD( previousTransform.m_vPosition.Base() );
	const fltx4 conjugate = MulSIMD( LoadAlignedSIMD( previousTransform.m_orientation.Base() ), _mm_setr_ps( -1.0f, -1.0f, -1.0f, 1.0f ) );
	const fltx4 length = quaternionLength( conjugate );
	const fltx4 ones = _mm_set1_ps( 1.0f );
	const fltx4 epsilon = _mm_set1_ps( FLT_EPSILON );
	const fltx4 inverseRotation = _mm_div_ps( conjugate, _mm_or_ps( length, _mm_and_ps( CmpEqSIMD( length, _mm_setzero_ps() ), epsilon ) ) );
	CTransform inverse;
	StoreAlignedSIMD( inverse.m_orientation.Base(), inverseRotation );
	if ( previousTransform.m_vPosition.w == 1.0f )
	{
		StoreAlignedSIMD( inverse.m_vPosition.Base(), SetWSIMD( SubSIMD( _mm_setzero_ps(), rotate( position, conjugate ) ), SplatWSIMD( position ) ) );
	}
	else
	{
		const fltx4 scale = SplatWSIMD( position );
		const fltx4 inverseScale = _mm_div_ps( ones, _mm_or_ps( scale, _mm_and_ps( epsilon, CmpEqSIMD( scale, _mm_setzero_ps() ) ) ) );
		StoreAlignedSIMD( inverse.m_vPosition.Base(), SetWSIMD( MulSIMD( rotate( position, inverseRotation ), SubSIMD( _mm_setzero_ps(), inverseScale ) ), inverseScale ) );
	}
	const CTransform delta = concatenate( currentTransform, inverse );
	for ( int i = 0; i < nBodyCount; ++i )
	{
		if ( IPhysicsBody *pBody = m_ControlBodies[i] )
			pBody->SetTransform( concatenate( delta, pBody->GetTransform() ) );
		if ( IPhysicsBody *pBody = m_Ragdoll->GetBodies()[i] )
			pBody->SetTransform( concatenate( delta, pBody->GetTransform() ) );
	}
}

#endif // IVPHYSICS2WORLD_H
