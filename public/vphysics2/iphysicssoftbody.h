#ifndef IPHYSICSSOFTBODY_H
#define IPHYSICSSOFTBODY_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "mathlib/vectoraligned.h"
#include "mathlib/ssemath.h"
#include "mathlib/transform.h"
#include "mathlib/aabb.h"
#include "tier0/utlstring.h"
#include "tier0/checksum_crc.h"
#include "tier0/utlvector.h"
#include "tier0/utlstringtoken.h"
#include "tier0/utlhashtable.h"
#include "tier0/utlvectorultraconservative.h"
#include "modellib/modelskeletondata.h"

class IVPhysics2World;
class CFeModel;
class CRnSoftbody;
class KeyValues3;
class CUtlBuffer;
struct RnCollisionAttr_t;

// AMNOTE: Valid interpolated properties; attraction scales default to one and the other properties default to zero.
enum class SoftbodyInterpolatedProperty_t : uint8
{
	PositionAnimationAttractionScale = 0,
	PositionAnimationAttractionBias = 1,
	VelocityAnimationAttractionScale = 2,
	VelocityAnimationAttractionBias = 3,
	GroundHeightOffset = 4,
};

// AMNOTE: Copies aligned positions while supplying the simulation transform. Zero selects the preceding frame; every nonzero value selects the current frame.
struct SetNodePositionsParams_t
{
	const VectorAligned *pPos;
	int nCount;
	CTransform tm;
	int nFrame;
};

struct VPhys2SoftbodyBoneMapping_t
{
	uint16 m_nBone;
	uint16 m_nControl;
};

// AMNOTE: Bone-merge indices; minus one denotes an unmapped bone.
struct VPhys2SoftbodyParentBoneMapping_t
{
	int32 m_nParentBone;
	int32 m_nChildBone;
};

// AMNOTE: Borrowed model skeleton for softbody transform filtering. The model owns its lifetime; do not construct, copy or destroy it.
class VPhys2SoftbodyModelSkeleton
{
public:
	const ModelSkeletonData_t &GetSkeletonData() const { return m_SkeletonData; }
	int GetBoneCount() const { return m_SkeletonData.m_nParent.Count(); }
	// AMNOTE: Requires a level of detail from zero through seven.
	uint16 GetBoneCountForLOD( int nLOD ) const { return m_nBoneCountPerLOD[nLOD]; }
	// AMNOTE: Returns minus one when the name does not match a bone.
	int FindBone( CUtlStringToken nName ) const { const uint16 *pBone = m_BoneNameMap.GetPtr( nName ); return pBone ? *pBone : -1; }
	int GetParentBone( int nBone ) const { return nBone >= 0 && nBone < GetBoneCount() ? m_SkeletonData.m_nParent[nBone] : -1; }
	const char *GetBoneName( int nBone ) const { return nBone >= 0 && nBone < GetBoneCount() ? m_SkeletonData.m_boneName[nBone].String() : "root"; }
	const CUtlVector<CTransform> &GetInverseBindPose() const { return m_InverseBindPose; }

private:
	VPhys2SoftbodyModelSkeleton() = delete;
	VPhys2SoftbodyModelSkeleton( const VPhys2SoftbodyModelSkeleton & ) = delete;
	VPhys2SoftbodyModelSkeleton &operator=( const VPhys2SoftbodyModelSkeleton & ) = delete;
	~VPhys2SoftbodyModelSkeleton() = delete;

	CUtlHashtable<CUtlStringToken, uint16> m_BoneNameMap;
	uint16 m_nBoneCountPerLOD[8];
	ModelSkeletonData_t m_SkeletonData;
	CUtlVector<CTransform> m_InverseBindPose;
};

// AMNOTE: Engine-owned model binding. The model owns its lifetime; do not construct, copy or destroy it.
class VPhys2SoftbodyAnimationBinding
{
public:
	const CFeModel *GetFeModel() const { return m_pModel; }
	// AMNOTE: Borrowed model skeleton for FilterTransformsParams_t; the model owns its lifetime.
	const VPhys2SoftbodyModelSkeleton *GetSkeleton() const { return m_pSkeleton; }
	uint16 GetBoneCount() const { return m_nBoneCount; }
	int GetCtrlCount() const { return m_ControlBoneMap.Count() - m_nBoneCount; }
	// AMNOTE: Borrowed maps contain minus one for unmapped bones or controls.
	const int16 *GetBoneToCtrlMap() const { return m_ControlBoneMap.Base(); }
	const int16 *GetCtrlToBoneMap() const { return m_ControlBoneMap.Base() ? m_ControlBoneMap.Base() + m_nBoneCount : nullptr; }
	const CUtlVectorConservative<int16> &GetMorphControllerMap() const { return m_MorphControllerMap; }
	const CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> &GetControlMappings() const { return m_ControlMappings; }
	const CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> &GetBoneSortedControlMappings() const { return m_BoneSortedControlMappings; }
	int GetParentBoneMappingCount() const { return m_pParentBoneMappings ? m_pParentBoneMappings->m_Size : 0; }
	const VPhys2SoftbodyParentBoneMapping_t *GetParentBoneMappings() const { return m_pParentBoneMappings ? m_pParentBoneMappings->m_Elements : nullptr; }

private:
	VPhys2SoftbodyAnimationBinding() = delete;
	VPhys2SoftbodyAnimationBinding( const VPhys2SoftbodyAnimationBinding & ) = delete;
	VPhys2SoftbodyAnimationBinding &operator=( const VPhys2SoftbodyAnimationBinding & ) = delete;
	~VPhys2SoftbodyAnimationBinding() = delete;

	const CFeModel *m_pModel;
	const VPhys2SoftbodyModelSkeleton *m_pSkeleton;
	CUtlVectorConservative<int16> m_ControlBoneMap;
	CUtlVectorConservative<int16> m_MorphControllerMap;
	CUtlVectorConservative<uint16> m_unk001;
	CUtlVectorConservative<uint16> m_unk002;
	CUtlVectorConservative<uint32> m_unk003;
	CUtlVectorConservative<uint32> m_RequiredBoneMask;
	CUtlVectorConservative<uint32> m_RequiredAncestorBoneMask;
	CUtlVectorConservative<uint32> m_MorphDrivenBoneMask;
	CUtlVectorConservative<uint16> m_unk101;
	CUtlVectorConservative<uint16> m_MorphFlexControllers;
	CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> m_unk201;
	CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> m_BoneSortedControlMappings;
	CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> m_ControlMappings;
	CUtlVectorConservative<float> m_EffectWeights;
	CUtlVectorConservative<VPhys2SoftbodyBoneMapping_t> m_FilteredControlMappings;
	CUtlVectorUltraConservative<VPhys2SoftbodyParentBoneMapping_t>::Data_t *m_pParentBoneMappings;
	uint16 m_nBoneCount;
	uint16 m_unk301;
	uint16 m_unk302;
	uint16 m_unk303;
	uint16 m_nPositionOnlyCtrlCount;
};

struct SetAnimatedTransformsParams_t
{
	const CTransform *pBones = nullptr;
	const VPhys2SoftbodyAnimationBinding *pBinding = nullptr;
	const int16 *pMorphControllerMap = nullptr;
	const float *pMorphControllerValues = nullptr;
	int nMorphControllerCount = 0;
	// AMNOTE: Bit 0 restores node positions from the bone pose and clears motion history; the other bits are unused.
	uint32 nFlags = 0;
};

struct FilterTransformsParams_t
{
	// AMNOTE: Use the matching binding's borrowed skeleton. Required when a local pose is supplied.
	const VPhys2SoftbodyModelSkeleton *pSkeleton = nullptr;
	CTransform *pBones = nullptr;
	const CTransform *pLocalPose = nullptr;
	// AMNOTE: Optional output arrays require space for GetNodeCount() transforms.
	CTransform *pSimulatedPoseOut = nullptr;
	CTransform *pAnimatedPoseOut = nullptr;
	CTransform tm;
	const VPhys2SoftbodyAnimationBinding *pBinding = nullptr;
	const VectorAligned *pNodePositions = nullptr;
	// AMNOTE: Optional words select bones by bit index; null selects all bones.
	const uint32 *pBoneMask = nullptr;
	uint32 nParentControlCount = 0;
	const uint32 *pParentControlIndices = nullptr;
	const CTransform *pParentPose = nullptr;
	const int16 *pMorphControllerMap = nullptr;
	const float *pMorphControllerValues = nullptr;
	int nMorphControllerCount = 0;
};

// AMNOTE: Borrowed engine-owned effect; remove it through IPhysicsSoftbody::RemoveEffect.
class ISoftbodyEffect;

// AMNOTE: Engine-owned softbody exposed by IVPhysics2World; do not implement or delete it. Private reservations retain operations whose full contracts remain unresolved.
abstract_class IPhysicsSoftbody
{
protected:
	virtual ~IPhysicsSoftbody() = default;

public:
	// AMNOTE: Returns the owning physics world, or null when no world is assigned.
	virtual IVPhysics2World *GetWorld() const = 0;
	// AMNOTE: Stores the model scale without clamping.
	virtual void SetModelScale( float flModelScale ) = 0;
	// AMNOTE: Returns the stored model scale.
	virtual float GetModelScale() const = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk001() = 0;

public:
	// AMNOTE: Returns the borrowed finite-element model.
	virtual CFeModel *GetFeModel() = 0;
	// AMNOTE: Returns the borrowed finite-element model.
	virtual const CFeModel *GetFeModel() const = 0;
	// AMNOTE: Returns the CRC32 of both node-position frames.
	virtual uint32 GetStateHash() const = 0;
	// AMNOTE: Marks the animated transforms as changed.
	virtual void TouchAnimatedTransforms() = 0;
	// AMNOTE: Copies mapped bone transforms into the animated control pose. A null transform array leaves the pose unchanged.
	virtual void SetAnimatedTransforms( const VPhys2SoftbodyAnimationBinding *pBinding, const CTransform *pBones ) = 0;
	// AMNOTE: Also evaluates mapped morph controllers. Resetting node positions requires a binding and bone array.
	virtual void SetAnimatedTransforms( const SetAnimatedTransformsParams_t &params ) = 0;

public:
	// AMNOTE: Returns borrowed simulated control transforms, updating them from the supplied or current node positions as needed.
	virtual CTransform *GetParticleTransforms( const VectorAligned *pInputNodePos = nullptr, uint32 nFlags = 0 ) = 0;
	// AMNOTE: Returns a borrowed array of animated control transforms.
	virtual CTransform *GetAnimatedTransforms() = 0;
	// AMNOTE: Returns a borrowed array of animated control transforms.
	virtual const CTransform *GetAnimatedTransforms() const = 0;
	// AMNOTE: Returns a borrowed array of simulated control transforms.
	virtual CTransform *GetSimulatedTransforms() = 0;
	// AMNOTE: Returns a borrowed array of simulated control transforms.
	virtual const CTransform *GetSimulatedTransforms() const = 0;
	// AMNOTE: Updates the activity state before transform filtering and reports whether filtering can proceed.
	virtual bool BeforeFilterTransforms() = 0;

	// AMNOTE: Filters the bone array using simulated controls and optional parent, local-pose and morph inputs. Requires a matching binding and bone array.
	virtual void FilterTransforms( const FilterTransformsParams_t &params ) = 0;
	// AMNOTE: Returns one of four cached simulation transforms. Requires a frame index from zero through three.
	virtual CTransform GetSimulationTransform( int nFrame ) const = 0;
	// AMNOTE: Updates the simulation transform and applies its movement to node positions; teleporting clears motion history.
	virtual void SetSimulationTransform( const CTransform &tm, bool bTeleport ) = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk101() = 0;
	virtual void unk102() = 0;

public:
	// AMNOTE: Sets the control pose from an array containing GetCtrlCount() transforms.
	virtual void SetPose( const CTransform *pPose ) = 0;
	// AMNOTE: Sets the control pose by composing each pose transform with tm.
	virtual void SetPose( const CTransform &tm, const CTransform *pPose ) = 0;

	// AMNOTE: Sets the mapped bone pose and restores node positions from it, clearing motion history. Requires a binding and transform array.
	virtual void SetPoseFromBones( const VPhys2SoftbodyAnimationBinding *pBinding, const CTransform *pBones ) = 0;
	// AMNOTE: Returns a borrowed node-position array; zero selects the preceding frame and every nonzero value selects the current frame.
	virtual const VectorAligned *GetNodePositions( int nFrame = 1 ) const = 0;
	// AMNOTE: Returns a borrowed node-position array; zero selects the preceding frame and every nonzero value selects the current frame.
	virtual VectorAligned *GetNodePositions( int nFrame = 1 ) = 0;
	// AMNOTE: Returns the borrowed node positions cached from the animated control pose.
	virtual const Vector *GetCachedNodePositions() const = 0;
	// AMNOTE: Copies node positions into the selected frame; zero selects the preceding frame and every nonzero value selects the current frame. A negative count selects all nodes.
	virtual void SetNodePositions( const Vector *pPos, int nCount, int nFrame = 1 ) = 0;
	// AMNOTE: Stores the supplied simulation transform and copies aligned node positions into the selected frame. A negative count selects all nodes.
	virtual void SetNodePositions( SetNodePositionsParams_t &params ) = 0;
	// AMNOTE: Writes GetNodeCount() aligned positions by interpolating the preceding and current frames; the factor is not clamped.
	virtual void ComputeInterpolatedNodePositions( float flFactor, VectorAligned *pPosOut ) = 0;
	// AMNOTE: Adds scaled velocity to every dynamic node by changing the preceding position frame; frozen and dormant bodies are unchanged.
	virtual void AddVelocity( fltx4 vVelocity, float flTimeStep ) = 0;
	// AMNOTE: Adds scaled velocity to a dynamic node by changing its preceding position; static, frozen and dormant nodes are unchanged. Requires a valid node index.
	virtual void AddNodeVelocity( uint32 nNode, const fltx4 &vVelocity, float flTimeStep ) = 0;
	// AMNOTE: Returns the number of nodes, including static nodes.
	virtual uint32 GetNodeCount() const = 0;
	// AMNOTE: Returns the difference between the current and preceding node positions divided by the last timestep. Requires a valid node index; a zero timestep is not guarded.
	virtual Vector GetNodeVelocity( uint32 nNode ) const = 0;
	// AMNOTE: Scales movement between node frames and limits its magnitude, then clears transform motion history. A nonpositive maximum or zero scale clears movement.
	virtual void LimitNodeDisplacements( float flMaxDistance, float flScale ) = 0;
	// AMNOTE: Returns the number of controls.
	virtual uint32 GetCtrlCount() const = 0;
	// AMNOTE: Reads named particle positions and velocities from text and initializes their two position frames.
	virtual void ParseParticleState( CUtlBuffer &buf, float flTimeStep ) = 0;
	// AMNOTE: Returns text containing each named control's current position, animated position and velocity.
	virtual CUtlString PrintParticleState() const = 0;
	// AMNOTE: Returns the control count.
	virtual int GetParticleCount() const = 0;
	// AMNOTE: Sets one control transform. Requires a valid control index.
	virtual void SetCtrl( int nCtrl, const CTransform &tm ) = 0;
	// AMNOTE: Returns the difference between the indexed current and preceding node positions divided by the last timestep. Requires a valid control index; a zero timestep is not guarded.
	virtual Vector GetCtrlVelocity( uint32 nCtrl ) const = 0;
	// AMNOTE: Stores the surface-stretch value without clamping.
	virtual void SetSurfaceStretch( float flSurfaceStretch ) = 0;
	// AMNOTE: Returns the stored surface-stretch value.
	virtual float GetSurfaceStretch() const = 0;
	// AMNOTE: Creates or updates a named model effect using optional settings; returns null when no effect matches or initialization fails.
	virtual ISoftbodyEffect *CreateEffect( uint32 nNameHash, const KeyValues3 *pSettings = nullptr ) = 0;
	// AMNOTE: Returns a borrowed effect with the requested name hash, or null when none matches.
	virtual ISoftbodyEffect *FindEffect( uint32 nNameHash ) = 0;
	// AMNOTE: Removes every attached effect with the requested name hash.
	virtual void RemoveEffect( uint32 nNameHash ) = 0;
	// AMNOTE: Enables or disables spring integration.
	virtual void EnableSprings( bool bEnable ) = 0;
	// AMNOTE: Returns whether spring integration is enabled.
	virtual bool AreSpringsEnabled() const = 0;

	// AMNOTE: Selects elastic energy for zero, kinetic energy for one, potential energy for two, and their sum otherwise.
	virtual float GetEnergy( int nEnergy ) const = 0;
	// AMNOTE: Computes elastic energy from the current node positions and the last timestep.
	virtual float GetElasticEnergy() const = 0;
	// AMNOTE: Computes gravitational potential energy for dynamic nodes.
	virtual float GetPotentialEnergy() const = 0;
	// AMNOTE: Computes kinetic energy from the two node-position frames and the last timestep.
	virtual float GetKinematicEnergy() const = 0;
	// AMNOTE: Stores the multiplier applied to per-node damping without clamping.
	virtual void SetDampingMultiplier( float flMul ) = 0;
	// AMNOTE: Returns the stored per-node damping multiplier.
	virtual float GetDampingMultiplier() const = 0;
	// AMNOTE: Activates a dormant body unless dormancy is forced.
	virtual void GoWakeup() = 0;
	// AMNOTE: Returns whether the body is in the active state.
	virtual bool IsActive() const = 0;
	// AMNOTE: Returns whether simulation is forcibly dormant.
	virtual bool IsFrozen() const = 0;
	// AMNOTE: Forces dormancy while frozen; unfreezing resumes activity when the awake override is enabled.
	virtual void SetFrozen( bool bFrozen ) = 0;
	// AMNOTE: Returns the stored velocity air-drag coefficient.
	virtual float GetVelAirDrag() const = 0;
	// AMNOTE: Stores the velocity air-drag coefficient without clamping.
	virtual void SetVelAirDrag( float flVelAirDrag ) = 0;
	// AMNOTE: Returns the stored exponential air-drag coefficient.
	virtual float GetExpAirDrag() const = 0;
	// AMNOTE: Stores the exponential air-drag coefficient without clamping.
	virtual void SetExpAirDrag( float flExpAirDrag ) = 0;
	// AMNOTE: Returns the stored quad velocity air-drag coefficient.
	virtual float GetVelQuadAirDrag() const = 0;
	// AMNOTE: Stores the quad velocity air-drag coefficient without clamping.
	virtual void SetVelQuadAirDrag( float flVelQuadAirDrag ) = 0;
	// AMNOTE: Returns the stored quad exponential air-drag coefficient.
	virtual float GetExpQuadAirDrag() const = 0;
	// AMNOTE: Stores the quad exponential air-drag coefficient without clamping.
	virtual void SetExpQuadAirDrag( float flExpQuadAirDrag ) = 0;
	// AMNOTE: Returns the stored quad velocity-smoothing rate.
	virtual float GetQuadVelocitySmoothRate() const = 0;
	// AMNOTE: Stores the quad velocity-smoothing rate without clamping.
	virtual void SetQuadVelocitySmoothRate( float flRate ) = 0;
	// AMNOTE: Returns the stored rod velocity-smoothing iteration count.
	virtual uint16 ReadRodVelocitySmoothIterations() const = 0;
	// AMNOTE: Stores the quad velocity-smoothing iteration count.
	virtual void SetQuadVelocitySmoothIterations( uint16 nIterations ) = 0;
	// AMNOTE: Returns the stored rod velocity-smoothing rate.
	virtual float GetRodVelocitySmoothRate() const = 0;
	// AMNOTE: Stores the rod velocity-smoothing rate without clamping.
	virtual void SetRodVelocitySmoothRate( float flRate ) = 0;
	// AMNOTE: Returns the rod velocity-smoothing iteration count.
	virtual uint16 GetRodVelocitySmoothIterations() const = 0;
	// AMNOTE: Stores the rod velocity-smoothing iteration count.
	virtual void SetRodVelocitySmoothIterations( uint16 nIterations ) = 0;

	// AMNOTE: Enables motion smoothing when the model supplies a positive smoothing coefficient.
	virtual void EnableMotionSmoothing( bool bEnable ) = 0;
	// AMNOTE: Returns whether motion smoothing is enabled, independently of the model coefficient.
	virtual bool IsMotionSmoothingEnabled() const = 0;

	// AMNOTE: Returns whether the body is in the dormant state.
	virtual bool IsDormant() const = 0;
	// AMNOTE: Requests dormancy unless the awake override is set.
	virtual void GoDormant() = 0;
	// AMNOTE: Increments an 8-bit counter; when its preceding value exceeds three, requests dormancy and returns true.
	virtual bool AdvanceSleepCounter() = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk201() = 0;
	virtual void unk202() = 0;
	virtual void unk203() = 0;
	virtual void unk204() = 0;

public:
	// AMNOTE: Returns current node bounds, using cached dynamic-node bounds when available.
	virtual AABB_t BuildBounds() const = 0;
	// AMNOTE: Returns the borrowed collision attributes.
	virtual const RnCollisionAttr_t &GetCollisionAttributes() const = 0;
	// AMNOTE: Stores the collision attributes unchanged.
	virtual void SetCollisionAttributes( const RnCollisionAttr_t &attr ) = 0;
	// AMNOTE: Returns zero in this engine build.
	virtual uint32 GetContactCount() const = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk301() = 0;
	// AMNOTE: Enables or disables ground collision for an indexed disable reason. The full parameter contract is unresolved; this reservation is not callable.
	virtual void EnableGroundCollision() = 0;

public:
	// AMNOTE: Returns whether no ground-collision disable reason is set.
	virtual bool IsGroundCollisionEnabled() const = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk401() = 0;
	virtual void unk402() = 0;

public:
	// AMNOTE: Sets the ground height and copies its horizontal origin from the simulation origin.
	virtual void SetGroundZ( float flGroundZ ) = 0;
	// AMNOTE: Returns the stored ground height.
	virtual float GetGroundZ() const = 0;
	// AMNOTE: Refreshes the cached ground trace when the supplied origin moves sufficiently far from the preceding trace origin and the model has collision spheres.
	virtual void UpdateGroundTrace( fltx4 vOrigin ) = 0;
	// AMNOTE: Stores the vertical offset added to the ground-trace start without clamping.
	virtual void SetGroundTraceStartOffset( float flOffset ) = 0;
	// AMNOTE: Returns the vertical offset added to the ground-trace start.
	virtual float GetGroundTraceStartOffset() const = 0;
	// AMNOTE: Returns the closest node inside the forward cone, or zero when none matches; the direction must be nonzero.
	virtual int CastCone( const Vector &vStart, const Vector &vDir, float flConePitch ) = 0;
	// AMNOTE: Returns whether the FTL rod-relaxation pass is enabled.
	virtual bool IsFtlPassEnabled() const = 0;
	// AMNOTE: Enables or disables the FTL rod-relaxation pass.
	virtual void EnableFtlPass( bool bEnable ) = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk501() = 0;
	virtual void unk502() = 0;

public:
	// AMNOTE: Restores node positions from the cached pose, copies the current frame to the preceding frame and clears motion history.
	virtual void ResetVelocities() = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk601() = 0;
	virtual void unk602() = 0;
	virtual void unk603() = 0;
	virtual void unk604() = 0;

public:
	// AMNOTE: Returns the last simulation timestep.
	virtual float GetTimeStep() const = 0;
	// AMNOTE: Enables or disables automatic sleep detection.
	virtual void EnableSleeping( bool bEnable ) = 0;
	// AMNOTE: Returns whether automatic sleep detection is enabled.
	virtual bool IsSleepingEnabled() const = 0;
	// AMNOTE: Controls the awake override; resumes a dormant body when it is not frozen.
	virtual void ForceAwake( bool bForceAwake ) = 0;
	// AMNOTE: Returns whether the awake override is enabled.
	virtual bool IsAwakeForced() const = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk701() = 0;

public:
	// AMNOTE: Stores a borrowed user pointer in one of two slots; indices outside the range are ignored.
	virtual void SetUserData( uint32 nIndex, void *pData ) = 0;
	// AMNOTE: Returns the user pointer in the requested slot, or null for an invalid index.
	virtual void *GetUserData( uint32 nIndex ) = 0;

	// AMNOTE: Starts changing a property's target now, replacing later updates for the selection. Speed is units per second; speed at or below FLT_EPSILON snaps to the target.
	// AMNOTE: A zero hash selects all ordinary vertex sets. Nonzero hashes select a named vertex set or effect, falling back to the default set when unmatched.
	virtual void SetInterpolatedProperty( SoftbodyInterpolatedProperty_t property, float flValue, float flSpeed, uint32 nNameHash ) = 0;
	// AMNOTE: Schedules a target change after a positive delay, or starts it now otherwise, replacing later updates for the same property and selection. Named effects can also be selected by hash.
	virtual void SetInterpolatedPropertyDelayed( SoftbodyInterpolatedProperty_t property, float flDelay, float flValue, float flSpeed, uint32 nNameHash ) = 0;
	// AMNOTE: Returns the current interpolated value for the named vertex set or effect; an unmatched hash selects the default set.
	virtual float GetInterpolatedProperty( SoftbodyInterpolatedProperty_t property, uint32 nNameHash ) const = 0;
	// AMNOTE: Appends matching scheduled times measured from now, preserving existing output entries. A zero hash selects broadcast updates only; a nonzero hash also includes updates for the selected vertex set or effect.
	virtual void AppendInterpolatedPropertyTimes( SoftbodyInterpolatedProperty_t property, CUtlVector<float> &times, uint32 nNameHash ) const = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk801() = 0;

public:
	// AMNOTE: Controls whether world softbody simulation skips this body.
	virtual void SetWorldSimulationDisabled( bool bDisabled ) = 0;
	// AMNOTE: Returns whether world softbody simulation skips this body.
	virtual bool IsWorldSimulationDisabled() const = 0;
	// AMNOTE: Adds an outgoing softbody link and its corresponding incoming link; self-links are ignored. Requires a non-null body.
	virtual void AddLinkedSoftbody( IPhysicsSoftbody *pOther ) = 0;
	// AMNOTE: Removes matching outgoing softbody links and their corresponding incoming links. Requires a non-null body.
	virtual void RemoveLinkedSoftbody( IPhysicsSoftbody *pOther ) = 0;
	// AMNOTE: Removes all outgoing softbody links and their corresponding incoming links.
	virtual void RemoveAllLinkedSoftbodies() = 0;
	// AMNOTE: Rebuilds model-bone links to a parent softbody; null clears the parent.
	virtual void SetLinkParent( IPhysicsSoftbody *pParent ) = 0;
	// AMNOTE: Returns whether a simulation object is present and active.
	virtual bool IsValidAndActive() const = 0;
	// AMNOTE: Has no effect in this engine build.
	virtual void SetDebugNameV( const char *pNameFormat, va_list args ) = 0;
	// AMNOTE: Returns null in this engine build.
	virtual const char *GetDebugName() const = 0;
	// AMNOTE: Returns the index in the owning softbody collection, or minus one before registration.
	virtual int GetIndexInWorld() const = 0;
	// AMNOTE: Returns the borrowed simulation object.
	virtual CRnSoftbody *GetSoftbody() = 0;

private:
	// AMNOTE: Unresolved source contracts; these reservations are not callable.
	virtual void unk901() = 0;
};

// AMNOTE: Engine-owned adapter. The engine supplies destruction and all operations not defined here.
class CPhysicsSoftbody : public IPhysicsSoftbody
{
public:
	inline uint32 GetStateHash() const override;
	inline Vector GetNodeVelocity( uint32 nNode ) const override;
	// AMNOTE: NaN payloads for nonfinite inputs can vary with the compiler.
	inline void ComputeInterpolatedNodePositions( float flFactor, VectorAligned *pPosOut ) override;
	CTransform *GetSimulatedTransforms() override { return GetAnimatedTransforms() + GetCtrlCount(); }
	const CTransform *GetSimulatedTransforms() const override { return GetAnimatedTransforms() + GetCtrlCount(); }

	using IPhysicsSoftbody::GetFeModel;
	const CFeModel *GetFeModel() const override { return const_cast<IPhysicsSoftbody *>( static_cast<const IPhysicsSoftbody *>( this ) )->GetFeModel(); }
	int GetParticleCount() const override { return static_cast<int>( GetCtrlCount() ); }
	Vector GetCtrlVelocity( uint32 nCtrl ) const override { return GetNodeVelocity( nCtrl ); }
	// AMNOTE: NaN bit patterns for nonfinite inputs can vary with the compiler.
	inline float GetEnergy( int nEnergy ) const override;
	uint16 ReadRodVelocitySmoothIterations() const override { return GetRodVelocitySmoothIterations(); }
	uint32 GetContactCount() const override { return 0; }
	bool IsValidAndActive() const override { return m_pSoftbody != nullptr && IsActive(); }
	void SetDebugNameV( const char *, va_list ) override {}
	const char *GetDebugName() const override { return nullptr; }

	CRnSoftbody *GetSoftbody() override { return m_pSoftbody; }
	const CRnSoftbody *GetSoftbody() const { return m_pSoftbody; }
	void SetUserData( uint32 nIndex, void *pData ) override { if ( nIndex < 2 ) m_pUserData[nIndex] = pData; }
	void *GetUserData( uint32 nIndex ) override { return nIndex < 2 ? m_pUserData[nIndex] : nullptr; }
	void SetInterpolatedProperty( SoftbodyInterpolatedProperty_t property, float flValue, float flSpeed, uint32 nNameHash ) override { SetInterpolatedPropertyDelayed( property, 0.0f, flValue, flSpeed, nNameHash ); }

private:
	CPhysicsSoftbody() = delete;
	CPhysicsSoftbody( const CPhysicsSoftbody & ) = delete;
	CPhysicsSoftbody &operator=( const CPhysicsSoftbody & ) = delete;
	~CPhysicsSoftbody() override;

	// AMNOTE: Borrowed simulation object; the engine controls its lifetime.
	CRnSoftbody *m_pSoftbody;
	void *m_pUserData[2];
};

inline void CPhysicsSoftbody::ComputeInterpolatedNodePositions( float flFactor, VectorAligned *pPosOut )
{
	const uint32 nCount = GetNodeCount();
	if ( nCount == 0 )
		return;

	const VectorAligned *pPrevious = GetNodePositions( 0 );
	const VectorAligned *pCurrent = GetNodePositions( 1 );
	const fltx4 vFactor = ReplicateX4( flFactor );
	const fltx4 vPreviousFactor = ReplicateX4( 1.0f - flFactor );
	for ( uint32 nNode = 0; nNode < nCount; ++nNode )
	{
		const fltx4 vCurrent = MulSIMD( LoadAlignedSIMD( pCurrent[nNode].Base() ), vFactor );
		const fltx4 vPrevious = MulSIMD( LoadAlignedSIMD( pPrevious[nNode].Base() ), vPreviousFactor );
		StoreAlignedSIMD( pPosOut[nNode].Base(), AddSIMD( vCurrent, vPrevious ) );
	}
}

inline Vector CPhysicsSoftbody::GetNodeVelocity( uint32 nNode ) const
{
	const fltx4 vCurrent = LoadAlignedSIMD( GetNodePositions( 1 )[nNode].Base() );
	const fltx4 vPrevious = LoadAlignedSIMD( GetNodePositions( 0 )[nNode].Base() );
	Vector vVelocity;
	StoreUnaligned3SIMD( vVelocity.Base(), DivSIMD( SubSIMD( vCurrent, vPrevious ), ReplicateX4( GetTimeStep() ) ) );
	return vVelocity;
}

inline float CPhysicsSoftbody::GetEnergy( int nEnergy ) const
{
	switch ( nEnergy )
	{
	case 0:
		return GetElasticEnergy();
	case 1:
		return GetKinematicEnergy();
	case 2:
		return GetPotentialEnergy();
	default:
#ifdef COMPILER_MSVC
		const float flFirst = GetKinematicEnergy();
		const float flSecond = GetElasticEnergy();
#else
		const float flFirst = GetElasticEnergy();
		const float flSecond = GetKinematicEnergy();
#endif
		const float flCombined = flFirst + flSecond;
		return GetPotentialEnergy() + flCombined;
	}
}

inline uint32 CPhysicsSoftbody::GetStateHash() const
{
	CRC32_t crc;
	CRC32_Init( &crc );
	const int nByteCount = GetNodeCount() * sizeof( VectorAligned );
	CRC32_ProcessBuffer( &crc, GetNodePositions( 0 ), nByteCount );
	CRC32_ProcessBuffer( &crc, GetNodePositions( 1 ), nByteCount );
	CRC32_Final( &crc );
	return crc;
}

#endif // IPHYSICSSOFTBODY_H
