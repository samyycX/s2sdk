#ifndef IPHYSAGGREGATEINSTANCE_H
#define IPHYSAGGREGATEINSTANCE_H

#ifdef _WIN32
#pragma once
#endif

#include "physicslib/physaggregatedata.h"

#include "tier0/platform.h"
#include "schemasystem/minschema.h"
#include "tier0/globalsymbol.h"
#include "gametrace.h"
#include "tier0/utlvector.h"
#include "tier0/utlstring.h"
#include "tier0/keyvalues3.h"
#include "mathlib/transform.h"
#include "mathlib/ssequaternion.h"
#include "vphysics2/iphysicsbody.h"
#include "vphysics2/iphysicsjoint.h"
#include "vphysics2/physicscast.h"

class IVPhysics2World;
class CPhysAggregateData;
class IPhysAggregateInstance;

struct ALIGN16 FourVectors2D
{
	fltx4 x;
	fltx4 y;
} ALIGN16_POST;

struct ALIGN4 CFeJiggleBone
{
	uint32 m_nFlags;
	float m_flLength;
	float m_flTipMass;
	float m_flYawStiffness;
	float m_flYawDamping;
	float m_flPitchStiffness;
	float m_flPitchDamping;
	float m_flAlongStiffness;
	float m_flAlongDamping;
	float m_flAngleLimit;
	float m_flMinYaw;
	float m_flMaxYaw;
	float m_flYawFriction;
	float m_flYawBounce;
	float m_flMinPitch;
	float m_flMaxPitch;
	float m_flPitchFriction;
	float m_flPitchBounce;
	float m_flBaseMass;
	float m_flBaseStiffness;
	float m_flBaseDamping;
	float m_flBaseMinLeft;
	float m_flBaseMaxLeft;
	float m_flBaseLeftFriction;
	float m_flBaseMinUp;
	float m_flBaseMaxUp;
	float m_flBaseUpFriction;
	float m_flBaseMinForward;
	float m_flBaseMaxForward;
	float m_flBaseForwardFriction;
	float m_flRadius0;
	float m_flRadius1;
	Vector m_vPoint0;
	Vector m_vPoint1;
	uint16 m_nCollisionMask;
} ALIGN4_POST;

struct ALIGN4 CFeIndexedJiggleBone
{
	uint32 m_nNode;
	uint32 m_nJiggleParent;
	CFeJiggleBone m_jiggleBone;
} ALIGN4_POST;

struct ALIGN4 FeAnimStrayRadius_t
{
	uint16 nNode[2];
	float flMaxDist;
	float flRelaxationFactor;
} ALIGN4_POST;

struct ALIGN4 FeAntiTunnelProbe_t
{
	float flWeight;
	uint32 nFlags;
	uint16 nProbeNode;
	uint16 nCount;
	uint32 nBegin;
	float flActivationDistance;
	float flCurvatureRadius;
	float flBias;
} ALIGN4_POST;

struct ALIGN4 FeAxialEdgeBend_t
{
	float te;
	float tv;
	float flDist;
	float flWeight[4];
	uint16 nNode[6];
} ALIGN4_POST;

struct ALIGN4 FeBoneMergeLink_t
{
	uint32 m_nParentHash;
	uint16 m_nChildNode;
} ALIGN4_POST;

struct ALIGN16 FeBoxRigid_t
{
	CTransform tmFrame2;
	uint16 nNode;
	uint16 nCollisionMask;
	Vector vSize;
	uint16 nVertexMapIndex;
	uint16 nFlags;
} ALIGN16_POST;

struct ALIGN4 FeCollisionPlane_t
{
	uint16 nCtrlParent;
	uint16 nChildNode;
	RnPlane_t m_Plane;
	float flStrength;
} ALIGN4_POST;

struct ALIGN4 FeCtrlOffset_t
{
	Vector vOffset;
	uint16 nCtrlParent;
	uint16 nCtrlChild;
} ALIGN4_POST;

struct ALIGN4 FeCtrlOsOffset_t
{
	uint16 nCtrlParent;
	uint16 nCtrlChild;
} ALIGN4_POST;

struct ALIGN4 FeCtrlSoftOffset_t
{
	uint16 nCtrlParent;
	uint16 nCtrlChild;
	Vector vOffset;
	float flAlpha;
} ALIGN4_POST;

struct FeDynKinLink_t
{
	uint16 m_nParent;
	uint16 m_nChild;
};

struct ALIGN16 FeFitMatrix_t
{
	CTransform bone;
	Vector vCenter;
	uint16 nEnd;
	uint16 nNode;
	uint16 nBeginDynamic;
} ALIGN16_POST;

struct ALIGN4 FeFitWeight_t
{
	float flWeight;
	uint16 nNode;
	uint16 nDummy;
} ALIGN4_POST;

struct ALIGN4 FeFollowNode_t
{
	uint16 nParentNode;
	uint16 nChildNode;
	float flWeight;
} ALIGN4_POST;

struct ALIGN4 FeHingeLimit_t
{
	uint16 nNode[6];
	uint32 nFlags;
	float flWeight4;
	float flWeight5;
	float flAngleCenter;
	float flAngleExtents;
} ALIGN4_POST;

struct ALIGN4 FeKelagerBend2_t
{
	float flWeight[3];
	float flHeight0;
	uint16 nNode[3];
	uint16 nReserved;
} ALIGN4_POST;

struct ALIGN4 FeNodeIntegrator_t
{
	float flPointDamping;
	float flAnimationForceAttraction;
	float flAnimationVertexAttraction;
	float flGravity;
} ALIGN4_POST;

struct ALIGN4 FeNodeReverseOffset_t
{
	Vector vOffset;
	uint16 nBoneCtrl;
	uint16 nTargetNode;
} ALIGN4_POST;

struct ALIGN16 FeNodeStrayBox_t
{
	Vector vMin;
	uint32 nFlags;
	Vector vMax;
	uint16 nNode[2];
} ALIGN16_POST;

struct FeNodeWindBase_t
{
	uint16 nNodeX0;
	uint16 nNodeX1;
	uint16 nNodeY0;
	uint16 nNodeY1;
};

struct ALIGN4 FePrism_t
{
	uint16 nNode[6];
	float flVolume;
} ALIGN4_POST;

struct ALIGN4 FeQuad_t
{
	uint16 nNode[4];
	float flSlack;
	Vector4D vShape[4];
} ALIGN4_POST;

struct FeRigidColliderIndices_t
{
	uint16 m_nTaperedCapsuleRigidIndex;
	uint16 m_nSphereRigidIndex;
	uint16 m_nBoxRigidIndex;
	uint16 m_nSDFRigidIndex;
	uint16 m_nCollisionPlaneIndex;
};

struct ALIGN4 FeRodConstraint_t
{
	uint16 nNode[2];
	float flMaxDist;
	float flMinDist;
	float flWeight0;
	float flRelaxationFactor;
} ALIGN4_POST;

struct ALIGN16 FeSimdAnimStrayRadius_t
{
	uint16 nNode[2][4];
	fltx4 flMaxDist;
	fltx4 flRelaxationFactor;
} ALIGN16_POST;

struct ALIGN16 FeSimdNodeBase_t
{
	uint16 nNode[4];
	uint16 nNodeX0[4];
	uint16 nNodeX1[4];
	uint16 nNodeY0[4];
	uint16 nNodeY1[4];
	uint16 nDummy[4];
	FourQuaternions qAdjust;
} ALIGN16_POST;

struct ALIGN16 FeSimdPrism_t
{
	uint16 nNode[6][4];
	fltx4 flVolume;
} ALIGN16_POST;

struct ALIGN16 FeSimdQuad_t
{
	uint16 nNode[4][4];
	fltx4 f4Slack;
	FourVectors vShape[4];
	fltx4 f4Weights[4];
} ALIGN16_POST;

struct ALIGN16 FeSimdRodConstraintAnim_t
{
	uint16 nNode[2][4];
	fltx4 f4Weight0;
	fltx4 f4RelaxationFactor;
} ALIGN16_POST;

struct ALIGN16 FeSimdRodConstraint_t
{
	uint16 nNode[2][4];
	fltx4 f4MaxDist;
	fltx4 f4MinDist;
	fltx4 f4Weight0;
	fltx4 f4RelaxationFactor;
} ALIGN16_POST;

struct ALIGN16 FeSimdSpringIntegrator_t
{
	uint16 nNode[2][4];
	fltx4 flSpringRestLength;
	fltx4 flSpringConstant;
	fltx4 flSpringDamping;
	fltx4 flNodeWeight0;
} ALIGN16_POST;

struct ALIGN16 FeSimdTri_t
{
	uint32 nNode[3][4];
	fltx4 w1;
	fltx4 w2;
	fltx4 v1x;
	FourVectors2D v2;
} ALIGN16_POST;

struct ALIGN16 FeSphereRigid_t
{
	fltx4 vSphere;
	uint16 nNode;
	uint16 nCollisionMask;
	uint16 nVertexMapIndex;
	uint16 nFlags;
} ALIGN16_POST;

struct ALIGN4 FeSpringIntegrator_t
{
	uint16 nNode[2];
	float flSpringRestLength;
	float flSpringConstant;
	float flSpringDamping;
	float flNodeWeight0;
} ALIGN4_POST;

struct ALIGN16 FeTaperedCapsuleRigid_t
{
	fltx4 vSphere[2];
	uint16 nNode;
	uint16 nCollisionMask;
	uint16 nVertexMapIndex;
	uint16 nFlags;
} ALIGN16_POST;

struct ALIGN4 FeTaperedCapsuleStretch_t
{
	uint16 nNode[2];
	uint16 nCollisionMask;
	uint16 nDummy;
	float flRadius[2];
} ALIGN4_POST;

struct FeTreeChildren_t
{
	uint16 nChild[2];
};

struct ALIGN4 FeTri_t
{
	uint16 nNode[3];
	float w1;
	float w2;
	float v1x;
	Vector2D v2;
} ALIGN4_POST;

struct ALIGN4 FeTwistConstraint_t
{
	uint16 nNodeOrient;
	uint16 nNodeEnd;
	float flTwistRelax;
	float flSwingRelax;
} ALIGN4_POST;

struct ALIGN4 FeWorldCollisionParams_t
{
	float flWorldFriction;
	float flGroundFriction;
	uint16 nListBegin;
	uint16 nListEnd;
} ALIGN4_POST;


struct ALIGN16 FeNodeBase_t
{
	uint16 nNode;
	uint16 nDummy[3];
	uint16 nNodeX0;
	uint16 nNodeX1;
	uint16 nNodeY0;
	uint16 nNodeY1;
	QuaternionStorage qAdjust;
} ALIGN16_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 FeEffectDesc_t
{
	CUtlString sName;
	uint32 nNameHash;
	int32 nType;
	KeyValues3 m_Params;

private:
	FeEffectDesc_t() = delete;
	FeEffectDesc_t( const FeEffectDesc_t & ) = delete;
	FeEffectDesc_t &operator=( const FeEffectDesc_t & ) = delete;
	~FeEffectDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 FeModelSelfCollisionLayer_t
{
	CUtlString m_Name;
	CUtlVector< uint16 > m_Nodes;
	float m_flParentReaction;
	uint32 m_nFlags;
	uint32 m_nEndIdx[4];

private:
	FeModelSelfCollisionLayer_t() = delete;
	FeModelSelfCollisionLayer_t( const FeModelSelfCollisionLayer_t & ) = delete;
	FeModelSelfCollisionLayer_t &operator=( const FeModelSelfCollisionLayer_t & ) = delete;
	~FeModelSelfCollisionLayer_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 FeMorphLayerDepr_t
{
	CUtlString m_Name;
	uint32 m_nNameHash;
	CUtlVector< uint16 > m_Nodes;
	CUtlVector< Vector > m_InitPos;
	CUtlVector< float > m_Gravity;
	CUtlVector< float > m_GoalStrength;
	CUtlVector< float > m_GoalDamping;
	uint32 m_nFlags;

private:
	FeMorphLayerDepr_t() = delete;
	FeMorphLayerDepr_t( const FeMorphLayerDepr_t & ) = delete;
	FeMorphLayerDepr_t &operator=( const FeMorphLayerDepr_t & ) = delete;
	~FeMorphLayerDepr_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 FeSDFRigid_t
{
	Vector vLocalMin;
	Vector vLocalMax;
	float flBounciness;
	uint16 nNode;
	uint16 nCollisionMask;
	uint16 nVertexMapIndex;
	uint16 nFlags;
	CUtlVector< float > m_Distances;
	int32 m_nWidth;
	int32 m_nHeight;
	int32 m_nDepth;

private:
	FeSDFRigid_t() = delete;
	FeSDFRigid_t( const FeSDFRigid_t & ) = delete;
	FeSDFRigid_t &operator=( const FeSDFRigid_t & ) = delete;
	~FeSDFRigid_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 FeVertexMapDesc_t
{
	CUtlString sName;
	uint32 nNameHash;
	uint32 nColor;
	uint32 nFlags;
	uint16 nVertexBase;
	uint16 nVertexCount;
	uint32 nMapOffset;
	uint32 nNodeListOffset;
	Vector vCenterOfMass;
	float flVolumetricSolveStrength;
	int16 nScaleSourceNode;
	uint16 nNodeListCount;

private:
	FeVertexMapDesc_t() = delete;
	FeVertexMapDesc_t( const FeVertexMapDesc_t & ) = delete;
	FeVertexMapDesc_t &operator=( const FeVertexMapDesc_t & ) = delete;
	~FeVertexMapDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 PhysFeModelDesc_t
{
	CUtlVector< uint32 > m_CtrlHash;
	CUtlVector< CUtlString > m_CtrlName;
	uint32 m_nStaticNodeFlags;
	uint32 m_nDynamicNodeFlags;
	float m_flLocalForce;
	float m_flLocalRotation;
	uint16 m_nNodeCount;
	uint16 m_nStaticNodes;
	uint16 m_nRotLockStaticNodes;
	uint16 m_nFirstPositionDrivenNode;
	uint16 m_nSimdTriCount1;
	uint16 m_nSimdTriCount2;
	uint16 m_nSimdQuadCount1;
	uint16 m_nSimdQuadCount2;
	uint16 m_nQuadCount1;
	uint16 m_nQuadCount2;
	uint16 m_nTreeDepth;
	uint16 m_nNodeBaseJiggleboneDependsCount;
	uint16 m_nRopeCount;
	CUtlVector< uint16 > m_Ropes;
	CUtlVector< FeNodeBase_t > m_NodeBases;
	CUtlVector< FeSimdNodeBase_t > m_SimdNodeBases;
	CUtlVector< FeQuad_t > m_Quads;
	CUtlVector< FeSimdQuad_t > m_SimdQuads;
	CUtlVector< FeSimdTri_t > m_SimdTris;
	CUtlVector< FePrism_t > m_Prisms;
	CUtlVector< FeSimdPrism_t > m_SimdPrisms;
	CUtlVector< FeSimdRodConstraint_t > m_SimdRods;
	CUtlVector< FeSimdRodConstraintAnim_t > m_SimdRodsAnim;
	CUtlVector< CTransform > m_InitPose;
	CUtlVector< FeRodConstraint_t > m_Rods;
	CUtlVector< FeTwistConstraint_t > m_Twists;
	CUtlVector< FeHingeLimit_t > m_HingeLimits;
	CUtlVector< uint32 > m_AntiTunnelBytecode;
	CUtlVector< FeDynKinLink_t > m_DynKinLinks;
	CUtlVector< FeBoneMergeLink_t > m_BoneMergeLinks;
	CUtlVector< FeAntiTunnelProbe_t > m_AntiTunnelProbes;
	CUtlVector< uint16 > m_AntiTunnelTargetNodes;
	CUtlVector< FeNodeStrayBox_t > m_NodeStrayBoxes;
	CUtlVector< FeAxialEdgeBend_t > m_AxialEdges;
	CUtlVector< float > m_NodeInvMasses;
	CUtlVector< FeCtrlOffset_t > m_CtrlOffsets;
	CUtlVector< FeCtrlOsOffset_t > m_CtrlOsOffsets;
	CUtlVector< FeFollowNode_t > m_FollowNodes;
	CUtlVector< FeCollisionPlane_t > m_CollisionPlanes;
	CUtlVector< FeNodeIntegrator_t > m_NodeIntegrator;
	CUtlVector< FeSpringIntegrator_t > m_SpringIntegrator;
	CUtlVector< FeSimdSpringIntegrator_t > m_SimdSpringIntegrator;
	CUtlVector< FeWorldCollisionParams_t > m_WorldCollisionParams;
	CUtlVector< float > m_LegacyStretchForce;
	CUtlVector< float > m_NodeCollisionRadii;
	CUtlVector< float > m_DynNodeFriction;
	CUtlVector< float > m_LocalRotation;
	CUtlVector< float > m_LocalForce;
	CUtlVector< FeTaperedCapsuleStretch_t > m_TaperedCapsuleStretches;
	CUtlVector< FeTaperedCapsuleRigid_t > m_TaperedCapsuleRigids;
	CUtlVector< FeSphereRigid_t > m_SphereRigids;
	CUtlVector< uint16 > m_WorldCollisionNodes;
	CUtlVector< uint16 > m_TreeParents;
	CUtlVector< uint16 > m_TreeCollisionMasks;
	CUtlVector< FeTreeChildren_t > m_TreeChildren;
	CUtlVector< uint16 > m_FreeNodes;
	CUtlVector< FeFitMatrix_t > m_FitMatrices;
	CUtlVector< FeFitWeight_t > m_FitWeights;
	CUtlVector< FeNodeReverseOffset_t > m_ReverseOffsets;
	CUtlVector< FeAnimStrayRadius_t > m_AnimStrayRadii;
	CUtlVector< FeSimdAnimStrayRadius_t > m_SimdAnimStrayRadii;
	CUtlVector< FeKelagerBend2_t > m_KelagerBends;
	CUtlVector< FeCtrlSoftOffset_t > m_CtrlSoftOffsets;
	CUtlVector< CFeIndexedJiggleBone > m_JiggleBones;
	CUtlVector< uint16 > m_SourceElems;
	CUtlVector< uint32 > m_GoalDampedSpringIntegrators;
	CUtlVector< FeTri_t > m_Tris;
	uint16 m_nTriCount1;
	uint16 m_nTriCount2;
	uint8 m_nReservedUint8;
	uint8 m_nExtraPressureIterations;
	uint8 m_nExtraGoalIterations;
	uint8 m_nExtraIterations;
	CUtlVector< FeSDFRigid_t > m_SDFRigids;
	CUtlVector< FeBoxRigid_t > m_BoxRigids;
	CUtlVector< uint8 > m_DynNodeVertexSet;
	CUtlVector< uint32 > m_VertexSetNames;
	CUtlVector< FeRigidColliderIndices_t > m_RigidColliderPriorities;
	CUtlVector< FeMorphLayerDepr_t > m_MorphLayers;
	CUtlVector< uint8 > m_MorphSetData;
	CUtlVector< FeVertexMapDesc_t > m_VertexMaps;
	CUtlVector< uint8 > m_VertexMapValues;
	CUtlVector< FeEffectDesc_t > m_Effects;
	CUtlVector< FeCtrlOffset_t > m_LockToParent;
	CUtlVector< uint16 > m_LockToGoal;
	CUtlVector< int16 > m_SkelParents;
	CUtlVector< FeNodeWindBase_t > m_DynNodeWindBases;
	CUtlVector< FeModelSelfCollisionLayer_t > m_SelfCollisionLayers;
	float m_flInternalPressure;
	float m_flDefaultTimeDilation;
	float m_flWindage;
	float m_flWindDrag;
	float m_flDefaultSurfaceStretch;
	float m_flDefaultThreadStretch;
	float m_flDefaultGravityScale;
	float m_flDefaultVelAirDrag;
	float m_flDefaultExpAirDrag;
	float m_flDefaultVelQuadAirDrag;
	float m_flDefaultExpQuadAirDrag;
	float m_flRodVelocitySmoothRate;
	float m_flQuadVelocitySmoothRate;
	float m_flAddWorldCollisionRadius;
	float m_flDefaultVolumetricSolveAmount;
	float m_flMotionSmoothCDT;
	float m_flLocalDrag1;
	uint16 m_nRodVelocitySmoothIterations;
	uint16 m_nQuadVelocitySmoothIterations;

private:
	PhysFeModelDesc_t() = delete;
	PhysFeModelDesc_t( const PhysFeModelDesc_t & ) = delete;
	PhysFeModelDesc_t &operator=( const PhysFeModelDesc_t & ) = delete;
	~PhysFeModelDesc_t() = delete;
} ALIGN8_POST;


struct PhysShapeMarkup_t
{
	int32 m_nBodyInAggregate;
	int32 m_nShapeInBody;
	CGlobalSymbol m_sHitGroup;
};

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnShapeDesc_t
{
	uint32 m_nCollisionAttributeIndex;
	uint32 m_nSurfacePropertyIndex;
	CUtlString m_UserFriendlyName;
	bool m_bUserFriendlyNameSealed;
	bool m_bUserFriendlyNameLong;
	uint32 m_nToolMaterialHash;

private:
	RnShapeDesc_t() = delete;
	RnShapeDesc_t( const RnShapeDesc_t & ) = delete;
	RnShapeDesc_t &operator=( const RnShapeDesc_t & ) = delete;
	~RnShapeDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnSphereDesc_t : public RnShapeDesc_t
{
	RnSphere_t m_Sphere;

private:
	RnSphereDesc_t() = delete;
	RnSphereDesc_t( const RnSphereDesc_t & ) = delete;
	RnSphereDesc_t &operator=( const RnSphereDesc_t & ) = delete;
	~RnSphereDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnCapsuleDesc_t : public RnShapeDesc_t
{
	RnCapsule_t m_Capsule;

private:
	RnCapsuleDesc_t() = delete;
	RnCapsuleDesc_t( const RnCapsuleDesc_t & ) = delete;
	RnCapsuleDesc_t &operator=( const RnCapsuleDesc_t & ) = delete;
	~RnCapsuleDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnHullDesc_t : public RnShapeDesc_t
{
	RnHull_t m_Hull;

private:
	RnHullDesc_t() = delete;
	RnHullDesc_t( const RnHullDesc_t & ) = delete;
	RnHullDesc_t &operator=( const RnHullDesc_t & ) = delete;
	~RnHullDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnMeshDesc_t : public RnShapeDesc_t
{
	RnMesh_t m_Mesh;

private:
	RnMeshDesc_t() = delete;
	RnMeshDesc_t( const RnMeshDesc_t & ) = delete;
	RnMeshDesc_t &operator=( const RnMeshDesc_t & ) = delete;
	~RnMeshDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 RnCompoundDesc_t : public RnShapeDesc_t
{
	RnCompound_t m_Compound;

private:
	RnCompoundDesc_t() = delete;
	RnCompoundDesc_t( const RnCompoundDesc_t & ) = delete;
	RnCompoundDesc_t &operator=( const RnCompoundDesc_t & ) = delete;
	~RnCompoundDesc_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 VPhysics2ShapeDef_t
{
	CUtlLeanVector< RnSphereDesc_t > m_spheres;
	CUtlLeanVector< RnCapsuleDesc_t > m_capsules;
	CUtlLeanVector< RnHullDesc_t > m_hulls;
	CUtlLeanVector< RnMeshDesc_t > m_meshes;
	CUtlLeanVector< RnCompoundDesc_t > m_compounds;
	CUtlVector< uint16 > m_CollisionAttributeIndices;

private:
	VPhysics2ShapeDef_t() = delete;
	VPhysics2ShapeDef_t( const VPhysics2ShapeDef_t & ) = delete;
	VPhysics2ShapeDef_t &operator=( const VPhysics2ShapeDef_t & ) = delete;
	~VPhysics2ShapeDef_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 VPhysXBodyPart_t
{
	uint32 m_nFlags;
	float m_flMass;
	VPhysics2ShapeDef_t m_rnShape;
	uint16 m_nCollisionAttributeIndex;
	uint16 m_nReserved;
	float m_flInertiaScale;
	float m_flLinearDamping;
	float m_flAngularDamping;
	float m_flLinearDrag;
	float m_flAngularDrag;
	bool m_bOverrideMassCenter;
	Vector m_vMassCenterOverride;

private:
	VPhysXBodyPart_t() = delete;
	VPhysXBodyPart_t( const VPhysXBodyPart_t & ) = delete;
	VPhysXBodyPart_t &operator=( const VPhysXBodyPart_t & ) = delete;
	~VPhysXBodyPart_t() = delete;
} ALIGN8_POST;

struct ALIGN4 VPhysXConstraintParams_t
{
	int8 m_nType;
	int8 m_nTranslateMotion;
	int8 m_nRotateMotion;
	int8 m_nFlags;
	Vector m_anchor[2];
	QuaternionStorage m_axes[2];
	float m_maxForce;
	float m_maxTorque;
	float m_linearLimitValue;
	float m_linearLimitRestitution;
	float m_linearLimitSpring;
	float m_linearLimitDamping;
	float m_twistLowLimitValue;
	float m_twistLowLimitRestitution;
	float m_twistLowLimitSpring;
	float m_twistLowLimitDamping;
	float m_twistHighLimitValue;
	float m_twistHighLimitRestitution;
	float m_twistHighLimitSpring;
	float m_twistHighLimitDamping;
	float m_swing1LimitValue;
	float m_swing1LimitRestitution;
	float m_swing1LimitSpring;
	float m_swing1LimitDamping;
	float m_swing2LimitValue;
	float m_swing2LimitRestitution;
	float m_swing2LimitSpring;
	float m_swing2LimitDamping;
	Vector m_goalPosition;
	QuaternionStorage m_goalOrientation;
	Vector m_goalAngularVelocity;
	float m_driveSpringX;
	float m_driveSpringY;
	float m_driveSpringZ;
	float m_driveDampingX;
	float m_driveDampingY;
	float m_driveDampingZ;
	float m_driveSpringTwist;
	float m_driveSpringSwing;
	float m_driveSpringSlerp;
	float m_driveDampingTwist;
	float m_driveDampingSwing;
	float m_driveDampingSlerp;
	int32 m_solverIterationCount;
	float m_projectionLinearTolerance;
	float m_projectionAngularTolerance;
} ALIGN4_POST;

struct ALIGN4 VPhysXConstraint2_t
{
	uint32 m_nFlags;
	uint16 m_nParent;
	uint16 m_nChild;
	VPhysXConstraintParams_t m_params;
} ALIGN4_POST;

struct ALIGN4 VPhysXRange_t
{
	float m_flMin;
	float m_flMax;
} ALIGN4_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN16 VPhysXJoint_t
{
	uint16 m_nType;
	uint16 m_nBody1;
	uint16 m_nBody2;
	uint16 m_nFlags;
	CTransform m_Frame1;
	CTransform m_Frame2;
	bool m_bEnableCollision;
	bool m_bIsLinearConstraintDisabled;
	bool m_bIsAngularConstraintDisabled;
	bool m_bEnableLinearLimit;
	VPhysXRange_t m_LinearLimit;
	bool m_bEnableLinearMotor;
	Vector m_vLinearTargetVelocity;
	float m_flMaxForce;
	bool m_bEnableSwingLimit;
	VPhysXRange_t m_SwingLimit;
	bool m_bEnableTwistLimit;
	VPhysXRange_t m_TwistLimit;
	bool m_bEnableAngularMotor;
	Vector m_vAngularTargetVelocity;
	float m_flMaxTorque;
	float m_flLinearFrequency;
	float m_flLinearDampingRatio;
	float m_flAngularFrequency;
	float m_flAngularDampingRatio;
	float m_flFriction;
	float m_flElasticity;
	float m_flElasticDamping;
	float m_flPlasticity;
	CUtlString m_Tag;

private:
	VPhysXJoint_t() = delete;
	VPhysXJoint_t( const VPhysXJoint_t & ) = delete;
	VPhysXJoint_t &operator=( const VPhysXJoint_t & ) = delete;
	~VPhysXJoint_t() = delete;
} ALIGN16_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 VPhysXCollisionAttributes_t
{
	int32 m_nIncludeDetailLayerCount;
	uint32 m_CollisionGroup;
	CUtlVector< uint32 > m_InteractAs;
	CUtlVector< uint32 > m_InteractWith;
	CUtlVector< uint32 > m_InteractExclude;
	CUtlVector< uint32 > m_DetailLayers;
	CUtlString m_CollisionGroupString;
	CUtlVector< CUtlString > m_InteractAsStrings;
	CUtlVector< CUtlString > m_InteractWithStrings;
	CUtlVector< CUtlString > m_InteractExcludeStrings;
	CUtlVector< CUtlString > m_DetailLayerStrings;

private:
	VPhysXCollisionAttributes_t() = delete;
	VPhysXCollisionAttributes_t( const VPhysXCollisionAttributes_t & ) = delete;
	VPhysXCollisionAttributes_t &operator=( const VPhysXCollisionAttributes_t & ) = delete;
	~VPhysXCollisionAttributes_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned data; obtain from a live engine instance. Do not construct, copy or destroy it.
struct ALIGN8 VPhysXAggregateData_t
{
	uint16 m_nFlags;
	uint16 m_nRefCounter;
	bool m_bCompoundsPacked;
	CUtlVector< uint32 > m_bonesHash;
	CUtlVector< CUtlString > m_boneNames;
	CUtlVector< uint16 > m_indexNames;
	CUtlVector< uint16 > m_indexHash;
	CUtlVector< matrix3x4a_t > m_bindPose;
	CUtlVector< VPhysXBodyPart_t > m_parts;
	CUtlVector< PhysShapeMarkup_t > m_shapeMarkups;
	CUtlVector< VPhysXConstraint2_t > m_constraints2;
	CUtlVector< VPhysXJoint_t > m_joints;
	PhysFeModelDesc_t *m_pFeModel;
	CUtlVector< uint16 > m_boneParents;
	CUtlVector< uint32 > m_surfacePropertyHashes;
	CUtlVector< VPhysXCollisionAttributes_t > m_collisionAttributes;
	CUtlVector< CUtlString > m_debugPartNames;
	CUtlString m_embeddedKeyvalues;

private:
	VPhysXAggregateData_t() = delete;
	VPhysXAggregateData_t( const VPhysXAggregateData_t & ) = delete;
	VPhysXAggregateData_t &operator=( const VPhysXAggregateData_t & ) = delete;
	~VPhysXAggregateData_t() = delete;
} ALIGN8_POST;

// AMNOTE: Engine-owned body-list interface; do not implement or delete it.
abstract_class IPhysicsBodyList
{
public:
	// Returns the engine-owned schema class binding for this interface.
	virtual SchemaMetaInfoHandle_t<CSchemaClassInfo> GetSchemaClassInfo() const = 0;

protected:
	virtual ~IPhysicsBodyList() = default;

public:
	virtual int GetBodyCount() const = 0;

private:
	// AMNOTE: Indexed body forwarding; source overload grouping and qualifiers remain unresolved.
	virtual void unk001() = 0;
	virtual void unk002() = 0;
};

// AMNOTE: Engine-owned body list. Obtain it from IPhysicsRagdollControl or the owning world; do not implement or delete it.
// AMNOTE: Unresolved virtual signatures remain private. Returned containers and pointers are borrowed.
abstract_class IPhysicsRagdoll : public IPhysicsBodyList
{
protected:
	~IPhysicsRagdoll() override = default;

public:
	virtual IVPhysics2World *GetWorld() const = 0;

private:
	// AMNOTE: Returns the skeleton association; its nominal type remains unresolved.
	virtual void unk011() = 0;
	// AMNOTE: Returns embedded resource data; its complete type remains unresolved.
	virtual void unk012() = 0;

public:
	virtual const CUtlVector<IPhysicsBody *> &GetBodies() const = 0;
	// AMNOTE: Returns the first matching body index, or minus one when absent.
	virtual int FindBody( const IPhysicsBody *pBody ) const = 0;
	// AMNOTE: Selects the body nearest vPosition by its world position, or null for an empty list.
	virtual IPhysicsBody *FindClosestBody( Vector vPosition ) const = 0;
	// AMNOTE: Requires a nonempty body list. Computes a centroid and principal orientation from body positions.
	virtual CTransform GetApproximateTransform() const = 0;

private:
	// AMNOTE: Creates a body with resource shapes; the complete creation parameters remain unresolved.
	virtual void unk111() = 0;

public:
	// AMNOTE: Appends a body and its skeleton bone index, then rebuilds bone mappings. Ragdoll destruction deletes bodies assigned to its owner index.
	virtual void AddBody( IPhysicsBody *pBody, int nBoneIndex ) = 0;
	// AMNOTE: Appends a joint and its two endpoint body indices; missing endpoints are minus one. Ragdoll destruction also destroys registered joints.
	virtual void AddJoint( IPhysicsJoint *pJoint, const IPhysicsBody *pBodyA, const IPhysicsBody *pBodyB ) = 0;
	// AMNOTE: Detaches a body and its associated joints without destroying them; absent bodies are ignored.
	virtual void RemoveBody( const IPhysicsBody *pBody ) = 0;
	// AMNOTE: Detaches a joint and its two endpoint indices without destroying the joint.
	virtual void RemoveJoint( const IPhysicsJoint *pJoint ) = 0;
	virtual const CUtlVector<int> &GetBodyBoneIndices() const = 0;
	// AMNOTE: Maps skeleton bones to body indices; negative entries have no directly mapped body.
	virtual const CUtlVector<int> &GetBoneToBodyMap() const = 0;

private:
	// AMNOTE: Returns an integer mapping used when traversing the body hierarchy; its full meaning remains unresolved.
	virtual void unk211() = 0;
	// AMNOTE: Returns another integer hierarchy mapping; its full meaning remains unresolved.
	virtual void unk212() = 0;

public:
	// AMNOTE: An empty ragdoll also returns true.
	virtual bool HasOnlyDynamicBodies() const = 0;
	// AMNOTE: Returns the sum of requested masses for bodies created during initialization; later body or mass changes do not update it.
	virtual float GetInitialMass() const = 0;

	// AMNOTE: Unions all body bounds; an empty list returns +FLT_MAX minima and -FLT_MAX maxima.
	virtual void GetWorldBounds( Vector &vMin, Vector &vMax ) const = 0;
	virtual void SetCollisionAttributes( const RnCollisionAttr_t &attributes ) = 0;
	virtual const RnCollisionAttr_t &GetCollisionAttributes() const = 0;

	// AMNOTE: Requires a valid, nonnull root body. Resets all body scales, then redistributes inverse mass/inertia within its connected joint component while preserving total requested mass.
	virtual void RedistributeMassAndInertia( int nRootBodyIndex ) = 0;
	virtual void ResetMassAndInertiaScale() = 0;
	// AMNOTE: Unions only enabled body bounds; an empty selection returns invalid bounds.
	virtual void GetEnabledBodyBounds( AABB_t &bounds ) const = 0;
	// AMNOTE: Forwards the body's game index; pBody must be nonnull.
	virtual uint16 GetBodyGameIndex( const IPhysicsBody *pBody ) const = 0;
	virtual int GetJointCount() const = 0;
	// AMNOTE: Returns null for an invalid index.
	virtual IPhysicsJoint *GetJoint( int nIndex ) = 0;
	virtual const IPhysicsJoint *GetJoint( int nIndex ) const = 0;
	virtual const CUtlVector<IPhysicsJoint *> &GetJoints() const = 0;
	// AMNOTE: Two consecutive body indices per joint, endpoint A followed by endpoint B; minus one denotes an absent body.
	virtual const CUtlVector<int> &GetJointBodyIndices() const = 0;
	// AMNOTE: Requires the original resource body prefix and its bone map to remain intact. Sets their scaled transforms from the skeleton pose and clears their velocities.
	virtual void SetBodyTransformsFromSkeletonPose( const CTransform *pSkeletonPose ) = 0;
	// AMNOTE: The array must contain GetBodyCount() scaled transforms; clears linear and angular velocities.
	virtual void SetBodyTransforms( const CTransform *pBodyTransforms ) = 0;
	// AMNOTE: Writes GetBodyCount() scaled body transforms.
	virtual void GetBodyTransforms( CTransform *pBodyTransforms ) const = 0;
	// AMNOTE: Observed bits: 2 dynamic, 4 static, 8 keyframed.
	virtual uint32 GetMotionTypeMask() const = 0;

	// AMNOTE: Updates angular motor friction for spherical, revolute and conical joints using their stored friction coefficient.
	virtual void UpdateJointFriction() = 0;
	// AMNOTE: Returns the borrowed aggregate adapter embedded in this ragdoll; ragdoll destruction invalidates it.
	virtual IPhysAggregateInstance *GetAggregateInstance() const = 0;
	// AMNOTE: Resets the fraction to one, then selects the nearest accepted hit. A miss leaves all other fields unchanged; initialize the result before a new query.
	virtual void CastShape( VPhys2CastResult_t &result, const Ray_t &ray, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
};

#include "vphysics2/ivphysics2world.h"

// AMNOTE: Engine-owned; obtain and destroy through IVPhysics2World.
abstract_class IPhysAggregateInstance : public IPhysicsBodyList
{
protected:
	~IPhysAggregateInstance() override = default;

public:
	// AMNOTE: Returns the borrowed creator/owning world.
	virtual IVPhysics2World *GetWorld() const = 0;

private:
	// AMNOTE: Inaccessible: mode enum, return mask and the complete source signature remain unresolved.
	virtual void SetMotionType() = 0;

public:
	// AMNOTE: ORs per-body motion bits: 2 dynamic, 4 static, 8 keyframed. An empty list returns zero.
	virtual uint32 GetMotionTypeMask() const = 0;

private:
	// AMNOTE: Inaccessible: classifies empty/all-dynamic/all-other/mixed state; the return enum remains unresolved.
	virtual void GetMotionState() const = 0;

public:
	// AMNOTE: Returns the borrowed aggregate resource without retaining it. Its complete native type remains unresolved; do not delete it.
	virtual const CPhysAggregateData *GetAggregateData() const = 0;
	// AMNOTE: CPhysAggregateInstance destroys joints first and bodies second; CPhysicsRagdollAggregate does no work.
	virtual void ClearBodiesAndJoints() = 0;
	// AMNOTE: CPhysAggregateInstance tests both pointer counts; CPhysicsRagdollAggregate always returns false.
	virtual bool IsEmpty() const = 0;
	// AMNOTE: Returns the first body position, or the zero vector when there are no bodies.
	virtual Vector GetPosition() const = 0;
	// AMNOTE: Tests the transform-update flag on CPhysAggregateInstance; CPhysicsRagdollAggregate returns false.
	virtual bool AreTransformUpdatesEnabled() const = 0;
	// AMNOTE: Disables the two aggregate transform-update operations; it does not change body enabled state.
	virtual void DisableTransformUpdates() = 0;

public:
	// Applies resource bind poses, including their scale, to keyframed bodies. bTeleport bypasses their shadow controllers; otherwise flTimeOffset is passed to the shadow target update.
	// Disabled transform updates and CPhysicsRagdollAggregate leave bodies unchanged.
	virtual void UpdateKeyframedTransforms( const CTransform &transform, float flTimeOffset, bool bTeleport ) = 0;
	// Applies resource bind poses, including their scale, to dynamic bodies. Disabled transform updates and CPhysicsRagdollAggregate leave bodies unchanged.
	virtual void UpdateDynamicTransforms( const CTransform &transform ) = 0;

	// AMNOTE: Returns the body sweep fraction. Only fractions below one move the bodies by the permitted displacement, then put them to sleep.
	virtual float MoveByOffset( const Vector &vDisplacement ) = 0;

public:
	// AMNOTE: Returns the stored scale on CPhysAggregateInstance, or the first body scale on CPhysicsRagdollAggregate; an empty ragdoll returns one.
	virtual float GetScale() const = 0;
	// AMNOTE: CPhysAggregateInstance updates positive scales unless their absolute difference is below 0.001; CPhysicsRagdollAggregate forwards every scale to its bodies.
	virtual void SetScale( float scale ) = 0;

public:
	// AMNOTE: Unions body geometry bounds. Empty aggregates return invalid bounds with FLT_MAX minima and -FLT_MAX maxima.
	virtual void GetWorldBounds( AABB_t &bounds ) const = 0;
public:
	// AMNOTE: CPhysAggregateInstance unions registered broadphase shapes; CPhysicsRagdollAggregate unions all body geometry bounds.
	virtual void GetBroadphaseBounds( AABB_t &bounds ) const = 0;
public:
	// AMNOTE: CPhysAggregateInstance uses scaled resource bind poses; CPhysicsRagdollAggregate leaves bounds unchanged.
	virtual void GetTransformedBounds( const CTransform &transform, AABB_t &bounds ) const = 0;
private:

public:
	// Returns the indexed body, or null for an invalid index. CPhysicsRagdollAggregate accepts minus three for the world's geometry body. The pointer is borrowed.
	virtual IPhysicsBody *GetBody( int nIndex ) = 0;
	virtual const IPhysicsBody *GetBody( int nIndex ) const = 0;

	// AMNOTE: Returns the first pointer match, or -1 when absent.
	virtual int FindBody( const IPhysicsBody *body ) const = 0;
	// AMNOTE: CPhysAggregateInstance requires a nonnegative index and returns a borrowed resource name or the empty string; CPhysicsRagdollAggregate returns null.
	virtual const char *GetPartName( int index ) const = 0;
	// AMNOTE: index must be nonnegative. Returns the resource name hash, or zero at/above the name count.
	virtual uint32 GetPartNameHash( int index ) const = 0;
	// AMNOTE: CPhysAggregateInstance resolves resource hashes, with zero selecting its first body; CPhysicsRagdollAggregate returns null.
	virtual IPhysicsBody *FindPartByNameHash( uint32 nameHash ) const = 0;

	// AMNOTE: Returns selected resource collision attributes' interaction mask, or zero when empty. attributes may be null; otherwise it receives the selected attributes with their resolved mask.
	virtual uint64 GetCollisionAttributes( RnCollisionAttr_t *attributes ) const = 0;

	// AMNOTE: Replaces collision attributes on every attached body shape. The supplied record is borrowed for this call.
	virtual void SetCollisionAttributes( const RnCollisionAttr_t &attributes ) = 0;

	// AMNOTE: Zero is a successful no-op. Otherwise applies resolved surface properties to all body shapes without updating mass; false leaves materials unchanged. Resolution may select the default properties.
	virtual bool SetSurfaceProperties( uint32 surfacePropertyHash ) = 0;

	// AMNOTE: shape must be nonnull. Returns its borrowed resource markup, or null when its body, shape index or markup is absent.
	// AMNOTE: Resource/source lifetime changes can invalidate the result.
	virtual const PhysShapeMarkup_t *FindShapeMarkup( const IPhysicsShape *shape ) const = 0;

private:
	// AMNOTE: Inaccessible: Returns a borrowed PhysShapeMarkup_t pointer/count view; its complete value type remains unresolved.
	virtual void GetShapeMarkupView() = 0;

public:
	// AMNOTE: CPhysAggregateInstance appends opposite contact entity IDs, then sorts and deduplicates the whole vector, retaining invalid IDs. CPhysicsRagdollAggregate does no work.
	virtual void AppendContactEntityIds( CUtlVector<VPhys2ContactEntityId_t> &entityIds ) const = 0;
	// AMNOTE: Counts the aggregate joint-adapter pointers.
	virtual int GetJointCount() const = 0;

	// nIndex must be nonnegative; indices at or above the joint count return null. The returned pointer is borrowed.
	virtual IPhysicsJoint *GetJoint( int nIndex ) = 0;
	virtual const IPhysicsJoint *GetJoint( int nIndex ) const = 0;
	// Borrows the retained CPhysAggregateInstance transform or a static identity transform for CPhysicsRagdollAggregate.
	virtual const CTransform &GetStoredTransform() const = 0;

private:
	// AMNOTE: Inaccessible: Tests every body through a native helper predicate with a DWORD argument. The predicate/parameter meaning remains unresolved.
	virtual void unk011() = 0;

public:
	// AMNOTE: CPhysAggregateInstance returns the mass-weighted center of dynamic bodies; CPhysicsRagdollAggregate returns zero.
	virtual Vector GetMassCenterWorldSpace() const = 0;

	// AMNOTE: Sums dynamic-body mass: effective inverse-mass reciprocals for CPhysAggregateInstance, requested masses for CPhysicsRagdollAggregate.
	virtual float GetMass() const = 0;

	// AMNOTE: Requires mass greater than 0.000001. A single body receives it directly; multiple dynamic bodies receive shares proportional to their geometry mass, preserving overrides only on excluded bodies.
	virtual void SetTotalMass( float flMass ) = 0;
	// AMNOTE: With at least two bodies, assigns total dynamic mass divided by the original body count to each body; no change for zero/one body.
	virtual void EqualizeMasses() = 0;
	// AMNOTE: Forwards the priority to each body.
	virtual void SetMassPriority( int8 priority ) = 0;
	// AMNOTE: Forwards the automatic-sleep control to each body.
	virtual void EnableAutoSleep( bool enable ) = 0;
	// AMNOTE: Wakes each body.
	virtual void Wake() = 0;
	// AMNOTE: Puts each body to sleep.
	virtual void Sleep() = 0;
	// AMNOTE: True when every body is asleep, including an empty aggregate.
	virtual bool IsAsleep() const = 0;
	// AMNOTE: Forwards the drag control to each body.
	virtual void EnableDrag( bool enable ) = 0;
	// AMNOTE: Forwards the coefficient to each body.
	virtual void SetLinearDragCoefficient( float coefficient ) = 0;
	// AMNOTE: Forwards the coefficient to each body.
	virtual void SetAngularDragCoefficient( float coefficient ) = 0;
	// AMNOTE: Forwards the damping to each body.
	virtual void SetLinearDamping( float damping ) = 0;
	// AMNOTE: Forwards the damping to each body.
	virtual void SetAngularDamping( float damping ) = 0;

private:
	// AMNOTE: Inaccessible: Sums a core-body float query with one explicit parameter. The source parameter and purpose remain unresolved.
	virtual void unk111() = 0;

public:
	// AMNOTE: Forwards the scale to each body.
	virtual void SetBuoyancyScale( float scale ) = 0;
	// AMNOTE: Forwards the scale to each body.
	virtual void SetLinearFluidDragScale( float scale ) = 0;
	// AMNOTE: Forwards the scale to each body.
	virtual void SetAngularFluidDragScale( float scale ) = 0;

public:
	// AMNOTE: CPhysAggregateInstance resets the fraction and replaces closer hits; CPhysicsRagdollAggregate leaves the result unchanged.
	virtual void CastShape( VPhys2CastResult_t &result, const Ray_t &ray, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const = 0;
public:
	// AMNOTE: Sweeps against static/keyframed objects using the conditionally-solid group and force-hit filtering. Returns the ordered fraction product; each next displacement uses the accumulated fraction.
	virtual float SweepByOffset( const Vector &vDisplacement ) const = 0;
public:
	// AMNOTE: CPhysAggregateInstance tests eligible attached shapes; CPhysicsRagdollAggregate returns false.
	virtual bool OverlapRay( const CTransform &shapeToQuery, const Ray_t &ray, const RnQueryAttr_t &query ) const = 0;
public:
	// AMNOTE: CPhysAggregateInstance tests its own body prefix; pCountSource supplies its length. CPhysicsRagdollAggregate returns false.
	virtual bool HasBodyPrefixOverlap( const IPhysicsBodyList *pCountSource ) const = 0;
public:
	// AMNOTE: Minimum convex distance in world space; FLT_MAX when no eligible shape exists.
	virtual float GetConvexDistanceToPoint( const Vector &worldPosition ) const = 0;
public:
	// AMNOTE: CPhysAggregateInstance requires pOther of the same implementation and returns FLT_MAX when no eligible pair exists; CPhysicsRagdollAggregate returns zero.
	virtual float GetConvexDistanceToAggregate( const IPhysAggregateInstance *pOther ) const = 0;
public:
	// AMNOTE: Tests resource bone hashes on CPhysAggregateInstance; CPhysicsRagdollAggregate returns true.
	virtual bool HasBoneHashes() const = 0;
public:
	// AMNOTE: Returns the body-position centroid and principal orientation. An empty aggregate produces NaN centroid components.
	virtual CTransform GetBodyPositionPrincipalTransform() const = 0;
public:
	// AMNOTE: Stores the filtering entity ID and forwards it to every body.
	virtual void SetEntityId( uint32 nEntityId ) = 0;
	// AMNOTE: Returns the stored filtering entity ID, initially all ones.
	virtual uint32 GetEntityId() const = 0;
	// AMNOTE: CPhysAggregateInstance formats and forwards body names; CPhysicsRagdollAggregate does no work.
	virtual void SetDebugName( const char *pPrefix ) = 0;

	// AMNOTE: CPhysAggregateInstance appends body and joint descriptions; CPhysicsRagdollAggregate leaves description unchanged.
	virtual void AppendDebugDescription( CUtlString &description ) const = 0;

public:
	void *m_pSkeleton;
	bool m_bIsAxisAligned;
};

// AMNOTE: Complete engine-owned adapter storage; resource release and other unimplemented native overrides remain in the engine.
class CPhysAggregateInstance : public IPhysAggregateInstance
{
private:
	// AMNOTE: Native destruction releases the aggregate resource and pointer buffers; its resource accessor is unrecovered.
	~CPhysAggregateInstance() override;

public:
	inline void GetWorldBounds( AABB_t &bounds ) const override;
	inline uint32 GetMotionTypeMask() const override;
	inline float MoveByOffset( const Vector &vDisplacement ) override;
	inline void CastShape( VPhys2CastResult_t &result, const Ray_t &ray, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const override;
	inline float SweepByOffset( const Vector &vDisplacement ) const override;
	inline bool OverlapRay( const CTransform &shapeToQuery, const Ray_t &ray, const RnQueryAttr_t &query ) const override;
	int GetBodyCount() const override { return m_Bodies.Count(); }
	IVPhysics2World *GetWorld() const override { return m_World; }
	const CPhysAggregateData *GetAggregateData() const override { return m_AggregateData; }
	inline void ClearBodiesAndJoints() override;
	bool IsEmpty() const override { return m_Bodies.Count() == 0 && m_Joints.Count() == 0; }
	Vector GetPosition() const override { return m_Bodies.Count() > 0 ? m_Bodies[0]->GetPosition() : Vector( 0.0f, 0.0f, 0.0f ); }
	bool AreTransformUpdatesEnabled() const override { return !m_DisableTransformUpdates; }
	void DisableTransformUpdates() override { m_DisableTransformUpdates = true; }
	float GetScale() const override { return m_flScale; }
	inline int FindBody( const IPhysicsBody *body ) const override;
	int GetJointCount() const override { return m_Joints.Count(); }
	inline void SetCollisionAttributes( const RnCollisionAttr_t &attributes ) override;

	inline float GetMass() const override;
	inline Vector GetMassCenterWorldSpace() const override;
	inline void EqualizeMasses() override;

	inline void SetMassPriority( int8 priority ) override;

	inline void EnableAutoSleep( bool enable ) override;

	inline void Wake() override;

	inline void Sleep() override;

	inline void EnableDrag( bool enable ) override;

	inline void SetLinearDragCoefficient( float coefficient ) override;

	inline void SetAngularDragCoefficient( float coefficient ) override;

	inline void SetLinearDamping( float damping ) override;

	inline void SetAngularDamping( float damping ) override;

	inline void SetBuoyancyScale( float scale ) override;

	inline void SetLinearFluidDragScale( float scale ) override;

	inline void SetAngularFluidDragScale( float scale ) override;

	inline bool IsAsleep() const override;

	inline void SetEntityId( uint32 nEntityId ) override;
	uint32 GetEntityId() const override { return m_nEntityId; }

	IPhysicsBody *GetBody( int index ) override { return index >= 0 && index < m_Bodies.Count() ? m_Bodies[index] : nullptr; }
	const IPhysicsBody *GetBody( int index ) const override { return index >= 0 && index < m_Bodies.Count() ? m_Bodies[index] : nullptr; }
	// AMNOTE: index must be nonnegative; the native joint lookup checks only the upper bound.
	IPhysicsJoint *GetJoint( int index ) override { return index < m_Joints.Count() ? m_Joints[index] : nullptr; }
	const IPhysicsJoint *GetJoint( int index ) const override { return index < m_Joints.Count() ? m_Joints[index] : nullptr; }
	const CTransform &GetStoredTransform() const override { return m_Transform; }

private:
	CPhysAggregateInstance() = delete;
	CPhysAggregateInstance( const CPhysAggregateInstance & ) = delete;
	CPhysAggregateInstance &operator=( const CPhysAggregateInstance & ) = delete;

	IVPhysics2World *m_World;
	CPhysAggregateData *m_AggregateData;
	float m_flScale;
	int m_WorldIndex;
	bool m_DisableTransformUpdates;
	CUtlVector<IPhysicsBody *> m_Bodies;
	CUtlVector<IPhysicsJoint *> m_Joints;
	CTransform m_Transform;
	uint32 m_nEntityId;
	uint16 m_nCallbackFlags;
};

// AMNOTE: Complete engine-owned adapter storage, embedded in CPhysicsRagdoll. Unrecovered native methods remain inherited; obtain it through GetAggregateInstance().
class CPhysicsRagdollAggregate : public IPhysAggregateInstance
{
public:
	int GetBodyCount() const override { return m_pRagdoll->GetBodies().Count(); }
	IVPhysics2World *GetWorld() const override { return m_pRagdoll->GetWorld(); }
	inline uint32 GetMotionTypeMask() const override;
	const CPhysAggregateData *GetAggregateData() const override { return nullptr; }
	void ClearBodiesAndJoints() override {}
	bool IsEmpty() const override { return false; }
	Vector GetPosition() const override { return GetBodyCount() > 0 ? m_pRagdoll->GetBodies()[0]->GetPosition() : Vector( 0.0f, 0.0f, 0.0f ); }
	bool AreTransformUpdatesEnabled() const override { return false; }
	void DisableTransformUpdates() override {}
	void UpdateKeyframedTransforms( const CTransform &, float, bool ) override {}
	void UpdateDynamicTransforms( const CTransform & ) override {}
	inline float MoveByOffset( const Vector &vDisplacement ) override;
	float GetScale() const override { return GetBodyCount() > 0 ? m_pRagdoll->GetBodies()[0]->GetScaledTransform().m_vPosition.w : 1.0f; }
	// AMNOTE: Unions only enabled body bounds.
	void GetWorldBounds( AABB_t &bounds ) const override { m_pRagdoll->GetEnabledBodyBounds( bounds ); }
	// AMNOTE: Unions all body geometry bounds, including disabled bodies.
	void GetBroadphaseBounds( AABB_t &bounds ) const override { m_pRagdoll->GetWorldBounds( bounds.m_vMinBounds, bounds.m_vMaxBounds ); }
	// AMNOTE: Leaves bounds unchanged.
	void GetTransformedBounds( const CTransform &, AABB_t & ) const override {}
	// AMNOTE: Minus three selects the world's registered geometry body; all other invalid indices return null.
	inline IPhysicsBody *GetBody( int nIndex ) override;
	inline const IPhysicsBody *GetBody( int nIndex ) const override;
	int FindBody( const IPhysicsBody *pBody ) const override { return m_pRagdoll->FindBody( pBody ); }
	const char *GetPartName( int ) const override { return nullptr; }
	uint32 GetPartNameHash( int ) const override { return 0; }
	IPhysicsBody *FindPartByNameHash( uint32 ) const override { return nullptr; }
	// AMNOTE: Leaves the optional output unchanged.
	uint64 GetCollisionAttributes( RnCollisionAttr_t * ) const override { return 0; }
	void SetCollisionAttributes( const RnCollisionAttr_t &attributes ) override { m_pRagdoll->SetCollisionAttributes( attributes ); }
	bool SetSurfaceProperties( uint32 ) override { return false; }
	void AppendContactEntityIds( CUtlVector<VPhys2ContactEntityId_t> & ) const override {}
	int GetJointCount() const override { return m_pRagdoll->GetJoints().Count(); }
	IPhysicsJoint *GetJoint( int nIndex ) override { return m_pRagdoll->GetJoint( nIndex ); }
	const IPhysicsJoint *GetJoint( int nIndex ) const override { return static_cast<const IPhysicsRagdoll *>( m_pRagdoll )->GetJoint( nIndex ); }
	// AMNOTE: Returns a static identity transform.
	inline const CTransform &GetStoredTransform() const override;
	Vector GetMassCenterWorldSpace() const override { return Vector( 0.0f, 0.0f, 0.0f ); }
	// AMNOTE: Sums requested masses of dynamic bodies, before inverse-mass scales.
	inline float GetMass() const override;
	inline void EqualizeMasses() override;
	inline void SetMassPriority( int8 priority ) override;
	inline void EnableAutoSleep( bool enable ) override;
	inline void Wake() override;
	inline void Sleep() override;
	inline bool IsAsleep() const override;
	inline void EnableDrag( bool enable ) override;
	inline void SetLinearDragCoefficient( float coefficient ) override;
	inline void SetAngularDragCoefficient( float coefficient ) override;
	inline void SetLinearDamping( float damping ) override;
	inline void SetAngularDamping( float damping ) override;
	inline void SetBuoyancyScale( float scale ) override;
	inline void SetLinearFluidDragScale( float scale ) override;
	inline void SetAngularFluidDragScale( float scale ) override;
	// AMNOTE: Leaves the result unchanged.
	void CastShape( VPhys2CastResult_t &, const Ray_t &, const Vector &, const Vector &, const RnQueryAttr_t & ) const override {}
	inline float SweepByOffset( const Vector &vDisplacement ) const override;
	bool OverlapRay( const CTransform &, const Ray_t &, const RnQueryAttr_t & ) const override { return false; }
	bool HasBodyPrefixOverlap( const IPhysicsBodyList * ) const override { return false; }
	float GetConvexDistanceToAggregate( const IPhysAggregateInstance * ) const override { return 0.0f; }
	bool HasBoneHashes() const override { return true; }
	// AMNOTE: Requires a nonempty body list.
	CTransform GetBodyPositionPrincipalTransform() const override { return m_pRagdoll->GetApproximateTransform(); }
	void SetDebugName( const char * ) override {}
	void AppendDebugDescription( CUtlString & ) const override {}

private:
	CPhysicsRagdollAggregate() = delete;
	CPhysicsRagdollAggregate( const CPhysicsRagdollAggregate & ) = delete;
	CPhysicsRagdollAggregate &operator=( const CPhysicsRagdollAggregate & ) = delete;
	~CPhysicsRagdollAggregate() override = default;

	IPhysicsRagdoll *m_pRagdoll;
};

inline uint32 CPhysAggregateInstance::GetMotionTypeMask() const
{
	const uint32 masks[] = { 4, 8, 2 };
	uint32 nMask = 0;
	const int nBodyCount = m_Bodies.Count();
	for ( int i = 0; i < nBodyCount; ++i )
		nMask |= masks[m_Bodies[i]->GetMotionType()];
	return nMask;
}

inline float CPhysAggregateInstance::MoveByOffset( const Vector &vDisplacement )
{
	const float flFraction = SweepByOffset( vDisplacement );
	if ( flFraction < 1.0f )
	{
		for ( int i = 0; i < m_Bodies.Count(); ++i )
		{
			const Vector vOffset( flFraction * vDisplacement.x, flFraction * vDisplacement.y, flFraction * vDisplacement.z );
			IPhysicsBody *pBody = m_Bodies[i];
			const Vector vPosition = pBody->GetPosition();
			pBody->SetPosition( Vector( vPosition.x + vOffset.x, vPosition.y + vOffset.y, vPosition.z + vOffset.z ) );
			m_Bodies[i]->Sleep();
		}
	}
	return flFraction;
}

inline void CPhysAggregateInstance::ClearBodiesAndJoints()
{
	for ( int i = 0; i < m_Joints.Count(); ++i )
		m_World->DestroyJoint( m_Joints[i] );
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_World->DestroyBody( m_Bodies[i] );
	m_Joints.RemoveAll();
	m_Bodies.RemoveAll();
}

inline int CPhysAggregateInstance::FindBody( const IPhysicsBody *body ) const
{
	const int count = m_Bodies.Count();
	if ( count <= 0 )
		return -1;
	IPhysicsBody *const *bodies = m_Bodies.Base();
	for ( int i = 0; i < count; ++i )
		if ( bodies[i] == body )
			return i;
	return -1;
}

inline void CPhysAggregateInstance::SetCollisionAttributes( const RnCollisionAttr_t &attributes )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetCollisionAttributes( attributes );
}

inline float CPhysAggregateInstance::GetMass() const
{
	float flMass = 0.0f;
	for ( int i = 0; i < m_Bodies.Count(); ++i )
	{
		const IPhysicsBody *pBody = m_Bodies[i];
		if ( pBody->GetMotionType() != BODY_DYNAMIC )
			continue;
		const float flInvMass = pBody->GetInvMass();
		if ( flInvMass != 0.0f )
			flMass = 1.0f / flInvMass + flMass;
	}
	return flMass;
}

inline Vector CPhysAggregateInstance::GetMassCenterWorldSpace() const
{
	Vector center;
	center.Zero();
	float flTotalMass = 0.0f;
	for ( int i = 0; i < m_Bodies.Count(); ++i )
	{
		const IPhysicsBody *pBody = m_Bodies[i];
		if ( pBody->GetMotionType() != BODY_DYNAMIC )
			continue;
		const float flInvMass = pBody->GetInvMass();
		if ( flInvMass == 0.0f )
			continue;
		const float flMass = 1.0f / flInvMass;
		const Vector bodyCenter = pBody->GetMassCenterWorldSpace();
		center.x = flMass * bodyCenter.x + center.x;
		center.y = flMass * bodyCenter.y + center.y;
		center.z = flMass * bodyCenter.z + center.z;
		flTotalMass = flMass + flTotalMass;
	}
	if ( flTotalMass > 0.0f )
	{
		center.x /= flTotalMass;
		center.y /= flTotalMass;
		center.z /= flTotalMass;
	}
	else
	{
		center.Zero();
	}
	return center;
}

inline void CPhysAggregateInstance::EqualizeMasses()
{
	const int nBodyCount = m_Bodies.Count();
	if ( nBodyCount <= 1 )
		return;

	const float flMass = GetMass() / float( nBodyCount );
	const int nUpdatedBodyCount = m_Bodies.Count();
	IPhysicsBody *const *pBodies = m_Bodies.Base();
	for ( int i = 0; i < nUpdatedBodyCount; ++i )
		pBodies[i]->SetMass( flMass, 0 );
}

inline void CPhysAggregateInstance::SetMassPriority( int8 priority )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetMassPriority( priority );
}

inline void CPhysAggregateInstance::EnableAutoSleep( bool enable )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->EnableAutoSleep( enable );
}

inline void CPhysAggregateInstance::Wake()
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->Wake();
}

inline void CPhysAggregateInstance::Sleep()
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->Sleep();
}

inline void CPhysAggregateInstance::EnableDrag( bool enable )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->EnableDrag( enable );
}

inline void CPhysAggregateInstance::SetLinearDragCoefficient( float coefficient )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetLinearDragCoefficient( coefficient );
}

inline void CPhysAggregateInstance::SetAngularDragCoefficient( float coefficient )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetAngularDragCoefficient( coefficient );
}

inline void CPhysAggregateInstance::SetLinearDamping( float damping )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetLinearDamping( damping );
}

inline void CPhysAggregateInstance::SetAngularDamping( float damping )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetAngularDamping( damping );
}

inline void CPhysAggregateInstance::SetBuoyancyScale( float scale )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetBuoyancyScale( scale );
}

inline void CPhysAggregateInstance::SetLinearFluidDragScale( float scale )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetLinearFluidDragScale( scale );
}

inline void CPhysAggregateInstance::SetAngularFluidDragScale( float scale )
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetAngularFluidDragScale( scale );
}

inline bool CPhysAggregateInstance::IsAsleep() const
{
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		if ( !m_Bodies[i]->IsAsleep() )
			return false;
	return true;
}

inline void CPhysAggregateInstance::SetEntityId( uint32 nEntityId )
{
	m_nEntityId = nEntityId;
	for ( int i = 0; i < m_Bodies.Count(); ++i )
		m_Bodies[i]->SetEntityId( m_nEntityId );
}

inline void CPhysAggregateInstance::GetWorldBounds( AABB_t &bounds ) const
{
	bounds.MakeInvalid();
	for ( int i = 0; i < m_Bodies.Count(); ++i )
	{
		const AABB_t bodyBounds = m_Bodies[i]->GetWorldBounds();
		for ( int j = 0; j < 3; ++j )
		{
			bounds.m_vMinBounds[j] = bounds.m_vMinBounds[j] < bodyBounds.m_vMinBounds[j] ? bounds.m_vMinBounds[j] : bodyBounds.m_vMinBounds[j];
			bounds.m_vMaxBounds[j] = bounds.m_vMaxBounds[j] > bodyBounds.m_vMaxBounds[j] ? bounds.m_vMaxBounds[j] : bodyBounds.m_vMaxBounds[j];
		}
	}
}

inline float CPhysAggregateInstance::SweepByOffset( const Vector &vDisplacement ) const
{
	RnQueryAttr_t query;
	query.m_nObjectSetMask = RNQUERY_OBJECTS_STATIC | RNQUERY_OBJECTS_KEYFRAMED;
	query.m_nCollisionGroup = COLLISION_GROUP_CONDITIONALLY_SOLID;
	query.m_bForceHitEverything = true;
	float fraction = 1.0f;
	IPhysicsBody *const *bodies = m_Bodies.Base();
	const int count = m_Bodies.Count();
	for ( int i = 0; i < count; ++i )
	{
		const IPhysicsBody *body = bodies[i];
		const CTransform start = body->GetTransform();
		CTransform end = start;
		end.m_vPosition.x = start.m_vPosition.x + fraction * vDisplacement.x;
		end.m_vPosition.y = start.m_vPosition.y + fraction * vDisplacement.y;
		end.m_vPosition.z = start.m_vPosition.z + fraction * vDisplacement.z;
		end.m_vPosition.w = 1.0f;
		fraction *= m_World->SweepBody( start, end, body, query, false ).m_flFraction;
	}
	return fraction;
}

inline bool CPhysAggregateInstance::OverlapRay( const CTransform &shapeToQuery, const Ray_t &ray, const RnQueryAttr_t &query ) const
{
	const int bodyCount = GetBodyCount();
	for ( int i = 0; i < bodyCount; ++i )
	{
		const IPhysicsBody *body = m_Bodies[i];
		const int shapeCount = body->GetShapeCount();
		for ( int j = 0; j < shapeCount; ++j )
		{
			const IPhysicsShape *shape = body->GetShape( j );
			if ( m_World->CanQueryShape( shape, query ) && shape->Overlap( shapeToQuery, ray ) )
				return true;
		}
	}
	return false;
}

inline void CPhysAggregateInstance::CastShape( VPhys2CastResult_t &result, const Ray_t &ray, const Vector &vStart, const Vector &vDisplacement, const RnQueryAttr_t &query ) const
{
	result.m_flFraction = 1.0f;
	VPhys2ShapeCastResult_t shapeResult;
	const int bodyCount = GetBodyCount();
	for ( int i = 0; i < bodyCount; ++i )
	{
		IPhysicsBody *body = m_Bodies[i];
		const CTransform transform = body->GetScaledTransform();
		matrix3x4_t bodyToWorld;
		const Quaternion &rotation = transform.m_orientation;
		const float xx = rotation.x * rotation.x;
		const float yy = rotation.y * rotation.y;
		const float zz = rotation.z * rotation.z;
		const float xy = rotation.x * rotation.y;
		const float xz = rotation.x * rotation.z;
		const float yz = rotation.y * rotation.z;
		const float wx = rotation.w * rotation.x;
		const float wy = rotation.w * rotation.y;
		const float wz = rotation.w * rotation.z;
		// AMNOTE: Preserve the stored quaternion and this float operation grouping.
		bodyToWorld[0][0] = 1.0f - ((yy + zz) + (yy + zz));
		bodyToWorld[0][1] = (xy - wz) + (xy - wz);
		bodyToWorld[0][2] = (xz + wy) + (xz + wy);
		bodyToWorld[0][3] = transform.m_vPosition.x;
		bodyToWorld[1][0] = (xy + wz) + (xy + wz);
		bodyToWorld[1][1] = 1.0f - ((xx + zz) + (xx + zz));
		bodyToWorld[1][2] = (yz - wx) + (yz - wx);
		bodyToWorld[1][3] = transform.m_vPosition.y;
		bodyToWorld[2][0] = (xz - wy) + (xz - wy);
		bodyToWorld[2][1] = (yz + wx) + (yz + wx);
		bodyToWorld[2][2] = 1.0f - ((xx + yy) + (xx + yy));
		bodyToWorld[2][3] = transform.m_vPosition.z;
		const int shapeCount = body->GetShapeCount();
		for ( int j = 0; j < shapeCount; ++j )
		{
			IPhysicsShape *shape = body->GetShape( j );
			if ( !m_World->CanQueryShape( shape, query ) || !shape->CastShape( shapeResult, bodyToWorld, vStart, vDisplacement, ray, result.m_flFraction ) || !(result.m_flFraction > shapeResult.m_flFraction) )
				continue;
			result.m_pBody = body;
			result.m_pShape = shape;
			result.m_flFraction = shapeResult.m_flFraction;
			VectorTransform( shapeResult.m_vPoint, bodyToWorld, result.m_vHitPoint );
			VectorRotate( shapeResult.m_vNormal, bodyToWorld, result.m_vHitNormal );
			result.m_flHitOffset = DotProduct( result.m_vHitPoint, result.m_vHitNormal );
			result.m_nConvexIndex = shapeResult.m_GeometryId.m_nConvexIndex;
			result.m_nTriangle = shapeResult.m_GeometryId.m_nTriangleIndex;
			result.m_bStartInSolid = shapeResult.m_bStartInSolid;
			const VPhys2ShapeMaterialBinding *material = shapeResult.m_pMaterial;
			if ( !material )
				material = &shape->GetMaterial();
			result.m_pSurfaceProperties = material->m_pSurfaceProperties;
		}
	}
}

inline uint32 CPhysicsRagdollAggregate::GetMotionTypeMask() const
{
	const uint32 masks[] = { 4, 8, 2 };
	uint32 nMask = 0;
	const int nBodyCount = m_pRagdoll->GetBodies().Count();
	for ( int i = 0; i < nBodyCount; ++i )
		nMask |= masks[m_pRagdoll->GetBodies()[i]->GetMotionType()];
	return nMask;
}

inline IPhysicsBody *CPhysicsRagdollAggregate::GetBody( int nIndex )
{
	if ( nIndex == -3 )
		return m_pRagdoll->GetWorld()->GetWorldBody();
	const CUtlVector<IPhysicsBody *> &bodies = m_pRagdoll->GetBodies();
	return nIndex >= 0 && nIndex < bodies.Count() ? bodies[nIndex] : nullptr;
}

inline const IPhysicsBody *CPhysicsRagdollAggregate::GetBody( int nIndex ) const
{
	if ( nIndex == -3 )
		return m_pRagdoll->GetWorld()->GetWorldBody();
	const CUtlVector<IPhysicsBody *> &bodies = m_pRagdoll->GetBodies();
	return nIndex >= 0 && nIndex < bodies.Count() ? bodies[nIndex] : nullptr;
}

inline const CTransform &CPhysicsRagdollAggregate::GetStoredTransform() const
{
	static const CTransform transform = []
	{
		CTransform result( Vector( 0.0f, 0.0f, 0.0f ), Quaternion( 0.0f, 0.0f, 0.0f, 1.0f ) );
		result.m_vPosition.w = 1.0f;
		return result;
	}();
	return transform;
}

inline float CPhysicsRagdollAggregate::GetMass() const
{
	float flMass = 0.0f;
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
	{
		const IPhysicsBody *pBody = m_pRagdoll->GetBodies()[i];
		if ( pBody->GetMotionType() == BODY_DYNAMIC )
			flMass = pBody->GetMass() + flMass;
	}
	return flMass;
}

inline void CPhysicsRagdollAggregate::EqualizeMasses()
{
	const int nBodyCount = m_pRagdoll->GetBodies().Count();
	if ( nBodyCount <= 1 )
		return;
	const float flMass = GetMass() / float( nBodyCount );
	const CUtlVector<IPhysicsBody *> &bodies = m_pRagdoll->GetBodies();
	const int nUpdatedBodyCount = bodies.Count();
	IPhysicsBody *const *pBodies = bodies.Base();
	for ( int i = 0; i < nUpdatedBodyCount; ++i )
		pBodies[i]->SetMass( flMass, 0 );
}

inline bool CPhysicsRagdollAggregate::IsAsleep() const
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		if ( !m_pRagdoll->GetBodies()[i]->IsAsleep() )
			return false;
	return true;
}

inline float CPhysicsRagdollAggregate::SweepByOffset( const Vector &vDisplacement ) const
{
	RnQueryAttr_t query;
	query.m_nObjectSetMask = RNQUERY_OBJECTS_STATIC | RNQUERY_OBJECTS_KEYFRAMED;
	query.m_nCollisionGroup = COLLISION_GROUP_CONDITIONALLY_SOLID;
	query.m_bForceHitEverything = true;
	float flFraction = 1.0f;
	const CUtlVector<IPhysicsBody *> &bodies = m_pRagdoll->GetBodies();
	IPhysicsBody *const *pBodies = bodies.Base();
	const int nBodyCount = bodies.Count();
	for ( int i = 0; i < nBodyCount; ++i )
	{
		const IPhysicsBody *pBody = pBodies[i];
		const CTransform start = pBody->GetTransform();
		CTransform end = start;
		end.m_vPosition.x = start.m_vPosition.x + flFraction * vDisplacement.x;
		end.m_vPosition.y = start.m_vPosition.y + flFraction * vDisplacement.y;
		end.m_vPosition.z = start.m_vPosition.z + flFraction * vDisplacement.z;
		end.m_vPosition.w = 1.0f;
		flFraction *= m_pRagdoll->GetWorld()->SweepBody( start, end, pBody, query, false ).m_flFraction;
	}
	return flFraction;
}

inline float CPhysicsRagdollAggregate::MoveByOffset( const Vector &vDisplacement )
{
	const float flFraction = SweepByOffset( vDisplacement );
	if ( flFraction < 1.0f )
	{
		for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		{
			const Vector vOffset( flFraction * vDisplacement.x, flFraction * vDisplacement.y, flFraction * vDisplacement.z );
			IPhysicsBody *pBody = m_pRagdoll->GetBodies()[i];
			const Vector vPosition = pBody->GetPosition();
			pBody->SetPosition( Vector( vPosition.x + vOffset.x, vPosition.y + vOffset.y, vPosition.z + vOffset.z ) );
			m_pRagdoll->GetBodies()[i]->Sleep();
		}
	}
	return flFraction;
}

inline void CPhysicsRagdollAggregate::SetMassPriority( int8 priority )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetMassPriority( priority );
}

inline void CPhysicsRagdollAggregate::EnableAutoSleep( bool enable )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->EnableAutoSleep( enable );
}

inline void CPhysicsRagdollAggregate::Wake()
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->Wake();
}

inline void CPhysicsRagdollAggregate::Sleep()
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->Sleep();
}

inline void CPhysicsRagdollAggregate::EnableDrag( bool enable )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->EnableDrag( enable );
}

inline void CPhysicsRagdollAggregate::SetLinearDragCoefficient( float coefficient )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetLinearDragCoefficient( coefficient );
}

inline void CPhysicsRagdollAggregate::SetAngularDragCoefficient( float coefficient )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetAngularDragCoefficient( coefficient );
}

inline void CPhysicsRagdollAggregate::SetLinearDamping( float damping )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetLinearDamping( damping );
}

inline void CPhysicsRagdollAggregate::SetAngularDamping( float damping )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetAngularDamping( damping );
}

inline void CPhysicsRagdollAggregate::SetBuoyancyScale( float scale )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetBuoyancyScale( scale );
}

inline void CPhysicsRagdollAggregate::SetLinearFluidDragScale( float scale )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetLinearFluidDragScale( scale );
}

inline void CPhysicsRagdollAggregate::SetAngularFluidDragScale( float scale )
{
	for ( int i = 0; i < m_pRagdoll->GetBodies().Count(); ++i )
		m_pRagdoll->GetBodies()[i]->SetAngularFluidDragScale( scale );
}

#endif // IPHYSAGGREGATEINSTANCE_H
