#ifndef IVPHYSICS2_H
#define IVPHYSICS2_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier0/utlleanvector.h"
#include "appframework/iappsystem.h"
#include "resourcesystem/iresourcesystem.h"
#include "vphysics2/iphysicsbody.h"
#include "vphysics2/physicscast.h"

#if ( defined( _MSVC_LANG ) && _MSVC_LANG > 201703L ) || __cplusplus > 201703L
#include <span>
#endif

class CVPhysics2Interface;
class IVPhysics2World;
class IPhysicsSoftbody;
class IIkSolver;
class IPhysSurfacePropertyController;
class IPhysIntersectionDictionary;
class IPhysIntersectionController;
class IResourceTypeManager;
class IVConComm;
class CCommand;
class CFeModel;
struct VPhysXWorldStepResults;
class IFnPhysicsContact;
class IFnPhysicsAnyJoint;
class IFnPhysicsWheelJoint;
class IFnPhysicsWeldJoint;
class IFnPhysicsPrismaticJoint;
class IFnPhysicsRevoluteJoint;
class IFnPhysicsConicalJoint;
class IFnPhysicsSpringJoint;
class IFnPhysicsSphericalJoint;
class IFnPhysicsPulleyJoint;
class IFnPhysicsGenericJoint;
class IFnPhysicsSplineJoint;
class IFnPhysicsAnyShape;
class IFnPhysicsSphereShape;
class IFnPhysicsCapsuleShape;
class IFnPhysicsHullShape;
class IFnPhysicsMeshShape;
class IFnPhysicsCompoundShape;
class IFnPhysicsBody;
class IFnPhysicsSoftbody;
class IFnPhysicsWorld;

class CVPhysics2SurfacePropertyManager;
class CKV3Arena;

// AMNOTE: Result of the surface-property resource manager's AsyncProcessData. Keep its owning unique_ptr alive while using the arena.
class CKV3ResourceBlockHelper : public IAsyncProcessedDataResult
{
public:
	CKV3Arena *GetArena() const { return m_pArena; }

private:
	CKV3ResourceBlockHelper() = delete;
	CKV3ResourceBlockHelper( const CKV3ResourceBlockHelper & ) = delete;
	CKV3ResourceBlockHelper &operator=( const CKV3ResourceBlockHelper & ) = delete;
	~CKV3ResourceBlockHelper() override;

	CKV3Arena *m_pArena;
};

struct VPhys2MeshCreationOptions_t
{
	bool m_bWeldVertices = true;
	// AMNOTE: A negative tolerance selects 1/32 unit along each axis.
	float m_flWeldTolerance = -1.0f;
	// AMNOTE: Nonpositive values disable simplification.
	float m_flMaxSimplificationError = 0.0f;
	// AMNOTE: Zero disables simplification. A negative limit uses the pre-simplification vertex count minus four, with a minimum of zero.
	int m_nMaxVertexCollapses = -1;
};

struct VPhys2IkSolverLink_t
{
	int m_nParentIndex;
	float m_flInverseMass;
	Vector m_vInverseInertia;
	Vector m_vCenterOfMass;
	Vector m_unk001;
	CTransform m_ParentFrame;
	CTransform m_ChildFrame;
	// AMNOTE: Angular limits are in radians in the joint frames.
	float m_flMinTwist;
	float m_flMaxTwist;
	// AMNOTE: A nonpositive value selects the hinge constraint.
	float m_flMaxSwing;
};

struct VPhys2IkSolverEffector_t
{
	int m_nLinkIndex;
	CTransform m_Target;
};

enum VPhys2HullSimplificationAlgorithm_t
{
	VPHYS2_HULL_SIMPLIFY_QUADRIC_ERROR_METRIC = 0,
	VPHYS2_HULL_SIMPLIFY_INCREMENTAL_VERTEX_REDUCTION,
	VPHYS2_HULL_SIMPLIFY_INCREMENTAL_FACE_REDUCTION,
};

struct VPhys2HullCreationOptions_t
{
	float m_flMinFaceAngleDegrees = 5.0f;
	// AMNOTE: Used for short-edge reduction and vertex welding. Relative welding scales this tolerance by each axis's extent.
	float m_flEdgeTolerance = 0.0f;
	int m_nMaxFaces = 256;
	int m_nMaxHalfEdges = 256;
	int m_nMaxVertices = 256;
	// AMNOTE: Zero permits one simplification attempt; a negative value skips simplification.
	int m_nMaxSimplificationRetries = 0;
	VPhys2HullSimplificationAlgorithm_t m_Algorithm = VPHYS2_HULL_SIMPLIFY_QUADRIC_ERROR_METRIC;
	// AMNOTE: Corrects edge alignment and requires at most 256 faces, half-edges and vertices afterward.
	bool m_bFixEdgeAlignment = true;
	bool m_bRelativeEdgeTolerance = true;
	bool m_bWarnOnSimplificationFailure = false;
};

struct VPhys2ConvexSimplexCache_t
{
	// AMNOTE: Length, area or volume of the cached simplex, according to its point count.
	float m_flMeasure = 0.0f;
	int m_nPointCount = 0;
	uint8 m_VertexIndicesA[4]{};
	uint8 m_VertexIndicesB[4]{};
	float m_Weights[4]{};
	Vector m_vSearchDirection{ 0.0f, 0.0f, 0.0f };
};

enum VPhys2HullFeatureType_t
{
	VPHYS2_HULL_FEATURE_VERTEX = 0,
	VPHYS2_HULL_FEATURE_HALF_EDGE,
	VPHYS2_HULL_FEATURE_FACE,
};

struct VPhys2HullFeature_t
{
	uint64 m_nKey = 0;
	VPhys2HullFeatureType_t GetType() const { return VPhys2HullFeatureType_t( m_nKey & 3 ); }
	uint32 GetIndex() const { return uint32( m_nKey >> 32 ); }
};

struct VPhys2HullFeaturePair_t
{
	VPhys2HullFeature_t m_FeatureA;
	VPhys2HullFeature_t m_FeatureB;
};

// AMNOTE: Engine-owned; obtain through IVPhysics2. Resource reloads can invalidate returned surface properties.
abstract_class IPhysSurfacePropertyController
{
protected:
	virtual ~IPhysSurfacePropertyController() = default;

public:
	// AMNOTE: An absent hash returns the default properties. The optional flag distinguishes a match from this fallback.
	virtual const CPhysSurfaceProperties *GetSurfacePropertyByNameHash( uint32 nNameHash, bool *pFound = nullptr ) const = 0;
	virtual int GetSurfacePropertyCount() const = 0;
	// AMNOTE: Returns null for an invalid index or an unloaded resource.
	virtual const CPhysSurfaceProperties *GetSurfacePropertyByIndex( int nIndex ) const = 0;
	// AMNOTE: Appends matching names and, optionally, their descriptions in the same order.
	virtual void AppendSurfacePropertyNames( CUtlVector<CUtlString> &names, CUtlVector<CUtlString> *pDescriptions = nullptr, bool bIncludeHidden = false ) const = 0;
	virtual const CPhysSurfaceProperties *GetDefaultSurfaceProperty() const = 0;
	// AMNOTE: Counts resource load attempts, including reloads.
	virtual uint32 GetResourceLoadCount() const = 0;
	// AMNOTE: A null input uses the default properties, or resets the binding if none are loaded. A nonnull input preserves the binding's unknown material value.
	virtual void PopulateShapeMaterialBinding( const CPhysSurfaceProperties *pProperties, VPhys2ShapeMaterialBinding &binding ) const = 0;

private:
	IPhysSurfacePropertyController() = default;
	IPhysSurfacePropertyController( const IPhysSurfacePropertyController & ) = delete;
	IPhysSurfacePropertyController &operator=( const IPhysSurfacePropertyController & ) = delete;
	friend class CVPhysics2SurfacePropertyManager;
};

// AMNOTE: Engine-owned; do not construct, implement or delete it. Private unknown methods reserve unresolved native entries.
abstract_class IVPhysics2 : public IAppSystem
{
protected:
	virtual ~IVPhysics2() = default;

public:
	// AMNOTE: Creates a world owned by this interface; destroy it with DestroyWorld.
	virtual IVPhysics2World *CreateWorld( uint32 nFlags ) = 0;
	virtual void DestroyWorld( IVPhysics2World *pWorld ) = 0;
	virtual int GetWorldCount() = 0;
	// AMNOTE: Requires an index below GetWorldCount().
	virtual IVPhysics2World *GetWorld( int nIndex ) = 0;
	// AMNOTE: The optional results object collects world step transitions; clear its vectors before a new collection.
	virtual void SimulateWorlds( IVPhysics2World **ppWorlds, int nWorldCount, float flTimeStep, int nSubsteps, bool bAllowThreading, VPhysXWorldStepResults *pResults ) = 0;
	virtual void SimulateSoftbodies( IVPhysics2World **ppWorlds, int nWorldCount, float flTimeStep, int nUnknown ) = 0;
	// AMNOTE: Updates or clears debugging records for each world.
	virtual void UpdateWorldDebugStreams() = 0;
	// AMNOTE: Threading and SIMD switches are process-wide.
	virtual bool IsMultithreadingEnabled() = 0;
	virtual void SetMultithreadingEnabled( bool bEnabled ) = 0;
	virtual bool IsSIMDEnabled() = 0;
	virtual void SetSIMDEnabled( bool bEnabled ) = 0;
	// AMNOTE: Returns the borrowed surface-property controller.
	virtual IPhysSurfacePropertyController *GetSurfacePropertyController() = 0;
	virtual void InitIntersectionDictionary() = 0;
	virtual void LoadCollisionDetailLayers( bool bUnknown ) = 0;
	virtual IPhysIntersectionDictionary *GetIntersectionDictionary() = 0;
	virtual const IPhysIntersectionDictionary *GetIntersectionDictionary() const = 0;
	virtual IPhysIntersectionController *GetIntersectionController() = 0;
	virtual const IPhysIntersectionController *GetIntersectionController() const = 0;
	// AMNOTE: Standalone shapes must be destroyed with DestroyShape.
	virtual IPhysicsShape *CreateSphereShape( const RnSphere_t &sphere, float flScale ) = 0;
	virtual IPhysicsShape *CreateCapsuleShape( const RnCapsule_t &capsule, float flScale ) = 0;
	// AMNOTE: Hull, mesh and compound geometry is borrowed and must outlive its shapes.
	virtual IPhysicsShape *CreateHullShape( const RnHull_t *pHull, float flScale ) = 0;
	virtual IPhysicsShape *CreateMeshShape( const RnMesh_t *pMesh, const Vector &vScale ) = 0;
	virtual IPhysicsShape *CreateCompoundShape( const RnCompound_t *pCompound, float flScale ) = 0;
	// AMNOTE: Destroys a standalone shape created by this interface.
	virtual void DestroyShape( IPhysicsShape *pShape ) = 0;
	// AMNOTE: Both shapes must be nonnull.
	virtual bool ShapesHaveSameCollisionFilter( IPhysicsShape *pShapeA, IPhysicsShape *pShapeB ) = 0;

private:
	virtual void unk001() = 0;
	virtual void unk002() = 0;
	virtual void unk003() = 0;
	virtual void unk004() = 0;
	virtual void unk005() = 0;
	virtual void unk006() = 0;
	virtual void unk007() = 0;

public:
	// AMNOTE: The scale multiplies the hull's plane offsets. Use a fresh result; the material field is unchanged.
	virtual bool CastLineAgainstHull( VPhys2ShapeCastResult_t &result, const RnHull_t &hull, float flScale, const Vector &vStart, const Vector &vDisplacement ) = 0;
	// AMNOTE: The cast geometry is translated by vStart; the target geometry uses targetTransform. Use a fresh result because geometry-only casts leave body, shape and surface fields unchanged.
	virtual bool CastRay( VPhys2CastResult_t &result, const Ray_t &castRay, const Vector &vStart, const Vector &vDisplacement, const Ray_t &targetRay, const CTransform &targetTransform ) = 0;

private:
	virtual void unk101() = 0;
	virtual void unk102() = 0;
	virtual void unk103() = 0;
	virtual void unk104() = 0;

public:
	// AMNOTE: Returns the distance between convex point clouds; finite distances at or below 1/8 unit become zero. Both clouds must be nonempty.
	virtual float ComputeConvexDistance( const Vector *pPointsA, int nPointCountA, const Vector *pPointsB, int nPointCountB ) = 0;
	virtual float ComputeConvexDistance( const matrix3x4_t &transformA, const Vector *pPointsA, int nPointCountA, const matrix3x4_t &transformB, const Vector *pPointsB, int nPointCountB ) = 0;
	// AMNOTE: Requires nonnull shapes supporting convex descriptors. Subtracts both descriptor radii from the transformed convex-core distance.
	virtual float ComputeShapeDistance( IPhysicsShape *pShapeA, const matrix3x4_t &transformA, IPhysicsShape *pShapeB, const matrix3x4_t &transformB ) = 0;
	// AMNOTE: Requires a cache with one to four points and valid vertex indices. A null hull supports only vertex or two-point edge selection. Small-weight points are removed from a copy of the cache.
	virtual VPhys2HullFeaturePair_t GetHullClosestFeatures( const VPhys2ConvexSimplexCache_t &cache, const RnHull_t *pHullA, const RnHull_t *pHullB ) = 0;

private:
	virtual void unk201() = 0;
	virtual void unk202() = 0;
	virtual void unk203() = 0;
	virtual void unk204() = 0;
	virtual void unk205() = 0;

public:
	// AMNOTE: A null destination allocates a new hull; a nonnull destination must be an engine-owned hull.
	virtual RnHull_t *CreateBoxHull( const Vector &vHalfExtents, const Vector &vCenter, RnHull_t *pDestHull ) = 0;
	virtual RnHull_t *CreateConeHull( const Vector &vStart, const Vector &vEnd, float flRadius, RnHull_t *pDestHull ) = 0;
	virtual RnHull_t *CreateCylinderHull( const Vector &vStart, const Vector &vEnd, float flRadius, RnHull_t *pDestHull ) = 0;
	// AMNOTE: A null destination allocates a hull; a nonnull destination must be engine-owned. Null options use their defaults. Recognized box corners use the box builder directly.
	virtual RnHull_t *CreateHull( int nPointCount, const Vector *pPoints, RnHull_t *pDestHull, const VPhys2HullCreationOptions_t *pOptions = nullptr ) = 0;

private:
	// AMNOTE: Reserves the strided point-input overload while its complete input class remains unresolved.
	virtual void CreateHull() = 0;

public:
	virtual RnHull_t *CreateRuntimeHull( int nPointCount, const Vector *pPoints ) = 0;
	virtual RnHull_t *CreateRuntimeHullFromPlanes( int nPlaneCount, const RnPlane_t *pPlanes, float flUnknown, const Vector &vUnknown ) = 0;
	virtual RnHull_t *CreateRandomHull( int nPointCount, float flRadius, float flMinimumVolume, RnHull_t *pDestHull ) = 0;
	// AMNOTE: The destination must be an engine-owned hull.
	virtual void CloneHull( RnHull_t *pDestHull, const RnHull_t *pSourceHull ) = 0;
	virtual void DestroyHull( RnHull_t *pHull ) = 0;
	virtual RnMesh_t *CreateGridMesh( int nHalfExtent, float flSpacing, float flHeight ) = 0;

	// AMNOTE: Copies triangle, vertex and optional per-triangle material data. A null destination allocates a mesh; a nonnull destination must be engine-owned. Null options use their default values.
	virtual RnMesh_t *CreateMesh( int nTriangleCount, const RnTriangle_t *pTriangles, const uint8 *pMaterials, int nVertexCount, const Vector *pVertices, RnMesh_t *pDestMesh, const VPhys2MeshCreationOptions_t *pOptions ) = 0;

private:
	// AMNOTE: Reserves the strided mesh-input overload while its complete input class remains unresolved.
	virtual void CreateMesh() = 0;

public:
	// AMNOTE: The destination must be an engine-owned mesh.
	virtual void CloneMesh( RnMesh_t *pDestMesh, const RnMesh_t *pSourceMesh ) = 0;
	virtual void DestroyMesh( RnMesh_t *pMesh ) = 0;
	virtual void UpdateMeshChangedVertices( RnMesh_t *pMesh ) = 0;

#if ( defined( _MSVC_LANG ) && _MSVC_LANG > 201703L ) || __cplusplus > 201703L
public:
	// AMNOTE: Bounds must cover every triangle referenced by the mesh. The native update ignores the span length.
	virtual void UpdateMeshTriangleAABBs( RnMesh_t *pMesh, std::span<const AABB_t> triangleBounds ) = 0;
	// AMNOTE: Removes these triangles from the hierarchy without erasing the mesh's triangle or vertex arrays.
	virtual void HideMeshTriangles( RnMesh_t *pMesh, std::span<const uint32> triangleIndices ) = 0;
	// AMNOTE: Clones the component geometry. Supplied materials have one byte per component, in sphere, capsule, hull, mesh order. A nonnull destination must be engine-owned.
	virtual RnCompound_t *CreateCompound( std::span<const RnSphere_t> spheres, std::span<const RnCapsule_t> capsules, std::span<const RnHull_t *const> hulls, std::span<const RnMesh_t *const> meshes, std::span<const uint8> materials, RnCompound_t *pDestCompound ) = 0;
#else
private:
	// AMNOTE: These native operations require C++20 std::span.
	virtual void UpdateMeshTriangleAABBs() = 0;
	virtual void HideMeshTriangles() = 0;
	virtual void CreateCompound() = 0;
#endif

public:
	// AMNOTE: The destination must be an engine-owned compound.
	virtual void CloneCompound( RnCompound_t *pDestCompound, const RnCompound_t *pSourceCompound ) = 0;
	virtual void DestroyCompound( RnCompound_t *pCompound ) = 0;

private:
	virtual void unk301() = 0;
	virtual void unk302() = 0;
	virtual void unk303() = 0;
	virtual void unk304() = 0;
	virtual void unk305() = 0;
	virtual void unk306() = 0;
	virtual void unk307() = 0;
	virtual void unk308() = 0;
	virtual void unk309() = 0;
	virtual void unk3010() = 0;

public:
	// AMNOTE: The model is borrowed and must outlive the softbody.
	virtual IPhysicsSoftbody *CreateSoftbody( const CFeModel *pModel ) = 0;
	virtual void DestroySoftbody( IPhysicsSoftbody *pSoftbody ) = 0;
	// AMNOTE: Creates a softbody tracked by the interface. Its model remains borrowed.
	virtual IPhysicsSoftbody *CreateListedSoftbody( const CFeModel *pModel ) = 0;
	// AMNOTE: Requires a live softbody created by CreateListedSoftbody.
	virtual void DestroyListedSoftbody( IPhysicsSoftbody *pSoftbody ) = 0;
	// AMNOTE: Counts listed softbodies and softbodies in this interface's worlds.
	virtual int GetSoftbodyCount() = 0;
	// AMNOTE: Output capacity must be at least GetSoftbodyCount(). Flags 0 includes all; 4 excludes listed softbodies. Bits 1 and 2 select additional native filters.
	virtual int GetSoftbodies( uint8 nFlags, IPhysicsSoftbody **ppSoftbodies ) = 0;

	// AMNOTE: Copies the links. Parent indices and subsequent pose arrays use this order. Destroy the returned solver with DestroyIkSolver.
	virtual IIkSolver *CreateIkSolver( const CUtlVector<VPhys2IkSolverLink_t> &links ) = 0;
	virtual void DestroyIkSolver( IIkSolver *pSolver ) = 0;

private:
	virtual void unk401() = 0;
	// AMNOTE: A positive count allocates uninitialized manifold storage with one retained reference; zero returns null.
	virtual void *AllocContactManifoldBuffer( int nManifoldCount ) = 0;
	// AMNOTE: Returns nonnull allocated storage to the native pool. Call only after its last retained reference has been released.
	virtual void FreeContactManifoldBuffer( void *pBuffer ) = 0;

public:
	// AMNOTE: Low two bits select hull (0), mesh (1) or compound (2).
	virtual void DestroyTaggedGeometry( uintp nTaggedGeometry ) = 0;

public:
	// AMNOTE: Returns local inertia, mass center and mass using the material's density and thickness. Zero density returns identity inertia and zero mass.
	virtual VPhys2MassProperties_t ComputeMassProperties( const RnSphere_t &sphere, const VPhys2ShapeMaterialBinding &material ) const = 0;
	virtual VPhys2MassProperties_t ComputeMassProperties( const RnCapsule_t &capsule, const VPhys2ShapeMaterialBinding &material ) const = 0;
	virtual VPhys2MassProperties_t ComputeMassProperties( const RnHull_t &hull, float flScale, const VPhys2ShapeMaterialBinding &material ) const = 0;

private:
	virtual void unk501() = 0;
	// AMNOTE: Every ConstructFnPhysics method requires caller-owned storage of at least 16 bytes, aligned to 8 bytes. Keep that storage alive while using the returned helper; do not delete the helper.
	virtual IFnPhysicsContact *ConstructFnPhysicsContact( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsAnyJoint *ConstructFnPhysicsAnyJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsWheelJoint *ConstructFnPhysicsWheelJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsWeldJoint *ConstructFnPhysicsWeldJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsPrismaticJoint *ConstructFnPhysicsPrismaticJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsRevoluteJoint *ConstructFnPhysicsRevoluteJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsConicalJoint *ConstructFnPhysicsConicalJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsSpringJoint *ConstructFnPhysicsSpringJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsSphericalJoint *ConstructFnPhysicsSphericalJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsPulleyJoint *ConstructFnPhysicsPulleyJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsGenericJoint *ConstructFnPhysicsGenericJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsSplineJoint *ConstructFnPhysicsSplineJoint( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsAnyShape *ConstructFnPhysicsAnyShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsSphereShape *ConstructFnPhysicsSphereShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsCapsuleShape *ConstructFnPhysicsCapsuleShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsHullShape *ConstructFnPhysicsHullShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsMeshShape *ConstructFnPhysicsMeshShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsCompoundShape *ConstructFnPhysicsCompoundShape( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsBody *ConstructFnPhysicsBody( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsSoftbody *ConstructFnPhysicsSoftbody( void *pMemory, int nMemorySize ) = 0;
	virtual IFnPhysicsWorld *ConstructFnPhysicsWorld( void *pMemory, int nMemorySize ) = 0;
	virtual void unk601() = 0;

public:
	virtual void SetVConComm( IVConComm *pVConComm ) = 0;
	virtual void RunVConCommFrame() = 0;
	virtual bool HasVConCommState() = 0;
	virtual bool StreamToPhysicsDebugger( bool bStart ) = 0;
	virtual bool IsStreamingToPhysicsDebugger() = 0;

private:
	virtual void unk701() = 0;
	virtual void unk702() = 0;
	virtual void unk703() = 0;
	virtual void unk704() = 0;
	virtual void unk705() = 0;

public:
	virtual void PrintInfo( const CCommand &args ) = 0;

private:
	virtual void unk801() = 0;
	virtual void unk802() = 0;

public:
	virtual void DestroyDebugSceneObject( void *pSceneObject ) = 0;

private:
	virtual void unk901() = 0;
	virtual void unk902() = 0;
	virtual void unk903() = 0;

public:
	virtual const char *GetMassDescription( float flMass ) = 0;
	// AMNOTE: Returns the borrowed surface-property resource manager.
	virtual IResourceTypeManager *GetSurfacePropertyTypeManager() = 0;

private:
	friend class CVPhysics2Interface;
	IVPhysics2() = default;
	IVPhysics2( const IVPhysics2 & ) = delete;
	IVPhysics2 &operator=( const IVPhysics2 & ) = delete;

	// AMNOTE: Borrowed pointers to native aggregate allocations, maintained by the engine.
	CUtlLeanVector<void *> m_AllocatedAggregateData;
};

#endif // IVPHYSICS2_H
