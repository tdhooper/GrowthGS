// Fill out your copyright notice in the Description page of Project Settings.

#include "DifferentialGrowthSimulation.h"
#include "DynamicMesh/MeshNormals.h"

ADifferentialGrowthSimulation::ADifferentialGrowthSimulation()
{
    PrimaryActorTick.bCanEverTick = true;
    bSimulate = true;
    SimulationFramerate = 30.0f;
    DragCoefficient = 0.05f;
    ForceMultiplier = 10.0f;
    EdgeSplitLength = 8.0f;
    bRestrictSimulationToGrowthAreas = true;
    bGrowthEnabled = true;
    GrowthRate = 0.05f;
    bStretchForceEnabled = true;
    StretchForceEdgePullFactor = 0.5f;
    bStretchForceDebugEnabled = false;
    bBendForceEnabled = true;
}

// Called when the game starts or when spawned
void ADifferentialGrowthSimulation::BeginPlay()
{
    Super::BeginPlay();

    FrameTimeAccumulator = 0.0f;
}

void ADifferentialGrowthSimulation::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bSimulate) return;

    UDynamicMesh* TargetMesh = DynamicMeshComponent->GetDynamicMesh();

    if (TargetMesh == nullptr) return;

    float SimDT = 1.0f / SimulationFramerate;

    int MaxSteps = 3;
    float FrameTime = FMath::Min(DeltaTime, SimDT * MaxSteps);

    FrameTimeAccumulator += FrameTime;

    UWorld* World = GetWorld();

    // TODO: Allow each step to add a EDynamicMeshAttributeChangeFlags
    TargetMesh->EditMesh([&](FDynamicMesh3& EditMesh)
        {
            SetUpAttributes(EditMesh);

            while (FrameTimeAccumulator > SimDT)
            {
                Solve(EditMesh, SimDT);
                FrameTimeAccumulator -= SimDT;
            }

            FlushPersistentDebugLines(World);
            DrawDebugOverlays(World, EditMesh);

            FGeometryScriptCalculateNormalsOptions CalculateNormalsOptions;
            RecomputeNormals(EditMesh, CalculateNormalsOptions);
        }
    );
}

void ADifferentialGrowthSimulation::SetUpAttributes(FDynamicMesh3& EditMesh)
{
    EditMesh.EnableAttributes();

    if (!EditMesh.Attributes()->HasAttachedAttribute("Force"))
    {
        ForceAttributes = new FVector3VertexAttribute(&EditMesh);
        EditMesh.Attributes()->AttachAttribute("Force", ForceAttributes);
    }
    else if (ForceAttributes == nullptr)
    {
        ForceAttributes = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute("Force"));
    }

    if (!EditMesh.Attributes()->HasAttachedAttribute("GrowthRate"))
    {
        GrowthRateAttributes = new FloatVertexAttribute(&EditMesh);

        for (int32 VertexID : EditMesh.VertexIndicesItr())
        {
            GrowthRateAttributes->SetValue(VertexID, 0.0f);
        }

        EditMesh.Attributes()->AttachAttribute("GrowthRate", GrowthRateAttributes);
    }
    else if (GrowthRateAttributes == nullptr)
    {
        GrowthRateAttributes = static_cast<FloatVertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute("GrowthRate"));
    }


    if (!EditMesh.Attributes()->HasAttachedAttribute("PreviousPosition"))
    {
        PreviousPositionAttributes = new FVector3VertexAttribute(&EditMesh);

        for (int32 VertexID : EditMesh.VertexIndicesItr())
        {
            FVector3d Position = EditMesh.GetVertex(VertexID);
            PreviousPositionAttributes->SetValue(VertexID, Position);
        }

        EditMesh.Attributes()->AttachAttribute("PreviousPosition", PreviousPositionAttributes);
    }
    else if (PreviousPositionAttributes == nullptr)
    {
        PreviousPositionAttributes = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute("PreviousPosition"));
    }

    if (!EditMesh.Attributes()->HasAttachedAttribute("EdgeLength"))
    {
        EdgeLengthAttributes = new FVector3TriangleAttribute(&EditMesh);

        for (int32 TriangleID : EditMesh.TriangleIndicesItr())
        {
            UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
            FVector3d VertexA = EditMesh.GetVertex(Triangle[0]);
            FVector3d VertexB = EditMesh.GetVertex(Triangle[1]);
            FVector3d VertexC = EditMesh.GetVertex(Triangle[2]);
            FVector3d TriangleEdgeLengths{
                (VertexA - VertexB).Length(),
                (VertexB - VertexC).Length(),
                (VertexC - VertexA).Length()
            };
            EdgeLengthAttributes->SetValue(TriangleID, TriangleEdgeLengths);
        }

        EditMesh.Attributes()->AttachAttribute("EdgeLength", EdgeLengthAttributes);
    }
    else if (EdgeLengthAttributes == nullptr)
    {
        EdgeLengthAttributes = static_cast<FVector3TriangleAttribute*>(EditMesh.Attributes()->GetAttachedAttribute("EdgeLength"));
    }
}

void ADifferentialGrowthSimulation::Solve(FDynamicMesh3& EditMesh, float DeltaSeconds)
{
    ForceAttributes->Initialize(0);
    AdjustGrowthRates(EditMesh);
    SplitLongEdges(EditMesh);
    StretchConstraint(EditMesh, DeltaSeconds);
    BendConstraint(EditMesh);
    Integrate(EditMesh, DeltaSeconds);
}

// Copied from MeshNormalsFunctions.cpp
void ADifferentialGrowthSimulation::RecomputeNormals(FDynamicMesh3& EditMesh, FGeometryScriptCalculateNormalsOptions CalculateOptions)
{
    UE::Geometry::FMeshNormals MeshNormals(&EditMesh);
    if (EditMesh.Attributes()->PrimaryNormals()->ElementCount() == 0)
    {
        //UE::Geometry::AppendWarning(Debug, EGeometryScriptErrorType::InvalidInputs, LOCTEXT("RecomputeNormals_NothingToRecompute", "RecomputeNormals: TargetMesh did not have normals to recompute; falling back to per-vertex normals. Consider using 'Set Mesh To Per Vertex Normals' or 'Compute Split Normals' instead."));
        EditMesh.Attributes()->PrimaryNormals()->CreateFromPredicate([](int, int, int)->bool {return true;}, 0.0f);
    }
    MeshNormals.RecomputeOverlayNormals(EditMesh.Attributes()->PrimaryNormals(), CalculateOptions.bAreaWeighted, CalculateOptions.bAngleWeighted);
    MeshNormals.CopyToOverlay(EditMesh.Attributes()->PrimaryNormals(), false);
}

// Fast Marching
// 
// Vertex data
//    Classification: Far, Known, Neighbour
//    T: float
// 
// 1. Starting boundary points have T = 0, classified as Known
//	  adjacent points to boundary are classified as Neighbour
//    all other points classified as Far
// 
// 2. Calculate T values for Neighbour points, if a T value already
//    exists, keep the smallest one
// 
// 3. Neighbour that has smallest T value is classified as Known
//    and its 'Far' adjacent points are classified as Neighbour
// 
// 4. Repeat 2
// 
// Calculating T values for vertices
// 
// for each triangle, where our vertex is C
//	if it has 1 Known adjacent vertex, A, use A + distance(A,C)
//	if it has 2 known adjacent vertices
//		if triangle is non-obtuse (has no angle greater than 90)
// 
//		C = (TA + TB + sqrt((T_A + T_B)^2 - (T_A^2 + T_B^2 - d_AC^2/v^2 - d_BC^2/v^2))) / 2
//			
// 
// pick the lowest T value of all triangles
// 
// We don't want to use the classifications as we want to constantaly update every vertex
// as the boundary moves
// 
// we'll have to make sure we only select upwind triangles to get data from?
// look at our SDF propagation approach


void ADifferentialGrowthSimulation::SplitLongEdges(FDynamicMesh3& EditMesh)
{
    // Split long edges

    // TODO: Paralellise
    // pass 1 identifies edges to split in parallell
    // pass 2 does splits
    // pass 3 updates edge data in parallel

    for (int32 EdgeID : EditMesh.EdgeIndicesItr())
    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Split Long Edges");

        FVector3d TriangleEdgeLengths;

        UE::Geometry::FDynamicMesh3::FEdge Edge = EditMesh.GetEdge(EdgeID);

        if (
            bRestrictSimulationToGrowthAreas
            && GrowthRateAttributes->GetValue(Edge.Vert.A) <= 0.0f
            && GrowthRateAttributes->GetValue(Edge.Vert.B) <= 0.0f
        )
        {
            continue;
        }

        UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(Edge.Tri[0]);

        FVector3d VertexA = EditMesh.GetVertex(Triangle.A);
        FVector3d VertexB = EditMesh.GetVertex(Triangle.B);
        FVector3d VertexC = EditMesh.GetVertex(Triangle.C);
        FVector3d VertexD;

        TriangleEdgeLengths[0] = (VertexA - VertexB).Length();
        TriangleEdgeLengths[1] = (VertexB - VertexC).Length();
        TriangleEdgeLengths[2] = (VertexC - VertexA).Length();

        int TriangleEdgeIndex = IndexUtil::FindEdgeIndexInTri(Edge.Vert.A, Edge.Vert.B, Triangle);
        float EdgeLength = TriangleEdgeLengths[TriangleEdgeIndex];
        float EdgeLength1 = TriangleEdgeLengths[(TriangleEdgeIndex + 1) % 3];
        float EdgeLength2 = TriangleEdgeLengths[(TriangleEdgeIndex + 2) % 3];

        if (EdgeLength < EdgeSplitLength || EdgeLength < EdgeLength1 || EdgeLength < EdgeLength2)
        {
            continue;
        }

        UE::Geometry::FIndex3i TriangleA = EditMesh.GetTriangle(Edge.Tri[0]);
        UE::Geometry::FIndex3i TriangleB;

        if (Edge.Tri[1] != UE::Geometry::FDynamicMesh3::InvalidID)
        {
            TriangleB = EditMesh.GetTriangle(Edge.Tri[1]);
        }

        int VtxA = Edge.Vert.A;
        int VtxB = Edge.Vert.B;
        int VtxC = IndexUtil::OrientTriEdgeAndFindOtherVtx(VtxA, VtxB, TriangleA);
        int VtxD;

        VertexA = EditMesh.GetVertex(VtxA);
        VertexB = EditMesh.GetVertex(VtxB);
        VertexC = EditMesh.GetVertex(VtxC);

        if (Edge.Tri[1] != UE::Geometry::FDynamicMesh3::InvalidID)
        {
            VtxD = IndexUtil::FindTriOtherVtx(VtxA, VtxB, TriangleB);
            VertexD = EditMesh.GetVertex(VtxD);
        }

        EdgeLengthAttributes->GetValue(Edge.Tri[0], TriangleEdgeLengths);
        float TriAEdgeLengthAB = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxA, VtxB, TriangleA)];
        float TriAEdgeLengthBC = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxB, VtxC, TriangleA)];
        float TriAEdgeLengthCA = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxC, VtxA, TriangleA)];

        float TriBEdgeLengthAB;
        float TriBEdgeLengthBD;
        float TriBEdgeLengthDA;
        if (Edge.Tri[1] != UE::Geometry::FDynamicMesh3::InvalidID)
        {
            EdgeLengthAttributes->GetValue(Edge.Tri[1], TriangleEdgeLengths);
            TriBEdgeLengthAB = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxA, VtxB, TriangleB)];
            TriBEdgeLengthBD = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxB, VtxD, TriangleB)];
            TriBEdgeLengthDA = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxD, VtxA, TriangleB)];
        }


        UE::Geometry::FDynamicMesh3::FEdgeSplitInfo SplitInfo;
        UE::Geometry::EMeshResult Result = EditMesh.SplitEdge(Edge.Vert[0], Edge.Vert[1], SplitInfo);
        if (Result != UE::Geometry::EMeshResult::Ok)
        {
            continue;
        }

        int VtxF = SplitInfo.NewVertex;
        FVector3d VertexF = EditMesh.GetVertex(VtxF);

        FVector3d AC = VertexC - VertexA;
        AC.Normalize();
        FVector3d AF = VertexF - VertexA;
        AF.Normalize();
        float TriACosineAngleA = AC | AF;

        float TriBCosineAngleB;
        if (!SplitInfo.bIsBoundary)
        {
            VertexD = EditMesh.GetVertex(VtxD);
            FVector3d BD = VertexD - VertexB;
            BD.Normalize();
            FVector3d BF = VertexF - VertexB;
            BF.Normalize();
            TriBCosineAngleB = BD | BF;
        }

        float TriAEdgeLengthAF = TriAEdgeLengthAB * .5f;
        float TriEdgeLengthCF = sqrt(TriAEdgeLengthCA * TriAEdgeLengthCA + TriAEdgeLengthAF * TriAEdgeLengthAF - 2.0f * TriAEdgeLengthCA * TriAEdgeLengthAF * TriACosineAngleA);

        float TriEdgeLengthDF;
        float TriBEdgeLengthBF;
        if (!SplitInfo.bIsBoundary)
        {
            TriBEdgeLengthBF = TriBEdgeLengthAB * .5f;
            TriEdgeLengthDF = sqrt(TriBEdgeLengthBD * TriBEdgeLengthBD + TriBEdgeLengthBF * TriBEdgeLengthBF - 2.0f * TriBEdgeLengthBD * TriBEdgeLengthBF * TriBCosineAngleB);
        }

        // Update edge attributes
        // (attributes for new vertex is interpolated automatically)

        UE::Geometry::FIndex3i OriginalTriangleA = EditMesh.GetTriangle(SplitInfo.OriginalTriangles.A);
        UE::Geometry::FIndex3i OriginalTriangleB;
        if (!SplitInfo.bIsBoundary)
        {
            OriginalTriangleB = EditMesh.GetTriangle(SplitInfo.OriginalTriangles.B);
        }

        EdgeLengthAttributes->GetValue(SplitInfo.OriginalTriangles.A, TriangleEdgeLengths);

        /*

            c ____________ b        c ____________ b
             |           /|          |\          /|
             | Original / |          | \  New   / |
             |    A    /  |    -->   |  \  A   /  |
             |        /   |          |   \    /   |
             |       /    |          |    \  /    |
             |      /     |          | Ori \/ New |
             |     /      |          |  A  /\  B  |
             |    /       |          |    / f\    |
             |   /        |          |   /    \   |
             |  /    B    |          |  /   B  \  |
             | / Original |          | /   Ori  \ |
             |/___________|          |/__________\|
            a              d        a              d
        
        */

        // Original triangle A shrunk edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxA, VtxF, OriginalTriangleA)
        ] = TriAEdgeLengthAF;

        // Original triangle A new edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxC, VtxF, OriginalTriangleA)
        ] = TriEdgeLengthCF;

        // Original triangle A unmodified edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxC, VtxA, OriginalTriangleA)
        ] = TriAEdgeLengthCA;

        EdgeLengthAttributes->SetValue(SplitInfo.OriginalTriangles.A, TriangleEdgeLengths);


        if (!SplitInfo.bIsBoundary)
        {
            EdgeLengthAttributes->GetValue(SplitInfo.OriginalTriangles.B, TriangleEdgeLengths);

            // Original triangle B shrunk edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxA, VtxF, OriginalTriangleB)
            ] = TriAEdgeLengthAF;

            // Original triangle B new edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxD, VtxF, OriginalTriangleB)
            ] = TriEdgeLengthDF;

            // Original triangle B unmodified edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxD, VtxA, OriginalTriangleB)
            ] = TriBEdgeLengthDA;

            EdgeLengthAttributes->SetValue(SplitInfo.OriginalTriangles.B, TriangleEdgeLengths);
        }


        UE::Geometry::FIndex3i NewTriangleA = EditMesh.GetTriangle(SplitInfo.NewTriangles.A);
        EdgeLengthAttributes->GetValue(SplitInfo.NewTriangles.A, TriangleEdgeLengths);

        // New triangle A shrunk edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxB, VtxF, NewTriangleA)
        ] = TriAEdgeLengthAF;

        // New triangle A new edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxC, VtxF, NewTriangleA)
        ] = TriEdgeLengthCF;

        // New triangle A unmodified edge
        TriangleEdgeLengths[
            IndexUtil::FindEdgeIndexInTri(VtxB, VtxC, NewTriangleA)
        ] = TriAEdgeLengthBC;

        EdgeLengthAttributes->SetValue(SplitInfo.NewTriangles.A, TriangleEdgeLengths);


        if (!SplitInfo.bIsBoundary)
        {
            UE::Geometry::FIndex3i NewTriangleB = EditMesh.GetTriangle(SplitInfo.NewTriangles.B);
            EdgeLengthAttributes->GetValue(SplitInfo.NewTriangles.B, TriangleEdgeLengths);

            // New triangle B shrunk edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxB, VtxF, NewTriangleB)
            ] = TriBEdgeLengthBF;

            // New triangle B new edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxD, VtxF, NewTriangleB)
            ] = TriEdgeLengthDF;

            // New triangle B unmodified edge
            TriangleEdgeLengths[
                IndexUtil::FindEdgeIndexInTri(VtxB, VtxD, NewTriangleB)
            ] = TriBEdgeLengthBD;

            EdgeLengthAttributes->SetValue(SplitInfo.NewTriangles.B, TriangleEdgeLengths);
        }
    }
}

void ADifferentialGrowthSimulation::AdjustGrowthRates(FDynamicMesh3& EditMesh)
{
    if (!bGrowthEnabled)
    {
        GrowthRateAttributes->Initialize(0);
        return;
    }

    ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("Adjust Growth Rate");

            if (!EditMesh.IsVertex(VertexID))
            {
                return;
            }

            FVector3d Vertex = EditMesh.GetVertex(VertexID);

            float PerlinScale = .03f;

            float Value = FMath::PerlinNoise3D(Vertex * PerlinScale - .5f);

            float Mask = FMath::Clamp(Vertex[2] * .01f, 0.0f, 1.0f);

            Value *= Mask;

            GrowthRateAttributes->SetValue(VertexID, Value);
        }
    );
}

void ADifferentialGrowthSimulation::StretchConstraint(FDynamicMesh3& EditMesh, float DeltaSeconds)
{
    if (!bStretchForceEnabled)
    {
        return;
    }

    if (bGrowthEnabled)
    {
        // Adjust Edge Length Constraints

        ParallelFor(EditMesh.MaxTriangleID(), [&](int32 TriangleID)
            {
                TRACE_CPUPROFILER_EVENT_SCOPE_STR("Adjust Edge Length Constraints");

                if (!EditMesh.IsTriangle(TriangleID))
                {
                    return;
                }

                UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
                FVector3d VertexA = EditMesh.GetVertex(Triangle[0]);
                FVector3d VertexB = EditMesh.GetVertex(Triangle[1]);
                FVector3d VertexC = EditMesh.GetVertex(Triangle[2]);

                float GrowthRateA = GrowthRateAttributes->GetValue(Triangle[0]);
                float GrowthRateB = GrowthRateAttributes->GetValue(Triangle[1]);
                float GrowthRateC = GrowthRateAttributes->GetValue(Triangle[2]);

                FVector3d MidpointA = (VertexA + VertexB) * .5f;
                FVector3d MidpointB = (VertexB + VertexC) * .5f;
                FVector3d MidpointC = (VertexC + VertexA) * .5f;

                float EdgeGrowthRateA = (GrowthRateA + GrowthRateB) * .5f;
                float EdgeGrowthRateB = (GrowthRateB + GrowthRateC) * .5f;
                float EdgeGrowthRateC = (GrowthRateC + GrowthRateA) * .5f;

                FVector3d TriangleEdgeLengths;
                EdgeLengthAttributes->GetValue(TriangleID, TriangleEdgeLengths);

                TriangleEdgeLengths[0] *= 1.0f + (GrowthRateA * .5f + .5f) * DeltaSeconds * GrowthRate;
                TriangleEdgeLengths[1] *= 1.0f + (GrowthRateB * .5f + .5f) * DeltaSeconds * GrowthRate;
                TriangleEdgeLengths[2] *= 1.0f + (GrowthRateC * .5f + .5f) * DeltaSeconds * GrowthRate;

                FVector3d Center = (VertexA + VertexB + VertexC) / 3.0f;
                FVector3d DirectionA = VertexA - VertexB;
                FVector3d DirectionB = VertexB - VertexC;
                FVector3d DirectionC = VertexC - VertexA;
                DirectionA.Normalize();
                DirectionB.Normalize();
                DirectionC.Normalize();

                EdgeLengthAttributes->SetValue(TriangleID, TriangleEdgeLengths);
            }
        );
    }

    // Stretch constraint

    ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("Stretch constraint");

            if (!EditMesh.IsVertex(VertexID))
            {
                return;
            }

            FVector3d Position = EditMesh.GetVertex(VertexID);
            FVector3d Force;
            ForceAttributes->GetValue(VertexID, Force);
            FVector3d OtherPosition{ 0 };
            FVector3d TargetPosition{ 0 };

            if (bRestrictSimulationToGrowthAreas && GrowthRateAttributes->GetValue(VertexID) <= 0.0f)
            {
                return;
            }

            EditMesh.EnumerateVertexTriangles(VertexID, [&](int32 TriangleID)
                {
                    UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
                    int32 TriangleVertexIndex = IndexUtil::FindTriIndex(VertexID, Triangle);

                    int32 OtherVertexID = Triangle[(TriangleVertexIndex + 1) % 3];
                    OtherPosition = EditMesh.GetVertex(OtherVertexID);

                    FVector3d TriangleEdgeLengths;
                    EdgeLengthAttributes->GetValue(TriangleID, TriangleEdgeLengths);
                    float NewTargetEdgeLength = TriangleEdgeLengths[TriangleVertexIndex];


                    FVector3d OtherPositionToPosition = Position - OtherPosition;
                    OtherPositionToPosition.Normalize();
                    OtherPositionToPosition *= NewTargetEdgeLength * (1.0f - StretchForceEdgePullFactor);
                    TargetPosition = OtherPosition + OtherPositionToPosition;
                    Force += TargetPosition - Position;
                }
            );

            ForceAttributes->SetValue(VertexID, Force);
        }
    );
}

void ADifferentialGrowthSimulation::BendConstraint(FDynamicMesh3& EditMesh)
{
    if (!bBendForceEnabled)
    {
        return;
    }

    ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("Bend Constraint");

            if (!EditMesh.IsVertex(VertexID))
            {
                return;
            }

            if (bRestrictSimulationToGrowthAreas && GrowthRateAttributes->GetValue(VertexID) <= 0.0f)
            {
                return;
            }

            FVector3d Position = EditMesh.GetVertex(VertexID);
            FVector3d Force;
            ForceAttributes->GetValue(VertexID, Force);

            EditMesh.EnumerateVertexTriangles(VertexID, [&](int32 TriangleID)
                {
                    UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
                    int32 TriangleVertexIndex = IndexUtil::FindTriIndex(VertexID, Triangle);

                    UE::Geometry::FIndex3i Edges = EditMesh.GetTriEdges(TriangleID);
                    int32 OppositeEdgeID = Edges[(TriangleVertexIndex + 1) % 3];

                    UE::Geometry::FDynamicMesh3::FEdge Edge = EditMesh.GetEdge(OppositeEdgeID);
                    int32 OppositeTriangleID = Edge.Tri.A == TriangleID ? Edge.Tri.B : Edge.Tri.A;

                    if (!EditMesh.IsTriangle(OppositeTriangleID))
                    {
                        return;
                    }

                    int32 EdgeVertexIDA = Triangle[(TriangleVertexIndex + 1) % 3];
                    int32 EdgeVertexIDB = Triangle[(TriangleVertexIndex + 2) % 3];

                    FVector3d EdgeVertexA = EditMesh.GetVertex(EdgeVertexIDA);
                    FVector3d EdgeVertexB = EditMesh.GetVertex(EdgeVertexIDB);

                    FVector3d EdgeNormal = EdgeVertexB - EdgeVertexA;
                    EdgeNormal.Normalize();

                    FVector3d ProjectedPosition = FVector::VectorPlaneProject(Position - EdgeVertexA, EdgeNormal);
                    ProjectedPosition.Normalize();


                    UE::Geometry::FIndex3i OppositeTriangle = EditMesh.GetTriangle(OppositeTriangleID);
                    int32 OppositeVertexID = IndexUtil::FindTriOtherVtxUnsafe(EdgeVertexIDA, EdgeVertexIDB, OppositeTriangle);

                    FVector3d OppositePosition = EditMesh.GetVertex(OppositeVertexID);
                    FVector3d OppositeProjectedPosition = FVector::VectorPlaneProject(OppositePosition - EdgeVertexA, EdgeNormal);
                    OppositeProjectedPosition.Normalize();

                    float Angle = FMath::Acos(ProjectedPosition | OppositeProjectedPosition);
                    float Direction = ((OppositeProjectedPosition ^ ProjectedPosition) | EdgeNormal) < 0.0f ? -1.0f : 1.0f;
                    float ToRotate = (UE_PI - Angle) * Direction * .5f;

                    FVector3d RotatedPosition = (Position - EdgeVertexA).RotateAngleAxisRad(ToRotate, EdgeNormal) + EdgeVertexA;

                    Force += RotatedPosition - Position;
                });

            ForceAttributes->SetValue(VertexID, Force);
        }
    );
}

void ADifferentialGrowthSimulation::Integrate(FDynamicMesh3& EditMesh, float DeltaSeconds)
{
    // Verlet integration

    ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("Verlet integration");

            if (!EditMesh.IsVertex(VertexID))
            {
                return;
            }

            if (bRestrictSimulationToGrowthAreas && GrowthRateAttributes->GetValue(VertexID) <= 0.0f)
            {
                return;
            }

            FVector3d Position = EditMesh.GetVertex(VertexID);

            FVector3d PreviousPosition;
            PreviousPositionAttributes->GetValue(VertexID, PreviousPosition);
            PreviousPositionAttributes->SetValue(VertexID, Position);

            FVector3d Force;
            ForceAttributes->GetValue(VertexID, Force);

            FVector3d Velocity = Position - PreviousPosition;
            Position = Position + Velocity * (1.0f - DragCoefficient) + Force * ForceMultiplier * DeltaSeconds * DeltaSeconds;

            EditMesh.SetVertex(VertexID, Position);
        }
    );
}


void ADifferentialGrowthSimulation::DrawDebugOverlays(UWorld* World, FDynamicMesh3& EditMesh)
{
    if (bStretchForceDebugEnabled)
    {
        for (int32 TriangleID : EditMesh.TriangleIndicesItr())
        {
            UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);

            FVector3d Vertices[3]{
                EditMesh.GetVertex(Triangle[0]),
                EditMesh.GetVertex(Triangle[1]),
                EditMesh.GetVertex(Triangle[2])
            };

            FVector3d TriangleEdgeLengths;
            EdgeLengthAttributes->GetValue(TriangleID, TriangleEdgeLengths);

            FVector3d Normal = (Vertices[1] - Vertices[0]) ^ (Vertices[2] - Vertices[0]);
            Normal.Normalize();

            const FTransform& TRS = ActorToWorld();

            for (int i = 0; i < 3; i++)
            {
                FVector3d Vertex = Vertices[i];
                FVector3d OtherVertex = Vertices[(i + 1) % 3];
                FVector3d Midpoint = FMath::Lerp(Vertex, OtherVertex, .5f);
                FVector3d Direction = Vertex - OtherVertex;
                Direction.Normalize();
                FVector3d Tangent = Direction ^ Normal;
                Midpoint += Tangent * .1f;
                FVector3d LineFrom = Midpoint - Direction * TriangleEdgeLengths[i] * .25f;
                FVector3d LineTo = Midpoint + Direction * TriangleEdgeLengths[i] * .25f;
                LineFrom = TRS.TransformPosition(LineFrom);
                LineTo = TRS.TransformPosition(LineTo);
                DrawDebugLine(World, LineFrom, LineTo, FColor::Yellow, true, -1.0f, 0, .1f);
            }
        }
    }
}


