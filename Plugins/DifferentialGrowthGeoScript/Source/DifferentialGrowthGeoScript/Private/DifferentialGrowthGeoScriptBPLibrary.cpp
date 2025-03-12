// Copyright Epic Games, Inc. All Rights Reserved.

#include "DifferentialGrowthGeoScriptBPLibrary.h"
#include "DifferentialGrowthGeoScript.h"
#include "Math/UnrealMathUtility.h"
#include "DrawDebugHelpers.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"


UDifferentialGrowthGeoScriptBPLibrary::UDifferentialGrowthGeoScriptBPLibrary(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{
}

float UDifferentialGrowthGeoScriptBPLibrary::DifferentialGrowthGeoScriptSampleFunction(float Param)
{
	return -1;
}

// TODO: Move solver into an UActorComponent
float FrameTimeAccumulator = 0.0f;

UDynamicMesh* SolveConstraintsStep(
	UDynamicMesh* TargetMesh,
	float DeltaSeconds,
	float SimulationFramerate,
	float DragCoefficient,
	float ForceMultiplier,
	float TargetEdgeLength,
	float EdgePullFactor,
	float GrowthRate,
	int32 DebugVertexID,
	int32 DebugTriangleIndex
)
{
	if (TargetMesh == nullptr) return TargetMesh;

	UWorld* World = TargetMesh->GetWorld();

	// Add attributes 

	//const FName VelocityAttributeName = "VelocityAttribute";
	const FName PreviousPositionAttributeName = "PreviousPositionAttribute";
	const FName ForcesAttributeName = "ForcesAttribute";
	const FName EdgeLengthAttributeName = "EdgeLengthAttribute";

	TargetMesh->EditMesh([&](FDynamicMesh3& EditMesh)
		{

			// Setup attributes
			// Cribbed from UPlaneCutTool::Setup

			EditMesh.EnableAttributes();

			if (!EditMesh.Attributes()->HasAttachedAttribute(ForcesAttributeName))
			{
				FVector3VertexAttribute* Forces = new FVector3VertexAttribute(&EditMesh);
				EditMesh.Attributes()->AttachAttribute(ForcesAttributeName, Forces);

				FrameTimeAccumulator = 0;
			}

			/*if (!EditMesh.Attributes()->HasAttachedAttribute(VelocityAttributeName))
			{
				FVector3VertexAttribute* Velocities = new FVector3VertexAttribute(&EditMesh);
				EditMesh.Attributes()->AttachAttribute(VelocityAttributeName, Velocities);
			}*/

			if (!EditMesh.Attributes()->HasAttachedAttribute(PreviousPositionAttributeName))
			{
				FVector3VertexAttribute* PreviousPositions = new FVector3VertexAttribute(&EditMesh);

				for (int32 VertexID : EditMesh.VertexIndicesItr())
				{
					FVector3d Position = EditMesh.GetVertex(VertexID);
					PreviousPositions->SetValue(VertexID, Position);
				}

				EditMesh.Attributes()->AttachAttribute(PreviousPositionAttributeName, PreviousPositions);
			}

			if (!EditMesh.Attributes()->HasAttachedAttribute(EdgeLengthAttributeName))
			{
				FVector3TriangleAttribute* EdgeLengths = new FVector3TriangleAttribute(&EditMesh);

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
					//TriangleEdgeLengths[0] = TargetEdgeLength;
					//TriangleEdgeLengths[1] = TargetEdgeLength;
					//TriangleEdgeLengths[2] = TargetEdgeLength;
					EdgeLengths->SetValue(TriangleID, TriangleEdgeLengths);
				}

				EditMesh.Attributes()->AttachAttribute(EdgeLengthAttributeName, EdgeLengths);
			}

			// Get attributes

			//FVector3VertexAttribute* Velocities = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(VelocityAttributeName));
			FVector3VertexAttribute* PreviousPositions = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(PreviousPositionAttributeName));
			FVector3VertexAttribute* Forces = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(ForcesAttributeName));
			FVector3TriangleAttribute* EdgeLengths = static_cast<FVector3TriangleAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(EdgeLengthAttributeName));


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
				UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(Edge.Tri[0]);

				FVector3d VertexA = EditMesh.GetVertex(Triangle.A);
				FVector3d VertexB = EditMesh.GetVertex(Triangle.B);
				FVector3d VertexC = EditMesh.GetVertex(Triangle.C);

				TriangleEdgeLengths[0] = (VertexA - VertexB).Length();
				TriangleEdgeLengths[1] = (VertexB - VertexC).Length();
				TriangleEdgeLengths[2] = (VertexC - VertexA).Length();

				int TriangleEdgeIndex = IndexUtil::FindEdgeIndexInTri(Edge.Vert.A, Edge.Vert.B, Triangle);
				float EdgeLengthA = TriangleEdgeLengths[TriangleEdgeIndex];
				float EdgeLengthB = TriangleEdgeLengths[(TriangleEdgeIndex + 1) % 3];
				float EdgeLengthC = TriangleEdgeLengths[(TriangleEdgeIndex + 2) % 3];

				if (EdgeLengthA < TargetEdgeLength || EdgeLengthA < EdgeLengthB || EdgeLengthA < EdgeLengthC)
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

				if (Edge.Tri[1] != UE::Geometry::FDynamicMesh3::InvalidID)
				{
					VtxD = IndexUtil::FindTriOtherVtx(VtxA, VtxB, TriangleB);
				}

				EdgeLengths->GetValue(Edge.Tri[0], TriangleEdgeLengths);
				float TriAEdgeLengthAB = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxA, VtxB, TriangleA)];
				float TriAEdgeLengthBC = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxB, VtxC, TriangleA)];
				float TriAEdgeLengthCA = TriangleEdgeLengths[IndexUtil::FindEdgeIndexInTri(VtxC, VtxA, TriangleA)];

				float TriBEdgeLengthAB;
				float TriBEdgeLengthBD;
				float TriBEdgeLengthDA;
				if (Edge.Tri[1] != UE::Geometry::FDynamicMesh3::InvalidID)
				{
					EdgeLengths->GetValue(Edge.Tri[1], TriangleEdgeLengths);
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

				// Update edge attributes
				// (attributes for new vertex is interpolated automatically)

				//UE_LOG(LogTemp, Warning, TEXT("EdgeIndexInTri: %f"), IndexUtil::FindEdgeIndexInTri(VtxA, VtxB, OriginalTriangleA));
				//UE_LOG(LogTemp, Warning, TEXT("TriangleEdgeLengths: %f %f %f"), TriangleEdgeLengths[0], TriangleEdgeLengths[1], TriangleEdgeLengths[2]);
				//UE_LOG(LogTemp, Warning, TEXT("OriginalTriangleASplitEdgeLength: %f"), OriginalTriangleASplitEdgeLength);

				UE::Geometry::FIndex3i OriginalTriangleA = EditMesh.GetTriangle(SplitInfo.OriginalTriangles.A);
				UE::Geometry::FIndex3i OriginalTriangleB;
				if (!SplitInfo.bIsBoundary)
				{
					OriginalTriangleB = EditMesh.GetTriangle(SplitInfo.OriginalTriangles.B);
				}

				EdgeLengths->GetValue(SplitInfo.OriginalTriangles.A, TriangleEdgeLengths);

				// Original triangle A shrunk edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxA, VtxF, OriginalTriangleA)
				] = TriAEdgeLengthAB * .5f;

				// Original triangle A new edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxC, VtxF, OriginalTriangleA)
				] = TriAEdgeLengthAB * .5f;

				// Original triangle A unmodified edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxC, VtxA, OriginalTriangleA)
				] = TriAEdgeLengthCA;

				EdgeLengths->SetValue(SplitInfo.OriginalTriangles.A, TriangleEdgeLengths);


				if (!SplitInfo.bIsBoundary)
				{
					EdgeLengths->GetValue(SplitInfo.OriginalTriangles.B, TriangleEdgeLengths);

					// Original triangle B shrunk edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxA, VtxF, OriginalTriangleB)
					] = TriBEdgeLengthAB * .5f;

					// Original triangle B new edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxD, VtxF, OriginalTriangleB)
					] = TriBEdgeLengthAB * .5f;

					// Original triangle B unmodified edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxD, VtxA, OriginalTriangleB)
					] = TriBEdgeLengthDA;

					EdgeLengths->SetValue(SplitInfo.OriginalTriangles.B, TriangleEdgeLengths);
				}


				UE::Geometry::FIndex3i NewTriangleA = EditMesh.GetTriangle(SplitInfo.NewTriangles.A);
				EdgeLengths->GetValue(SplitInfo.NewTriangles.A, TriangleEdgeLengths);

				// New triangle A shrunk edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxB, VtxF, NewTriangleA)
				] = TriAEdgeLengthAB * .5f;

				// New triangle A new edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxC, VtxF, NewTriangleA)
				] = TriAEdgeLengthAB * .5f;

				// New triangle A unmodified edge
				TriangleEdgeLengths[
					IndexUtil::FindEdgeIndexInTri(VtxB, VtxC, NewTriangleA)
				] = TriAEdgeLengthBC;

				EdgeLengths->SetValue(SplitInfo.NewTriangles.A, TriangleEdgeLengths);


				if (!SplitInfo.bIsBoundary)
				{
					UE::Geometry::FIndex3i NewTriangleB = EditMesh.GetTriangle(SplitInfo.NewTriangles.B);
					EdgeLengths->GetValue(SplitInfo.NewTriangles.B, TriangleEdgeLengths);

					// New triangle B shrunk edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxB, VtxF, NewTriangleB)
					] = TriBEdgeLengthAB * .5f;

					// New triangle B new edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxD, VtxF, NewTriangleB)
					] = TriBEdgeLengthAB * .5f;

					// New triangle B unmodified edge
					TriangleEdgeLengths[
						IndexUtil::FindEdgeIndexInTri(VtxB, VtxD, NewTriangleB)
					] = TriBEdgeLengthBD;

					EdgeLengths->SetValue(SplitInfo.NewTriangles.B, TriangleEdgeLengths);
				}
			}


			float DT = DeltaSeconds;

			FlushPersistentDebugLines(World);


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

					FVector3d MidpointA = (VertexA + VertexB) * .5f;
					FVector3d MidpointB = (VertexB + VertexC) * .5f;
					FVector3d MidpointC = (VertexC + VertexA) * .5f;

					FVector3d TriangleEdgeLengths;
					EdgeLengths->GetValue(TriangleID, TriangleEdgeLengths);

					float PerlinScale = .05f;

					//float ValueA = FMath::PerlinNoise3D(MidpointA * PerlinScale - .5f);
					//float ValueB = FMath::PerlinNoise3D(MidpointB * PerlinScale - .5f);
					//float ValueC = FMath::PerlinNoise3D(MidpointC * PerlinScale - .5f);

					float ValueA = FMath::Clamp(MidpointA[2] * PerlinScale, -1.0f, 1.0f);
					float ValueB = FMath::Clamp(MidpointB[2] * PerlinScale, -1.0f, 1.0f);
					float ValueC = FMath::Clamp(MidpointC[2] * PerlinScale, -1.0f, 1.0f);

					TriangleEdgeLengths[0] *= 1.0f + (ValueA * .5f + .5f) * DT * GrowthRate;
					TriangleEdgeLengths[1] *= 1.0f + (ValueB * .5f + .5f) * DT * GrowthRate;
					TriangleEdgeLengths[2] *= 1.0f + (ValueC * .5f + .5f) * DT * GrowthRate;

					FVector3d Center = (VertexA + VertexB + VertexC) / 3.0f;
					FVector3d DirectionA = VertexA - VertexB;
					FVector3d DirectionB = VertexB - VertexC;
					FVector3d DirectionC = VertexC - VertexA;
					DirectionA.Normalize();
					DirectionB.Normalize();
					DirectionC.Normalize();
					//DrawDebugLine(World, FMath::Lerp(MidpointA, Center, .1f) - DirectionA * TriangleEdgeLengths[0] * .25f, FMath::Lerp(MidpointA, Center, .1f) + DirectionA * TriangleEdgeLengths[0] * .25f, FColor::Yellow, true, -1.0f, 0, .1f);
					//DrawDebugLine(World, FMath::Lerp(MidpointB, Center, .1f) - DirectionB * TriangleEdgeLengths[1] * .25f, FMath::Lerp(MidpointB, Center, .1f) + DirectionB * TriangleEdgeLengths[1] * .25f, FColor::Yellow, true, -1.0f, 0, .1f);
					//DrawDebugLine(World, FMath::Lerp(MidpointC, Center, .1f) - DirectionC * TriangleEdgeLengths[2] * .25f, FMath::Lerp(MidpointC, Center, .1f) + DirectionC * TriangleEdgeLengths[2] * .25f, FColor::Yellow, true, -1.0f, 0, .1f);


					//DrawDebugPoint(World, MidpointA, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueA * .5f + .5f).ToRGBE(), true);
					//DrawDebugPoint(World, MidpointB, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueB * .5f + .5f).ToRGBE(), true);
					//DrawDebugPoint(World, MidpointC, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueC * .5f + .5f).ToRGBE(), true);

					EdgeLengths->SetValue(TriangleID, TriangleEdgeLengths);
				}
			);

			// Calculate forces


			// Reset Forces

			{
				TRACE_CPUPROFILER_EVENT_SCOPE_STR("Reset Forces");
				Forces->Initialize(0);
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
					Forces->GetValue(VertexID, Force);
					FVector3d OtherPosition{ 0 };
					FVector3d TargetPosition{ 0 };

					EditMesh.EnumerateVertexTriangles(VertexID, [&](int32 TriangleID)
						{
							UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
							int32 TriangleVertexIndex = IndexUtil::FindTriIndex(VertexID, Triangle);

							int32 OtherVertexID = Triangle[(TriangleVertexIndex + 1) % 3];
							OtherPosition = EditMesh.GetVertex(OtherVertexID);

							FVector3d TriangleEdgeLengths;
							EdgeLengths->GetValue(TriangleID, TriangleEdgeLengths);
							float NewTargetEdgeLength = TriangleEdgeLengths[TriangleVertexIndex];


							FVector3d OtherPositionToPosition = Position - OtherPosition;
							OtherPositionToPosition.Normalize();
							OtherPositionToPosition *= NewTargetEdgeLength * (1.0f - EdgePullFactor);
							TargetPosition = OtherPosition + OtherPositionToPosition;
							FVector3d ThisForce = TargetPosition - Position;


							/*
							FVector3d Direction = Position - OtherPosition;
							Direction.Normalize();
							FVector3d Midpoint = (Position + OtherPosition) * .5f;
							TargetPosition = Midpoint + Direction * NewTargetEdgeLength * .25f;
							FVector3d ThisForce = TargetPosition - Position;
							*/


							//if (VertexID == DebugVertexID)
							{
								//DrawDebugLine(World, Midpoint - Direction * NewTargetEdgeLength * .125f, Midpoint + Direction * NewTargetEdgeLength * .125f, FColor::Yellow, true, -1.0f, 0, .15f);
								//DrawDebugLine(World, Position, OtherPosition + Direction * NewTargetEdgeLength * .5f, FColor::Orange, true, -1.0f, 0, .1f);
								//DrawDebugLine(World, Position, Position + (OtherPosition - Position) * .25f, FColor::Magenta, true, -1.0f, 2, .05f);
								//DrawDebugLine(World, Position, Position + ThisForce, FColor::Magenta, true, -1.0f, 0, .05f);
							}

							Force += ThisForce;
						}
					);

					Forces->SetValue(VertexID, Force);
				}
			);


			// Bend Constraint

			ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
				{
					TRACE_CPUPROFILER_EVENT_SCOPE_STR("Bend Constraint");

					if (!EditMesh.IsVertex(VertexID))
					{
						return;
					}

					FVector3d Position = EditMesh.GetVertex(VertexID);
					FVector3d Force;
					Forces->GetValue(VertexID, Force);

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
							float Direction = FMath::Sign((OppositeProjectedPosition ^ ProjectedPosition) | EdgeNormal);
							float ToRotate = (UE_PI - Angle) * Direction * .5f;

							FVector3d RotatedPosition = (Position - EdgeVertexA).RotateAngleAxisRad(ToRotate, EdgeNormal) + EdgeVertexA;

							Force += RotatedPosition - Position;
						});

					Forces->SetValue(VertexID, Force);
				});


			// Verlet integration

			ParallelFor(EditMesh.MaxVertexID(), [&](int32 VertexID)
				{
					TRACE_CPUPROFILER_EVENT_SCOPE_STR("Verlet integration");

					if (!EditMesh.IsVertex(VertexID))
					{
						return;
					}

					FVector3d PreviousPosition;
					PreviousPositions->GetValue(VertexID, PreviousPosition);

					FVector3d Position = EditMesh.GetVertex(VertexID);

					//FVector3d Velocity;
					//Velocities->GetValue(VertexID, Velocity);

					FVector3d Force;
					Forces->GetValue(VertexID, Force);

					PreviousPositions->SetValue(VertexID, Position);

					FVector3d Velocity = Position - PreviousPosition;
					Position = Position + Velocity * (1.0f - DragCoefficient) + Force * (1.0f - DragCoefficient) * DT * DT;

					//DrawDebugLine(World, Position, PreviousPosition + Force, FColor::Green, true);
					//DrawDebugLine(World, Position, Position + Velocity, FColor::Red, true);

					EditMesh.SetVertex(VertexID, Position);
				}
			);
		}
	);

	//FVector3d Position = EditMesh.GetVertex(VertexID);
	//FVector3d PreviousPosition;
	//PreviousPositions->GetValue(VertexID, PreviousPosition);
	//FVector3d Force;
	//Forces->GetValue(VertexID, Force);

	//FVector3d TempPosition = Position;
	//FVector3d Velocity = Position - PreviousPosition;

	// Drag
	//Force -= Velocity * DragCoefficient;

	//Position += Velocity + Force * DeltaTime * DeltaTime;
	//EditMesh.SetVertex(VertexID, Position);

	//PreviousPositions->SetValue(VertexID, TempPosition);


	// what's the difference between Overlay and Attribute?

	// FDynamicMesh3
	// --> FDynamicMeshAttributeSet
	//		--> TDynamicMeshOverlay
	//		--> TDynamicMeshTriangleAttribute
	//		--> TDynamicMeshVertexAttribute

	//PreviousPositionOverlay->CreateFromPredicate([](int ParentVID, int TriIDA, int TriIDB) {return true;}, 0.f);


	
	return TargetMesh;
}

UDynamicMesh* UDifferentialGrowthGeoScriptBPLibrary::SolveConstraints(
	UDynamicMesh* TargetMesh,
	float DeltaSeconds,
	float SimulationFramerate,
	float DragCoefficient,
	float ForceMultiplier,
	float TargetEdgeLength,
	float EdgePullFactor,
	float GrowthRate,
	int32 DebugVertexID,
	int32 DebugTriangleIndex
)
{
	float SimDT = 1.0f / SimulationFramerate;

	int MaxSteps = 3;
	float FrameTime = FMath::Min(DeltaSeconds, SimDT * MaxSteps);

	FrameTimeAccumulator += FrameTime;

	int Iterations = 0;

	while (FrameTimeAccumulator > SimDT)
	{
		TargetMesh = SolveConstraintsStep(
			TargetMesh,
			SimDT,
			SimulationFramerate,
			DragCoefficient,
			ForceMultiplier,
			TargetEdgeLength,
			EdgePullFactor,
			GrowthRate,
			DebugVertexID,
			DebugTriangleIndex
		);

		FrameTimeAccumulator -= SimDT;
		Iterations++;
	}

	//UE_LOG(LogTemp, Warning, TEXT("Iterations: %d"), Iterations);

	return TargetMesh;
}
