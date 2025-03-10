// Copyright Epic Games, Inc. All Rights Reserved.

#include "DifferentialGrowthGeoScriptBPLibrary.h"
#include "DifferentialGrowthGeoScript.h"
#include "Math/UnrealMathUtility.h"
#include "DrawDebugHelpers.h"


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

UDynamicMesh* UDifferentialGrowthGeoScriptBPLibrary::SolveConstraints(
	UDynamicMesh* TargetMesh,
	float DeltaSeconds,
	float SimulationFramerate,
	float DragCoefficient,
	float ForceMultiplier,
	float TargetEdgeLength,
	int32 DebugVertexID,
	int32 DebugTriangleIndex
)
{
	if (TargetMesh == nullptr) return TargetMesh;

	UWorld* World = TargetMesh->GetWorld();

	// Add attributes 

	const FName VelocityAttributeName = "VelocityAttribute";
	const FName SimPositionAttributeName = "SimPositionAttribute";
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

		if (!EditMesh.Attributes()->HasAttachedAttribute(VelocityAttributeName))
		{
			FVector3VertexAttribute* Velocities = new FVector3VertexAttribute(&EditMesh);
			EditMesh.Attributes()->AttachAttribute(VelocityAttributeName, Velocities);
		}

		if (!EditMesh.Attributes()->HasAttachedAttribute(SimPositionAttributeName))
		{
			FVector3VertexAttribute* SimPositions = new FVector3VertexAttribute(&EditMesh);

			for (int32 VertexID : EditMesh.VertexIndicesItr())
			{
				FVector3d Position = EditMesh.GetVertex(VertexID);
				SimPositions->SetValue(VertexID, Position);
			}

			EditMesh.Attributes()->AttachAttribute(SimPositionAttributeName, SimPositions);
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
				FVector3d TriangleEdgeLengths {
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


		float FrameTime = DeltaSeconds;
		if (FrameTime > .25f)
		{
			FrameTime = .25f;
		}

		//UE_LOG(LogTemp, Warning, TEXT("FrameTime: %f"), DeltaSeconds);

		FrameTimeAccumulator += FrameTime;

		float SimDT = 1.0f / SimulationFramerate;

		int32 Iterations = (int32)(FrameTimeAccumulator / SimDT);

		FrameTimeAccumulator -= SimDT * (float)Iterations;

		if (Iterations == 0)
		{
			return;
		}

		//UE_LOG(LogTemp, Warning, TEXT("Iterations: %d * %f = %f"), Iterations, SimDT, Iterations * SimDT);

		FlushPersistentDebugLines(World);

		// Get attributes

		FVector3VertexAttribute* Velocities = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(VelocityAttributeName));
		FVector3VertexAttribute* SimPositions = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(SimPositionAttributeName));
		FVector3VertexAttribute* Forces = static_cast<FVector3VertexAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(ForcesAttributeName));
		FVector3TriangleAttribute* EdgeLengths = static_cast<FVector3TriangleAttribute*>(EditMesh.Attributes()->GetAttachedAttribute(EdgeLengthAttributeName));

		// Adjust Constraints

		for (int32 TriangleID : EditMesh.TriangleIndicesItr())
		{
			UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
			FVector3d VertexA = EditMesh.GetVertex(Triangle[0]);
			FVector3d VertexB = EditMesh.GetVertex(Triangle[1]);
			FVector3d VertexC = EditMesh.GetVertex(Triangle[2]);

			FVector3d MidpointA = (VertexA + VertexB) * .5f;
			FVector3d MidpointB = (VertexB + VertexC) * .5f;
			FVector3d MidpointC = (VertexC + VertexA) * .5f;

			FVector3d TriangleEdgeLengths;
			EdgeLengths->GetValue(TriangleID, TriangleEdgeLengths);

			
			FVector3d DirectionA = VertexA - VertexB;
			FVector3d DirectionB = VertexB - VertexC;
			FVector3d DirectionC = VertexC - VertexA;
			DirectionA.Normalize();
			DirectionB.Normalize();
			DirectionC.Normalize();
			//DrawDebugLine(World, MidpointA - DirectionA * TriangleEdgeLengths[0] * .25f, MidpointA + DirectionA * TriangleEdgeLengths[0] * .25f, FColor::Purple, true, -1.0f, 0, .1f);
			//DrawDebugLine(World, MidpointB - DirectionB * TriangleEdgeLengths[1] * .25f, MidpointB + DirectionB * TriangleEdgeLengths[1] * .25f, FColor::Purple, true, -1.0f, 0, .1f);
			//DrawDebugLine(World, MidpointC - DirectionC * TriangleEdgeLengths[2] * .25f, MidpointC + DirectionC * TriangleEdgeLengths[2] * .25f, FColor::Purple, true, -1.0f, 0, .1f);
			

			float PerlinScale = .05f;

			//float ValueA = FMath::PerlinNoise3D(MidpointA * PerlinScale - .5f);
			//float ValueB = FMath::PerlinNoise3D(MidpointB * PerlinScale - .5f);
			//float ValueC = FMath::PerlinNoise3D(MidpointC * PerlinScale - .5f);

			float ValueA = FMath::Clamp(MidpointA[0] * PerlinScale, -1.0f, 1.0f);
			float ValueB = FMath::Clamp(MidpointB[0] * PerlinScale, -1.0f, 1.0f);
			float ValueC = FMath::Clamp(MidpointC[0] * PerlinScale, -1.0f, 1.0f);

			float GrowthRate = .05f;

			TriangleEdgeLengths[0] *= 1.0f + (ValueA * .5f + .5f) * (SimDT * Iterations) * GrowthRate;
			TriangleEdgeLengths[1] *= 1.0f + (ValueB * .5f + .5f) * (SimDT * Iterations) * GrowthRate;
			TriangleEdgeLengths[2] *= 1.0f + (ValueC * .5f + .5f) * (SimDT * Iterations) * GrowthRate;

			//DrawDebugPoint(World, MidpointA, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueA * .5f + .5f).ToRGBE(), true);
			//DrawDebugPoint(World, MidpointB, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueB * .5f + .5f).ToRGBE(), true);
			//DrawDebugPoint(World, MidpointC, 10.0f, FLinearColor::LerpUsingHSV(FLinearColor{ 1, 0, 0, 1 }, FLinearColor{ 0, 0, 1, 1 }, ValueC * .5f + .5f).ToRGBE(), true);

			EdgeLengths->SetValue(TriangleID, TriangleEdgeLengths);
		}

		// Calculate forces

		// Reset Forces

		for (int32 VertexID : EditMesh.VertexIndicesItr())
		{
			FVector3d Force {0};
			Forces->SetValue(VertexID, Force);
		}

		// Stretch constraint

		for (int32 VertexID : EditMesh.VertexIndicesItr())
		{
			FVector3d Position = EditMesh.GetVertex(VertexID);
			FVector3d Force;
			Forces->GetValue(VertexID, Force);
			FVector3d OtherPosition {0};
			FVector3d TargetPosition{ 0 };
			FVector3d PositionToOtherPosition{ 0 };
			FVector3d OtherPositionToPosition{ 0 };

			EditMesh.EnumerateVertexTriangles(VertexID, [&](int32 TriangleID)
				{
					UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
					int32 TriangleVertexIndex = (
						Triangle[0] == VertexID ? 0 :
						Triangle[1] == VertexID ? 1 :
						2
					);

					int32 OtherVertexID = Triangle[(TriangleVertexIndex + 1) % 3];
					OtherPosition = EditMesh.GetVertex(OtherVertexID);

					FVector3d TriangleEdgeLengths;
					EdgeLengths->GetValue(TriangleID, TriangleEdgeLengths);
					float NewTargetEdgeLength = TriangleEdgeLengths[TriangleVertexIndex];
					
					OtherPositionToPosition = Position - OtherPosition;
					OtherPositionToPosition.Normalize();
					OtherPositionToPosition *= NewTargetEdgeLength * .5f;

					TargetPosition = OtherPosition + OtherPositionToPosition;

					FVector3d ThisForce = TargetPosition - Position;
					//ThisForce *= 2.0f;

					//if (VertexID == DebugVertexID)
					{
						//DrawDebugLine(World, Position, Position + ThisForce, FColor::Green, true, -1.0f, 0, .1f);
					}

					Force += ThisForce;
				}
			);

			Forces->SetValue(VertexID, Force);
		}

		// Bend Constraint
		
		for (int32 VertexID : EditMesh.VertexIndicesItr())
		{
			FVector3d Position = EditMesh.GetVertex(VertexID);
			FVector3d Force;
			Forces->GetValue(VertexID, Force);

			int32 TriangleIndex = 0;

			EditMesh.EnumerateVertexTriangles(VertexID, [&](int32 TriangleID)
			{
				UE::Geometry::FIndex3i Triangle = EditMesh.GetTriangle(TriangleID);
				int32 TriangleVertexIndex = (
					Triangle[0] == VertexID ? 0 :
					Triangle[1] == VertexID ? 1 :
					2
				);

				UE::Geometry::FIndex3i Edges = EditMesh.GetTriEdges(TriangleID);
				int32 OppositeEdgeID = Edges[(TriangleVertexIndex + 1) % 3];

				int32 EdgeVertexIDA = Triangle[(TriangleVertexIndex + 1) % 3];
				int32 EdgeVertexIDB = Triangle[(TriangleVertexIndex + 2) % 3];
				
				FVector3d EdgeVertexA = EditMesh.GetVertex(EdgeVertexIDA);
				FVector3d EdgeVertexB = EditMesh.GetVertex(EdgeVertexIDB);

				//if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
				{
					//DrawDebugLine(World, EdgeVertexA, EdgeVertexB, FColor::Green, true);
				}

				FVector3d EdgeNormal = EdgeVertexB - EdgeVertexA;
				EdgeNormal.Normalize();

				FVector3d ProjectedPosition = FVector::VectorPlaneProject(Position - EdgeVertexA, EdgeNormal);
				ProjectedPosition.Normalize();

				//if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
				{
					//DrawDebugLine(World, EdgeVertexA, EdgeVertexA + ProjectedPosition, FColor::Blue, true);
				}

				float ToRotate = 0;
				int32 OppositeTriangleCount = 0;

				EditMesh.EnumerateEdgeTriangles(OppositeEdgeID, [&](int32 OppositeTriangleID)
				{
					if (TriangleID == OppositeTriangleID)
					{
						return;
					}

					UE::Geometry::FIndex3i OppositeTriangle = EditMesh.GetTriangle(OppositeTriangleID);
					int32 OppositeVertexID = (
						OppositeTriangle[0] != EdgeVertexIDA && OppositeTriangle[0] != EdgeVertexIDB ? OppositeTriangle[0] : 
						OppositeTriangle[1] != EdgeVertexIDA && OppositeTriangle[1] != EdgeVertexIDB ? OppositeTriangle[1] : 
						OppositeTriangle[2]
					);

					FVector3d OppositePosition = EditMesh.GetVertex(OppositeVertexID);
					FVector3d OppositeProjectedPosition = FVector::VectorPlaneProject(OppositePosition - EdgeVertexA, EdgeNormal);
					OppositeProjectedPosition.Normalize();

					//if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
					{
						//DrawDebugLine(World, EdgeVertexA, EdgeVertexA + OppositeProjectedPosition, FColor::Blue, true);
					}

					// we get stuck when faces invert, or even are < 90 degrees
					float Angle = FMath::Acos(ProjectedPosition | OppositeProjectedPosition);
					float Direction = FMath::Sign((OppositeProjectedPosition ^ ProjectedPosition) | EdgeNormal);
					ToRotate += (UE_PI - Angle) * Direction * .5f;
					OppositeTriangleCount++;
				});

				if (OppositeTriangleCount > 0)
				{
					ToRotate /= OppositeTriangleCount;
					//ToRotate *= .0005f;

					FVector3d RotatedPosition = (Position - EdgeVertexA).RotateAngleAxisRad(ToRotate, EdgeNormal) + EdgeVertexA;

					//if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
					{
					//	DrawDebugPoint(World, Position, 10.1f, FColor::Red, true);
						//DrawDebugLine(World, Position, RotatedPosition, FColor::Red, true);
					}

					Force += RotatedPosition - Position;
				}

				TriangleIndex++;
			});

			Forces->SetValue(VertexID, Force);
		}
		
		// Verlet integration

		for (int32 VertexID : EditMesh.VertexIndicesItr())
		{
			FVector3d SimPosition;
			SimPositions->GetValue(VertexID, SimPosition);

			FVector3d Velocity;
			Velocities->GetValue(VertexID, Velocity);

			FVector3d Force;
			Forces->GetValue(VertexID, Force);

			FVector3d FromPosition { 0 };

			Force -= Velocity * DragCoefficient;

			Force *= ForceMultiplier;

			for (int32 i = 0; i < Iterations; ++i)
			{
				FromPosition = SimPosition;

				// Integrate
				SimPosition += Velocity * SimDT;
				SimPosition += Force * SimDT * SimDT * 0.5f;
				Velocity += Force * SimDT;
			//	Velocity *= DragCoefficient;
			}

			SimPositions->SetValue(VertexID, SimPosition);
			Velocities->SetValue(VertexID, Velocity);

			float Alpha = FrameTimeAccumulator / SimDT;
			FVector3d Position = FMath::Lerp(FromPosition, SimPosition, Alpha);
			EditMesh.SetVertex(VertexID, Position);
		}
	});

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
