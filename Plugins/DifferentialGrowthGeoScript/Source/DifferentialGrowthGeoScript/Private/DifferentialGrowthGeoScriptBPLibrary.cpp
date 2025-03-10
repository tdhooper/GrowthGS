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
	const FName InitialPositionAttributeName = "InitialPositionAttribute";
	const FName SimPositionAttributeName = "SimPositionAttribute";
	const FName ForcesAttributeName = "ForcesAttribute";

	TargetMesh->EditMesh([&](FDynamicMesh3& EditMesh)
	{

		// Setup attributes
		// Cribbed from UPlaneCutTool::Setup

		EditMesh.EnableAttributes();

		if (!EditMesh.Attributes()->HasAttachedAttribute(ForcesAttributeName))
		{
			FVector3Attribute* Forces = new FVector3Attribute(&EditMesh);
			EditMesh.Attributes()->AttachAttribute(ForcesAttributeName, Forces);

			FrameTimeAccumulator = 0;
		}

		if (!EditMesh.Attributes()->HasAttachedAttribute(VelocityAttributeName))
		{
			FVector3Attribute* Velocities = new FVector3Attribute(&EditMesh);
			EditMesh.Attributes()->AttachAttribute(VelocityAttributeName, Velocities);
		}


		if (!EditMesh.Attributes()->HasAttachedAttribute(InitialPositionAttributeName))
		{
			FVector3Attribute* InitialPositions = new FVector3Attribute(&EditMesh);

			for (int32 VertexID : EditMesh.VertexIndicesItr())
			{
				FVector3d Position = EditMesh.GetVertex(VertexID);
				InitialPositions->SetValue(VertexID, Position);
			}

			EditMesh.Attributes()->AttachAttribute(InitialPositionAttributeName, InitialPositions);
		}

		if (!EditMesh.Attributes()->HasAttachedAttribute(SimPositionAttributeName))
		{
			FVector3Attribute* SimPositions = new FVector3Attribute(&EditMesh);

			for (int32 VertexID : EditMesh.VertexIndicesItr())
			{
				FVector3d Position = EditMesh.GetVertex(VertexID);
				SimPositions->SetValue(VertexID, Position);
			}

			EditMesh.Attributes()->AttachAttribute(SimPositionAttributeName, SimPositions);
		}


		float FrameTime = DeltaSeconds;
		if (FrameTime > .25f)
		{
			FrameTime = .25f;
		}

		UE_LOG(LogTemp, Warning, TEXT("FrameTime: %f"), DeltaSeconds);

		FrameTimeAccumulator += FrameTime;

		float SimDT = 1.0f / SimulationFramerate;

		int32 Iterations = (int32)(FrameTimeAccumulator / SimDT);

		FrameTimeAccumulator -= SimDT * (float)Iterations;

		if (Iterations == 0)
		{
			return;
		}

		UE_LOG(LogTemp, Warning, TEXT("Iterations: %d * %f = %f"), Iterations, SimDT, Iterations * SimDT);

		FlushPersistentDebugLines(World);

		// Get attributes

		FVector3Attribute* Velocities = static_cast<FVector3Attribute*>(EditMesh.Attributes()->GetAttachedAttribute(VelocityAttributeName));
		FVector3Attribute* InitialPositions = static_cast<FVector3Attribute*>(EditMesh.Attributes()->GetAttachedAttribute(InitialPositionAttributeName));
		FVector3Attribute* SimPositions = static_cast<FVector3Attribute*>(EditMesh.Attributes()->GetAttachedAttribute(SimPositionAttributeName));
		FVector3Attribute* Forces = static_cast<FVector3Attribute*>(EditMesh.Attributes()->GetAttachedAttribute(ForcesAttributeName));

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
			FVector3d Midpoint{ 0 };
			FVector3d MidpointToPosition{ 0 };
			FVector3d PositionToOtherPosition{ 0 };
			FVector3d TargetPosition{ 0 };
			float SquaredTargetLength = TargetEdgeLength * TargetEdgeLength;

			EditMesh.EnumerateVertexVertices(VertexID, [&](int32 OtherVertexID)
			{
				OtherPosition = EditMesh.GetVertex(OtherVertexID);
				//Midpoint = (Position + OtherPosition) / 2.0f;
				//MidpointToPosition = Position - Midpoint;
				//TargetPosition = Midpoint + MidpointToPosition * (SquaredTargetLength / MidpointToPosition.SquaredLength());
				//Force += (TargetPosition - Position) * .01f;
				PositionToOtherPosition = OtherPosition - Position;
				float f = PositionToOtherPosition.Length() - TargetEdgeLength * .5f;
				PositionToOtherPosition.Normalize();
				FVector3d ThisForce = PositionToOtherPosition * f;
				Force += ThisForce;

				//DrawDebugLine(World, Position, Position + ThisForce, FColor::Blue, true, -1.f, 0, .1f);
			});


			Forces->SetValue(VertexID, Force);
		}

		/*
		
		// Pin Constraint

		for (int32 VertexID : EditMesh.VertexIndicesItr())
		{
			FVector3d Position = EditMesh.GetVertex(VertexID);
			FVector3d OtherPosition;
			InitialPositions->GetValue(VertexID, OtherPosition);
			FVector3d Force;
			Forces->GetValue(VertexID, Force);
			FVector3d Midpoint{ 0 };
			FVector3d MidpointToPosition{ 0 };
			FVector3d PositionToOtherPosition{ 0 };
			FVector3d TargetPosition{ 0 };
			float SquaredTargetLength = TargetEdgeLength * TargetEdgeLength;

			PositionToOtherPosition = OtherPosition - Position;
			Force += PositionToOtherPosition * .5f;

			Forces->SetValue(VertexID, Force);
		}
		*/



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

				if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
				{
					//DrawDebugLine(World, EdgeVertexA, EdgeVertexB, FColor::Green, true);
				}

				FVector3d EdgeNormal = EdgeVertexB - EdgeVertexA;
				EdgeNormal.Normalize();

				FVector3d ProjectedPosition = FVector::VectorPlaneProject(Position - EdgeVertexA, EdgeNormal);
				ProjectedPosition.Normalize();

				if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
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

					if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
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

					if (VertexID == DebugVertexID && TriangleIndex == DebugTriangleIndex)
					{
					//	DrawDebugPoint(World, Position, 10.1f, FColor::Red, true);
					//	DrawDebugLine(World, Position, RotatedPosition, FColor::Red, true);
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
