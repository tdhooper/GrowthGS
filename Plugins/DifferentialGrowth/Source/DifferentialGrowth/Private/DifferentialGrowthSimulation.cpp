// Fill out your copyright notice in the Description page of Project Settings.

#include "DifferentialGrowthSimulation.h"

ADifferentialGrowthSimulation::ADifferentialGrowthSimulation()
{
 	PrimaryActorTick.bCanEverTick = true;
	
	DynamicMeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("DynamicMeshComponent"));

	SimulationFramerate = 30.0f;
	DragCoefficient = 0.05f;
	ForceMultiplier = 10.0f;
	EdgeSplitLength = 8.0f;
	bRestrictSimulationToGrowthAreas = true;
	bGrowthEnabled = true;
	GrowthRate = 0.05f;
	bStretchForceEnabled = true;
	StretchForceEdgePullFactor = 0.5f;
	bBendForceEnabled = true;
}

// Called when the game starts or when spawned
void ADifferentialGrowthSimulation::BeginPlay()
{
	Super::BeginPlay();

	FrameTimeAccumulator = 0.0f;

	// Setup attributes

	UDynamicMesh* TargetMesh = DynamicMeshComponent->GetDynamicMesh();

	TargetMesh->EditMesh([&](FDynamicMesh3& EditMesh)
		{
			EditMesh.EnableAttributes();

			if (!EditMesh.Attributes()->HasAttachedAttribute("Force"))
			{
				ForceAttributes = new FVector3VertexAttribute(&EditMesh);
				EditMesh.Attributes()->AttachAttribute("Force", ForceAttributes);
			}
			else
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
			else
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

				EditMesh.Attributes()->AttachAttribute("PreviousPosition", ForceAttributes);
			}
			else
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
			else
			{
				EdgeLengthAttributes = static_cast<FVector3TriangleAttribute*>(EditMesh.Attributes()->GetAttachedAttribute("EdgeLength"));
			}
		}
	);
}

void ADifferentialGrowthSimulation::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	float SimDT = 1.0f / SimulationFramerate;

	int MaxSteps = 3;
	float FrameTime = FMath::Min(DeltaTime, SimDT * MaxSteps);

	FrameTimeAccumulator += FrameTime;

	int Iterations = 0;

	while (FrameTimeAccumulator > SimDT)
	{
		Solve(SimDT);
		FrameTimeAccumulator -= SimDT;
		Iterations++;
	}
}

void ADifferentialGrowthSimulation::Solve(float DeltaSeconds)
{
	UDynamicMesh* TargetMesh = DynamicMeshComponent->GetDynamicMesh();

	if (TargetMesh == nullptr) return;

	TargetMesh->EditMesh([&](FDynamicMesh3& EditMesh)
		{
			Integrate(EditMesh, DeltaSeconds);
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

			float VertexGrowthRate = GrowthRateAttributes->GetValue(VertexID);

			if (bRestrictSimulationToGrowthAreas && VertexGrowthRate <= 0.0f)
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


