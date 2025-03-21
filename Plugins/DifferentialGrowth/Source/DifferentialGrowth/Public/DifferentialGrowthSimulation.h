// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DynamicMeshActor.h"
#include "UDynamicMesh.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMeshScalarVertexAttribute.h"
#include "GeometryScript/MeshNormalsFunctions.h"
#include "DifferentialGrowthSimulation.generated.h"

typedef UE::Geometry::TDynamicMeshVertexAttribute<float, 3> FVector3VertexAttribute;
typedef UE::Geometry::TDynamicMeshTriangleAttribute<float, 3> FVector3TriangleAttribute;
typedef UE::Geometry::TDynamicMeshScalarVertexAttribute<float> FloatVertexAttribute;

UCLASS()
class DIFFERENTIALGROWTH_API ADifferentialGrowthSimulation : public ADynamicMeshActor
{
	GENERATED_BODY()
	
public:	
	UPROPERTY(EditAnywhere)
	bool bSimulate;

	UPROPERTY(EditAnywhere)
	float SimulationFramerate;

	UPROPERTY(EditAnywhere)
	float DragCoefficient;

	UPROPERTY(EditAnywhere)
	float ForceMultiplier;

	UPROPERTY(EditAnywhere)
	float EdgeSplitLength;

	UPROPERTY(EditAnywhere)
	bool bRestrictSimulationToGrowthAreas;

	UPROPERTY(EditAnywhere, Category = "Forces|Growth", DisplayName = "Enabled")
	bool bGrowthEnabled;

	UPROPERTY(EditAnywhere, Category = "Forces|Growth", DisplayName = "Rate")
	float GrowthRate;

	UPROPERTY(EditAnywhere, Category="Forces|Stretch", DisplayName="Enabled")
	bool bStretchForceEnabled;

	UPROPERTY(EditAnywhere, Category = "Forces|Stretch", DisplayName = "Force")
	float StretchForceMultiplier;

	UPROPERTY(EditAnywhere, Category = "Forces|Stretch", DisplayName = "Pull Factor")
	float StretchForceEdgePullFactor;

	UPROPERTY(EditAnywhere, Category = "Forces|Stretch", DisplayName = "Show Debug")
	bool bStretchForceDebugEnabled;

	UPROPERTY(EditAnywhere, Category = "Forces|Bend", DisplayName = "Enabled")
	bool bBendForceEnabled;

	UPROPERTY(EditAnywhere, Category = "Forces|Bend", DisplayName = "Force")
	float BendForceMultiplier;

	ADifferentialGrowthSimulation();

protected:

	float FrameTimeAccumulator;
	bool bSimulationExploded;

	FloatVertexAttribute* GrowthRateAttributes;
	FVector3VertexAttribute* PreviousPositionAttributes;
	FVector3VertexAttribute* ForceAttributes;
	FVector3TriangleAttribute* EdgeLengthAttributes;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void SetUpAttributes(FDynamicMesh3& EditMesh);

	virtual void Solve(FDynamicMesh3& EditMesh, float DeltaSeconds);

	virtual void RecomputeNormals(FDynamicMesh3& EditMesh, FGeometryScriptCalculateNormalsOptions CalculateOptions);

	virtual void SplitLongEdges(FDynamicMesh3& EditMesh);

	virtual void AdjustGrowthRates(FDynamicMesh3& EditMesh);

	virtual void StretchConstraint(FDynamicMesh3& EditMesh, float DeltaSeconds);

	virtual void BendConstraint(FDynamicMesh3& EditMesh);

	virtual void Integrate(FDynamicMesh3& EditMesh, float DeltaSeconds);

	virtual void DrawDebugOverlays(UWorld* World, FDynamicMesh3& EditMesh);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};