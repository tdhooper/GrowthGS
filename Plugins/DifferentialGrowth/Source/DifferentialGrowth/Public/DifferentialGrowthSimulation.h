// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UDynamicMesh.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMeshScalarVertexAttribute.h"
#include "DifferentialGrowthSimulation.generated.h"

typedef UE::Geometry::TDynamicMeshVertexAttribute<float, 3> FVector3VertexAttribute;
typedef UE::Geometry::TDynamicMeshTriangleAttribute<float, 3> FVector3TriangleAttribute;
typedef UE::Geometry::TDynamicMeshScalarVertexAttribute<float> FloatVertexAttribute;

UCLASS()
class DIFFERENTIALGROWTH_API ADifferentialGrowthSimulation : public AActor
{
	GENERATED_BODY()
	
public:	
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UDynamicMeshComponent> DynamicMeshComponent;

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

	UPROPERTY(EditAnywhere, Category = "Forces|Stretch", DisplayName = "Pull Factor")
	float StretchForceEdgePullFactor;

	UPROPERTY(EditAnywhere, Category = "Forces|Bend", DisplayName = "Enabled")
	bool bBendForceEnabled;

	ADifferentialGrowthSimulation();

protected:

	float FrameTimeAccumulator;

	FloatVertexAttribute* GrowthRateAttributes;
	FVector3VertexAttribute* PreviousPositionAttributes;
	FVector3VertexAttribute* ForceAttributes;
	FVector3TriangleAttribute* EdgeLengthAttributes;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void Solve(float DeltaSeconds);

	virtual void Integrate(FDynamicMesh3& EditMesh, float DeltaSeconds);


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};