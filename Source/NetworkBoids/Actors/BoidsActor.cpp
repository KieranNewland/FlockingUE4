// Fill out your copyright notice in the Description page of Project Settings.

#include "BoidsActor.h"


// Sets default values
ABoidsActor::ABoidsActor()
{
	bCanBeDamaged = false;

	//Add sphere component
	static ConstructorHelpers::FObjectFinder<UStaticMesh> pMesh (TEXT("/Engine/BasicShapes/Sphere"));

	m_pSphereComponent = CreateDefaultSubobject<UStaticMeshComponent>(FName("Display Sphere"));
	m_pSphereComponent->SetStaticMesh(pMesh.Object);

	m_pSphereComponent->bOwnerNoSee = false;
	m_pSphereComponent->CastShadow = true;
	m_pSphereComponent->SetHiddenInGame(false);

	m_pSphereComponent->BodyInstance.SetCollisionEnabled(ECollisionEnabled::NoCollision);

	RootComponent = m_pSphereComponent;
}

// Called when the game starts or when spawned
void ABoidsActor::BeginPlay()
{
	Super::BeginPlay();
	
}

void ABoidsActor::FollowTarget(const float DeltaTime, const FVector pTargetPosition, const TArray<ABoidsActor*> aOtherActors, const TArray<ABoidsObstacle*> aObstacles)
{
	FVector pOurLocation = GetActorLocation();

	//Find desired velocity
	FVector pDesiredVelocity = (pTargetPosition - pOurLocation).GetSafeNormal();
	pDesiredVelocity += CalculateSeperationVector(aOtherActors);
	pDesiredVelocity += CalculateAlignmentVector(aOtherActors);
	pDesiredVelocity += CalculateCohesionVector(aOtherActors);
	pDesiredVelocity += CalculateAvoidanceVector(aObstacles);

	if (pDesiredVelocity.Size() == 0)
		return;

	//Smooth our current rotation towards the desired velocity
	FQuat pCurrentRotation = m_pVelocity.ToOrientationQuat();
	FQuat pDesiredRotation = pDesiredVelocity.ToOrientationQuat();
	FQuat pNewRotation = FQuat::Slerp(pCurrentRotation, pDesiredRotation, m_nTurnSpeed * DeltaTime);

	//Accelerate movement to a max, slow down when turning
	m_nCurrentSpeed = FMath::Clamp<float>(m_nCurrentSpeed + m_nAcceleration * DeltaTime, 0, m_nFlightSpeed);

	m_pVelocity = pNewRotation * FVector::ForwardVector * m_nCurrentSpeed * DeltaTime;

	SetActorLocation(pOurLocation + m_pVelocity);
	SetActorRotation(FRotationMatrix::MakeFromX(m_pVelocity).Rotator());
}

FVector ABoidsActor::CalculateSeperationVector(const TArray<ABoidsActor*> aBoids)
{
	//Aim away from all local boids
	FVector pTargetDirection = FVector(0, 0, 0);

	if (aBoids.Num() < 2)
		return pTargetDirection;

	FVector pOurLocation = GetActorLocation();

	FVector pCurrentDifference;
	float nWeight;
	for (int i = 0; i < aBoids.Num(); i++)
	{
		if (aBoids[i] == this)
			continue;

		pCurrentDifference = aBoids[i]->GetActorLocation() - pOurLocation;
		nWeight = FMath::Clamp<float>(1 - (pCurrentDifference.Size() / m_nSeperationDistance), 0, 1);

		pTargetDirection -= pCurrentDifference.GetSafeNormal() * nWeight;
	}

	return pTargetDirection.GetSafeNormal() * m_nSeperationStrength;
}

FVector ABoidsActor::CalculateAlignmentVector(const TArray<ABoidsActor*> aBoids)
{
	//Aim to match the average of local boids velocities
	int nCount = 0;
	FVector pOurLocation = GetActorLocation();
	FVector pTargetDirection = FVector(0, 0, 0);

	float nWeight;
	FVector pOtherVelocity;
	FVector pDifference;
	for (int i = 0; i < aBoids.Num(); i++)
	{
		if (aBoids[i] == this)
			continue;

		pDifference = aBoids[i]->GetActorLocation() - pOurLocation;

		if (pDifference.Size() > m_nAlignmentDist || pDifference.Size() == 0)
			continue;

		pOtherVelocity = aBoids[i]->GetFlockingVelocity();

		if (pOtherVelocity.Size() == 0)
			continue;

		nCount++;
		nWeight = 1 - FMath::Clamp<float>(pDifference.Size() / m_nAlignmentDist, 0, 1);
		pTargetDirection += pOtherVelocity.GetSafeNormal() * nWeight;
	}

	if (nCount == 0)
		return pTargetDirection;

	return pTargetDirection.GetSafeNormal() * m_nAlignmentStrength;
}

FVector ABoidsActor::CalculateCohesionVector(const TArray<ABoidsActor*> aBoids)
{
	//Aim towards the average position of local boids
	int nCount = 0;
	FVector pOurLocation = GetActorLocation();
	FVector pCenterLocation = FVector(0, 0, 0);

	float nDistance;
	FVector pOtherPosition;
	for (int i = 0; i < aBoids.Num(); i++)
	{
		pOtherPosition = aBoids[i]->GetActorLocation();
		nDistance = (pOtherPosition - pOurLocation).Size();

		if (nDistance > m_nCohesionDistance)
			continue;

		nCount++;
		pCenterLocation += pOtherPosition;
	}

	pCenterLocation = pCenterLocation / nCount;

	return (pCenterLocation - pOurLocation).GetSafeNormal() * m_nCohesionStrength;
}

FVector ABoidsActor::CalculateAvoidanceVector(const TArray<ABoidsObstacle*> aObstacles)
{
	//Get vector aiming perpendicular from obstacles
	FVector pOurLocation = GetActorLocation();
	FVector pTotalAvoidanceDirection = FVector(0, 0, 0);

	FVector pObstacleLocation;
	FVector pClosestPoint;
	FVector pAvoidanceDirection;
	FVector pCurrentDirection = m_pVelocity.GetSafeNormal();
	int nCount = 0;
	float nWeight;
	FVector pDifference;
	for (int i = 0; i < aObstacles.Num(); i++)
	{
		pObstacleLocation = aObstacles[i]->GetActorLocation();
		pDifference = pObstacleLocation - pOurLocation;

		if (pDifference.Size() > m_nAvoidanceDistance)
			continue;

		nCount++;
		nWeight = 1 - FMath::Clamp<float>(pDifference.Size() / m_nAvoidanceDistance, 0, 1);

		//Calculate perpendicular direction
		//Get closest point on direction vector to obstacle
		pClosestPoint = pOurLocation + pCurrentDirection * FVector::DotProduct(pObstacleLocation, pCurrentDirection);

		//Direction from center of the obstacle to our closest point, this create a right angle to aim away from
		pAvoidanceDirection = (pClosestPoint - pObstacleLocation).GetSafeNormal();
		pTotalAvoidanceDirection += pAvoidanceDirection * nWeight;
	}

	return pTotalAvoidanceDirection.GetSafeNormal() * m_nAvoidanceStrength;
}