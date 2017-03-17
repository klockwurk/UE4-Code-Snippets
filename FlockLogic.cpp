/*******************************************************************************/
/*!
\file   Aggro.cs
\author Khan Sweetman
\par    All content © 2015 DigiPen (USA) Corporation, all rights reserved.
\par    Team Arcus
\brief
    Basic implementation for flocking.

*/
/*******************************************************************************/

#include "Visceral.h"
#include "FlockLogic.h"
#include "Runtime/Engine/Public/StaticMeshResources.h"
#include "Runtime/Core/Public/Templates/ScopedPointer.h"

// ------------------------------------------------- Life Cycle -------------------------------------------------- //
UFlockLogic::UFlockLogic() :
    Controller(NULL),
    Spline(NULL),
    FormationMesh(NULL),
    DiveAcceleration(300.0f),
    DiveRotationSpeed(2.0f),
    DiveTime(2.0f),
    GameTimer(0.0f)
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
    // off to improve performance if you don't need them.
    bWantsBeginPlay = true;
    PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts
void UFlockLogic::BeginPlay()
{
    Super::BeginPlay();
    Controller = (AAIController*)GetOwner()->GetInstigatorController();
    Spline = GetOwner()->FindComponentByClass<USplineComponent>();
    TArray<UActorComponent*> meshes = GetOwner()->GetComponentsByClass(UStaticMeshComponent::StaticClass());
    for (int32 i = 0; i < meshes.Num(); ++i)
    {
        if (meshes[i]->ComponentHasTag(FName("Flocking")))
        {
            FormationMesh = (UStaticMeshComponent*)meshes[i];
            break;
        }
    }
    if (FormationMesh != NULL && FormationMesh->StaticMesh != NULL && FormationMesh->StaticMesh->RenderData.GetOwnedPointer() != NULL)
    {
        FPositionVertexBuffer* buffer = &FormationMesh->StaticMesh->RenderData.GetOwnedPointer()->LODResources[0].PositionVertexBuffer;
        for (uint32 i = 0; i < buffer->GetNumVertices(); ++i)
        {
            FormationVertices.Add(buffer->VertexPosition(i));
        }
    }
    Rand.GenerateNewSeed();

    // Populate Flock
    for (int i = 0; i < NumStartingAgents; ++i)
    {
        FVector pos;
        // Spawn around spline if we're using it
        if (LineFollowWeight != 0.0f)
        {
            float length = Spline->GetSplineLength();
            pos = Spline->GetLocationAtDistanceAlongSpline(Rand.GetFraction() * length, ESplineCoordinateSpace::World);
        }
        else
        {
            pos = GetOwner()->GetActorLocation() + Rand.GetUnitVector() * Rand.FRandRange(StartDistMin, StartDistMax);
        }
        Flock.Emplace(SpawnBoid(pos, FRotator::ZeroRotator));
    }

    // Determine active flocking functions
    if (AlignmentWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::AlignmentI);
    if (CohesionWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::CohesionI);
    if (CurrentDirWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::CurrentDirectionI);
    if (SeparationWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::SeparationI);
    if (LineFollowWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::LineFollowI);
    if (WanderWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::WanderI);
    if (FormationWeight != 0.0f)
        ActiveFlockFuncs.Add(&UFlockLogic::FormationI);
    ogLineFollowWeight = LineFollowWeight;

    // Default update func
    CurrUpdateFunc = &UFlockLogic::NormalUpdate;

#if UE_BUILD_DEBUG || DEBUG_FLOCK
    bool error = true;
    if (BoidArchetype == NULL)
    {
        GEngine->AddOnScreenDebugMessage(INDEX_NONE, 8.0f, FColor::Red, "FLOCKING ERROR: BoidArchetype not set on: " + GetOwner()->GetArchetype()->GetName());
        error = true;
    }
    if (FormationMesh == NULL && FormationWeight != 0.0f)
    {
        GEngine->AddOnScreenDebugMessage(INDEX_NONE, 8.0f, FColor::Red, "FLOCKING ERROR: FormationMesh not set on: " + GetOwner()->GetArchetype()->GetName() + ". To set the FormationMesh, please add a UStaticMeshComponent to the blueprint, and give it the \"Flocking\" tag.");
        error = true;
    }
    if (ActiveFlockFuncs.Num() == 0)
    {
        GEngine->AddOnScreenDebugMessage(INDEX_NONE, 8.0f, FColor::Yellow, "FLOCKING WARNING: " + GetOwner()->GetArchetype()->GetName() + " does not have any active flocking functions");
        error = true;
    }
    PrimaryComponentTick.bCanEverTick = error;
#endif
}

void UFlockLogic::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    UpdateCenterOfMass();
    UpdateFlockAlignment();
    ((this->*CurrUpdateFunc))(DeltaTime, TickType, ThisTickFunction);
}

void UFlockLogic::NormalUpdate(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
#if UE_BUILD_DEBUG || DEBUG_FLOCK
    if (DrawSplineOn)
    {
        DrawSpline(Spline, FColor::Orange, 3);
    }
#endif
    GameTimer += DeltaTime;

    // Sum all active flocking functions
    for (int32 i = 0; i < Flock.Num(); ++i)
    {
        // Oscillate line weight
        float scale = FMath::Abs(FMath::Sin(GameTimer * 0.2f + 0.1f * static_cast<float>(i)));
        LineFollowWeight = (ogLineFollowWeight * 0.5f) + (ogLineFollowWeight * scale * 0.5f);

        // Calculate direction 
        FVector dir = FVector(0.0f, 0.0f, 0.0f);
        for (int32 j = 0; j < ActiveFlockFuncs.Num(); ++j)
        {
            dir += (this->*ActiveFlockFuncs[j])(Flock[i]);
        }
#if UE_BUILD_DEBUG || DEBUG_FLOCK
        if (DebugMovement)
        {
            DrawDebugLine(GetWorld(), Flock[i]->GetActorLocation(), Flock[i]->GetActorLocation() + dir.GetSafeNormal() * Acceleration * 10.0f, FColor::Orange);
        }
#endif
        dir = (dir + dir - Flock[i]->GetActorForwardVector()).GetSafeNormal(); // overcorrect for rotation to lean into the turns
        RotateBoid(Flock[i], dir, DeltaTime * RotationSpeed);
        MoveBoid(Flock[i], Flock[i]->GetActorForwardVector() * DeltaTime * Acceleration);
    }
}

// Diving temporarily overrides other flocking behaviors
void UFlockLogic::DiveUpdate(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    if (DiveTimer > 0.0f)
    {
        for (int32 i = 0; i < Flock.Num(); ++i)
        {
            FVector dir = DiveTarget - Flock[i]->GetActorLocation();
            if (dir.SizeSquared() > 10)
            {
                MoveBoid(Flock[i], dir.GetSafeNormal() * DeltaTime * DiveAcceleration);
                RotateBoid(Flock[i], dir, DeltaTime * DiveRotationSpeed);
            }
        }
        DiveTimer -= DeltaTime;
    }
    else
    {
        CurrUpdateFunc = &UFlockLogic::NormalUpdate;
    }
}

// ------------------------------------------------- Flocking Functions -------------------------------------------------- //
FVector UFlockLogic::AlignmentI(AActor* target)
{
    return FlockAlignment * AlignmentWeight;
}

FVector UFlockLogic::CohesionI(AActor* target)
{
    // Go towards the flock's center of mass
    return (CenterOfMass - target->GetActorLocation()).GetSafeNormal() * CohesionWeight;
}

FVector UFlockLogic::CurrentDirectionI(AActor* target)
{
    return target->GetActorForwardVector().GetSafeNormal() * CurrentDirWeight;
}

FVector UFlockLogic::SeparationI(AActor* target)
{
    AActor* closest = NULL;
    float minDist = FLT_MAX;
    float dist = 0.f;

    // Move away from closest boid
    int count = Flock.Num();
    for (int i = 0; i < count; ++i)
    {
        if (Flock[i] == target)
            continue;

        dist = (Flock[i]->GetActorLocation() - target->GetActorLocation()).SizeSquared();
        if (dist < minDist)
        {
            minDist = dist;
            closest = Flock[i];
        }
    }

    return (target->GetActorLocation() - closest->GetActorLocation()).GetSafeNormal() * SeparationWeight;
}

FVector UFlockLogic::LineFollowI(AActor* target)
{
    // get vector towards closest point on line
    FVector myPos = target->GetActorLocation();
    FVector closestPoint = Spline->FindLocationClosestToWorldLocation(myPos + target->GetActorForwardVector() * 50.0f, ESplineCoordinateSpace::World);
    FVector towardsClosest = (closestPoint - myPos).GetSafeNormal();

    // get direction of spline at closest point on line
    FVector dirAtClosest = Spline->FindDirectionClosestToWorldLocation(myPos, ESplineCoordinateSpace::World).GetSafeNormal();

    // combine vectors based on proximity to line
    float dist = (myPos - closestPoint).Size();
    float splineWeight = LineDirWeight != nullptr ? LineDirWeight->GetFloatValue(dist) : 0.5f;
    FVector avg = dirAtClosest * splineWeight + towardsClosest * (1.0f - splineWeight);

#if UE_BUILD_DEBUG || DEBUG_FLOCK
    if (DebugLineFollow)
    {
        DrawDebugLine(GetWorld(), myPos, myPos + avg * 50.f, FColor::Orange, false, -1.0f, -1);
    }
#endif
    return avg * LineFollowWeight;
}

FVector UFlockLogic::WanderI(AActor* target)
{
    float x, y, z;
    FMath::SinCos(&x, &y, WanderTimer);
    z = (x + y) * 0.5f;
    WanderTimer += GetWorld()->GetDeltaSeconds() * WanderJitterSpeed;
    FVector sphereVec = FVector(x, y, z).GetSafeNormal();
    FVector wanderVec = (target->GetActorForwardVector() * WanderDist + sphereVec * WanderRadius).GetSafeNormal();

#if UE_BUILD_DEBUG || DEBUG_FLOCK
    if (DebugWander)
    {
        DrawDebugLine(GetWorld(), target->GetActorLocation(), target->GetActorLocation() + wanderVec, FColor::Orange);
    }
#endif

    return wanderVec * WanderWeight;
}

FVector UFlockLogic::FormationI(AActor* target)
{
    FVector formationVec = FVector(0.0f, 0.0f, 0.0f);
    return formationVec * FormationWeight;
}

// ------------------------------------------------- Static Flocking Functions -------------------------------------------------- //
FVector UFlockLogic::LineFollowS(AActor* target, USplineComponent* spline)
{
    // get vector towards closest point on line
    FVector myPos = target->GetActorLocation();
    FVector closestPoint = spline->FindLocationClosestToWorldLocation(myPos, ESplineCoordinateSpace::World);
    FVector towardsClosest = (closestPoint - myPos).GetSafeNormal();

    // get direction of spline at closest point on line
    FVector dirAtClosest = spline->FindDirectionClosestToWorldLocation(myPos, ESplineCoordinateSpace::World).GetSafeNormal();

    // combine vectors based on proximity to line
    float dist = (myPos - closestPoint).Size();
    float splineWeight = 0.5f;
    FVector avg = dirAtClosest * splineWeight + towardsClosest * (1.0f - splineWeight);

    return avg;
}

// ------------------------------------------------- Non-Flocking Flock Functions -------------------------------------------------- //
void UFlockLogic::DiveToPoint(FVector point)
{
    DiveTarget = point;
    DiveTimer = DiveTime;
    CurrUpdateFunc = &UFlockLogic::DiveUpdate;
}

// ------------------------------------------------- (Some) Debug Drawers -------------------------------------------------- //
void UFlockLogic::DrawSpline(const USplineComponent* spline, FColor color, int32 pointsPerSegment)
{
    if (spline != nullptr)
    {
        int32 numPoints = spline->GetNumberOfSplinePoints();
        FVector prevPoint = spline->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);
        FVector currPoint;
        for (int32 i = 1; i < numPoints; ++i)
        {
            float prevPointDist = spline->GetDistanceAlongSplineAtSplinePoint(i - 1);
            float currPointDist = spline->GetDistanceAlongSplineAtSplinePoint(i);
            for (int32 j = 0; j <= pointsPerSegment; ++j)
            {
                float currDist = prevPointDist + ((currPointDist - prevPointDist) / (float)pointsPerSegment * j);
                currPoint = spline->GetLocationAtDistanceAlongSpline(currDist, ESplineCoordinateSpace::World);
                DrawDebugLine(spline->GetOwner()->GetWorld(), prevPoint, currPoint, color);
                prevPoint = currPoint;
            }
        }
        if (spline->IsClosedLoop())
        {
            currPoint = spline->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);
            DrawDebugLine(spline->GetOwner()->GetWorld(), prevPoint, currPoint, color);
        }
    }
}

// ------------------------------------------------- Helper Functions -------------------------------------------------- //
AActor* UFlockLogic::SpawnBoid(FVector pos, FRotator rot)
{
    UWorld* const world = GetWorld();
    if (world)
    {
        FActorSpawnParameters spawnParams;
        spawnParams.Owner = GetOwner();
        spawnParams.Instigator = GetOwner()->Instigator;
        spawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        AActor* boid = world->SpawnActor<AActor>(BoidArchetype, pos, rot, spawnParams);
        return boid;
    }

    return NULL;
}

void UFlockLogic::MoveBoid(AActor* boid, const FVector& moveVec)
{
    switch (FlockingMovementType)
    {
    case EFlockingMovementType::FM_DirectlySet:
        boid->SetActorLocation(boid->GetActorLocation() + moveVec);
        break;

    case EFlockingMovementType::FM_Accelerate:
        UPrimitiveComponent* root = ((UPrimitiveComponent*)boid->GetRootComponent());
        root->SetPhysicsLinearVelocity(root->ComponentVelocity + moveVec);
        break;
    }

    // Speed cap
    if (MaxSpeed != -1.f)
    {
        UPrimitiveComponent* root = ((UPrimitiveComponent*)boid->GetRootComponent());
        FVector vel = root->ComponentVelocity;
        if (vel.SizeSquared() > MaxSpeed * MaxSpeed)
            root->SetPhysicsLinearVelocity(vel.GetSafeNormal() * MaxSpeed);
    }
}

void UFlockLogic::RotateBoid(AActor* boid, const FVector& dir, float rotSpeed)
{
    FQuat rot = FQuat::FastLerp(boid->GetActorQuat(), dir.ToOrientationQuat(), rotSpeed); // TODO: math out actual rotation speed
    boid->SetActorRotation(rot);
}

void UFlockLogic::UpdateCenterOfMass()
{
    CenterOfMass = FVector::ZeroVector;
    int count = Flock.Num();
    for (int i = 0; i < count; ++i)
        CenterOfMass += Flock[i]->GetActorLocation();
    CenterOfMass /= static_cast<float>(count);
}

void UFlockLogic::UpdateFlockAlignment()
{
    FlockAlignment = FVector::ZeroVector;
    int count = Flock.Num();
    for (int i = 0; i < count; ++i)
        FlockAlignment = FlockAlignment + Flock[i]->GetActorForwardVector();
    FlockAlignment = FlockAlignment / static_cast<float>(count);
}

// ------------------------------------------------- Getters -------------------------------------------------- //
FVector UFlockLogic::GetCenterOfMass()
{
    return CenterOfMass;
}

// ------------------------------------------------- Setters -------------------------------------------------- //
void UFlockLogic::SetPathTo(FVector des)
{
    // ...
}
