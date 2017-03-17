/*******************************************************************************/
/*!
\file   Aggro.cs
\author Khan Sweetman
\par    All content © 2015 DigiPen (USA) Corporation, all rights reserved.
\par    Team Arcus
\brief
    Basic logic for flocking.

*/
/*******************************************************************************/

#pragma once

#include "Components/ActorComponent.h"
#include "Components/SplineComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Runtime/Core/Public/Math/RandomStream.h"
#include "Runtime/AIModule/Classes/AIController.h"
#include "FlockLogic.generated.h"

#define DEBUG_FLOCK 1 /* set to 1 to allow debug in release mode */
#define CALL_MEMBER_FN(object, ptrToMember)  ((object).*(ptrToMember))

UENUM(BlueprintType)
enum class EFlockingMovementType : uint8
{
  FM_DirectlySet UMETA(DisplayName= "Directly Set"),
  FM_Accelerate UMETA(DisplayName = "Accelerate")
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VISCERAL_API UFlockLogic : public UActorComponent
{
    GENERATED_BODY()

public:
    typedef FVector(UFlockLogic::*FlockFunc)(AActor*);
    typedef void (UFlockLogic::*UpdateFunc)(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);

    // ------------------------------------------------- Variables -------------------------------------------------- //
    float GameTimer;
    float TimeSinceMassUpdate;
    float WanderTimer;
    float DiveTimer;
    FVector CenterOfMass;
    FVector FlockAlignment;
    FVector DiveTarget;
    FRandomStream Rand;
    AAIController* Controller;
    USplineComponent* Spline;
    UStaticMeshComponent* FormationMesh;
    TArray<FVector> FormationVertices;
    TArray<FlockFunc> ActiveFlockFuncs;
    UpdateFunc CurrUpdateFunc;

    // ------------------------------------------------- Flocking Parameters -------------------------------------------------- //
    UPROPERTY(BlueprintReadWrite, Category = "Flocking")
        TArray<AActor*> Flock;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Starting Parameters")
        TSubclassOf<class AActor> BoidArchetype;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Starting Parameters")
        int32 NumStartingAgents;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Starting Parameters")
        float StartDistMin;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Starting Parameters")
        float StartDistMax;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Movement")
        EFlockingMovementType FlockingMovementType;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Movement")
        float MaxSpeed; // set to -1 for no max speed
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Movement")
        float Acceleration;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Movement")
        float RotationSpeed; // degrees per second
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Movement")
        bool DebugMovement; // degrees per second
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Alignment")
        float AlignmentWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Cohesion")
        float CohesionWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Current Direction")
        float CurrentDirWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Formation")
        float FormationWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Line Follow")
        float LineFollowWeight; float ogLineFollowWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Line Follow")
        UDistributionFloatConstantCurve* LineDirWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Line Follow")
        bool DebugLineFollow;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Line Follow")
        bool DrawSplineOn;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Line Follow")
        bool DrawSplineFieldOn;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Separation")
        float SeparationWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Wander")
        float WanderWeight;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Wander")
        float WanderRadius;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Wander")
        float WanderDist;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Wander")
        float WanderJitterSpeed;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Wander")
        bool DebugWander;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Other")
        float DiveAcceleration;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Other")
        float DiveRotationSpeed;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flocking|Other")
        float DiveTime;

    // ------------------------------------------------- Life Cycle -------------------------------------------------- //
    UFlockLogic();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    void NormalUpdate(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);
    void DiveUpdate(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);

    // ------------------------------------------------- Flocking Functions -------------------------------------------------- //
    // Individual boid flocking functions
    // These functions return weighted, desired direction for individual boids
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector AlignmentI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector CohesionI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector CurrentDirectionI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector FormationI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector LineFollowI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector SeparationI(AActor* target);
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector WanderI(AActor* target);

    // ------------------------------------------------- Static Flocking Functions -------------------------------------------------- //
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        static FVector LineFollowS(AActor* target, USplineComponent* spline);

    // ------------------------------------------------- Non-Flocking Flock Functions -------------------------------------------------- //
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        void DiveToPoint(FVector point);

    // ------------------------------------------------- (Some) Debug Drawers -------------------------------------------------- //
    UFUNCTION(BlueprintCallable, Category = "Debug")
        static void DrawSpline(const USplineComponent* spline, FColor color, int32 pointsPerSegment = 1);

    // ------------------------------------------------- Helpers -------------------------------------------------- //
    AActor* SpawnBoid(FVector pos, FRotator rot = FRotator::ZeroRotator);
    void UpdateCenterOfMass();
    void UpdateFlockAlignment();
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        void MoveBoid(AActor* boid, const FVector& moveVec);
    void RotateBoid(AActor* boid, const FVector& dir, float rotSpeed);

    // ------------------------------------------------- Getters -------------------------------------------------- //
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        FVector GetCenterOfMass();

    // ------------------------------------------------- Setters -------------------------------------------------- //
    UFUNCTION(BlueprintCallable, Category = "Flocking")
        void SetPathTo(FVector des);
};

