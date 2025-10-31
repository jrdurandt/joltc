#include <iostream>
#include "joltc.h"

static void TraceImpl(const char *message) {
    // Print to the TTY
    std::cout << message << std::endl;
}

namespace Layers {
    static constexpr JPH_ObjectLayer NON_MOVING = 0;
    static constexpr JPH_ObjectLayer MOVING = 1;
    static constexpr JPH_ObjectLayer NUM_LAYERS = 2;
};

namespace BroadPhaseLayers {
    static constexpr JPH_BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH_BroadPhaseLayer MOVING(1);
    static constexpr uint32_t NUM_LAYERS(2);
}

void tireMaxImpulseCallback(uint32_t index, float& longI, float& latI, float susI, float longF, float latF, float longS, float latS, float dt) {
    std::cout << "Called:" << index << std::endl;
    longI = 10.0f * longF * susI;
    latI = latF * susI;
}

int main(void) {
    if (!JPH_Init())
        return 1;

    JPH_SetTraceHandler(TraceImpl);
    //JPH_SetAssertFailureHandler(JPH_AssertFailureFunc handler);

    JPH_JobSystem *jobSystem = JPH_JobSystemThreadPool_Create(nullptr);

    // We use only 2 layers: one for non-moving objects and one for moving objects
    JPH_ObjectLayerPairFilter *objectLayerPairFilterTable = JPH_ObjectLayerPairFilterTable_Create(2);
    JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers::NON_MOVING, Layers::MOVING);
    JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers::MOVING, Layers::NON_MOVING);

    // We use a 1-to-1 mapping between object layers and broadphase layers
    JPH_BroadPhaseLayerInterface *broadPhaseLayerInterfaceTable = JPH_BroadPhaseLayerInterfaceTable_Create(2, 2);
    JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers::NON_MOVING,
                                                                 BroadPhaseLayers::NON_MOVING);
    JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers::MOVING,
                                                                 BroadPhaseLayers::MOVING);

    JPH_ObjectVsBroadPhaseLayerFilter *objectVsBroadPhaseLayerFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
        broadPhaseLayerInterfaceTable, 2, objectLayerPairFilterTable, 2);

    JPH_PhysicsSystemSettings settings = {};
    settings.maxBodies = 65536;
    settings.numBodyMutexes = 0;
    settings.maxBodyPairs = 65536;
    settings.maxContactConstraints = 65536;
    settings.broadPhaseLayerInterface = broadPhaseLayerInterfaceTable;
    settings.objectLayerPairFilter = objectLayerPairFilterTable;
    settings.objectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter;
    JPH_PhysicsSystem *system = JPH_PhysicsSystem_Create(&settings);
    JPH_BodyInterface *bodyInterface = JPH_PhysicsSystem_GetBodyInterface(system);

    JPH_BodyID floorId = {}; {
        // Next we can create a rigid body to serve as the floor, we make a large box
        // Create the settings for the collision volume (the shape).
        // Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
        JPH_Vec3 boxHalfExtents = {100.0f, 1.0f, 100.0f};
        JPH_BoxShape *floorShape = JPH_BoxShape_Create(&boxHalfExtents, JPH_DEFAULT_CONVEX_RADIUS);

        JPH_Vec3 floorPosition = {0.0f, -1.0f, 0.0f};
        JPH_BodyCreationSettings *floorSettings = JPH_BodyCreationSettings_Create3(
            (const JPH_Shape *) floorShape,
            &floorPosition,
            nullptr, // Identity,
            JPH_MotionType_Static,
            Layers::NON_MOVING);

        // Create the actual rigid body
        floorId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, floorSettings, JPH_Activation_DontActivate);
        JPH_BodyCreationSettings_Destroy(floorSettings);
    }

    //
    const float wheel_radius = 0.3f;
    const float wheel_width = 0.1f;
    const float half_vehicle_length = 2.0f;
    const float half_vehicle_width = 0.9f;
    const float half_vehicle_height = 0.2f;
    const JPH_Vec3 half_vehicle_size = {half_vehicle_width, half_vehicle_height, half_vehicle_length};

    const JPH_Vec3 up = {0.0f, 1.0f, 0.0f};

    JPH_VehicleCollisionTester *mTesters[3]{};
    // Create collision testers
    mTesters[0] = reinterpret_cast<JPH_VehicleCollisionTester *>(JPH_VehicleCollisionTesterRay_Create(
        Layers::MOVING, &up, 1));
    mTesters[1] = reinterpret_cast<JPH_VehicleCollisionTester *>(JPH_VehicleCollisionTesterCastSphere_Create(
        Layers::MOVING, 0.5f * wheel_width, &up, 1));
    mTesters[2] = reinterpret_cast<JPH_VehicleCollisionTester *>(JPH_VehicleCollisionTesterCastCylinder_Create(
        Layers::MOVING, JPH_DEFAULT_CONVEX_RADIUS));

    // Create vehicle body
    JPH_RVec3 position{0, 2, 0};
    JPH_Vec3 angles = {0, 0, 0};
    JPH_Quat rotation;
    JPH_Quat_FromEulerAngles(&angles, &rotation);
    auto body_box_shape = JPH_BoxShape_Create(&half_vehicle_size, JPH_DEFAULT_CONVEX_RADIUS);
    JPH_Vec3 offset{0, -half_vehicle_height, 0};
    auto car_shape = JPH_OffsetCenterOfMassShape_Create(&offset, reinterpret_cast<JPH_Shape *>(body_box_shape));

    auto car_body_settings = JPH_BodyCreationSettings_Create3(
        reinterpret_cast<const JPH_Shape *>(car_shape),
        &position,
        &rotation,
        JPH_MotionType_Dynamic,
        Layers::MOVING
    );

    JPH_BodyCreationSettings_SetOverrideMassProperties(
        car_body_settings,
        JPH_OverrideMassProperties_CalculateInertia
    );

    auto mass_property_override = JPH_MassProperties{
        .mass = 1500
    };

    JPH_BodyCreationSettings_SetMassPropertiesOverride(
        car_body_settings,
        &mass_property_override
    );

    auto car_body = JPH_BodyInterface_CreateBody(bodyInterface, car_body_settings);
    auto car_body_id = JPH_Body_GetID(car_body);
    JPH_BodyInterface_AddBody(bodyInterface, car_body_id, JPH_Activation_Activate);

    // Create vehicle constraint
    JPH_VehicleConstraintSettings vehicle{};
    JPH_VehicleConstraintSettings_Init(&vehicle);
    vehicle.base.drawConstraintSize = 0.1f;
    vehicle.maxPitchRollAngle = JPH_M_PI;

    // Suspension direction
    JPH_Vec3 suspension_dir = {0, -1, 0};
    JPH_Vec3 steering_axis = {0, 1, 0};
    JPH_Vec3 wheel_up = {0, 1, 0};
    JPH_Vec3 wheel_forward = {0, 0, 1};

    JPH_SpringSettings spring_settings = {
        .mode = JPH_SpringMode_FrequencyAndDamping,
        .frequencyOrStiffness = 1.5f,
        .damping = 0.5
    };

    JPH_WheelSettingsWV *w1 = JPH_WheelSettingsWV_Create();
    JPH_Vec3 front_left_wheel_pos = {
        half_vehicle_width, -0.9f * half_vehicle_height, half_vehicle_length - 2.0f * wheel_radius
    };
    JPH_WheelSettings_SetPosition(reinterpret_cast<JPH_WheelSettings *>(w1), &front_left_wheel_pos);
    JPH_WheelSettings_SetSuspensionDirection(reinterpret_cast<JPH_WheelSettings *>(w1), &suspension_dir);
    JPH_WheelSettings_SetSteeringAxis(reinterpret_cast<JPH_WheelSettings *>(w1), &steering_axis);
    JPH_WheelSettings_SetWheelUp(reinterpret_cast<JPH_WheelSettings *>(w1), &wheel_up);
    JPH_WheelSettings_SetWheelForward(reinterpret_cast<JPH_WheelSettings *>(w1), &wheel_forward);
    JPH_WheelSettings_SetSuspensionMinLength(reinterpret_cast<JPH_WheelSettings *>(w1), 0.3f);
    JPH_WheelSettings_SetSuspensionMaxLength(reinterpret_cast<JPH_WheelSettings *>(w1), 0.5f);
    JPH_WheelSettings_SetSuspensionSpring(reinterpret_cast<JPH_WheelSettings *>(w1), &spring_settings);
    JPH_WheelSettingsWV_SetMaxSteerAngle(w1,  1);
    JPH_WheelSettingsWV_SetMaxHandBrakeTorque(w1, 0.0f);

    JPH_WheelSettingsWV *w2 = JPH_WheelSettingsWV_Create();
    JPH_Vec3 front_right_wheel_pos = {
        -half_vehicle_width, -0.9f * half_vehicle_height, half_vehicle_length - 2.0f * wheel_radius
    };
    JPH_WheelSettings_SetPosition(reinterpret_cast<JPH_WheelSettings *>(w2), &front_right_wheel_pos);
    JPH_WheelSettings_SetSuspensionDirection(reinterpret_cast<JPH_WheelSettings *>(w2), &suspension_dir);
    JPH_WheelSettings_SetSteeringAxis(reinterpret_cast<JPH_WheelSettings *>(w2), &steering_axis);
    JPH_WheelSettings_SetWheelUp(reinterpret_cast<JPH_WheelSettings *>(w2), &wheel_up);
    JPH_WheelSettings_SetWheelForward(reinterpret_cast<JPH_WheelSettings *>(w2), &wheel_forward);
    JPH_WheelSettings_SetSuspensionMinLength(reinterpret_cast<JPH_WheelSettings *>(w2), 0.3f);
    JPH_WheelSettings_SetSuspensionMaxLength(reinterpret_cast<JPH_WheelSettings *>(w2), 0.5f);
    JPH_WheelSettings_SetSuspensionSpring(reinterpret_cast<JPH_WheelSettings *>(w2), &spring_settings);
    JPH_WheelSettingsWV_SetMaxSteerAngle(w2, 1);
    JPH_WheelSettingsWV_SetMaxHandBrakeTorque(w2, 0.0f);

    JPH_WheelSettingsWV *w3 = JPH_WheelSettingsWV_Create();
    JPH_Vec3 rear_left_wheel_pos = {
        half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 2.0f * wheel_radius
    };
    JPH_WheelSettings_SetPosition(reinterpret_cast<JPH_WheelSettings *>(w3), &rear_left_wheel_pos);
    JPH_WheelSettings_SetSuspensionDirection(reinterpret_cast<JPH_WheelSettings *>(w3), &suspension_dir);
    JPH_WheelSettings_SetSteeringAxis(reinterpret_cast<JPH_WheelSettings *>(w3), &steering_axis);
    JPH_WheelSettings_SetWheelUp(reinterpret_cast<JPH_WheelSettings *>(w3), &wheel_up);
    JPH_WheelSettings_SetWheelForward(reinterpret_cast<JPH_WheelSettings *>(w3), &wheel_forward);
    JPH_WheelSettings_SetSuspensionMinLength(reinterpret_cast<JPH_WheelSettings *>(w3), 0.3f);
    JPH_WheelSettings_SetSuspensionMaxLength(reinterpret_cast<JPH_WheelSettings *>(w3), 0.5f);
    JPH_WheelSettings_SetSuspensionSpring(reinterpret_cast<JPH_WheelSettings *>(w3), &spring_settings);
    JPH_WheelSettingsWV_SetMaxSteerAngle(w3, 0);
    JPH_WheelSettingsWV_SetMaxHandBrakeTorque(w3, 5000.0f);

    JPH_WheelSettingsWV *w4 = JPH_WheelSettingsWV_Create();
    JPH_Vec3 rear_right_wheel_pos = {
        half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 2.0f * wheel_radius
    };
    JPH_WheelSettings_SetPosition(reinterpret_cast<JPH_WheelSettings *>(w4), &rear_right_wheel_pos);
    JPH_WheelSettings_SetSuspensionDirection(reinterpret_cast<JPH_WheelSettings *>(w4), &suspension_dir);
    JPH_WheelSettings_SetSteeringAxis(reinterpret_cast<JPH_WheelSettings *>(w4), &steering_axis);
    JPH_WheelSettings_SetWheelUp(reinterpret_cast<JPH_WheelSettings *>(w4), &wheel_up);
    JPH_WheelSettings_SetWheelForward(reinterpret_cast<JPH_WheelSettings *>(w4), &wheel_forward);
    JPH_WheelSettings_SetSuspensionMinLength(reinterpret_cast<JPH_WheelSettings *>(w4), 0.3f);
    JPH_WheelSettings_SetSuspensionMaxLength(reinterpret_cast<JPH_WheelSettings *>(w4), 0.5f);
    JPH_WheelSettings_SetSuspensionSpring(reinterpret_cast<JPH_WheelSettings *>(w4), &spring_settings);
    JPH_WheelSettingsWV_SetMaxSteerAngle(w4, 0);
    JPH_WheelSettingsWV_SetMaxHandBrakeTorque(w4, 5000.0f);

    JPH_WheelSettings* wheels[4] = {
        reinterpret_cast<JPH_WheelSettings *>(w1),
        reinterpret_cast<JPH_WheelSettings *>(w2),
        reinterpret_cast<JPH_WheelSettings *>(w3),
        reinterpret_cast<JPH_WheelSettings *>(w4)
    };

    vehicle.wheelsCount = 4;
    vehicle.wheels = wheels;

    for (int i = 0; i < vehicle.wheelsCount; i++) {
        auto w = vehicle.wheels[i];
        JPH_WheelSettings_SetWidth(w, wheel_width);
        JPH_WheelSettings_SetRadius(w, wheel_radius);
    }

    auto controller_settings = JPH_WheeledVehicleControllerSettings_Create();
    vehicle.controller = reinterpret_cast<JPH_VehicleControllerSettings *>(controller_settings);

    JPH_VehicleDifferentialSettings diff;
    JPH_VehicleDifferentialSettings_Init(&diff);
    diff.leftWheel = 2;
    diff.rightWheel = 3;

    JPH_WheeledVehicleControllerSettings_SetDifferentialsCount(controller_settings, 1);
    JPH_WheeledVehicleControllerSettings_SetDifferential(controller_settings, 0, &diff);

    auto engine = JPH_VehicleEngineSettings_Create();
    JPH_VehicleEngineSettings_SetMaxTorque(engine, 500);
    JPH_VehicleEngineSettings_SetMinRPM(engine, 1000);
    JPH_VehicleEngineSettings_SetMaxRPM(engine, 6000);
    JPH_WheeledVehicleControllerSettings_SetEngine(controller_settings, engine);

    auto trans = JPH_VehicleTransmissionSettings_Create();
    JPH_VehicleTransmissionSettings_SetClutchStrength(trans, 10.0f);
    JPH_VehicleTransmissionSettings_SetMode(trans, JPH_TransmissionMode_Auto);
    JPH_WheeledVehicleControllerSettings_SetTransmission(controller_settings, trans);

    // // Anti rollbars
    // if (sAntiRollbar) {
    //     vehicle.mAntiRollBars.resize(2);
    //     vehicle.mAntiRollBars[0].mLeftWheel = 0;
    //     vehicle.mAntiRollBars[0].mRightWheel = 1;
    //     vehicle.mAntiRollBars[1].mLeftWheel = 2;
    //     vehicle.mAntiRollBars[1].mRightWheel = 3;
    // }

    // mVehicleConstraint = new VehicleConstraint(*mCarBody, vehicle);

    // The vehicle settings were tweaked with a buggy implementation of the longitudinal tire impulses, this meant that PhysicsSettings::mNumVelocitySteps times more impulse
    // could be applied than intended. To keep the behavior of the vehicle the same we increase the max longitudinal impulse by the same factor. In a future version the vehicle
    // will be retweaked.
    // static_cast<WheeledVehicleController *>(mVehicleConstraint->GetController())->SetTireMaxImpulseCallback(
    //     [](uint, float &outLongitudinalImpulse, float &outLateralImpulse, float inSuspensionImpulse,
    //        float inLongitudinalFriction, float inLateralFriction, float, float, float) {
    //         outLongitudinalImpulse = 10.0f * inLongitudinalFriction * inSuspensionImpulse;
    //         outLateralImpulse = inLateralFriction * inSuspensionImpulse;
    //     });
    auto vehicle_constraint = JPH_VehicleConstraint_Create(car_body, &vehicle);
    JPH_PhysicsSystem_AddConstraint(system, reinterpret_cast<JPH_Constraint *>(vehicle_constraint));

    auto controller = JPH_VehicleConstraint_GetController(vehicle_constraint);
    JPH_WheeledVehicleController_SetTireMaxImpulseCallback(reinterpret_cast<JPH_WheeledVehicleController *>(controller), tireMaxImpulseCallback);

    auto step_listener = JPH_VehicleConstraint_AsPhysicsStepListener(vehicle_constraint);
    JPH_PhysicsSystem_AddStepListener(system, step_listener);



    // Remove and destroy the floor
    JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, floorId);

    JPH_JobSystem_Destroy(jobSystem);

    JPH_PhysicsSystem_Destroy(system);
    JPH_Shutdown();
    return 0;
}
