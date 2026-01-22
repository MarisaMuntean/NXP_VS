#include "purePursuit.h"
#include "geometry2D.h"
#include "powerTrain.h"
#include <math.h>

float purePursuit_computeSteering(Point2D robot_pos, float robot_heading_rad, Point2D lookahead_point)
{
    // Vector de la poziția robotului la punctul de urmărire
    Vector2D_components vec_to_target = getVector2D_componentsFromPoints(robot_pos, lookahead_point);

    // Distanța până la punctul de urmărire
    float Ld = sqrtf(vec_to_target.i * vec_to_target.i + vec_to_target.j * vec_to_target.j);
    if (floatCmp(Ld, 0.0f) == 0) {
        return 0.0f; // robotul este chiar pe punct
    }

    // Unghiul dintre direcția robotului și linia spre punct
    float angle_to_point = atan2f(vec_to_target.j, vec_to_target.i);
    float alpha = NormalizePiToNegPi(angle_to_point - robot_heading_rad);

    // Formula Pure Pursuit:
    // δ = arctan(2 * L * sin(α) / Ld)
    float delta = atanf((2.0f * WHEELBASE_M * sinf(alpha)) / Ld);

    // Limităm unghiul (siguranță)
    if (delta > M_PI_4) delta = M_PI_4;
    if (delta < -M_PI_4) delta = -M_PI_4;

    return delta;
}

PurePursuitInfo purePursuitComputeABC(
    Point2D rearAxePosition,
    LineABC wayLine,
    float wheelBase,
    float lookAheadDist)
{
    PurePursuitInfo info = {0};

    
    float dist_to_line = distance2lineABC(rearAxePosition, wayLine);

    // lookahead minim = 125% din distanta pana la linie
    if (floatCmp(dist_to_line, lookAheadDist) >= 0)
        lookAheadDist = dist_to_line * 1.25f;


    
    IntersectionPoints2D_2 inters =
        intersectionLineCircleABC(rearAxePosition, lookAheadDist, wayLine);

    if (inters.numPoints == 0) {
        // No intersection - line is too far
        // Project point onto line as fallback
        Point2D projected = projectPointOnLineABC(rearAxePosition, wayLine);
        info.nextWayPoint = projected;
        info.lookAheadDistance = distance2lineABC(rearAxePosition, wayLine);
    }
    else {
        // CRITICAL FIX: Choose point with HIGHER Y (farther ahead)
        // After coordinate inversion, higher Y = farther from robot
        Point2D nextWayPoint;
        if (floatCmp(inters.point1.y, inters.point2.y) > 0)
            nextWayPoint = inters.point1;  // Higher Y = farther ahead
        else
            nextWayPoint = inters.point2;

        info.nextWayPoint = nextWayPoint;
        info.lookAheadDistance = lookAheadDist;
    }

    // Calculate steering
    float dx = info.nextWayPoint.x - rearAxePosition.x;
    float dy = info.nextWayPoint.y - rearAxePosition.y;

    float distanceToWaypoint = euclidianDistance(rearAxePosition, info.nextWayPoint);
    
    // Avoid division by zero
    if (floatCmp(distanceToWaypoint, 0.0f) == 0) {
        info.steeringAngle = 0.0f;
        info.rearWheelTurnRadius = 0.0f;
        return info;
    }

    // Pure Pursuit formula
    // Assuming robot is always pointing "up" (along +Y axis)
    float angleToWaypoint = atan2f(dy, dx);
    float robotHeading = M_PI_2;  // Pointing up (90 degrees)
    float alpha = angleToWaypoint - robotHeading;
    
    // Normalize alpha to [-π, π]
    alpha = NormalizePiToNegPi(alpha);

    // Steering angle
    float steeringAngle = atanf((2.0f * wheelBase * sinf(alpha)) / distanceToWaypoint);

    info.rearAxePosition = rearAxePosition;
    info.distanceToWayPoint = distanceToWaypoint;
    info.steeringAngle = steeringAngle;

    info.rearWheelTurnRadius =
        (floatCmp(sinf(steeringAngle), 0.0f) == 0)
        ? 0.0f
        : fabsf(wheelBase / sinf(steeringAngle));

    return info;
}

