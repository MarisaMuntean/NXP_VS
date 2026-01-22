#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "geometry2D.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculează unghiul de viraj (steering angle) conform algoritmului Pure Pursuit.
 *
 * @param robot_pos         Poziția actuală a robotului (x, y) în metri.
 * @param robot_heading_rad Unghiul actual al robotului (radiani, 0 = orientat pe axa X pozitivă).
 * @param lookahead_point   Punctul de urmărire (lookahead) pe traseu.
 * @return float            Unghiul de direcție (radiani, negativ = stânga, pozitiv = dreapta).
 */
float purePursuit_computeSteering(Point2D robot_pos, float robot_heading_rad, Point2D lookahead_point);

typedef struct PurePursuitInfo {
    Point2D rearAxePosition;
    Point2D nextWayPoint;

    float steeringAngle;        // rad
    float lookAheadDistance;    // m
    float distanceToWayPoint;   // m
    float rearWheelTurnRadius;  // m
} PurePursuitInfo;


PurePursuitInfo purePursuitComputeABC(
    Point2D rearAxePosition,
    LineABC wayLine,
    float wheelBase,
    float lookAheadDist
);

#ifdef __cplusplus
}
#endif

#endif // PURE_PURSUIT_H
