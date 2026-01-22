#ifndef ROBOT_FSM_H
#define ROBOT_FSM_H

#include <Arduino.h>

// starile robotului
typedef enum {
    FSM_FOLLOW_LINE = 0,
    FSM_SEARCH_LEFT,
    FSM_SEARCH_RIGHT,
    FSM_STOP
} RobotState;


// 300 ms pana la oprire reala
#define LINE_LOST_TIMEOUT_MS 800

typedef struct {
    RobotState currentState;
    float lastSteering;
    uint32_t lastSeenTime;
} RobotFSM;


// Initializare FSM
static inline void FSM_Init(RobotFSM *fsm)
{
    fsm->currentState = FSM_FOLLOW_LINE;
    fsm->lastSteering = 0.0f;
    fsm->lastSeenTime = millis();
}


// Cand linia este pierduta (1 frame)
static inline void FSM_OnLineLost(RobotFSM *fsm)
{
    if (fsm->lastSteering < 0)
        fsm->currentState = FSM_SEARCH_LEFT;
    else
        fsm->currentState = FSM_SEARCH_RIGHT;
}


// 300ms fara linie: STOP
static inline bool FSM_ShouldStop(RobotFSM *fsm)
{
    return (millis() - fsm->lastSeenTime) > LINE_LOST_TIMEOUT_MS;
}

#endif
