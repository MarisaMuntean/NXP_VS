#include "powerTrain.h"
#include "geometry2D.h"
#include <math.h>


#define LEFT_BACK_PIN 3
#define LEFT_FRONT_PIN 5

#define RIGHT_BACK_PIN 6
#define RIGHT_FRONT_PIN 9

float debug_left_motor = 0;
float debug_right_motor = 0;


void writeLeftMotor(float throtter) //acceleratie (-1,1)
{
	// clamp [-1,1]
    if (throtter > 1.0f) throtter = 1.0f;
    if (throtter < -1.0f) throtter = -1.0f;


    int pwm = (int)(fabs(throtter) * 255.0f);

   // if (pwm > 0 && pwm < 40) pwm = 40;

    if (throtter > 0)
    {
        // Mergi înainte cu motorul stânga:

        analogWrite(LEFT_FRONT_PIN, 0);
        analogWrite(LEFT_BACK_PIN, pwm);
    }
    else if (throtter < 0)
    {
        // Mergi înapoi:
        analogWrite(LEFT_FRONT_PIN, pwm);
        analogWrite(LEFT_BACK_PIN, 0);
    }
    else
    {
        // STOP
        analogWrite(LEFT_FRONT_PIN, 0);
        analogWrite(LEFT_BACK_PIN, 0);
    }
}

void writeRightMotor(float throtter) //acceleratie (-1,1)
{
	if (throtter > 1.0f) throtter = 1.0f;
    if (throtter < -1.0f) throtter = -1.0f;

    int pwm = (int)(fabs(throtter) * 255.0f);

   // if (pwm > 0 && pwm < 40) pwm = 40;


    if (throtter > 0)
    {
        // înainte
        analogWrite(RIGHT_FRONT_PIN, pwm);
        analogWrite(RIGHT_BACK_PIN, 0);
    }
    else if (throtter < 0)
    {
        // înapoi
        analogWrite(RIGHT_FRONT_PIN, 0);
        analogWrite(RIGHT_BACK_PIN, pwm);
    }
    else
    {
        // STOP
        analogWrite(RIGHT_FRONT_PIN, 0);
        analogWrite(RIGHT_BACK_PIN, 0);
    }

}

static float RearWheelTurnRadius(float wheelBase, float turnAngle) {
	float angle;
	//float temp_sin = sinf(turnAngle);
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return -1.0f;
	}
	float temp_sin = tanf(turnAngle);
	if (floatCmp(temp_sin, 0.0f) == 0) {
		return 0.0f;
	}

	//angle = (wheelBase / tanf(turnAngle));
	angle = (wheelBase / temp_sin);

	angle = fabsf(angle);
	return angle;
}

float steeringAngle2TurnRadius(float wheelbase_meters, float steeringAngle_rad)
{
    float temp_float = sinf(steeringAngle_rad);
    if (floatCmp(temp_float, 0.0f)==0) {
        return 0.0f;
    }
    float result = wheelbase_meters/temp_float;
    result = fabsf(result);
    return result;
}



// left_right_turn: negative if turning left, positive if turning right
// speed_ms is used as throttle and has a range [-1;1]
	void SetSpeedRequest(float speed_ms, float turn_radius, int left_right_turn){
		float left_wheel_turn_radius;
		float right_wheel_turn_radius;
		float left_wheel_turn_circonference;
		float right_wheel_turn_circonference;
		float car_trun_circonference;
		float left_wheel_speed_request_m;
		float right_wheel_speed_request_m;

		turn_radius = fabs(turn_radius);

		if (floatCmp(turn_radius, 0.0) == 0 || left_right_turn == 0)	// going straight
		{
			left_wheel_speed_request_m = speed_ms;
			right_wheel_speed_request_m = speed_ms;
		}
		else{
			if (left_right_turn < 0)	// left turn
			{
				left_wheel_turn_radius = turn_radius - (TRACKWIDTH_M / 2.0);
				right_wheel_turn_radius = turn_radius + (TRACKWIDTH_M / 2.0);
			}
			else if (left_right_turn > 0)	// right turn
			{
				left_wheel_turn_radius = turn_radius + (TRACKWIDTH_M / 2.0);
				right_wheel_turn_radius = turn_radius - (TRACKWIDTH_M / 2.0);
			}

			left_wheel_turn_circonference = (2.0 * left_wheel_turn_radius) * M_PI;
			right_wheel_turn_circonference = (2.0 * right_wheel_turn_radius) * M_PI;
			car_trun_circonference = (2.0 * turn_radius) * M_PI;
			
			left_wheel_speed_request_m =(left_wheel_turn_circonference / car_trun_circonference) * speed_ms;
			right_wheel_speed_request_m =(right_wheel_turn_circonference / car_trun_circonference) * speed_ms;
		}
		
        // înainte de writeLeftMotor / writeRightMotor:
        float minVel = 0.15f; // 15% din viteza maximă

        if (fabs(left_wheel_speed_request_m) < minVel && left_wheel_speed_request_m != 0.0f)
            left_wheel_speed_request_m = (left_wheel_speed_request_m > 0 ? minVel : -minVel);

        if (fabs(right_wheel_speed_request_m) < minVel && right_wheel_speed_request_m != 0.0f)
            right_wheel_speed_request_m = (right_wheel_speed_request_m > 0 ? minVel : -minVel);


		writeLeftMotor(left_wheel_speed_request_m);
		writeRightMotor(right_wheel_speed_request_m);
	}


// steering_angle_rad: negative if turning left, positive if turning right
float write_powertrain(float throttle, float steering_angle_rad){
    float turn_radius;
    float turn_radius_RL;
    float turn_radius_RR;
    int left_right_turn;

    left_right_turn = 0;
    if (steering_angle_rad < 0.0f)
    {
        left_right_turn = -1;
    }
    else if(steering_angle_rad > 0.0f)
    {
        left_right_turn = 1;
    }
    

    turn_radius = steeringAngle2TurnRadius(WHEELBASE_M, steering_angle_rad);
    SetSpeedRequest(throttle, turn_radius, left_right_turn);
}
