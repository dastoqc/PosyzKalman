/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman.h"

Kalman::Kalman(int i) {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_value = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    value = 0.0f; // Reset the value
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting value (use setvalue), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
	P[0][2] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
	P[1][2] = 0.0f;
	P[2][0] = 0.0f;
	P[2][1] = 0.0f;
	P[2][2] = 0.0f;

	isPosition = (bool)i;
};

// The value should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getvalue(float newvalue, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	if (!isPosition) {
		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rate = newRate - bias;
		value += dt * rate;

	
		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_value);
		P[0][1] -= dt * P[1][1];
		P[0][2] -= dt * dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;
		P[1][2] -= dt * P[1][1];
		P[2][0] -= dt * dt * P[1][1];
		P[2][1] -= dt * P[1][1];
		P[2][2] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S = P[0][0] + R_measure; // Estimate error
		/* Step 5 */
		float K[3]; // Kalman gain - This is a 3x1 vector
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;	//vel.
		K[2] = P[2][0] / S;	//accel.

		// Calculate value and bias - Update estimate with measurement zk (newvalue)
		/* Step 3 */
		float y = newvalue - value; // value difference
		/* Step 6 */
		value += K[0] * y;
		bias += K[1] * y;	//from vel.

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp = P[0][0];
		float P01_temp = P[0][1];	//vel.
		float P02_temp = P[0][2];	//accel.

		P[0][0] -= K[0] * P00_temp;
		P[0][1] -= K[0] * P01_temp;
		P[0][2] -= K[0] * P02_temp;
		P[1][0] -= K[1] * P00_temp;
		P[1][1] -= K[1] * P01_temp;
		P[1][2] -= K[1] * P02_temp;
		P[2][0] -= K[2] * P00_temp;
		P[2][1] -= K[2] * P01_temp;
		P[2][2] -= K[2] * P02_temp;
	}
	else {
		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rate = newRate - bias;
		value += dt * dt * rate;


		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_value);
		P[0][1] -= dt * P[1][1];
		P[0][2] -= dt * dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;
		P[1][2] -= dt * P[1][1];
		P[2][0] -= dt * dt * P[1][1];
		P[2][1] -= dt * P[1][1];
		P[2][2] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S = P[0][0] + R_measure; // Estimate error
									   /* Step 5 */
		float K[3]; // Kalman gain - This is a 3x1 vector
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;	//vel.
		K[2] = P[2][0] / S;	//accel.

		// Calculate value and bias - Update estimate with measurement zk (newvalue)
		/* Step 3 */
		float y = newvalue - value; // value difference
									/* Step 6 */
		value += K[0] * y;
		bias += K[2] * y;	//from accel.

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp = P[0][0];
		float P01_temp = P[0][1];	//vel.
		float P02_temp = P[0][2];	//accel.

		P[0][0] -= K[0] * P00_temp;
		P[0][1] -= K[0] * P01_temp;
		P[0][2] -= K[0] * P02_temp;
		P[1][0] -= K[1] * P00_temp;
		P[1][1] -= K[1] * P01_temp;
		P[1][2] -= K[1] * P02_temp;
		P[2][0] -= K[2] * P00_temp;
		P[2][1] -= K[2] * P01_temp;
		P[2][2] -= K[2] * P02_temp;
	}

    return value;
};

void Kalman::setvalue(float value) { this->value = value; }; // Used to set value, this should be set as the starting value
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQvalue(float Q_value) { this->Q_value = Q_value; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQvalue() { return this->Q_value; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };
