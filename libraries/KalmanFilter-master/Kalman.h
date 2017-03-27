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

#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {
public:
    Kalman(int isPosition);

    // The value should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getvalue(float newvalue, float newRate, float dt);

    void setvalue(float value); // Used to set value, this should be set as the starting value
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQvalue(float Q_value);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQvalue();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_value; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float value; // The value calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getvalue to update the rate

    float P[3][3]; // Error covariance matrix - This is a 2x2 matrix

	bool isPosition;
};

#endif
