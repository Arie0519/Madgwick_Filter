package com.example.madgwick_filter;

public class MadgwickFilter {
    private static final float BETA = 0.04f; // Filter gain
    private static final float GRAVITY = 9.81f;

    private float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternion
    private float[] accel = new float[3];
    private float[] gyro = new float[3];

    public void update(float[] accel, float[] gyro, float sampleFreq) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Convert gyroscope degrees/sec to radians/sec
        float gx = gyro[0] * 0.017453292f;
        float gy = gyro[1] * 0.017453292f;
        float gz = gyro[2] * 0.017453292f;
        float ax = accel[0];
        float ay = accel[1];
        float az = accel[2];

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= BETA * s0;
            qDot2 -= BETA * s1;
            qDot3 -= BETA * s2;
            qDot4 -= BETA * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        // Update instance variables
        this.accel = accel;
        this.gyro = gyro;
    }

    public float[] getQuaternion() {
        return new float[]{q0, q1, q2, q3};
    }

    public float[] getGravityVector() {
        float[] gravityVector = new float[3];
        gravityVector[0] = 2 * (q1 * q3 - q0 * q2);
        gravityVector[1] = 2 * (q0 * q1 + q2 * q3);
        gravityVector[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        return gravityVector;
    }

    public float[] getWorldAcceleration() {
        float[] worldAccel = new float[3];
        float[] gravityVector = getGravityVector();

        // ローカル座標系から世界座標系への変換行列を計算
        float[] R = new float[9];
        R[0] = 1 - 2 * (q2 * q2 + q3 * q3);
        R[1] = 2 * (q1 * q2 - q0 * q3);
        R[2] = 2 * (q1 * q3 + q0 * q2);
        R[3] = 2 * (q1 * q2 + q0 * q3);
        R[4] = 1 - 2 * (q1 * q1 + q3 * q3);
        R[5] = 2 * (q2 * q3 - q0 * q1);
        R[6] = 2 * (q1 * q3 - q0 * q2);
        R[7] = 2 * (q2 * q3 + q0 * q1);
        R[8] = 1 - 2 * (q1 * q1 + q2 * q2);

        // ローカル加速度から重力を除去
        float[] localAccelWithoutGravity = new float[3];
        for (int i = 0; i < 3; i++) {
            localAccelWithoutGravity[i] = accel[i] - gravityVector[i] * GRAVITY;
        }

        // 重力を除去したローカル加速度を世界座標系に変換
        for (int i = 0; i < 3; i++) {
            worldAccel[i] = R[i*3] * localAccelWithoutGravity[0] +
                    R[i*3+1] * localAccelWithoutGravity[1] +
                    R[i*3+2] * localAccelWithoutGravity[2];
        }

        // 小さな値をゼロにする（ノイズ除去）
        float EPSILON = 0.01f; // 閾値、適宜調整が必要
        for (int i = 0; i < 3; i++) {
            if (Math.abs(worldAccel[i]) < EPSILON) {
                worldAccel[i] = 0;
            }
        }

        return worldAccel;
    }

    public void reset() {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        accel = new float[3];
        gyro = new float[3];
    }

    private float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = Float.intBitsToFloat(0x5f3759df - (Float.floatToIntBits(x) >> 1));
        return y * (1.5f - halfx * y * y);
    }
}