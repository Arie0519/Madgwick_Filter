package com.example.madgwick_filter;

public class MadgwickFilter {
    // フィルタゲイン
    private static final float BETA = 0.1f;
    // サンプリング周波数（Hz）
    private static final float SAMPLE_FREQ = 100.0f;

    private static final float GRAVITY = 9.81f;

    // クォータニオン
    private float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    private float[] accel = new float[3];
    private float[] gyro = new float[3];

    public float[] getAdjustedWorldAcceleration() {
        float[] worldAccel = getWorldAcceleration();
        float[] adjustedWorldAccel = new float[3];

        // 重力ベクトルを計算
        float[] gravityVector = getGravityVector();

        // 重力の影響を除去
        for (int i = 0; i < 3; i++) {
            adjustedWorldAccel[i] = worldAccel[i] - gravityVector[i] * GRAVITY;
        }

        return adjustedWorldAccel;
    }

    public float[] getWorldAcceleration() {
        float[] worldAccel = new float[3];

        // センサー座標系から世界座標系への変換
        worldAccel[0] = 2 * (q1*q3 - q0*q2) * accel[0] + 2 * (q0*q1 + q2*q3) * accel[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3) * accel[2];
        worldAccel[1] = 2 * (q2*q3 + q0*q1) * accel[0] + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * accel[1] + 2 * (q1*q3 - q0*q2) * accel[2];
        worldAccel[2] = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * accel[0] + 2 * (q1*q2 - q0*q3) * accel[1] + 2 * (q0*q1 + q2*q3) * accel[2];

        return worldAccel;
    }

    public float[] getGravityVector() {
        float[] gravityVector = new float[3];
        gravityVector[0] = 2 * (q1 * q3 - q0 * q2);
        gravityVector[1] = 2 * (q0 * q1 + q2 * q3);
        gravityVector[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        return gravityVector;
    }

    // 加速度データの更新
    public void updateAccel(float[] accelData) {
        System.arraycopy(accelData, 0, accel, 0, 3);
    }

    // ジャイロデータの更新
    public void updateGyro(float[] gyroData) {
        System.arraycopy(gyroData, 0, gyro, 0, 3);
    }

    // フィルタの更新
    public void update() {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // ジャイロスコープの度/秒をラジアン/秒に変換
        float gx = gyro[0] * 0.0174533f;
        float gy = gyro[1] * 0.0174533f;
        float gz = gyro[2] * 0.0174533f;
        float ax = accel[0];
        float ay = accel[1];
        float az = accel[2];

        // ジャイロスコープからのクォータニオンの変化率
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // 加速度計の測定値が有効な場合のみフィードバックを計算（加速度計の正規化でNaNを避ける）
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // 加速度計の測定値を正規化
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // 繰り返しの計算を避けるための補助変数
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

            // 勾配降下法による修正ステップ
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            // ステップの大きさを正規化
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // フィードバックステップの適用
            qDot1 -= BETA * s0;
            qDot2 -= BETA * s1;
            qDot3 -= BETA * s2;
            qDot4 -= BETA * s3;
        }

        // クォータニオンの変化率を積分してクォータニオンを得る
        q0 += qDot1 * (1.0f / SAMPLE_FREQ);
        q1 += qDot2 * (1.0f / SAMPLE_FREQ);
        q2 += qDot3 * (1.0f / SAMPLE_FREQ);
        q3 += qDot4 * (1.0f / SAMPLE_FREQ);

        // クォータニオンを正規化
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    // クォータニオンの取得
    public float[] getQuaternion() {
        return new float[]{q0, q1, q2, q3};
    }

    // フィルタのリセット
    public void reset() {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }

    // 高速逆平方根関数
    private float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        int i = Float.floatToIntBits(y);
        i = 0x5f3759df - (i >> 1);
        y = Float.intBitsToFloat(i);
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
}