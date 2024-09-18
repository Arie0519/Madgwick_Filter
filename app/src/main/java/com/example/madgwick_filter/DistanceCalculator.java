package com.example.madgwick_filter;

import android.util.Log;

public class DistanceCalculator {
    private static final String TAG = "DistanceCalculator";
    private static final float ACCELERATION_THRESHOLD = 0.05f; // m/s^2
    private static final float VELOCITY_THRESHOLD = 0.03f; // m/s
    private static final float VELOCITY_DECAY_FACTOR = 0.9f; // 速度の減衰係数

    private float[] velocity = new float[3];
    private float[] position = new float[3];
    private float totalDistance = 0;
    private long lastUpdateTime = 0;
    private boolean isInitialized = false;

    public float[] calculateMotion(float[] worldAccel, boolean isMoving, long timestamp) {
        if (!isInitialized) {
            lastUpdateTime = timestamp;
            isInitialized = true;
            return new float[]{0, 0, 0, 0, 0, 0, 0};
        }

        float dt = (timestamp - lastUpdateTime) / 1e9f; // ナノ秒を秒に変換
        lastUpdateTime = timestamp;

        float[] newVelocity = new float[3];
        float[] newPosition = new float[3];
        float distanceIncrement = 0;

        if (isMoving) {
            for (int i = 0; i < 2; i++) { // x-y平面のみを考慮
                // ノイズ除去のための閾値処理
                float acceleration = Math.abs(worldAccel[i]) > ACCELERATION_THRESHOLD ? worldAccel[i] : 0;

                // 速度の計算
                newVelocity[i] = velocity[i] + acceleration * dt;

                // 速度の減衰
                newVelocity[i] *= VELOCITY_DECAY_FACTOR;

                // 速度の閾値処理
                newVelocity[i] = Math.abs(newVelocity[i]) > VELOCITY_THRESHOLD ? newVelocity[i] : 0;

                // 位置の計算
                newPosition[i] = position[i] + newVelocity[i] * dt;
            }

            // z軸の処理（高さ方向は距離計算に含めない）
            newVelocity[2] = 0;
            newPosition[2] = position[2];

            // 2次元平面上（x-y平面）での移動距離を計算
            float dx = newPosition[0] - position[0];
            float dy = newPosition[1] - position[1];
            distanceIncrement = (float) Math.sqrt(dx * dx + dy * dy);
        } else {
            // 静止中は速度を徐々に減衰
            for (int i = 0; i < 3; i++) {
                newVelocity[i] = velocity[i] * VELOCITY_DECAY_FACTOR;
                if (Math.abs(newVelocity[i]) < VELOCITY_THRESHOLD) {
                    newVelocity[i] = 0;
                }
            }
            newPosition = position.clone();
        }

        // 状態を更新
        velocity = newVelocity;
        position = newPosition;
        totalDistance += distanceIncrement;

        Log.d(TAG, String.format("2D Distance: %.4f, Velocity: [%.2f, %.2f, %.2f]",
                totalDistance, velocity[0], velocity[1], velocity[2]));

        // 結果を返す: [vx, vy, vz, px, py, pz, totalDistance]
        return new float[]{
                velocity[0], velocity[1], velocity[2],
                position[0], position[1], position[2],
                totalDistance
        };
    }


    public void reset() {
        velocity = new float[3];
        position = new float[3];
        totalDistance = 0;
        lastUpdateTime = 0;
        isInitialized = false;
        Log.d(TAG, "DistanceCalculator reset");
    }

    public void resetDistance() {
        totalDistance = 0;
        Log.d(TAG, "Distance reset to 0");
    }
}