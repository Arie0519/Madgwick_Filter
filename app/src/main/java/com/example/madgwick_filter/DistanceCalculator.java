package com.example.madgwick_filter;

public class DistanceCalculator {
    private static final float ACCELERATION_THRESHOLD = 0.05f; // m/s^2
    private static final float VELOCITY_THRESHOLD = 0.01f; // m/s
    private static final float ALPHA = 0.8f; // ローパスフィルタの係数

    private float[] lastMotionData = new float[7];
    private float[] lastAccel = new float[3];
    private float[] velocity = new float[3];
    private float[] position = new float[3];
    private float totalDistance = 0;
    private long lastUpdateTime = 0;

    public float[] calculateMotion(float[] worldAccel, boolean isMoving) {
        long currentTime = System.nanoTime();

        if (lastUpdateTime == 0) {
            lastUpdateTime = currentTime;
            System.arraycopy(worldAccel, 0, lastAccel, 0, 3);
            return new float[]{0, 0, 0, 0, 0, 0, 0};
        }

        float dt = (currentTime - lastUpdateTime) / 1e9f; // ナノ秒を秒に変換
        lastUpdateTime = currentTime;

        float[] newVelocity = new float[3];
        float[] newPosition = new float[3];
        float distanceIncrement = 0;

        if (isMoving) {
            for (int i = 0; i < 3; i++) {
                // ローパスフィルタを適用
                float filteredAccel = ALPHA * lastAccel[i] + (1 - ALPHA) * worldAccel[i];

                // ノイズ除去のための閾値処理
                float acceleration = Math.abs(filteredAccel) > ACCELERATION_THRESHOLD ? filteredAccel : 0;

                // 台形法による速度の積分
                newVelocity[i] = velocity[i] + (acceleration + lastAccel[i]) / 2 * dt;

                // 速度の閾値処理
                newVelocity[i] = Math.abs(newVelocity[i]) > VELOCITY_THRESHOLD ? newVelocity[i] : 0;

                // 速度による位置の計算
                newPosition[i] = position[i] + (velocity[i] + newVelocity[i]) / 2 * dt;

                // 距離の増分を計算
                distanceIncrement += Math.abs(newPosition[i] - position[i]);

                // 更新された加速度を保存
                lastAccel[i] = filteredAccel;
            }
        } else {
            // 静止中は速度をゼロにリセット
            newVelocity = new float[]{0, 0, 0};
            newPosition = position.clone();
            System.arraycopy(worldAccel, 0, lastAccel, 0, 3);
        }

        // 状態を更新
        velocity = newVelocity;
        position = newPosition;
        totalDistance += distanceIncrement;

        // 結果を返す: [vx, vy, vz, px, py, pz, totalDistance]
        lastMotionData[0] = velocity[0];
        lastMotionData[1] = velocity[1];
        lastMotionData[2] = velocity[2];
        lastMotionData[3] = position[0];
        lastMotionData[4] = position[1];
        lastMotionData[5] = position[2];
        lastMotionData[6] = totalDistance;

        return lastMotionData;
    }

    public float[] getLastMotionData() {
        return lastMotionData;
    }

    public void reset() {
        lastAccel = new float[3];
        velocity = new float[3];
        position = new float[3];
        totalDistance = 0;
        lastUpdateTime = 0;
        lastMotionData = new float[7];
    }
}