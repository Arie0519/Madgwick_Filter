package com.example.madgwick_filter;

import java.util.LinkedList;
import java.util.Queue;

public class DistanceCalculator {
    private static final String TAG = "DistanceCalculator";
    private static final int QUEUE_SIZE = 4;

    private Queue<Float> queueAccelX = new LinkedList<>();
    private Queue<Float> queueAccelY = new LinkedList<>();
    private Queue<Float> queueAccelZ = new LinkedList<>();
    private Queue<Long> queueAccTime = new LinkedList<>();

    private Queue<Float> queueVelocityX = new LinkedList<>();
    private Queue<Float> queueVelocityY = new LinkedList<>();
    private Queue<Float> queueVelocityZ = new LinkedList<>();
    private Queue<Long> queueVeloTime = new LinkedList<>();

    private float saveVeloX = 0, saveVeloY = 0, saveVeloZ = 0;
    private float saveDistX = 0, saveDistY = 0, saveDistZ = 0;
    private float totalDistance = 0;
    private long initTime = 0;

    public float[] calculateMotion(float[] worldAccel, boolean isMoving, long timestamp) {
        if (initTime == 0) {
            initTime = timestamp;
        }

        long sampleTime = timestamp - initTime;

        if (!isMoving) {
            // 静止状態では速度をリセットし、距離計算をスキップ
            saveVeloX = saveVeloY = saveVeloZ = 0;
            clearQueues();
            return new float[]{0, 0, 0, saveDistX, saveDistY, saveDistZ, totalDistance};
        }

        // 加速度データをキューに追加
        queueAccelX.add(worldAccel[0]);
        queueAccelY.add(worldAccel[1]);
        queueAccelZ.add(worldAccel[2]);
        queueAccTime.add(sampleTime);

        if (queueAccelX.size() == QUEUE_SIZE) {
            // 速度の計算（シンプソン則）
            saveVeloX = simpson4point(queueAccelX, queueAccTime);
            saveVeloY = simpson4point(queueAccelY, queueAccTime);
            saveVeloZ = simpson4point(queueAccelZ, queueAccTime);

            // 速度をキューに追加
            queueVelocityX.add(saveVeloX);
            queueVelocityY.add(saveVeloY);
            queueVelocityZ.add(saveVeloZ);
            queueVeloTime.add(sampleTime);

            removeOldData(queueAccelX);
            removeOldData(queueAccelY);
            removeOldData(queueAccelZ);
            removeOldData(queueAccTime);
        }

        if (queueVelocityX.size() == QUEUE_SIZE) {
            // 距離の計算（シンプソン則）
            float distX = simpson4point(queueVelocityX, queueVeloTime);
            float distY = simpson4point(queueVelocityY, queueVeloTime);
            float distZ = simpson4point(queueVelocityZ, queueVeloTime);

            saveDistX += distX;
            saveDistY += distY;
            saveDistZ += distZ;

            // 2D距離の計算 (x-y平面)
            float distanceIncrement = (float) Math.sqrt(distX * distX + distY * distY);
            totalDistance += distanceIncrement;

            removeOldData(queueVelocityX);
            removeOldData(queueVelocityY);
            removeOldData(queueVelocityZ);
            removeOldData(queueVeloTime);
        }

        // 結果を返す: [vx, vy, vz, px, py, pz, totalDistance]
        return new float[]{
                saveVeloX, saveVeloY, saveVeloZ,
                saveDistX, saveDistY, saveDistZ,
                totalDistance
        };
    }

    private float simpson4point(Queue<Float> values, Queue<Long> times) {
        if (values.size() != 4 || times.size() != 4) {
            throw new IllegalArgumentException("Queues must contain exactly 4 points");
        }

        float[] y = new float[4];
        long[] t = new long[4];
        int i = 0;
        for (Float value : values) {
            y[i++] = value;
        }
        i = 0;
        for (Long time : times) {
            t[i++] = time;
        }

        // ナノ秒を秒に変換
        float totalTimeInSeconds = (t[3] - t[0]) / 1e9f;
        float h = totalTimeInSeconds / 3f; // 3等分

        return (h / 3f) * (y[0] + 4*y[1] + 2*y[2] + y[3]);
    }

    private void removeOldData(Queue<?> queue) {
        if (queue.size() > QUEUE_SIZE) {
            queue.remove();
        }
    }

    private void clearQueues() {
        queueAccelX.clear();
        queueAccelY.clear();
        queueAccelZ.clear();
        queueAccTime.clear();
        queueVelocityX.clear();
        queueVelocityY.clear();
        queueVelocityZ.clear();
        queueVeloTime.clear();
    }

    public void reset() {
        clearQueues();
        saveVeloX = saveVeloY = saveVeloZ = 0;
        saveDistX = saveDistY = saveDistZ = 0;
        totalDistance = 0;
        initTime = 0;
    }

    public void resetDistance() {
        saveDistX = saveDistY = saveDistZ = 0;
        totalDistance = 0;
    }
}