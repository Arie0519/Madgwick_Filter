package com.example.madgwick_filter;

import android.util.Log;
import java.util.LinkedList;
import java.util.Queue;

public class MovementDetector {
    private static final String TAG = "MovementDetector";
    private static final int WINDOW_SIZE = 5; // 0.05秒 * 100Hz
    private static final float STD_DEV_THRESHOLD = 0.05f;
    private static final float MEAN_THRESHOLD = 0.08f;
    private static final int STATIC_COUNT_THRESHOLD = 5; // 即座に静止状態を検出

    private Queue<Float> magnitudeWindow = new LinkedList<>();
    private boolean isMoving = false;
    private int staticCount = 0;

    public boolean update(float[] worldAccel) {
        float magnitude = (float) Math.sqrt(
                worldAccel[0] * worldAccel[0] +
                        worldAccel[1] * worldAccel[1] +
                        worldAccel[2] * worldAccel[2]
        );

        magnitudeWindow.offer(magnitude);
        if (magnitudeWindow.size() > WINDOW_SIZE) {
            magnitudeWindow.poll();
        }

        if (magnitudeWindow.size() == WINDOW_SIZE) {
            float sum = 0;
            float squareSum = 0;

            for (float mag : magnitudeWindow) {
                sum += mag;
                squareSum += mag * mag;
            }

            float mean = sum / WINDOW_SIZE;
            float variance = (squareSum / WINDOW_SIZE) - (mean * mean);
            float stdDev = (float) Math.sqrt(variance);

            boolean previousState = isMoving;

            if (stdDev > STD_DEV_THRESHOLD || mean > MEAN_THRESHOLD) {
                isMoving = true;
                staticCount = 0;
            } else {
                staticCount++;
                if (staticCount >= STATIC_COUNT_THRESHOLD) {
                    isMoving = false;
                }
            }

            Log.d(TAG, String.format("Mean: %.4f, StdDev: %.4f, StaticCount: %d, IsMoving: %b",
                    mean, stdDev, staticCount, isMoving));

            if (isMoving != previousState) {
                Log.d(TAG, "Movement state changed: " + (isMoving ? "Moving" : "Static"));
            }
        }

        return isMoving;
    }

    public boolean isMoving() {
        return isMoving;
    }

    public void reset() {
        magnitudeWindow.clear();
        isMoving = false;
        staticCount = 0;
        Log.d(TAG, "MovementDetector reset");
    }
}