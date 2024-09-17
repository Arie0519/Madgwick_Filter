package com.example.madgwick_filter;

import java.util.Queue;
import java.util.LinkedList;

public class MovementDetector {
    private static final float MOVEMENT_THRESHOLD = 1.0f; // m/s^2
    private static final long WINDOW_SIZE_MS = 200;
    private static final float GRAVITY = 9.81f; // m/s^2
    private boolean isMoving = false;
    private final Queue<AccelData> accelDataWindow = new LinkedList<>();

    private static class AccelData {
        long timestamp;
        float[] values;

        AccelData(long timestamp, float[] values) {
            this.timestamp = timestamp;
            this.values = values;
        }
    }

    public void update(long timestamp, float[] accelData) {
        accelDataWindow.offer(new AccelData(timestamp, accelData.clone()));

        while (!accelDataWindow.isEmpty() &&
                timestamp - accelDataWindow.peek().timestamp > WINDOW_SIZE_MS * 1000000) {
            accelDataWindow.poll();
        }

        if (accelDataWindow.size() < 2) {
            isMoving = false;
            return;
        }

        float totalMagnitude = 0;
        for (AccelData data : accelDataWindow) {
            float magnitude = (float) Math.sqrt(
                    Math.pow(data.values[0], 2) +
                            Math.pow(data.values[1], 2) +
                            Math.pow(data.values[2] - GRAVITY, 2)
            );
            totalMagnitude += magnitude;
        }

        float averageMagnitude = totalMagnitude / accelDataWindow.size();
        isMoving = averageMagnitude > MOVEMENT_THRESHOLD;
    }

    public boolean isMoving() {
        return isMoving;
    }

    public void reset() {
        isMoving = false;
        accelDataWindow.clear();
    }
}