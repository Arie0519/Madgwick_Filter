package com.example.madgwick_filter;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "MainActivity";
    private static final int PERMISSION_REQUEST_CODE = 1;
    private SensorManager sensorManager;
    private Sensor accelerometer, gyroscope;
    private TextView tvQuaternion, tvAcceleration, tvState, tvDistance;
    private Button btnStart, btnStop, btnReset;
    private boolean isRunning = false;
    private MadgwickFilter madgwickFilter;
    private MovementDetector movementDetector;
    private DistanceCalculator distanceCalculator;
    private FileWriter csvWriter;
    private static final long SAMPLING_PERIOD_US = 100000; // 100ms = 10Hz
    private static final float SAMPLING_FREQUENCY = 10f; // Hz
    private static final long CSV_WRITE_INTERVAL_NS = 200000000; // 200ms
    private static final long MOVEMENT_DETECTION_INTERVAL_NS = 200000000; // 200ms
    private long lastMovementDetectionTime = 0;
    private long lastCsvWriteTime = 0;
    private long startTime = 0;
    private float[] lastAcceleration = new float[3];
    private float[] lastGyroscope = new float[3];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeViews();
        initializeSensors();
        requestPermissions();

        madgwickFilter = new MadgwickFilter();
        movementDetector = new MovementDetector();
        distanceCalculator = new DistanceCalculator();

        // アプリ起動時にセンサーリスナーを登録
        startSensorListening();
    }

    private void initializeViews() {
        tvQuaternion = findViewById(R.id.tvQuaternion);
        tvAcceleration = findViewById(R.id.tvAcceleration);
        tvState = findViewById(R.id.tvState);
        tvDistance = findViewById(R.id.tvDistance);
        btnStart = findViewById(R.id.btnStart);
        btnStop = findViewById(R.id.btnStop);
        btnReset = findViewById(R.id.btnReset);

        btnStart.setOnClickListener(v -> startMeasurement());
        btnStop.setOnClickListener(v -> stopMeasurement());
        btnReset.setOnClickListener(v -> resetMeasurement());
    }

    private void initializeSensors() {
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager == null) {
            Log.e(TAG, "Failed to get SensorManager");
            Toast.makeText(this, "センサーマネージャーの取得に失敗しました", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        if (accelerometer == null || gyroscope == null) {
            Log.e(TAG, "Required sensors are not available");
            Toast.makeText(this, "必要なセンサーが利用できません", Toast.LENGTH_SHORT).show();
            finish();
        }
    }

    private void startSensorListening() {
        if (sensorManager != null) {
            sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
            sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        }
    }

    private void stopSensorListening() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }

    private void requestPermissions() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    PERMISSION_REQUEST_CODE);
        }
    }

    private void startMeasurement() {
        if (!isRunning) {
            createCsvFile();
            isRunning = true;
            startTime = System.nanoTime();
            distanceCalculator.resetDistance(); // 距離を0にリセット
            startSensorListening();
            updateUI(new float[4], new float[3], "静止", 0); // UIも0に更新
            Toast.makeText(this, "測定開始", Toast.LENGTH_SHORT).show();
        }
    }

    private void stopMeasurement() {
        if (isRunning) {
            stopSensorListening();
            closeCsvFile();
            isRunning = false;
            Toast.makeText(this, "測定終了", Toast.LENGTH_SHORT).show();
        }
    }

    private void resetMeasurement() {
        stopMeasurement();
        madgwickFilter.reset();
        movementDetector.reset();
        distanceCalculator.reset();
        updateUI(new float[4], new float[3], "静止", 0);
        Toast.makeText(this, "リセット完了", Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        try {
            if (event.values == null || event.values.length < 3) {
                Log.e(TAG, "Invalid sensor data received");
                return;
            }

            if (madgwickFilter == null || movementDetector == null || distanceCalculator == null) {
                Log.e(TAG, "One or more required objects are null");
                return;
            }

            long currentTime = System.nanoTime();
            long elapsedTime = currentTime - startTime;

            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                System.arraycopy(event.values, 0, lastAcceleration, 0, 3);
            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                System.arraycopy(event.values, 0, lastGyroscope, 0, 3);
            }

            madgwickFilter.update(lastAcceleration, lastGyroscope, SAMPLING_FREQUENCY);
            float[] quaternion = madgwickFilter.getQuaternion();
            float[] adjustedWorldAccel = madgwickFilter.getWorldAcceleration();

            boolean isMoving = movementDetector.update(adjustedWorldAccel);
            String state = isMoving ? "歩行" : "静止";

            // デバッグログ
            Log.d(TAG, String.format("Adjusted World Accel: %.2f, %.2f, %.2f, Moving: %b",
                    adjustedWorldAccel[0], adjustedWorldAccel[1], adjustedWorldAccel[2], isMoving));

            float[] motionData = distanceCalculator.calculateMotion(adjustedWorldAccel, isMoving, event.timestamp);

            updateUI(quaternion, adjustedWorldAccel, state, motionData[6]);

            if (isRunning && currentTime - lastCsvWriteTime >= CSV_WRITE_INTERVAL_NS) {
                writeToCsv(elapsedTime, lastAcceleration, lastGyroscope, quaternion, adjustedWorldAccel, state, motionData);
                lastCsvWriteTime = currentTime;
            }
        } catch (Exception e) {
            Log.e(TAG, "Error in onSensorChanged", e);
            if (isRunning) {
                stopMeasurement();
            }
            Toast.makeText(this, "センサーデータの処理中にエラーが発生しました: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }

    private void updateUI(float[] quaternion, float[] adjustedWorldAccel, String state, float distance) {
        runOnUiThread(() -> {
            tvQuaternion.setText(String.format(Locale.getDefault(),
                    "%.2f, %.2f, %.2f, %.2f", quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
            tvAcceleration.setText(String.format(Locale.getDefault(),
                    "X=%.2f, Y=%.2f, Z=%.2f", adjustedWorldAccel[0], adjustedWorldAccel[1], adjustedWorldAccel[2]));
            tvState.setText(state);
            tvDistance.setText(String.format(Locale.getDefault(), "%.2f m", distance));
        });
    }

    private void createCsvFile() {
        String fileName = "sensor_data_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date()) + ".csv";
        File file = new File(getExternalFilesDir(null), fileName);
        try {
            csvWriter = new FileWriter(file);
            csvWriter.append("Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,WorldAccelX,WorldAccelY,WorldAccelZ,VelocityX,VelocityY,VelocityZ,PositionX,PositionY,PositionZ,Distance\n");
            Toast.makeText(this, "CSVファイルが作成されました: " + file.getAbsolutePath(), Toast.LENGTH_LONG).show();
        } catch (IOException e) {
            Log.e(TAG, "CSVファイルの作成に失敗しました", e);
            Toast.makeText(this, "CSVファイルの作成に失敗しました: " + e.getMessage(), Toast.LENGTH_SHORT).show();
        }
    }

    private void writeToCsv(long elapsedTime, float[] acceleration, float[] gyroscope, float[] quaternion, float[] adjustedWorldAccel, String state, float[] motionData) {
        if (csvWriter == null) return;

        try {
            StringBuilder sb = new StringBuilder();
            sb.append(elapsedTime).append(",");

            // 加速度計の値
            for (float value : acceleration) {
                sb.append(value).append(",");
            }

            // ジャイロスコープの値
            for (float value : gyroscope) {
                sb.append(value).append(",");
            }

            // クォータニオン
            for (float q : quaternion) {
                sb.append(q).append(",");
            }

            // 調整済みワールド座標系の加速度
            for (float wa : adjustedWorldAccel) {
                sb.append(wa).append(",");
            }

            // 速度 (3D)
            sb.append(motionData[0]).append(",").append(motionData[1]).append(",").append(motionData[2]).append(",");

            // 位置 (3D)
            sb.append(motionData[3]).append(",").append(motionData[4]).append(",").append(motionData[5]).append(",");

            // 総移動距離 (2D: x-y平面上)
            sb.append(motionData[6]);

            sb.append("\n");

            csvWriter.write(sb.toString());
        } catch (IOException e) {
            Log.e(TAG, "CSVファイルへの書き込みに失敗しました", e);
        }
    }

    private void closeCsvFile() {
        if (csvWriter != null) {
            try {
                csvWriter.flush();
                csvWriter.close();
            } catch (IOException e) {
                Log.e(TAG, "CSVファイルのクローズに失敗しました", e);
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 使用しない
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                Toast.makeText(this, "ストレージ権限が許可されました", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "ストレージ権限が必要です", Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (isRunning) {
            startSensorListening();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        stopSensorListening();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopMeasurement();
        sensorManager = null;
        madgwickFilter = null;
        movementDetector = null;
        distanceCalculator = null;
    }
}