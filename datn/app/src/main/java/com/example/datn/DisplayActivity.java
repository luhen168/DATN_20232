package com.example.datn;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.icu.text.SimpleDateFormat;
import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class DisplayActivity extends AppCompatActivity {
    private static final int LOCATION_PERMISSION_REQUEST_CODE = 1001;
    private LocationManager locationManager;
    private boolean isLogging = false;
    private static final String TAG = "GnssDataService";
    private static final String CSV_HEADER = "utcTimeMillis,TimeNanos,LeapSecond,TimeUncertaintyNanos,FullBiasNanos,BiasNanos,BiasUncertaintyNanos,DriftNanosPerSecond,DriftUncertaintyNanosPerSecond,HardwareClockDiscontinuityCount,Svid,TimeOffsetNanos,State,ReceivedSvTimeNanos,ReceivedSvTimeUncertaintyNanos,Cn0DbHz,PseudorangeRateMetersPerSecond,PseudorangeRateUncertaintyMetersPerSecond,AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,AccumulatedDeltaRangeUncertaintyMeters,CarrierFrequencyHz,CarrierCycles,CarrierPhase,CarrierPhaseUncertainty,MultipathIndicator,SnrInDb,ConstellationType,AgcDb,BasebandCn0DbHz,FullInterSignalBiasNanos,FullInterSignalBiasUncertaintyNanos,SatelliteInterSignalBiasNanos,SatelliteInterSignalBiasUncertaintyNanos,CodeType,ChipsetElapsedRealtimeNanos";
    private FileWriter csvWriter;
    private String csvFilePath;
    private List<String> gnssDataList = new ArrayList<>();
    TextView textView;
    Button startBtn, stopBtn;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_display);
        textView = findViewById(R.id.rawDataView);
        startBtn = findViewById(R.id.startBtn);
        stopBtn = findViewById(R.id.stopBtn);
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION}, LOCATION_PERMISSION_REQUEST_CODE);
        }
        startBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startService();
            }
        });
        stopBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                stopService();
            }
        });
    }
    private void startService() {
        if(!isLogging) {
            if (locationManager == null) {
                locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
            }

            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
                Log.i("Location manager", "Co vao location manager");
                locationManager.registerGnssMeasurementsCallback(gnssMeasurementsEventListener);
            }
            isLogging = true;
            Toast.makeText(DisplayActivity.this, "GNSS Service started", Toast.LENGTH_SHORT).show();
        }
        else {
            Toast.makeText(DisplayActivity.this, "Logging", Toast.LENGTH_SHORT).show();
        }
    }

    private void stopService() {
        if(isLogging) {
            locationManager.unregisterGnssMeasurementsCallback(gnssMeasurementsEventListener);
            isLogging = false;
            closeCsvWriter();
            Toast.makeText(DisplayActivity.this, "GNSS Service stopped", Toast.LENGTH_SHORT).show();
        }
        else {
            Toast.makeText(DisplayActivity.this, "Not logging", Toast.LENGTH_SHORT).show();
        }
    }
    private final GnssMeasurementsEvent.Callback gnssMeasurementsEventListener = new GnssMeasurementsEvent.Callback() {
        @Override
        public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {
            List<GnssMeasurement> measurements = (List<GnssMeasurement>) event.getMeasurements();
            GnssClock gnssClock = event.getClock();
            Log.i("Location manager", "Co vao gnss measurement");
            executePythonCode(measurements, gnssClock);
            saveToCsv(measurements, gnssClock);

        }
    };
    private void saveToCsv(List<GnssMeasurement> measurements, GnssClock gnssClock) {
        if (csvWriter == null) {
            try {
                String timeStamp = new SimpleDateFormat("yyyy_MM_dd_HH_mm", Locale.getDefault()).format(new Date());
                String fileName = "gnss_" + timeStamp + ".csv";
                File csvFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), fileName);
                csvFilePath = csvFile.getAbsolutePath();
                csvWriter = new FileWriter(csvFile);
                csvWriter.append(CSV_HEADER).append("\n");
                Log.i(TAG, "CSV file created");
            } catch (IOException e) {
                Log.e(TAG, "Error creating CSV file", e);
            }
        }
        try {
            for (GnssMeasurement measurement : measurements) {
                String line = convertMeasurementToCsvLine(measurement, gnssClock);
                csvWriter.append(line);
                csvWriter.append("\n");
            }
            csvWriter.flush();
        } catch (IOException e) {
            Log.e(TAG, "Error writing to CSV file", e);
        }
    }
    private void closeCsvWriter() {
        if (csvWriter != null) {
            try {
                csvWriter.close();
                Toast.makeText(getApplicationContext(), "CSV file saved to: " + csvFilePath, Toast.LENGTH_LONG).show();
            } catch (IOException e) {
                Log.e(TAG, "Error closing CSV writer", e);
            }
        }
    }
    private void executePythonCode(List<GnssMeasurement> measurements, GnssClock gnssClock) {
        gnssDataList.clear();
        for (GnssMeasurement measurement : measurements) {
            String line = convertMeasurementToCsvLine(measurement, gnssClock);
            gnssDataList.add(line);
        }
        if (!Python.isStarted()) {
            Python.start(new AndroidPlatform(this));
        }
        Python py = Python.getInstance();

        PyObject pyModule = py.getModule("test");

        String joinedData = String.join(";", gnssDataList);

        PyObject result = pyModule.callAttr("process_data", joinedData);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                textView.setText(result.toString());
            }
        });
    }
    private String convertMeasurementToCsvLine(GnssMeasurement measurement, GnssClock gnssClock) {
        StringBuilder builder = new StringBuilder();
        long utcTimeMillis = System.currentTimeMillis();
        long TimeNanos = gnssClock.getTimeNanos();
        int LeapSecond = gnssClock.hasLeapSecond() ? gnssClock.getLeapSecond() : 0;
        double TimeUncertaintyNanos = gnssClock.hasTimeUncertaintyNanos() ? gnssClock.getTimeUncertaintyNanos() : 0;
        long FullBiasNanos = gnssClock.getFullBiasNanos();
        double BiasNanos = gnssClock.hasBiasNanos() ? gnssClock.getBiasNanos() : 0;
        double BiasUncertaintyNanos = gnssClock.hasBiasUncertaintyNanos() ? gnssClock.getBiasUncertaintyNanos() : 0;
        double DriftNanosPerSecond = gnssClock.hasDriftNanosPerSecond() ? gnssClock.getDriftNanosPerSecond() : 0;
        double DriftUncertaintyNanosPerSecond = gnssClock.hasDriftUncertaintyNanosPerSecond() ? gnssClock.getDriftUncertaintyNanosPerSecond() : 0;
        double HardwareClockDiscontinuityCount = gnssClock.getHardwareClockDiscontinuityCount();
        builder.append(utcTimeMillis).append(",");
        builder.append(TimeNanos).append(",");
        builder.append(LeapSecond).append(",");
        builder.append(TimeUncertaintyNanos).append(",");
        builder.append(FullBiasNanos).append(",");
        builder.append(BiasNanos).append(",");
        builder.append(BiasUncertaintyNanos).append(",");
        builder.append(DriftNanosPerSecond).append(",");
        builder.append(DriftUncertaintyNanosPerSecond).append(",");
        builder.append(HardwareClockDiscontinuityCount).append(",");
        builder.append(measurement.getSvid()).append(",");
        builder.append(measurement.getTimeOffsetNanos()).append(",");
        builder.append(measurement.getState()).append(",");
        builder.append(measurement.getReceivedSvTimeNanos()).append(",");
        builder.append(measurement.getReceivedSvTimeUncertaintyNanos()).append(",");
        builder.append(measurement.getCn0DbHz()).append(",");
        builder.append(measurement.getPseudorangeRateMetersPerSecond()).append(",");
        builder.append(measurement.getPseudorangeRateUncertaintyMetersPerSecond()).append(",");
        builder.append(measurement.getAccumulatedDeltaRangeState()).append(",");
        builder.append(measurement.getAccumulatedDeltaRangeMeters()).append(",");
        builder.append(measurement.getAccumulatedDeltaRangeUncertaintyMeters()).append(",");
        builder.append(measurement.hasCarrierFrequencyHz() ? measurement.getCarrierFrequencyHz() : 0).append(",");
        builder.append(measurement.hasCarrierCycles() ? measurement.getCarrierCycles() : 0).append(",");
        builder.append(measurement.hasCarrierPhase() ? measurement.getCarrierPhase() : 0).append(",");
        builder.append(measurement.hasCarrierPhaseUncertainty() ? measurement.getCarrierPhaseUncertainty() : 0).append(",");
        builder.append(measurement.getMultipathIndicator()).append(",");
        builder.append(measurement.hasSnrInDb() ? measurement.getSnrInDb() : 0).append(",");
        builder.append(measurement.getConstellationType()).append(",");
        builder.append(measurement.hasAutomaticGainControlLevelDb() ? measurement.getAutomaticGainControlLevelDb():0).append(",");
        builder.append(measurement.getBasebandCn0DbHz()).append(",");
        builder.append(measurement.getFullInterSignalBiasNanos()).append(",");
        builder.append(measurement.getFullInterSignalBiasUncertaintyNanos()).append(",");
        builder.append(measurement.getSatelliteInterSignalBiasNanos()).append(",");
        builder.append(measurement.getSatelliteInterSignalBiasUncertaintyNanos()).append(",");
        builder.append(measurement.getCodeType()).append(",");
        builder.append(gnssClock.getElapsedRealtimeNanos());
        Log.i("Log data", "Logging data");
        return builder.toString();
    }
}
