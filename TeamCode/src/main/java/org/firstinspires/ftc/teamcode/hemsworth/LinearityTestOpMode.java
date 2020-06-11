package org.firstinspires.ftc.teamcode.hemsworth;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ftc9974.thorcore.robot.MotorType;
import org.ftc9974.thorcore.util.MathUtilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Linearity Test")
public class LinearityTestOpMode extends LinearOpMode {

    private AnalogInput ma3;
    private DcMotorEx slewMotor;

    private double ma3Offset;
    private int quadOffset;
    private boolean offsetsInitialized;

    private double[] directionDataset, voltageDataset;
    private int numDataPoints;
    private ElapsedTime timer;

    private List<LynxModule> revHubs;

    @Override
    public void runOpMode() {
        ma3 = hardwareMap.analogInput.get("SM-left-encoder");
        slewMotor = hardwareMap.get(DcMotorEx.class, "SM-left-slewMotor");

        directionDataset = new double[(int) Math.floor(7 * 4 * 71.2) + 1];
        for (int i = 0; i < directionDataset.length; i++) {
            directionDataset[i] = -1;
        }
        voltageDataset = directionDataset.clone();

        ma3Offset = quadOffset = 0;

        timer = new ElapsedTime();

        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule revHub : revHubs) {
            revHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (!isStopRequested() && !isStarted());
        if (isStopRequested()) return;

        timer.reset();
        slewMotor.setPower(0.1);

        while (!isStopRequested() && timer.seconds() < 0.5) {
            telemetry.addLine("Preloading Gears...");
            telemetry.update();
        }
        if (isStopRequested()) return;

        final int collectionThreshold = 1960;
        while (!isStopRequested() && numDataPoints < collectionThreshold) {
            for (LynxModule revHub : revHubs) {
                revHub.clearBulkCache();
            }

            double ma3Position = ma3.getVoltage();
            int quadPosition = slewMotor.getCurrentPosition() - quadOffset;

            if (!offsetsInitialized) {
                ma3Offset = ma3Position;
                quadOffset = quadPosition;

                quadPosition = 0;

                offsetsInitialized = true;
            }

            double ma3Direction = calculateMA3Direction(ma3Position);
            double quadDirection = calculateQuadDirection(quadPosition);
            double difference = ma3Direction - quadDirection;
            difference %= 2.0 * Math.PI;
            if (Math.abs(difference) > Math.PI) {
                if (difference > 0) {
                    difference -= 2.0 * Math.PI;
                } else {
                    difference += 2.0 * Math.PI;
                }
            }

            int wrappedQuadPosition = quadPosition;
            while (wrappedQuadPosition < 0) {
                wrappedQuadPosition += MotorType.YELLOWJACKET_71_2.ticksPerRevolution;
            }
            wrappedQuadPosition %= MotorType.YELLOWJACKET_71_2.ticksPerRevolution;

            if (wrappedQuadPosition < directionDataset.length && voltageDataset[wrappedQuadPosition] == -1) {
                directionDataset[wrappedQuadPosition] = ma3Direction;
                voltageDataset[wrappedQuadPosition] = ma3Position;
                numDataPoints++;
            }

            telemetry.addLine("Collecting Data.");
            telemetry.addData("Progress", String.format(Locale.getDefault(), "%d%% (%d/%d)", (int) ((100.0 * numDataPoints) / (double) collectionThreshold), numDataPoints, collectionThreshold));
            telemetry.update();
        }
        if (isStopRequested()) return;

        telemetry.log().add("Saving...");
        telemetry.update();

        try {
            File dataFile = new File(AppUtil.FIRST_FOLDER + "/linearityDataset.csv");
            if (dataFile.exists()) {
                dataFile.delete();
            }
            dataFile.createNewFile();
            FileWriter writer = new FileWriter(dataFile);

            for (int i = 0; i < directionDataset.length; i++) {
                double direction = directionDataset[i];
                double voltage = voltageDataset[i];

                if (voltage == -1) continue;

                // format:
                // ticks,quad encoder direction,ma3 direction,ma3 voltage
                writer.write(
                        String.format(Locale.US,
                                "%d,%f,%f,%f\n",
                                i,
                                calculateQuadDirection(i),
                                direction,
                                voltage)
                );
            }
            writer.flush();
            writer.close();
            telemetry.log().add("Saved %d data points.", numDataPoints);
        } catch (IOException e) {
            RobotLog.ee("LinearityTest", e, "IOException while saving dataset");
            telemetry.log().add("Failed to save.");
        }
        telemetry.update();
    }

    private double calculateMA3Direction(double currentVoltage) {
        double wrappedVoltage = currentVoltage - ma3Offset;
        if (wrappedVoltage < 0) {
            wrappedVoltage += 5;
        }
        return MathUtilities.map(wrappedVoltage, 5, 0, 0, 2.0 * Math.PI);
    }

    private double calculateQuadDirection(int currentPosition) {
        double ticksPerRevolution = MotorType.YELLOWJACKET_71_2.ticksPerRevolution;
        return MathUtilities.map(
                currentPosition % ticksPerRevolution,
                0,
                ticksPerRevolution,
                0,
                2 * Math.PI
        );
    }
}
