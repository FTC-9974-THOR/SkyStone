package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;

@Autonomous(name = "Stone CV Tests (Red)", group = "RND")
@Disabled
public class StoneVisionTestRed extends LinearOpMode {

    private VuMarkNavSource vuMarkNavSource;

    private StoneVision cv;

    @Override
    public void runOpMode() throws InterruptedException {
        vuMarkNavSource = new VuMarkNavSource(
                hardwareMap.appContext,
                VuforiaKey.KEY,
                VuforiaLocalizer.CameraDirection.BACK,
                OpenGLMatrix.translation(0, 0, 0)
        );

        cv = new StoneVision(vuMarkNavSource.getVuforiaLocalizer(), StoneVision.Tunings.RED);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        cv.beginProcessing();

        while (!isStopRequested()) {
            if (!cv.isProcessingComplete()) {
                telemetry.addLine("Processing...");
            } else {
                telemetry.addData("Time", cv.time);
            }
            telemetry.addData("Position", cv.getStonePosition());
            telemetry.update();
        }
    }
}
