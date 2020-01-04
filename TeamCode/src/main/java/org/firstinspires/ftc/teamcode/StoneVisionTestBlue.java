package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;

@Autonomous(name = "Stone CV Tests (Blue)", group = "RND")
@Disabled
public class StoneVisionTestBlue extends LinearOpMode {

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

        cv = new StoneVision(vuMarkNavSource.getVuforiaLocalizer(), StoneVision.Tunings.BLUE);

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
