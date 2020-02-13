package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;

@Autonomous(name = "Stone CV Tests (Red)", group = "RND")
//@Disabled
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

        telemetry.addLine("Initializing Hardware Acceleration...");
        telemetry.update();

        RenderScriptContextManager rscm = RenderScriptContextManager.getInstance();
        rscm.init();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        StonePosition correctPosition = StonePosition.LEFT;
        int correctCount, testCount;
        correctCount = testCount = 0;

        while (!isStopRequested()) {
            cv.beginProcessing();

            while (!cv.isProcessingComplete() && !isStopRequested()) {
                if (testCount > 0) telemetry.addData("Accuracy", 100 * correctCount / testCount);
                telemetry.addData("Correct", correctCount);
                telemetry.addData("Total", testCount);
                telemetry.addLine("Processing...");
                telemetry.update();
            }
            testCount++;
            if (cv.getStonePosition() == correctPosition) {
                correctCount++;
            }

            telemetry.addData("Accuracy", 100 * correctCount / testCount);
            telemetry.addData("Correct", correctCount);
            telemetry.addData("Total", testCount);
            telemetry.log().add("Time: %.3f", cv.time);
            telemetry.log().add("Position: %s", cv.getStonePosition().toString());
            telemetry.update();
        }
    }
}
