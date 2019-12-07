package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.RunningAverageFilter;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Line Up on Stone")
@Disabled
public class LineUpAuto extends LinearOpMode {

    private MecanumDrive rb;
    private VuMarkNavSource vuforia;
    private IMUNavSource imu;
    private PIDF alignPid, headingPid;
    private RunningAverageFilter filter;
    private Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(false, false, true, true);
        rb.resetEncoders();

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);
        imu = new IMUNavSource(hardwareMap);

        alignPid = new PIDF(0.003, 0, 0, 0);
        alignPid.setNominalOutputForward(0.2);
        alignPid.setNominalOutputReverse(-0.2);
        alignPid.setPeakOutputForward(0.7);
        alignPid.setPeakOutputReverse(-0.7);
        alignPid.setAtTargetThreshold(10);
        alignPid.setPhase(true);
        alignPid.setSetpoint(6.35);

        headingPid = new PIDF(0.3, 0, 0, 0);
        headingPid.setInputFunction(imu::getHeading);

        filter = new RunningAverageFilter(15);

        arm = new Arm(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            telemetry.addData("Can See Stone", vuforia.canSeeStone());
            update();
            telemetry.update();
        }
        if (isStopRequested()) {
            shutdown();
            return;
        }

        TimingUtilities.blockUntil(this, vuforia::canSeeStone, null, null);
        if (isStopRequested()) return;

        long onTargetStart = SystemClock.uptimeMillis();
        while (!isStopRequested() && vuforia.canSeeStone()/* && SystemClock.uptimeMillis() - onTargetStart < 100*/) {
            OpenGLMatrix stoneLocation = vuforia.getRelativeStoneLocation();
            if (stoneLocation != null) {
                VectorF translation = stoneLocation.getTranslation();
                double pidInput = filter.update(translation.get(0));
                double pidOutput = alignPid.update(pidInput);
                telemetry.addData("Input", pidInput);
                telemetry.addData("Output", pidOutput);
                if (Double.isNaN(pidOutput)) {
                    telemetry.log().add("Got NaN pid output!");
                }
                telemetry.update();
                rb.drive(pidOutput, 0, headingPid.update());
            }
            if (!alignPid.atTarget()) {
                onTargetStart = SystemClock.uptimeMillis();
            }
        }
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        /*arm.setTargetPosition(0.489);
        arm.setClosedLoopEnabled(true);

        TimingUtilities.blockUntil(this, arm::shoulderAtTarget, this::update, this::shutdown);
        if (isStopRequested()) return;*/

        TimingUtilities.blockUntil(this, this::isStopRequested, this::update, this::shutdown);
    }

    private void update() {
        arm.update();
    }

    private void shutdown() {
        arm.setClosedLoopEnabled(false);
        arm.setShoulderPower(0);
    }
}
