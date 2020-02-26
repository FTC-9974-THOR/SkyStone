package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.StoneVision;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.OdometerNavSource;
import org.ftc9974.thorcore.control.navigation.PIDFMovementStrategy;
import org.ftc9974.thorcore.control.navigation.SynchronousNavigator;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.List;

@Autonomous(name = "Odometer Tuning", group = "RND")
public class OdometerTuningOpMode extends LinearOpMode {

    private MecanumDrive rb;
    private Intake intake;
    private OdometerNavSource odometerNavSource;
    private PIDFMovementStrategy pidfMovementStrategy;
    private SynchronousNavigator navigator;

    //private VuMarkNavSource vuforia;
    //private StoneVision stoneVision;

    private StoneArm stoneArm;

    private ServoImplEx gearServo;

    private List<LynxModule> revHubs;

    private Arm arm;

    private FoundationClaw foundationClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);

        intake = new Intake(hardwareMap);

        odometerNavSource = new OdometerNavSource(
                hardwareMap,
                intake.intake0,
                intake.intake1,
                new Vector2(MathUtilities.inchesToMM(-9) + 25, MathUtilities.inchesToMM(-9) + 155),
                new Vector2(MathUtilities.inchesToMM(9) - 25, MathUtilities.inchesToMM(9) - 188),
                50,
                8192);
        odometerNavSource.setXInversion(false);
        odometerNavSource.setYInversion(true);
        // tuned heuristically with the Ziegler-Nichols Method - PD
        pidfMovementStrategy = new PIDFMovementStrategy(
                0.003, 0, 0.00005, 0,
                0.0028, 0, 0.0001, 0,
                0.6 * 5, 0, 0.6 / 8, 0,
                0.0,
                0.0,
                0.1,
                13,
                13,
                0.02
        );
        pidfMovementStrategy.setContinuity(-Math.PI, Math.PI);
        pidfMovementStrategy.setXPeriod(0.05);
        pidfMovementStrategy.setYPeriod(0.05);
        //pidfMovementStrategy.setThetaPeriod(0.6 / 8);
        navigator = new SynchronousNavigator(odometerNavSource, rb, pidfMovementStrategy);
        navigator.setEnabled(false);

        /*OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);

        stoneVision = new StoneVision(vuforia.getVuforiaLocalizer(), StoneVision.Tunings.BLUE);*/

        stoneArm = new StoneArm(hardwareMap);

        gearServo = hardwareMap.get(ServoImplEx.class, "O-gearServo");
        gearServo.setPwmRange(new PwmControl.PwmRange(1875, 2160));
        gearServo.setPosition(1);

        stoneArm.release();
        stoneArm.retract();

        revHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule revHub : revHubs) {
            revHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        arm = new Arm(hardwareMap);
        arm.release();

        foundationClaw = new FoundationClaw(hardwareMap);
        foundationClaw.retract();

        pidfMovementStrategy.setSpeedLimit(1);

        odometerNavSource.resetOdometers();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }

        /*double xCoefAccum = 0, yCoefAccum = 0;
        int coefCount = 0;

        while (!isStopRequested()) {
            odometerNavSource.resetOdometers();

            double startHeading = odometerNavSource.getHeading();

            navigator.setTarget(new Vector2(0, 0), startHeading + 0.5 * Math.PI);
            navigator.setAllowMovement(false);
            navigator.setEnabled(true);

            ElapsedTime timer = new ElapsedTime();
            double lastHeading = odometerNavSource.getHeading();
            double periodAccumulator = 0;
            double periodNumber = -1;

            do {
                clearHubCaches();
                double currentHeading = odometerNavSource.getHeading();
                if ((currentHeading > 0.5 * Math.PI && lastHeading < -0.5 * Math.PI) ||
                        (currentHeading < -0.5 * Math.PI && lastHeading > 0.5 * Math.PI)) {
                    if (periodNumber == -1) {
                        periodNumber = 0;
                    } else {
                        periodAccumulator += timer.seconds();
                        periodNumber += 0.5;
                    }
                    timer.reset();
                }
                lastHeading = currentHeading;
                if (periodNumber != -1) {
                    telemetry.addData("Period", periodAccumulator / periodNumber);
                }
                if (coefCount > 0) {
                    telemetry.addData("X Coef", xCoefAccum / coefCount);
                    telemetry.addData("Y Coef", yCoefAccum / coefCount);
                    telemetry.addData("Count", coefCount);
                }
                telemetry.addData("Current Position", odometerNavSource.getLocation());
                telemetry.addData("Target Position", navigator.getTargetPosition());
                telemetry.addData("Current Heading", odometerNavSource.getHeading());
                telemetry.addData("Target Heading", pidfMovementStrategy.getHeadingTarget());
                telemetry.update();
                navigator.update();
            } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget());

            rb.drive(0, 0, 0);

            clearHubCaches();
            double xPosition = odometerNavSource.getXEncoderReading();
            double yPosition = odometerNavSource.getYEncoderReading();
            double theta = odometerNavSource.getHeading() - startHeading;
            // xPosition = theta * coef
            xCoefAccum += xPosition / theta;
            yCoefAccum += yPosition / theta;
            coefCount++;
            RobotLog.ii("OdometerTest", "XCoef: %.8f YCoef: %.8f Count: %d", xCoefAccum / coefCount, yCoefAccum / coefCount, coefCount);

            odometerNavSource.resetOdometers();
            TimingUtilities.sleep(this, 0.5, null, null);
        }*/

        navigator.setTarget(new Vector2(0, 500), 0);
        navigator.setEnabled(true);

        do {
            clearHubCaches();
            navigator.update();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
        } while (!isStopRequested());

        rb.drive(0, 0, 0);
        /*TimingUtilities.blockUntil(this, this::isStopRequested, () -> {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
        }, null);*/
    }

    private void clearHubCaches() {
        for (LynxModule revHub : revHubs) {
            revHub.clearBulkCache();
        }
    }

    private boolean navigatorWithin(double distance) {
        return navigator.getTargetPosition().subtract(odometerNavSource.getLocation()).getMagnitude() < distance;
    }
}
