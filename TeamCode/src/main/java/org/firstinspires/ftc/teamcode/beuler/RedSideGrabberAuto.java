package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Fusion2;
import org.firstinspires.ftc.teamcode.RenderScriptContextManager;
import org.firstinspires.ftc.teamcode.StonePosition;
import org.firstinspires.ftc.teamcode.StoneVision;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Red Skystone & Foundation", group = "C")
public class RedSideGrabberAuto extends LinearOpMode {

    private enum FailsafeCondition {
        OK,
        STOP_REQUESTED,
        QUARRY_SIDE,
        BUILD_SIDE,
        ULTRASONIC_FAILURE
    }

    private MecanumDrive rb;
    private Fusion2 fusion2;
    private VuMarkNavSource vuforia;
    private StoneArm stoneArm;
    private FoundationClaw foundationClaw;

    private StoneVision vision;

    private Blinkin blinkin;

    private ParkingTape parkingTape;

    private Intake intake;
    private Odometer odometer;

    private AutonomousSensorManager asm;
    private PIDF headingPid, sidePid;

    private StonePosition stonePosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(true, true, false, false);
        rb.resetEncoders();

        fusion2 = new Fusion2(hardwareMap, rb);

        if (fusion2.getNavSource().isFallback()) {
            telemetry.log().add("Warning: Primary IMU failure, using fallback!");
            telemetry.update();
        }

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);

        stoneArm = new StoneArm(hardwareMap);
        foundationClaw = new FoundationClaw(hardwareMap);

        blinkin = new Blinkin(hardwareMap);
        blinkin.defaultPattern();

        vision = new StoneVision(vuforia.getVuforiaLocalizer(), StoneVision.Tunings.RED);

        parkingTape = new ParkingTape(hardwareMap);

        intake = new Intake(hardwareMap);
        odometer = new Odometer(hardwareMap, intake.intake0);
        odometer.extend();

        asm = fusion2.getASM();
        headingPid = fusion2.headingPid;
        //headingPid.setNominalOutputForward(0.15);
        //headingPid.setNominalOutputReverse(-0.15);
        headingPid.setAtTargetThreshold(Math.toRadians(0.5));

        sidePid = new PIDF(0.04, 0, 0, 0);
        sidePid.setAtTargetThreshold(10);
        sidePid.setPhase(false);
        sidePid.setPeakOutputForward(0.25);
        sidePid.setPeakOutputReverse(-0.25);
        sidePid.setNominalOutputForward(0.15);
        sidePid.setNominalOutputReverse(-0.15);

        telemetry.addLine("Initializing Hardware Acceleration...");
        telemetry.update();
        RenderScriptContextManager.getInstance().init();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.addData("Update Interval", telemetry.getMsTransmissionInterval());
            telemetry.addData("Front Distance", asm.getFrontDistance());
            telemetry.addData("Ultrasonic Distance", asm.getUltrasonicDistance());
            telemetry.update();
        }
        if (isStopRequested()) return;
        odometer.resetOdometer();

        vision.beginProcessing();
        while (!isStopRequested() && !vision.isProcessingComplete()) {
            telemetry.addLine("Processing...");
            telemetry.update();
        }
        if (isStopRequested()) return;

        stonePosition = vision.getStonePosition();

        telemetry.log().add("Stone Position: %s", stonePosition.toString());
        telemetry.update();

        FailsafeCondition condition = normalOperation();

        if (condition != FailsafeCondition.OK && condition != FailsafeCondition.STOP_REQUESTED) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            TimingUtilities.runAfterDelay(blinkin::defaultPattern, 500);
        }

        parkingTape.setPower(0);

        odometer.retract();

        if (condition == FailsafeCondition.STOP_REQUESTED) {
            return;
        } else if (condition == FailsafeCondition.QUARRY_SIDE) {
            rb.drive(0, 0, 0);
            {
                sidePid.setSetpoint(400);
                sidePid.setAtTargetThreshold(10);
                sidePid.setPeakOutputForward(0.2);
                sidePid.setPeakOutputReverse(-0.2);

                while (!isStopRequested()) {
                    rb.drive(sidePid.update(asm.getUltrasonicDistance()), 0, 0);

                    if (sidePid.atTarget()) {
                        break;
                    }
                }
            }
            if (isStopRequested()) return;

            fusion2.driveBackwardsToTape(this, null);
            if (isStopRequested()) return;
        } else if (condition == FailsafeCondition.ULTRASONIC_FAILURE) {
            rb.drive(0, 0, 0);

            ElapsedTime timer = new ElapsedTime();
            fusion2.driveBackwardsToTape(this, () -> {
                if (timer.seconds() > 10) {
                    requestOpModeStop();
                }
            });
            if (isStopRequested()) return;
        }

        TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
    }

    private FailsafeCondition normalOperation() {
        stoneArm.release();
        stoneArm.retract();

        {
            double y = 0;
            if (stonePosition == StonePosition.LEFT) {
                y = 200;
            } else if (stonePosition == StonePosition.RIGHT) {
                y = -250;
            }
            fusion2.drive(this, new Vector2(650, y), null, 1);
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        {
            double sideDistance = 716;
            double frontDistance = 710;
            if (stonePosition == StonePosition.LEFT) {
                frontDistance = 710;
            } else if (stonePosition == StonePosition.CENTER) {
                frontDistance = 920;
            } else if (stonePosition == StonePosition.RIGHT) {
                frontDistance = 1130;
            }

            sidePid.setSetpoint(sideDistance);

            double approachDirection = (asm.getFrontDistance() > frontDistance) ? 1 : -1;

            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
            /*while (!isStopRequested() && asm.getWallLaser1Distance() > 2000) {
                rb.drive(0, 0.5, headingPid.update());
            }
            if (isStopRequested()) {
                rb.drive(0, 0, 0);
                return;
            }*/

            boolean targetReached = false;
            while (!isStopRequested()) {
                double laserDistance = asm.getFrontDistance();
                double distance = -(frontDistance - laserDistance);
                double ultrasonicDistance = odometer.getOdometerPosition();

                double xCorrection = sidePid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", sidePid.atTarget());
                telemetry.update();

                double speed = 0.7;

                double effectiveSpeed = speed * approachDirection;
                double nominalSpeed = 0.1;
                double slowdownPoint = 400;
                double endCurvePoint = 200;
                double absDistance = Math.abs(distance);
                if (absDistance < slowdownPoint && absDistance > endCurvePoint) {
                    effectiveSpeed = MathUtilities.map(absDistance, slowdownPoint, endCurvePoint, effectiveSpeed, Math.copySign(nominalSpeed, effectiveSpeed));
                } else if (absDistance < endCurvePoint) {
                    effectiveSpeed = Math.copySign(nominalSpeed, effectiveSpeed);
                }

                if ((laserDistance > 8000 && ultrasonicDistance > 500) || laserDistance > 30000) {
                    telemetry.log().add("Sanity Check Failure: Wall Laser 1: %f", laserDistance);
                    telemetry.update();
                    return FailsafeCondition.QUARRY_SIDE;
                }

                if (ultrasonicDistance < 250) {
                    telemetry.log().add("Sanity Check Failure: Ultrasonic: %f", ultrasonicDistance);
                    telemetry.update();
                    return FailsafeCondition.ULTRASONIC_FAILURE;
                }

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (sidePid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.release();
        stoneArm.lower();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.grab();

        TimingUtilities.sleep(this, 0.3, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        /*{
            sidePid.setSetpoint(590);
            sidePid.setAtTargetThreshold(10);
            sidePid.setPeakOutputForward(0.5);
            sidePid.setPeakOutputReverse(-0.5);

            while (!isStopRequested()) {
                rb.drive(sidePid.update(asm.getUltrasonicDistance()), 0, 0);

                if (sidePid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        if (stonePosition != StonePosition.RIGHT) {
            fusion2.drive(this, new Vector2(0, -2650 + asm.getFrontDistance()), null, 1);
        } else {
            fusion2.drive(this, new Vector2(0, -2650 + 1130), null, 1);
        }
        rb.drive(0, 0, 0);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(200, 0), null, 0.5);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.place();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.release();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.retract();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(-200, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        /*{
            sidePid.setSetpoint(590);
            sidePid.setAtTargetThreshold(10);

            while (!isStopRequested()) {
                rb.drive(sidePid.update(asm.getUltrasonicDistance()), 0, 0);

                if (sidePid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        stoneArm.release();
        {
            double sideDistance = 680;
            double frontDistance = 107;

            if (stonePosition == StonePosition.LEFT) {
                frontDistance = 107;
                //sideDistance = 600;
            } else if (stonePosition == StonePosition.CENTER) {
                frontDistance = 312;
            } else {
                frontDistance = 525;
            }

            sidePid.setSetpoint(sideDistance);
            sidePid.setPeakOutputForward(0.25);
            sidePid.setPeakOutputReverse(-0.25);

            double approachDirection = (asm.getFrontDistance() > frontDistance) ? 1 : -1;

            ElapsedTime failureTimer = new ElapsedTime();
            failureTimer.reset();

            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
            while (!isStopRequested() && (failureTimer.seconds() < 1 || asm.getFrontDistance() > 2000)) {
                if (failureTimer.seconds() > 10) {
                    return FailsafeCondition.QUARRY_SIDE;
                }
                rb.drive(0, 1, headingPid.update());
            }
            if (isStopRequested()) {
                rb.drive(0, 0, 0);
                return FailsafeCondition.STOP_REQUESTED;
            }

            //sidePid.setAtTargetThreshold(70);

            while (!isStopRequested()) {
                double laserDistance = asm.getFrontDistance();
                double distance = -(frontDistance - laserDistance);
                double sideDistanceReading = odometer.getOdometerPosition();

                double xCorrection = sidePid.update(sideDistanceReading);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", sideDistanceReading);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", sidePid.atTarget());
                telemetry.update();

                double speed = 0.7;

                double effectiveSpeed = speed * approachDirection;
                double nominalSpeed = 0.1;
                double slowdownPoint = 600;
                double endCurvePoint = 200;
                double absDistance = Math.abs(distance);
                if (absDistance < slowdownPoint && absDistance > endCurvePoint) {
                    effectiveSpeed = MathUtilities.map(absDistance, slowdownPoint, endCurvePoint, effectiveSpeed, Math.copySign(nominalSpeed, effectiveSpeed));
                } else if (absDistance < endCurvePoint) {
                    effectiveSpeed = Math.copySign(nominalSpeed, effectiveSpeed);
                }

                if (laserDistance > 8000) {
                    telemetry.log().add("Sanity Check Failure: Front Laser");
                    telemetry.update();
                    return FailsafeCondition.QUARRY_SIDE;
                }

                if (sideDistanceReading < 200) {
                    telemetry.log().add("Sanity Check Failure: Ultrasonic: %f", sideDistanceReading);
                    telemetry.update();
                    return FailsafeCondition.ULTRASONIC_FAILURE;
                }

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (sidePid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        odometer.retract();

        // grab second stone

        stoneArm.lower();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.grab();

        TimingUtilities.sleep(this, 0.3, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        /*{
            sidePid.setSetpoint(590);
            sidePid.setAtTargetThreshold(10);
            sidePid.setPeakOutputForward(0.5);
            sidePid.setPeakOutputReverse(-0.5);

            while (!isStopRequested()) {
                rb.drive(sidePid.update(asm.getUltrasonicDistance()), 0, 0);

                if (sidePid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, -2900 + asm.getFrontDistance()), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(200, 0), null, 0.5);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.place();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.release();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        foundationClaw.ready();
        fusion2.drive(this, new Vector2(-100, 0), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.turnToHeading(this, 0.5 * Math.PI, foundationClaw::update);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        //fusion2.drive(this, new Vector2(300, 0), foundationClaw::update, 1);
        //if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, -150), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        foundationClaw.extend();

        TimingUtilities.sleep(this, 0.15, foundationClaw::update, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        //fusion2.driveForwardsToWall(this, foundationClaw::update);
        //if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 475), foundationClaw::update, 0.6);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            headingPid.setSetpoint(0);
            headingPid.setAtTargetThreshold(Math.toRadians(10));
            while (!isStopRequested()) {
                foundationClaw.update();
                // idea: stop driving forwards after turning a certain amount?
                rb.drive(0, 0.4, headingPid.update());
                if (headingPid.atTarget()) {
                    break;
                }
            }
            headingPid.setAtTargetThreshold(Math.toRadians(0.5));
        }
        rb.drive(0, 0, 0);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        foundationClaw.retract();
        parkingTape.setPower(1);

        TimingUtilities.runAfterDelay(() -> {
            parkingTape.setPower(0);
        }, 5000);

        fusion2.drive(this, new Vector2(325, 0), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, -550), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(260, 0), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, 675), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, 270), null, 0.35);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        return FailsafeCondition.OK;
    }
}
