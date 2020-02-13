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

@Autonomous(name = "Red Skystone", group = "B")
public class RedSideStoneAuto extends LinearOpMode {

    private enum FailsafeCondition {
        OK,
        STOP_REQUESTED,
        QUARRY_SIDE,
        BUILD_SIDE
    }

    private MecanumDrive rb;
    private Fusion2 fusion2;
    private VuMarkNavSource vuforia;
    private StoneArm stoneArm;
    private FoundationClaw foundationClaw;

    private StoneVision vision;

    private Blinkin blinkin;

    private ParkingTape parkingTape;

    private AutonomousSensorManager asm;
    private PIDF headingPid, ultrasonicPid;

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

        asm = fusion2.getASM();
        headingPid = fusion2.headingPid;
        //headingPid.setNominalOutputForward(0.15);
        //headingPid.setNominalOutputReverse(-0.15);
        headingPid.setAtTargetThreshold(Math.toRadians(0.5));

        ultrasonicPid = new PIDF(0.05, 0, 0, 0);
        ultrasonicPid.setAtTargetThreshold(10);
        ultrasonicPid.setPhase(false);
        ultrasonicPid.setPeakOutputForward(0.25);
        ultrasonicPid.setPeakOutputReverse(-0.25);
        ultrasonicPid.setNominalOutputForward(0.15);
        ultrasonicPid.setNominalOutputReverse(-0.15);

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

        if (condition == FailsafeCondition.STOP_REQUESTED) {
            return;
        } else if (condition == FailsafeCondition.QUARRY_SIDE) {
            rb.drive(0, 0, 0);
            {
                ultrasonicPid.setSetpoint(400);
                ultrasonicPid.setAtTargetThreshold(10);
                ultrasonicPid.setPeakOutputForward(0.2);
                ultrasonicPid.setPeakOutputReverse(-0.2);

                while (!isStopRequested()) {
                    rb.drive(ultrasonicPid.update(asm.getUltrasonicDistance()), 0, 0);

                    if (ultrasonicPid.atTarget()) {
                        break;
                    }
                }
            }
            if (isStopRequested()) return;

            fusion2.driveBackwardsToTape(this, null);
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
            double sideDistance = 427;
            double frontDistance = 710;
            if (stonePosition == StonePosition.LEFT) {
                frontDistance = 710;
            } else if (stonePosition == StonePosition.CENTER) {
                frontDistance = 920;
            } else if (stonePosition == StonePosition.RIGHT) {
                frontDistance = 1130;
            }

            ultrasonicPid.setSetpoint(sideDistance);

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
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
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

                if ((laserDistance > 8000 && ultrasonicDistance > 400) || laserDistance > 30000) {
                    telemetry.log().add("Sanity Check Failure: Wall Laser 1: %f", laserDistance);
                    telemetry.update();
                    return FailsafeCondition.QUARRY_SIDE;
                }

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (ultrasonicPid.atTarget() || xCorrection == 0) {
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
            ultrasonicPid.setSetpoint(590);
            ultrasonicPid.setAtTargetThreshold(10);
            ultrasonicPid.setPeakOutputForward(0.5);
            ultrasonicPid.setPeakOutputReverse(-0.5);

            while (!isStopRequested()) {
                rb.drive(ultrasonicPid.update(asm.getUltrasonicDistance()), 0, 0);

                if (ultrasonicPid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            double frontDistance = asm.getFrontDistance();
            if (frontDistance < 8000) {
                if (stonePosition != StonePosition.RIGHT) {
                    fusion2.drive(this, new Vector2(0, -2000 + frontDistance), null, 1);
                } else {
                    fusion2.drive(this, new Vector2(0, -2000 + 1130), null, 1);
                }
            } else {
                if (stonePosition == StonePosition.LEFT) {
                    fusion2.drive(this, new Vector2(0, -2000 + 710), null, 1);
                } else if (stonePosition == StonePosition.CENTER) {
                    fusion2.drive(this, new Vector2(0, -2000 + 920), null, 1);
                } else {
                    fusion2.drive(this, new Vector2(0, -2000 + 1130), null, 1);
                }
            }
            rb.drive(0, 0, 0);
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        //fusion2.drive(this, new Vector2(200, 0), null, 0.5);
        //if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.place();

        TimingUtilities.sleep(this, 0.25, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.release();

        TimingUtilities.sleep(this, 0.25, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.retract();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        //fusion2.drive(this, new Vector2(-200, 0), null, 1);
        //if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        /*{
            ultrasonicPid.setSetpoint(590);
            ultrasonicPid.setAtTargetThreshold(10);

            while (!isStopRequested()) {
                rb.drive(ultrasonicPid.update(asm.getUltrasonicDistance()), 0, 0);

                if (ultrasonicPid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        stoneArm.release();
        {
            double sideDistance = 427;
            double frontDistance = 107;

            if (stonePosition == StonePosition.LEFT) {
                frontDistance = 107;
                //sideDistance = 600;
            } else if (stonePosition == StonePosition.CENTER) {
                frontDistance = 312;
            } else {
                frontDistance = 525;
            }

            ultrasonicPid.setSetpoint(sideDistance);
            ultrasonicPid.setPeakOutputForward(0.25);
            ultrasonicPid.setPeakOutputReverse(-0.25);

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

            //ultrasonicPid.setAtTargetThreshold(70);

            while (!isStopRequested()) {
                double laserDistance = asm.getFrontDistance();
                double distance = -(frontDistance - laserDistance);
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
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

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (ultrasonicPid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        // grab second stone

        stoneArm.lower();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.grab();

        TimingUtilities.sleep(this, 0.3, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        /*{
            ultrasonicPid.setSetpoint(590);
            ultrasonicPid.setAtTargetThreshold(10);
            ultrasonicPid.setPeakOutputForward(0.5);
            ultrasonicPid.setPeakOutputReverse(-0.5);

            while (!isStopRequested()) {
                rb.drive(ultrasonicPid.update(asm.getUltrasonicDistance()), 0, 0);

                if (ultrasonicPid.atTarget()) {
                    break;
                }
            }
        }
        if (isStopRequested()) return;*/

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            double frontDistance = asm.getFrontDistance();
            if (frontDistance < 8000) {
                fusion2.drive(this, new Vector2(0, -2000 + frontDistance), null, 1);
            } else {
                if (stonePosition == StonePosition.LEFT) {
                    fusion2.drive(this, new Vector2(0, -2000 + 107), null, 1);
                } else if (stonePosition == StonePosition.CENTER) {
                    fusion2.drive(this, new Vector2(0, -2000 + 312), null, 1);
                } else {
                    fusion2.drive(this, new Vector2(0, -2000 + 525), null, 1);
                }
            }
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        //TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
        //if (isStopRequested()) {
        //    return FailsafeCondition.STOP_REQUESTED;
        //}

        stoneArm.place();

        TimingUtilities.sleep(this, 0.25, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.release();

        TimingUtilities.sleep(this, 0.25, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        {
            double sideDistance = 427;
            double frontDistance = 1130;

            if (stonePosition == StonePosition.RIGHT) {
                frontDistance = 910;
                //sideDistance = 600;
            }

            ultrasonicPid.setSetpoint(sideDistance);
            ultrasonicPid.setPeakOutputForward(0.25);
            ultrasonicPid.setPeakOutputReverse(-0.25);

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

            //ultrasonicPid.setAtTargetThreshold(70);

            while (!isStopRequested()) {
                double laserDistance = asm.getFrontDistance();
                double distance = -(frontDistance - laserDistance);
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
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

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (ultrasonicPid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        // grab second stone

        stoneArm.lower();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.grab();

        TimingUtilities.sleep(this, 0.3, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            double frontDistance = asm.getFrontDistance();
            if (frontDistance < 8000) {
                fusion2.drive(this, new Vector2(0, -2000 + frontDistance), null, 1);
            } else {
                if (stonePosition == StonePosition.RIGHT) {
                    fusion2.drive(this, new Vector2(0, -2000 + 910), null, 1);
                } else {
                    fusion2.drive(this, new Vector2(0, -2000 + 1130), null, 1);
                }
            }
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        stoneArm.release();

        {
            double sideDistance = 427;
            double frontDistance = 910;

            if (stonePosition == StonePosition.RIGHT) {
                frontDistance = 720;
                //sideDistance = 600;
            }

            ultrasonicPid.setSetpoint(sideDistance);
            ultrasonicPid.setPeakOutputForward(0.25);
            ultrasonicPid.setPeakOutputReverse(-0.25);

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

            //ultrasonicPid.setAtTargetThreshold(70);

            while (!isStopRequested()) {
                double laserDistance = asm.getFrontDistance();
                double distance = -(frontDistance - laserDistance);
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
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

                if (distance * approachDirection < 20) {
                    effectiveSpeed = 0;
                    if (ultrasonicPid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        // grab second stone

        stoneArm.lower();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.grab();

        TimingUtilities.sleep(this, 0.3, null, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        stoneArm.lift();

        fusion2.drive(this, new Vector2(-150, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            double frontDistance = asm.getFrontDistance();
            if (frontDistance < 8000) {
                fusion2.drive(this, new Vector2(0, -2000 + frontDistance), null, 1);
            } else {
                if (stonePosition == StonePosition.RIGHT) {
                    fusion2.drive(this, new Vector2(0, -2000 + 910), null, 1);
                } else {
                    fusion2.drive(this, new Vector2(0, -2000 + 720), null, 1);
                }
            }
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        stoneArm.release();

        fusion2.drive(this, new Vector2(0, 500), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        //TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
        //if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        return FailsafeCondition.OK;
    }
}
