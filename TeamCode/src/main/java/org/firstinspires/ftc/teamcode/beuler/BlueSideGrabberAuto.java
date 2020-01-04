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
import org.firstinspires.ftc.teamcode.StonePosition;
import org.firstinspires.ftc.teamcode.StoneVision;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Blue Skystone & Foundation", group = "C")
public class BlueSideGrabberAuto extends LinearOpMode {

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

        vision = new StoneVision(vuforia.getVuforiaLocalizer(), StoneVision.Tunings.BLUE);

        asm = fusion2.getASM();
        headingPid = fusion2.headingPid;
        //headingPid.setNominalOutputForward(0.15);
        //headingPid.setNominalOutputReverse(-0.15);
        headingPid.setAtTargetThreshold(Math.toRadians(0.5));

        ultrasonicPid = new PIDF(0.05, 0, 0, 0);
        ultrasonicPid.setAtTargetThreshold(70);
        ultrasonicPid.setPhase(false);
        ultrasonicPid.setPeakOutputForward(0.25);
        ultrasonicPid.setPeakOutputReverse(-0.25);
        ultrasonicPid.setNominalOutputForward(0.15);
        ultrasonicPid.setNominalOutputReverse(-0.15);

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

        if (condition == FailsafeCondition.STOP_REQUESTED) {
            return;
        } else if (condition == FailsafeCondition.QUARRY_SIDE) {
            rb.drive(0, 0, 0);
            {
                ultrasonicPid.setSetpoint(590);
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

            fusion2.driveForwardsToTape(this, null);
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
                y = 120;
            } else if (stonePosition == StonePosition.CENTER) {
                y = -165;
            } else if (stonePosition == StonePosition.RIGHT) {
                y = -220;
            }
            fusion2.drive(this, new Vector2(700, y), null, 1);
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        {
            double sideDistance = 630;
            double backDistance = 650;
            if (stonePosition == StonePosition.LEFT) {
                backDistance = 650;
                sideDistance = 633;
            } else if (stonePosition == StonePosition.CENTER) {
                backDistance = 370;
            } else if (stonePosition == StonePosition.RIGHT) {
                backDistance = 380;
            }

            ultrasonicPid.setSetpoint(sideDistance);

            double approachDirection = (asm.getBackDistance() > backDistance) ? 1 : -1;

            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
            /*while (!isStopRequested() && asm.getWallLaser1Distance() > 2000) {
                rb.drive(0, 0.5, headingPid.update());
            }
            if (isStopRequested()) {
                rb.drive(0, 0, 0);
                return;
            }*/

            while (!isStopRequested()) {
                double laserDistance = asm.getBackDistance();
                double distance = -(backDistance - laserDistance);
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Front Distance", laserDistance);
                telemetry.addData("Front Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
                telemetry.update();

                double speed = -0.7;

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
                    telemetry.log().add("Sanity Check Failure: Wall Laser 1");
                    telemetry.update();
                    return FailsafeCondition.QUARRY_SIDE;
                }

                if (distance * approachDirection < 20 || stonePosition == StonePosition.CENTER || stonePosition == StonePosition.LEFT) {
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

        fusion2.drive(this, new Vector2(0, 2350 - asm.getBackDistance()), null, 1);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(250, 0), null, 0.7);
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

        fusion2.drive(this, new Vector2(-250, 0), null, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
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
            double laserDistance = asm.getFrontDistance();
            if (laserDistance < 2000) {
                fusion2.drive(this, new Vector2(0, -1800 + asm.getFrontDistance()), () -> {
                    telemetry.addData("Back Distance", asm.getBackDistance());
                    telemetry.update();
                }, 1, false);
            } else {
                fusion2.drive(this, new Vector2(0, -1200), () -> {
                    telemetry.addData("Back Distance", asm.getBackDistance());
                    telemetry.update();
                }, 1, false);
            }
            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
        }

        {
            double sideDistance = 595;
            double backDistance = 200;

            ultrasonicPid.setSetpoint(sideDistance);
            ultrasonicPid.setPeakOutputForward(0.25);
            ultrasonicPid.setPeakOutputReverse(-0.25);

            double approachDirection = (asm.getBackDistance() > backDistance) ? 1 : -1;

            ElapsedTime failureTimer = new ElapsedTime();
            failureTimer.reset();

            if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;
            while (!isStopRequested() && (failureTimer.seconds() < 1 || asm.getBackDistance() > 2000)) {
                telemetry.addData("Back Distance", asm.getBackDistance());
                telemetry.update();
                if (failureTimer.seconds() > 3) {
                    return FailsafeCondition.QUARRY_SIDE;
                }
                rb.drive(0, -0.7, headingPid.update());
            }
            if (isStopRequested()) {
                rb.drive(0, 0, 0);
                return FailsafeCondition.STOP_REQUESTED;
            }

            ultrasonicPid.setAtTargetThreshold(70);

            while (!isStopRequested()) {
                double laserDistance = asm.getBackDistance();
                double distance = -(backDistance - laserDistance);
                double ultrasonicDistance = asm.getUltrasonicDistance();

                double xCorrection = ultrasonicPid.update(ultrasonicDistance);

                telemetry.addData("Back Distance", laserDistance);
                telemetry.addData("Back Error", distance);
                telemetry.addData("Side Distance", ultrasonicDistance);
                telemetry.addData("X Correction", xCorrection);
                telemetry.addData("X At Target", ultrasonicPid.atTarget());
                telemetry.update();

                double speed = -0.5;

                double effectiveSpeed = speed;
                double nominalSpeed = 0.1;
                double slowdownPoint = 600;
                double endCurvePoint = 200;
                double absDistance = Math.abs(distance);
                if (absDistance < slowdownPoint && absDistance > endCurvePoint) {
                    effectiveSpeed = MathUtilities.map(absDistance, slowdownPoint, endCurvePoint, effectiveSpeed, Math.copySign(nominalSpeed, effectiveSpeed));
                } else if (absDistance < endCurvePoint) {
                    effectiveSpeed = Math.copySign(nominalSpeed, effectiveSpeed);
                }

                if ((laserDistance > 8000 && ultrasonicDistance > 500) || laserDistance > 30000) {
                    telemetry.log().add("Sanity Check Failure: Wall Laser 1");
                    telemetry.update();
                    return FailsafeCondition.QUARRY_SIDE;
                }

                if (distance < 20 || asm.getBackDistance() < backDistance + 20) {
                    effectiveSpeed = 0;
                    if (ultrasonicPid.atTarget() || xCorrection == 0) {
                        break;
                    }
                }

                rb.drive(xCorrection, effectiveSpeed, headingPid.update());
            }
            //rb.drive(0, -0.7, 0);
            //TimingUtilities.blockUntil(this, () -> asm.getBackDistance() < 1000, null, null);
            rb.drive(0, 0, 0);
        }
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(150, 75), null, 1);
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

        fusion2.drive(this, new Vector2(0, 2600 - asm.getBackDistance()), null, 1);
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

        fusion2.drive(this, new Vector2(150, 0), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, -300), foundationClaw::update, 0.5);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        foundationClaw.extend();

        TimingUtilities.sleep(this, 0.3, foundationClaw::update, null);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        //fusion2.driveForwardsToWall(this, foundationClaw::update);
        //if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 375), foundationClaw::update, 0.6);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        {
            headingPid.setSetpoint(Math.PI);
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
        fusion2.drive(this, new Vector2(0, -550), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(-50, 0), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, 675), foundationClaw::update, 1);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        fusion2.drive(this, new Vector2(0, 270), null, 0.35);
        if (isStopRequested()) return FailsafeCondition.STOP_REQUESTED;

        return FailsafeCondition.OK;
    }
}
