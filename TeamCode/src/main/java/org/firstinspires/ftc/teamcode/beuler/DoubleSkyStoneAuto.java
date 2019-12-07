package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Fusion2;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Double SkyStone")
@Disabled
public class DoubleSkyStoneAuto extends LinearOpMode {

    private enum StonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private MecanumDrive rb;
    private Arm arm;
    private Fusion2 fusion2;
    private VuMarkNavSource vuforia;
    private Intake intake;
    private FoundationClaw claw;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(false, false, true, true);
        rb.resetEncoders();

        arm = new Arm(hardwareMap);

        fusion2 = new Fusion2(hardwareMap, rb);

        if (fusion2.getNavSource().isFallback()) {
            telemetry.log().add("Warning: Primary IMU failure, using fallback!");
            telemetry.update();
        }

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);

        intake = new Intake(hardwareMap);

        claw = new FoundationClaw(hardwareMap);

        timer = new ElapsedTime();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }
        timer.reset();

        arm.holdCapstone();

        claw.homeEncoder();
        claw.retract();
        claw.setClosedLoopEnabled(true);

        arm.setTargetPosition(3);
        arm.setClosedLoopEnabled(true);

        fusion2.drive(this, new Vector2(0, -350), this::update, 0.7);
        if (isStopRequested()) return;

        if (!vuforia.canSeeStone()) {
            telemetry.log().add("Cannot find stone.");
            telemetry.update();
            TimingUtilities.blockUntil(this, vuforia::canSeeStone, this::update, null);
            if (isStopRequested()) return;
        }

        double stoneOffset;
        {
            OpenGLMatrix stoneLocation = vuforia.getRelativeStoneLocation();
            if (stoneLocation == null) {
                telemetry.log().add("If you see this, something terrible has happened");
                telemetry.update();
                TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
                return;
            }
            VectorF translation = stoneLocation.getTranslation();
            stoneOffset = translation.get(0);
        }

        fusion2.drive(this, new Vector2(0, -200), this::update, 0.7);
        if (isStopRequested()) return;

        double wallDistance = fusion2.getASM().getLeftDistance();

        StonePosition stoneLocation = StonePosition.RIGHT;

        double vuforiaOffset = 10 * fusion2.getASM().getLeftDistance() + stoneOffset;
        if (wallDistance > 150 || vuforiaOffset < 300) {
            telemetry.log().add("Sanity Check failed. Compensating...");
            telemetry.update();
            if (stoneOffset > 100) {
                stoneLocation = StonePosition.LEFT;
            } else if (stoneOffset > -50) {
                stoneLocation = StonePosition.CENTER;
            } else {
                stoneLocation = StonePosition.RIGHT;
            }
        } else {
            telemetry.log().add("Vuforia Stone Offset: %f", stoneOffset);
            stoneOffset = vuforiaOffset;
            if (stoneOffset > 850) {
                stoneLocation = StonePosition.LEFT;
            } else if (stoneOffset > 650) {
                stoneLocation = StonePosition.CENTER;
            } else {
                stoneLocation = StonePosition.RIGHT;
            }
        }

        telemetry.log().add("Wall Distance: %f", wallDistance);
        telemetry.log().add("Stone Offset: %f", stoneOffset);
        telemetry.log().add("Stone Location: %s", stoneLocation.toString());
        telemetry.update();

        if (stoneLocation != StonePosition.LEFT) {
            fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
            if (isStopRequested()) return;

            if (fusion2.getASM().getUltrasonicDistance() < 200) {
                RobotLog.setGlobalWarningMessage("Sanity Check Failed. Localization lost. Abort.");
                telemetry.update();
                arm.setShoulderPower(0);
                claw.setPower(0);
                TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
                return;
            }

            if (stoneLocation == StonePosition.CENTER) {
                fusion2.driveToWallDistance(this, 820, this::update, 0.6);
            } else {
                fusion2.driveToWallDistance(this, 690, this::update, 0.6);
            }
            if (isStopRequested()) return;

            arm.release();

            //fusion2.drive(this, new Vector2(-200, 0), this::update, 0.5);
            //if (isStopRequested()) return;

            rb.drive(-0.6, 0, 0);
            TimingUtilities.blockUntil(this, () -> !(Double.isNaN(fusion2.getASM().getWall0Distance()) && Double.isNaN(fusion2.getASM().getWall1Distance())), () -> {
                rb.drive(-0.6, 0, fusion2.headingPid.update());
                update();
            }, null);
            rb.drive(0, 0, 0);
            if (isStopRequested()) return;

            if (!Double.isNaN(fusion2.getASM().getWall0Distance())) {
                fusion2.drive(this, new Vector2(-150, 0), this::update, 0.6);
            } else {
                fusion2.drive(this, new Vector2(250, 0), this::update, 0.6);
            }
            if (isStopRequested()) return;

            {
                double startTime = timer.seconds();
                intake.intake(1);

                rb.drive(0, 0.4, 0);
                TimingUtilities.blockUntil(this, () -> timer.seconds() - startTime > 1.5 || fusion2.getASM().isStonePresent(), () -> {
                    rb.drive(0, 0.4, 0);
                    update();
                }, null);
                rb.drive(0, 0, 0);
                if (isStopRequested()) return;
                if (timer.seconds() - startTime > 1.5) {
                    telemetry.addLine("Intake Sequence Failure.");
                    telemetry.update();
                    intake.stop();
                    TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
                    return;
                }
            }

            arm.setTargetPosition(3.3);

            TimingUtilities.runAfterDelay(() -> {
                intake.stop();
            }, 500);

            fusion2.drive(this, new Vector2(0, -100), this::update, 1);
            if (isStopRequested()) return;

            arm.grab();

            fusion2.turnToHeading(this, 0, this::update);
            if (isStopRequested()) return;

            fusion2.drive(this, new Vector2(0, 520), this::update, 0.7);
            if (isStopRequested()) return;
        }

        fusion2.turnToHeading(this, -0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        {
            final double armTarget = 0.7;
            AtomicBoolean armTargetUpdated = new AtomicBoolean(false);
            fusion2.drive(this, new Vector2(0, 1500), () -> {
                telemetry.addData("Arm Updated", armTargetUpdated.get());
                update();
                /*if (!armTargetUpdated.get()) {
                    double ultrasonicDistance = fusion2.getASM().getUltrasonicDistance();
                    if (ultrasonicDistance != 0 && ultrasonicDistance < 1100) {
                        armTargetUpdated.set(true);
                        arm.setTargetPosition(armTarget);
                    }
                }*/
            }, 1);
            if (isStopRequested()) return;

            if (!armTargetUpdated.get()) {
                arm.setTargetPosition(armTarget);
            }
        }

        TimingUtilities.sleep(this, 0.25, this::update, null);
        if (isStopRequested()) return;

        if (fusion2.getASM().getUltrasonicDistance() < 200) {
            telemetry.log().add("Sanity Check Failed. Localization lost. Abort.");
            telemetry.update();
            arm.setShoulderPower(0);
            claw.setPower(0);
            TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
            return;
        }

        fusion2.driveToWallDistance(this, 300, this::update);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, 0, this::update);
        if (isStopRequested()) return;

        fusion2.driveToWallDistance(this, 720, this::update);
        if (isStopRequested()) return;

        arm.release();
        claw.extend();

        TimingUtilities.runAfterDelay(() -> {
            arm.setTargetPosition(3.2);
        }, 250);

        TimingUtilities.sleep(this, 0.25, this::update, null);
        if (isStopRequested()) return;

        fusion2.driveForwardsToWall(this, this::update);
        if (isStopRequested()) return;

        claw.retract();
        fusion2.drive(this, new Vector2(-850, 0), this::update, 1);
        if (isStopRequested()) return;

        //fusion2.driveToWallDistance(this, 430, this::update);
        fusion2.drive(this, new Vector2(0, -360), this::update, 1);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(100, 1200), this::update, 1);
        if (isStopRequested()) return;

        switch (stoneLocation) {
            case LEFT:
                fusion2.driveToWallDistance(this, 700, this::update, 0.6);
                break;
            case CENTER:
                fusion2.driveToWallDistance(this, 540, this::update, 0.6);
                break;
            case RIGHT:
                fusion2.driveToWallDistance(this, 350, this::update, 0.6);
                break;
        }
        if (isStopRequested()) return;

        rb.drive(-0.6, 0, 0);
        TimingUtilities.blockUntil(this, () -> !(Double.isNaN(fusion2.getASM().getWall0Distance()) && Double.isNaN(fusion2.getASM().getWall1Distance())), () -> {
            rb.drive(-0.6, 0, fusion2.headingPid.update());
            update();
        }, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        if (!Double.isNaN(fusion2.getASM().getWall0Distance())) {
            fusion2.drive(this, new Vector2(-200, 0), this::update, 1);
        } else {
            fusion2.drive(this, new Vector2(200, 0), this::update, 1);
        }
        if (isStopRequested()) return;

        intake.intake(1);

        rb.drive(0, 0.5, 0);
        TimingUtilities.blockUntil(this, fusion2.getASM()::isStonePresent, () -> {
            rb.drive(0, 0.5, 0);
            update();
        }, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        arm.setTargetPosition(3.4);

        TimingUtilities.runAfterDelay(() -> {
            intake.stop();
            //arm.grab();
        }, 500);

        fusion2.drive(this, new Vector2(550, 0), this::update, 0.7);
        if (isStopRequested()) return;

        arm.setTargetPosition(3.15);

        //fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
        //if (isStopRequested()) return;

        //fusion2.driveToWallDistance(this, 670, this::update, 0.6);
        //if (isStopRequested()) return;

        if (stoneLocation == StonePosition.LEFT) {

        } else if (stoneLocation == StonePosition.CENTER) {
            fusion2.drive(this, new Vector2(0, -1900), this::update, 1);
        } else if (stoneLocation == StonePosition.RIGHT) {
            fusion2.drive(this, new Vector2(0, -2100), this::update, 1);
        }
        if (isStopRequested()) return;

        rb.drive(0, -1, 0);
        TimingUtilities.sleep(this, 0.25, this::update, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 450), this::update, 1);
        if (isStopRequested()) return;

        TimingUtilities.blockUntil(this, this::isStopRequested, this::update, null);
    }

    private void update() {
        arm.update();
        claw.update();
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
    }
}
