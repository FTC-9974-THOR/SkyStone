package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Red SkyStone", group = "SkyStone")
public class RedSkystoneAuto extends LinearOpMode {

    private enum StonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private MecanumDrive rb;
    private Arm arm;
    private Fusion2 fusion2;
    private VuMarkNavSource vuforia;
    private FoundationClaw claw;
    private Intake intake;
    private ElapsedTime timer;

    private Blinkin blinkin;

    private StonePosition stoneLocation = StonePosition.RIGHT;

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

        blinkin = new Blinkin(hardwareMap);

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }
        timer.reset();

        arm.holdCapstone();
        arm.release();

        claw.setClosedLoopEnabled(false);
        claw.setPower(0.3);
        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::isStalled).withMinimumTime(0.1), this::update, null);
        claw.setPower(0);
        if (isStopRequested()) return;

        claw.homeEncoder();
        claw.retract();
        claw.setClosedLoopEnabled(true);

        arm.setTargetPosition(3);
        arm.setClosedLoopEnabled(true);

        blinkin.lamp();

        fusion2.drive(this, new Vector2(0, -350), this::update, 0.4);
        if (isStopRequested()) return;

        boolean stoneFound = vuforia.canSeeStone();
        if (!stoneFound) {
            telemetry.log().add("Cannot find stone.");
            telemetry.update();
            TimingUtilities.blockUntil(this, ((CompositeFunction) vuforia::canSeeStone).withTimeout(2), this::update, null);
            if (isStopRequested()) return;
            stoneFound = vuforia.canSeeStone();
        }

        double stoneOffset = 0;
        if (stoneFound) {
            OpenGLMatrix stoneLocation = vuforia.getRelativeStoneLocation();
            if (stoneLocation == null) {
                telemetry.log().add("If you see this, something terrible has happened");
                telemetry.update();
                TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
                return;
            }
            VectorF translation = stoneLocation.getTranslation();
            stoneOffset = -translation.get(0);
        }

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);

        fusion2.drive(this, new Vector2(0, -150), this::update, 0.7);
        if (isStopRequested()) return;


        double wallDistance = fusion2.getASM().getRightDistance();

        if (stoneFound) {
            double vuforiaOffset = 10 * fusion2.getASM().getRightDistance() + stoneOffset;
            if (wallDistance > 150 || vuforiaOffset < 300) {
                telemetry.log().add("Sanity Check failed. Compensating...");
                telemetry.update();
                if (stoneOffset > 100) {
                    stoneLocation = StonePosition.RIGHT;
                } else if (stoneOffset > -50) {
                    stoneLocation = StonePosition.CENTER;
                } else {
                    stoneLocation = StonePosition.LEFT;
                }
            } else {
                telemetry.log().add("Vuforia Stone Offset: %f", stoneOffset);
                stoneOffset = vuforiaOffset;
                if (stoneOffset > 850) {
                    stoneLocation = StonePosition.RIGHT;
                } else if (stoneOffset > 650) {
                    stoneLocation = StonePosition.CENTER;
                } else {
                    stoneLocation = StonePosition.LEFT;
                }
            }

            //telemetry.log().add("Wall Distance: %f", wallDistance);
            //telemetry.log().add("Stone Offset: %f", stoneOffset);
            telemetry.log().add("Stone Location: %s", stoneLocation.toString());
            telemetry.update();
        }

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, -0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        //telemetry.log().add("Wall Distance: %s", fusion2.getASM().getUltrasonicDistance());
        //telemetry.update();
        boolean ultrasonicSane = fusion2.getASM().getUltrasonicDistance() > 200;
        if (ultrasonicSane) {
            if (stoneLocation == StonePosition.RIGHT) {
                //fusion2.driveToWallDistance(this,  610, this::update, 0.6);
                fusion2.drive(this, new Vector2(0, -30), this::update, 0.7);
            } else if (stoneLocation == StonePosition.CENTER) {
                fusion2.driveToWallDistance(this, 840, this::update, 0.6);
            } else { // left
                fusion2.driveToWallDistance(this, 660, this::update, 0.6);
            }
        }

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        rb.drive(0.6, 0, 0);
        TimingUtilities.blockUntil(this, () -> !(Double.isNaN(fusion2.getASM().getWall0Distance()) && Double.isNaN(fusion2.getASM().getWall1Distance())), () -> {
            rb.drive(0.6, 0, fusion2.headingPid.update());
            update();
        }, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        if (!Double.isNaN(fusion2.getASM().getWall1Distance())) {
            fusion2.drive(this, new Vector2(220, 0), this::update, 0.6);
        } else {
            fusion2.drive(this, new Vector2(-200, 0), this::update, 0.6);
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
            }
            intake.stop();
        }

        fusion2.drive(this, new Vector2(-500, 0), this::update, 0.7);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -300), this::update, 0.7);
        if (isStopRequested()) return;

        fusion2.driveBackwardsToTape(this, this::update);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(400, 0), this::update, 0.7);
        if (isStopRequested()) return;

        intake.outtake(1);
        TimingUtilities.blockWhile(this, fusion2.getASM()::isStonePresent, this::update, intake::stop);
        if (isStopRequested()) return;
        TimingUtilities.sleep(this, 0.5, this::update, null);
        intake.stop();
        if (isStopRequested()) return;

        if (stoneLocation == StonePosition.RIGHT) {
            fusion2.driveLeftToTape(this, this::update);
            rb.drive(0, 0, 0);
            return;
        }

        fusion2.turnToHeading(this, -0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 800), this::update, 1);
        if (isStopRequested()) return;

        if (stoneLocation == StonePosition.CENTER) {
            fusion2.driveToWallDistance(this, 580, this::update, 0.6);
        } else {
            fusion2.driveToWallDistance(this, 450, this::update, 0.6);
        }
        if (isStopRequested()) return;

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        rb.drive(0.6, 0, 0);
        TimingUtilities.blockUntil(this, () -> !(Double.isNaN(fusion2.getASM().getWall0Distance()) && Double.isNaN(fusion2.getASM().getWall1Distance())), () -> {
            rb.drive(0.6, 0, fusion2.headingPid.update());
            update();
        }, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        if (!Double.isNaN(fusion2.getASM().getWall1Distance())) {
            fusion2.drive(this, new Vector2(200, 0), this::update, 0.6);
        } else {
            fusion2.drive(this, new Vector2(-200, 0), this::update, 0.6);
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
            }
            intake.stop();
        }

        fusion2.drive(this, new Vector2(-470, 0), this::update, 0.7);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -750), this::update, 1);
        if (isStopRequested()) return;

        fusion2.driveBackwardsToTape(this, this::update);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(500, 0), this::update, 0.7);
        if (isStopRequested()) return;

        intake.outtake(1);
        TimingUtilities.blockWhile(this, fusion2.getASM()::isStonePresent, this::update, intake::stop);
        if (isStopRequested()) return;
        TimingUtilities.sleep(this, 0.5, this::update, null);
        intake.stop();
        if (isStopRequested()) return;

        fusion2.driveLeftToTape(this, this::update);

        TimingUtilities.blockUntil(this, this::isStopRequested, this::update, null);
    }

    private void update() {
        arm.update();
        claw.update();
        telemetry.addData("Time", timer.seconds());
        telemetry.addData("Stone Position", stoneLocation);
        telemetry.update();
    }
}
