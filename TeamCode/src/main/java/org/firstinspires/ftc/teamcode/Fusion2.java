package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.beuler.AutonomousSensorManager;
import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

public class Fusion2 {

    private static final double MAX_SANE_DISTANCE = 7000;

    private IMUNavSource imu;
    private AutonomousSensorManager asm;
    private HolonomicDrivetrain rb;
    public PIDF headingPid, leftStrafePid, rightStrafePid;
    private MecanumEncoderCalculator calculator;

    private Context appContext;

    private int errorSoundId;
    private boolean errorSoundFound;


    public Fusion2(HardwareMap hw, HolonomicDrivetrain drivetrain) {
        try {
            imu = new IMUNavSource(hw);
        } catch (RuntimeException e) {
            RobotLog.setGlobalErrorMsg(e, "Both IMUs have failed!");
            throw e;
        }
        asm = new AutonomousSensorManager(hw);
        rb = drivetrain;
        calculator = new MecanumEncoderCalculator(13.7 * 2.0, 96);

        appContext = hw.appContext;

        headingPid = new PIDF(0.95, 0, 0.01, 0);
        headingPid.setPhase(false);
        headingPid.setPeakOutputForward(1);
        headingPid.setPeakOutputReverse(-1);
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);
        headingPid.setAtTargetThreshold(Math.toRadians(3));

        leftStrafePid = new PIDF(0.25, 0, 0, 0);
        leftStrafePid.setNominalOutputForward(0.1);
        leftStrafePid.setNominalOutputReverse(-0.1);
        leftStrafePid.setPeakOutputForward(0.5);
        leftStrafePid.setPeakOutputReverse(-0.5);
        leftStrafePid.setPhase(false);
        leftStrafePid.setAtTargetThreshold(1);

        rightStrafePid = new PIDF(0.25, 0, 0, 0);
        rightStrafePid.setNominalOutputForward(0.1);
        rightStrafePid.setNominalOutputReverse(-0.1);
        rightStrafePid.setPeakOutputForward(0.5);
        rightStrafePid.setPeakOutputReverse(-0.5);
        rightStrafePid.setPhase(false);
        rightStrafePid.setAtTargetThreshold(1);

        headingPid.setInputFunction(imu::getHeading);
        leftStrafePid.setInputFunction(asm::getLeftDistance);
        rightStrafePid.setInputFunction(asm::getRightDistance);

        errorSoundId = hw.appContext.getResources().getIdentifier("warningmessage", "raw", hw.appContext.getPackageName());
        if (errorSoundId != -1) {
            errorSoundFound = SoundPlayer.getInstance().preload(hw.appContext, errorSoundId);
        }
    }

    public IMUNavSource getNavSource() {
        return imu;
    }

    public AutonomousSensorManager getASM() {
        return asm;
    }

    public void drive(LinearOpMode opMode, Vector2 target, Runnable whileWaiting, double speed) {
        drive(opMode, target, whileWaiting, speed, true);
    }

    /**
     * Drives the robot to the target position.
     * @param opMode opmode that called the method
     * @param target target position vector
     * @param whileWaiting callback executed repeatedly while
     *                     the function is running
     */
    public void drive(LinearOpMode opMode, Vector2 target, Runnable whileWaiting, double speed, boolean stop) {
        rb.resetEncoders();

        // calculate new encoder targets. These are the target
        // encoder values we are seeking.
        int[] targets = calculator.calculate(target);

        double absMaxTarget = MathUtilities.absMax(targets);
        double absMinTarget = MathUtilities.absMin(targets);

        double[] merpWeights = new double[targets.length];
        for (int i = 0; i < targets.length; i++) {
            merpWeights[i] = targets[i] / absMaxTarget;
        }


        int maxTargetIndex = -1;
        {
            int maxTarget = -1;
            for (int i = 0; i < targets.length; i++) {
                int absTarget = Math.abs(targets[i]);
                if (absTarget > maxTarget) {
                    maxTarget = absTarget;
                    maxTargetIndex = i;
                }
            }
        }

        // calculate the normalized version of the target vector.
        // a normalized vector is a vector with a magnitude of 1.
        // this vector is used us a drive input for the drivetrain.
        Vector2 normalizedTargetVector = target.scalarDivide(target.getMagnitude());

        // drive towards the target position. the normalized target vector
        // points towards the target position, but it has a magnitude of 1.
        // this allows us to easily change our speed by simply multiplying
        // the normalized vector by the speed we want to go.
        rb.drive(normalizedTargetVector.getX(), normalizedTargetVector.getY(), headingPid.update());

        // allocate arrays to hold the current error and progress for each wheel.
        // error is the difference between the target and current position.
        // progress is a value that represents how far each wheel has progressed to
        // the target, with 0 being at the starting point and 1 being the target point.
        int[] errors = new int[4];
        double[] progress = new double[4];

        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            // utility variable that holds the current encoder positions
            int[] positions = rb.getEncoderPositions();

            // iterate through each wheel
            for (int i = 0; i < targets.length; i++) {
                // calculate both error and progress for the wheel
                // note that errors is an absolute difference!
                errors[i] = Math.abs(targets[i] - positions[i]);
                progress[i] = (double) positions[i] / (double) targets[i];
            }

            // calculate the average progress amount. This is used as
            // a way to check how far overall the robot has moved to
            // the target, and is part of the end state logic.
            double averageProgress = progress[maxTargetIndex];//MathUtilities.average(progress);

            // end state logic: determine when we are "at target"
            // the current implementation defines "at target" as:
            // -- The largest error is less than 50
            // ~ or ~
            // -- Average progress is greater than or equal to 1
            if (MathUtilities.min(errors) < 50 || averageProgress >= 1) {
                // end the loop, since we are at our target
                break;
            }

            // estimate the remaining distance to the target point.
            // we assume that there is a 1:1 relationship between average
            // progress and interpolation between the start and end points.
            // logically, this line is the same as:
            // Vector2 currentPosition = Vector2.lerp(Vector2.ZERO, target, averageProgress);
            // Vector2 toTarget = Vector2.sub(target, currentPosition);
            // double remainingDistance = toTarget.getMagnitude();
            // however, this implementation is simpler and runs faster.
            double remainingDistance = (1 - averageProgress) * target.getMagnitude();

            // motion profile variables
            double effectiveSpeed = speed;             // speed the robot starts at
            double nominalSpeed = 0.2;      // speed to stop at. this should be
                                            // the slowest the robot can go
                                            // without the drive motors stalling.
            double slowdownPoint = 300;     // distance at which to start slowing down.

            // if we are within slowing distance, use the slowdown logic
            if (remainingDistance < slowdownPoint && stop) {
                // linearly interpolate between max speed and min speed.
                // as the robot approaches the target, speed will ramp down,
                // ending at nominal speed when distance = 0.
                effectiveSpeed = MathUtilities.map(remainingDistance, 0, slowdownPoint, nominalSpeed, effectiveSpeed);
                if (effectiveSpeed < nominalSpeed) {
                    // paranoia: ensure speed never goes below nominal speed
                    effectiveSpeed = nominalSpeed;
                }
            }

            // finally, drive towards the target at the calculated speed.
            // we also use the heading pid to stay pointed the correct
            // direction.
            rb.drive(
                    effectiveSpeed * normalizedTargetVector.getX(),
                    effectiveSpeed * normalizedTargetVector.getY(),
                    headingPid.update()
            );

            /*for (int i = 0; i < progress.length; i++) {
                opMode.telemetry.addData(String.format("Progress[%d]", i), progress[i]);
            }
            opMode.telemetry.addData("Average Progress", averageProgress);*/
        }
        if (stop) {
            // stop the robot at the target
            rb.drive(0, 0, 0);
        }

        opMode.telemetry.log().add("Action Completed: Drive %s", target.toString());
        opMode.telemetry.update();
    }

    public void driveToLeftDistance(LinearOpMode opMode, double target, Runnable whileWaiting) {
        double approachDirection = (asm.getLeftDistance() > target) ? 1 : -1;
        while (!opMode.isStopRequested()) {
            double distance = target - asm.getLeftDistance();

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.getLeftDistance() > MAX_SANE_DISTANCE) {
                if (errorSoundFound) {
                    SoundPlayer.getInstance().startPlaying(appContext, errorSoundId);
                }
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
            }

            if (Math.abs(distance) < 10 || distance * approachDirection > 0) {
                break;
            }

            double speed = -0.7 * approachDirection;
            double nominalSpeed = 0.2;
            double slowdownPoint = 100;
            if (Math.abs(distance) < slowdownPoint) {
                speed = MathUtilities.map(distance, slowdownPoint, 0, speed, Math.copySign(nominalSpeed, speed));
                if (Math.abs(speed) < nominalSpeed) {
                    speed = Math.copySign(nominalSpeed, speed);
                }
            }
            rb.drive(speed, 0, headingPid.update());
        }
        rb.drive(0, 0, 0);
        opMode.telemetry.log().add("Action Completed: Drive To Left Distance (%f)", target);
        opMode.telemetry.update();
    }

    public void driveToRightDistance(LinearOpMode opMode, double target, Runnable whileWaiting) {
        double approachDirection = (asm.getRightDistance() > target) ? 1 : -1;
        while (!opMode.isStopRequested()) {
            double distance = target - asm.getRightDistance();

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.getRightDistance() > MAX_SANE_DISTANCE) {
                if (errorSoundFound) {
                    SoundPlayer.getInstance().startPlaying(appContext, errorSoundId);
                }
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
            }

            if (Math.abs(distance) < 10 || distance * approachDirection > 0) {
                break;
            }

            double speed = 0.7 * approachDirection;
            double nominalSpeed = 0.2;
            double slowdownPoint = 100;
            if (Math.abs(distance) < slowdownPoint) {
                speed = MathUtilities.map(distance, slowdownPoint, 0, speed, Math.copySign(nominalSpeed, speed));
                if (Math.abs(speed) < nominalSpeed) {
                    speed = Math.copySign(nominalSpeed, speed);
                }
            }
            rb.drive(speed, 0, headingPid.update());
        }
        rb.drive(0, 0, 0);
        opMode.telemetry.log().add("Action Completed: Drive To Right Distance (%f)", target);
        opMode.telemetry.update();
    }

    public void driveToFrontDistance(LinearOpMode opMode, double target, Runnable whileWaiting) {
        double approachDirection = (asm.getFrontDistance() > target) ? 1 : -1;
        while (!opMode.isStopRequested()) {
            double distance = target - asm.getFrontDistance();

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.getLeftDistance() > MAX_SANE_DISTANCE) {
                if (errorSoundFound) {
                    SoundPlayer.getInstance().startPlaying(appContext, errorSoundId);
                }
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
            }

            if (Math.abs(distance) < 10 || distance * approachDirection > 0) {
                break;
            }

            double speed = 0.7 * approachDirection;
            double nominalSpeed = 0.2;
            double slowdownPoint = 100;
            if (Math.abs(distance) < slowdownPoint) {
                speed = MathUtilities.map(distance, slowdownPoint, 0, speed, Math.copySign(nominalSpeed, speed));
                if (Math.abs(speed) < nominalSpeed) {
                    speed = Math.copySign(nominalSpeed, speed);
                }
            }
            rb.drive(0, speed, headingPid.update());
        }
        rb.drive(0, 0, 0);
        opMode.telemetry.log().add("Action Completed: Drive To Front Distance (%f)", target);
        opMode.telemetry.update();
    }

    public void driveToBackDistance(LinearOpMode opMode, double target, Runnable whileWaiting) {
        double approachDirection = (asm.getBackDistance() > target) ? 1 : -1;
        while (!opMode.isStopRequested()) {
            double distance = target - asm.getBackDistance();

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.getLeftDistance() > MAX_SANE_DISTANCE) {
                if (errorSoundFound) {
                    SoundPlayer.getInstance().startPlaying(appContext, errorSoundId);
                }
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
            }

            if (Math.abs(distance) < 10 || distance * approachDirection > 0) {
                break;
            }

            double speed = -0.7 * approachDirection;
            double nominalSpeed = 0.2;
            double slowdownPoint = 100;
            if (Math.abs(distance) < slowdownPoint) {
                speed = MathUtilities.map(distance, slowdownPoint, 0, speed, Math.copySign(nominalSpeed, speed));
                if (Math.abs(speed) < nominalSpeed) {
                    speed = Math.copySign(nominalSpeed, speed);
                }
            }
            rb.drive(0, speed, headingPid.update());
        }
        rb.drive(0, 0, 0);
        opMode.telemetry.log().add("Action Completed: Drive To Back Distance (%f)", target);
        opMode.telemetry.update();
    }

    public void turnToHeading(LinearOpMode opMode, double heading, Runnable whileWaiting) {
        headingPid.setSetpoint(heading);
        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (Math.abs(headingPid.getLastError()) < Math.toRadians(6)) {
                break;
            }

            rb.drive(0, 0, headingPid.update());
        }
        rb.drive(0, 0, 0);
        opMode.telemetry.log().add("Action Completed: Turn To Heading (%f)", heading);
        opMode.telemetry.update();
    }

    /*public void driveForwardsToWall(LinearOpMode opMode, Runnable whileWaiting) {
        while (!opMode.isStopRequested()) {
            double distance = asm.getWallDistance();

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.frontAtWall()) {
                break;
            }

            double speed = 1;
            if (!Double.isNaN(distance) || getASM().getUltrasonicDistance() < 200) {
                speed = 0.4;
            }
            rb.drive(0, speed, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }*/

    public void driveForwardsToTape(LinearOpMode opMode, Runnable whileWaiting) {
        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.onTape()) {
                break;
            }

            rb.drive(0, 0.4, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }

    public void driveBackwardsToTape(LinearOpMode opMode, Runnable whileWaiting) {
        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.onTape()) {
                break;
            }

            rb.drive(0, -0.4, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }

    public void driveLeftToTape(LinearOpMode opMode, Runnable whileWaiting) {
        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.onTape()) {
                break;
            }

            rb.drive(-0.4, 0, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }

    public void driveRightToTape(LinearOpMode opMode, Runnable whileWaiting) {
        while (!opMode.isStopRequested()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }

            if (asm.onTape()) {
                break;
            }

            rb.drive(0.4, 0, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }

    public void driveToWallDistance(LinearOpMode opMode, double target, Runnable whileWaiting) {
        driveToWallDistance(opMode, target, whileWaiting, 0.4);
    }

    public void driveToWallDistance(LinearOpMode opMode, double target, Runnable whileWaiting, double speed) {
        double lastDistance = asm.getUltrasonicDistance();
        int numFailedChecks = 0;
        while (lastDistance < 1 || lastDistance > 10000) {
            lastDistance = asm.getUltrasonicDistance();
            numFailedChecks++;
            if (numFailedChecks >= 5) {
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
                rb.drive(0, 0, 0);
                return;
            }
        }
        numFailedChecks = 0;
        double approachDirection = (lastDistance > target) ? 1 : -1;
        while (!opMode.isStopRequested()) {
            double newDistance = asm.getUltrasonicDistance();
            if (newDistance > 1 && newDistance < 10000) {
                lastDistance = newDistance;
                numFailedChecks = 0;
            } else {
                numFailedChecks++;
                if (numFailedChecks >= 5) {
                    opMode.telemetry.log().add("Sanity Check Failed!");
                    opMode.telemetry.update();
                    opMode.requestOpModeStop();
                    rb.drive(0, 0, 0);
                    return;
                }
            }
            double distance = target - lastDistance;

            if (whileWaiting != null) {
                whileWaiting.run();
            }

            /*if (asm.getRightDistance() > MAX_SANE_DISTANCE) {
                if (errorSoundFound) {
                    SoundPlayer.getInstance().startPlaying(appContext, errorSoundId);
                }
                opMode.telemetry.log().add("Sanity Check Failed!");
                opMode.telemetry.update();
                opMode.requestOpModeStop();
            }*/

            if (Math.abs(distance) < 10 || distance * approachDirection > 0) {
                break;
            }

            double effectiveSpeed = speed * approachDirection;
            double nominalSpeed = 0.2;
            double slowdownPoint = 10;
            if (Math.abs(distance) < slowdownPoint) {
                effectiveSpeed = MathUtilities.map(distance, slowdownPoint, 0, effectiveSpeed, Math.copySign(nominalSpeed, effectiveSpeed));
                if (Math.abs(effectiveSpeed) < nominalSpeed) {
                    effectiveSpeed = Math.copySign(nominalSpeed, effectiveSpeed);
                }
            }
            rb.drive(0, effectiveSpeed, headingPid.update());
        }
        rb.drive(0, 0, 0);
    }
}
