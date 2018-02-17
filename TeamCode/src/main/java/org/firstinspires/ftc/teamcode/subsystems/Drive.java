package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
import static java.lang.Math.PI;
import static java.lang.Thread.sleep;

/**
 * Created by joshua9889 on 12/10/2017.
 * Used for the Drivetrain
 */

public class Drive implements Subsystem {
    /* Motors */
    private DcMotor leftMotor, rightMotor = null;

    // Used to output telemetry and to stop when stop is pressed
    private LinearOpMode linearOpMode = null;

    protected ElapsedTime runtime = new ElapsedTime();
    // FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.

    private static final double COUNTS_PER_MOTOR_REV = 1120;     // AndyMark NeveRest 40
    private static final double DRIVE_GEAR_REDUCTION = 0.6666 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference w/ wheel base
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.141592);
    private static final double WHEEL_BASE = 12.625;

    @Override
    public void init(LinearOpMode linearOpMode, boolean auto) {
        this.linearOpMode = linearOpMode;
        leftMotor = linearOpMode.hardwareMap.dcMotor.get("left_drive");
        rightMotor  = linearOpMode.hardwareMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        if(auto){
            zeroSensors();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            // We are running teleop, we don't need them
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void zeroSensors() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.yield();
    }

    @Override
    public void stop() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setLeftRightPowers(0,0);
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

        telemetry.addData("Left Power", leftMotor.getPower());
        telemetry.addData("Right Power", rightMotor.getPower());
        //telemetry.addData("Left Pos", leftMotor.getCurrentPosition());
        //telemetry.addData("Right Pos", rightMotor.getCurrentPosition());

    }

    /**
     * Basic setPower method for both sides of the drive train.
     *
     * @param left  left power
     * @param right right power
     */

    public void setLeftRightPowers(double left, double right){
        left = Range.clip(left, -1., 1.);
        right = Range.clip(right, -1., 1.);

        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public void setRunMode(DcMotor.RunMode runMode){
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        leftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Method to perform a Clockwise turn
     *
     * @param degrees       The degrees (in a circle) to turn.
     * @param speed         The speed that the motors are moving.
     * @param timeoutS      The amount of time this method is allowed to execute.
     */
    public void turnCW(double degrees, double speed, double timeoutS){

        double leftDistance = (WHEEL_BASE*PI*degrees)/-360;
        double rightDistance = (WHEEL_BASE*PI*degrees)/360;

        encoderDrive(speed, leftDistance, rightDistance, timeoutS);

    }


    public void turnDirectCW(double degrees, double speed, double timeoutS){
        double leftDistance = (WHEEL_BASE*PI*degrees)/-360;
        double rightDistance = (WHEEL_BASE*PI*degrees)/360;

        encoderDirectDrive(speed, leftDistance, rightDistance, timeoutS);
    }


    /**
     * Method to perform a Counter-clockwise turn
     *
     * @param degrees       The degrees (in a circle) to turn.
     * @param speed         The speed that the motors are moving.
     * @param timeoutS      The amount of time this method is allowed to execute.
     */

    public void turnCCW(double degrees, double speed, double timeoutS){

        double leftDistance = (WHEEL_BASE*PI*degrees)/360;
        double rightDistance = (WHEEL_BASE*PI*degrees)/-360;

        encoderDrive(speed, leftDistance, rightDistance, timeoutS);

    }


    public void turnDirectCCW(double degrees, double speed, double timeoutS){
        double leftDistance = (WHEEL_BASE*PI*degrees)/360;
        double rightDistance = (WHEEL_BASE*PI*degrees)/-360;

        encoderDirectDrive(speed, leftDistance, rightDistance, timeoutS);
    }


    /**
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the opmode running.
     *
     * @param speed       The speed that the motors are moving.
     * @param leftInches  The distance that the robot should move to the left.
     * @param rightInches The distance that the robot should move to the right.
     * @param timeoutS    The amount of time this method is allowed to execute.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            //int counter1 = 0;
            //int counter2 = 0;

            // Turn On RUN_TO_POSITION
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setLeftRightPowers(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() || rightMotor.isBusy())) {
                //slow the motors down to half the original speed when we get within 4 inches of our target and the speed is greater than 0.1.
                if ((Math.abs(newLeftTarget - leftMotor.getCurrentPosition()) < (4.0 * COUNTS_PER_INCH))
                        && (Math.abs(newLeftTarget - rightMotor.getCurrentPosition()) < (4.0 * COUNTS_PER_INCH))
                        && speed > 0.1) {
                    setLeftRightPowers(Math.abs(speed * 0.5), Math.abs(speed * 0.5));
                }
                //slow the motors down to 0.35 of the original speed when we get within 2 inches of our target and the speed is greater than 0.1.
                if ((Math.abs(newLeftTarget - leftMotor.getCurrentPosition()) < (2.0 * COUNTS_PER_INCH))
                        && (Math.abs(newLeftTarget - rightMotor.getCurrentPosition()) < (2.0 * COUNTS_PER_INCH))
                        && speed > 0.1) {
                    setLeftRightPowers(Math.abs(speed * 0.35), Math.abs(speed * 0.35));
                }
            }
            // Stop all motion;
            setLeftRightPowers(0,0);

            // Turn off RUN_TO_POSITION
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  linearOpMode.sleep(250);   // optional pause after each move
        }
    }


    //same as encoderDrive except without the slowdown before reaching target
    public void encoderDirectDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setLeftRightPowers(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() || rightMotor.isBusy())) {}

            // Stop all motion;
            setLeftRightPowers(0,0);

            // Turn off RUN_TO_POSITION
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  linearOpMode.sleep(250);   // optional pause after each move
        }
    }


    //only implements Kp right now.
    //if only Kp returns a desired resutl, i will leave it like that
    public void proportionControl(double leftTarget, double rightTarget, double speed, double P){
        double leftError = Math.abs(leftTarget - leftMotor.getCurrentPosition());
        double rightError = Math.abs(rightTarget - rightMotor.getCurrentPosition());
    }
}