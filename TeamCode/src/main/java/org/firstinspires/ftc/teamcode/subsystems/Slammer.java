package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/25/2018.
 */

public class Slammer implements Subsystem{

    private LinearOpMode linearOpMode = null;
    private DcMotor slamMotor = null;

    protected ElapsedTime runtime = new ElapsedTime();// FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.


    private static final double COUNTS_PER_MOTOR_REV = 1680; //AndyMark NeveRest 60


    @Override
    public void init(LinearOpMode linearOpMode, boolean auto) {
        this.linearOpMode = linearOpMode;
        slamMotor = linearOpMode.hardwareMap.dcMotor.get("slammer");
        slamMotor.setDirection(FORWARD);
        setRunMode(RUN_WITHOUT_ENCODER);
        stop();
    }

    @Override
    public void zeroSensors() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.yield();
    }

    @Override
    public void stop() {
        slamMotor.setZeroPowerBehavior(BRAKE);
        setPower(0);
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    public void setRunMode(DcMotor.RunMode runMode){slamMotor.setMode(runMode);}

    public void setPower(double power){slamMotor.setPower(power);}

    /*
    public void setTarget(boolean direction, double angle){
        int newTarget = 0;

        setRunMode(RUN_TO_POSITION);
        if(direction)
            newTarget = slamMotor.getCurrentPosition() + (int) (COUNTS_PER_MOTOR_REV * angle)/360;
        else if(!direction)
            newTarget = slamMotor.getCurrentPosition() + (int) (-1 * (COUNTS_PER_MOTOR_REV * angle)/360);
        slamMotor.setTargetPosition(newTarget);
    }

    public void angleTurn(double speed, boolean direction, double angle, double timeoutS){

        int newTarget = 0;
        int counter = 0;

        if(linearOpMode.opModeIsActive()){

            if(direction)
                newTarget = slamMotor.getCurrentPosition() + (int) (COUNTS_PER_MOTOR_REV * angle)/360;
            if(!direction)
                newTarget = slamMotor.getCurrentPosition() + (int) (-1 * (COUNTS_PER_MOTOR_REV * angle)/360);

            slamMotor.setTargetPosition(newTarget);
            setRunMode(RUN_TO_POSITION);
            runtime.reset();
            setPower(Math.abs(speed));
            while(linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    slamMotor.isBusy()){

                linearOpMode.telemetry.addData("Target", "Running to %7d", newTarget);
                linearOpMode.telemetry.addData("Distance", Math.abs(newTarget - slamMotor.getCurrentPosition()));
                linearOpMode.telemetry.addData("Counter", counter);
                counter++;

            }
            setPower(0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
    */
}
