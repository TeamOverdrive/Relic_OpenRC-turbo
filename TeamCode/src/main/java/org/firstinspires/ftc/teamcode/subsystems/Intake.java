package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/25/2018.
 */

public class Intake implements Subsystem{

    private LinearOpMode linearOpMode = null;
    private DcMotor intakeMotor = null;
    private Servo intakeRelease = null;

    @Override
    public void init(LinearOpMode linearOpMode, boolean auto) {
        this.linearOpMode = linearOpMode;
        intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intake_motor");
        intakeMotor.setDirection(REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRelease = linearOpMode.hardwareMap.servo.get("intake_servo");
        releaseLock();
        zeroSensors();
        stop();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        intakeMotor.setZeroPowerBehavior(FLOAT);
        setPower(0);
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake Power", intakeMotor.getPower());
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }

    public void intake(){setPower(1);}

    public void reverse(){setPower(-1);}

    public void releaseLock(){intakeRelease.setPosition(0.16);}

    public void releaseIntake(){intakeRelease.setPosition(0.35);}
}
