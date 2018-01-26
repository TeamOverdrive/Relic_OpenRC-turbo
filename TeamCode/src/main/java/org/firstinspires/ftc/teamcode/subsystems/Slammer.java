package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/25/2018.
 */

public class Slammer implements Subsystem{

    private LinearOpMode linearOpMode = null;
    private DcMotor slamMotor = null;

    @Override
    public void init(LinearOpMode linearOpMode, boolean auto) {
        this.linearOpMode = linearOpMode;
        slamMotor = linearOpMode.hardwareMap.dcMotor.get("slammer");
        slamMotor.setDirection(FORWARD);
        slamMotor.setMode(RUN_WITHOUT_ENCODER);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        slamMotor.setZeroPowerBehavior(BRAKE);
        slamMotor.setPower(0);
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    public void setPower(double power){
        slamMotor.setPower(power);
    }
}
