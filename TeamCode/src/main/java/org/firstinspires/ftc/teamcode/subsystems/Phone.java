package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/29/2018.
 */


//For the phone servo only lel
public class Phone implements Subsystem{

    private LinearOpMode linearOpMode = null;
    private Servo phoneServo = null;

    private static final double INITPOS = 0.5;
    private static final double JEWELPOS = 0;
    private static final double CRYPTOPOS = 0;

    @Override
    public void init(LinearOpMode linearOpMode, boolean auto) {
        this.linearOpMode = linearOpMode;
        phoneServo = linearOpMode.hardwareMap.servo.get("phone_servo");
       initPosition();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        initPosition();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    public void initPosition(){
        phoneServo.setPosition(INITPOS);
    }

    public void jewelPosition(){
        phoneServo.setPosition(JEWELPOS);
    }

    public void cryptoPosition(){phoneServo.setPosition(CRYPTOPOS);}

}
