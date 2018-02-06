package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import static java.lang.Math.abs;
import static java.lang.Math.round;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.TELEOP;

/**
 * Created by joshua9889 on 12/10/2017.
 *
 * Replicated 2753's TeleopMain_Relic with new backend.
 *Edited by David Zheng | 2753 Team Overdrive
 */

@TeleOp(name = "Teleop")
public class Teleop extends Team2753Linear {

    private static final boolean UP = true;
    private static final boolean DOWN = false;
    boolean slamState;

    @Override
    public void runOpMode() throws InterruptedException {

        //Set up Telemetry
        telemetry.setAutoClear(true);
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        Telemetry.Item currentOpMode = telemetry.addData("Running", "UNKOWN");
        Telemetry.Item phase = telemetry.addData("Phase", "Init Routine");
        telemetry.update();

        //Initialize Robot
        status.setValue("Initializing...");
        currentOpMode.setValue("Teleop");
        telemetry.update();
        initializeRobot(this, TELEOP);
        //slamState = DOWN;

        //Waiting for Start
        status.setValue("Initialized, Waiting for Start");
        telemetry.update();
        waitForStart(this);
        status.setValue("Running OpMode");
        currentOpMode.setValue("Teleop");
        phase.setValue("Driver Control");
        telemetry.update();

        // Loop while we are running Teleop
        while (opModeIsActive()){
/*
                     _______                                    _______
                  __/_______\_____                       ______/_______\__
                /                 \                     /                  \
               /                   \___________________/                    \
              |         __                                                   |
              |      __|  |__                                 ( Y )          |
              |     |__    __|                           ( X )     ( B )     |
             |         |__|                                   ( A )           |
             |                                                                |
             |                                                                |
             |                                                                |
            |                                                                  |
            |                                                                  |
            |                                                                  |
            |                                                                  |
           |                                                                    |
           |                                                                    |
           |                                                                    |
           |                                                                    |
           |                                                                    |
            \________/                                                \________/
*/

            /*Gamepad 1 Controls*/

            /* Drivetrain Controls */ //Gamepad 1 joysticks
            float leftThrottle = gamepad1.left_stick_y;
            float rightThrottle = gamepad1.right_stick_y;

            /* Clip the left and right throttle values so that they never exceed +/- 1.  */
            leftThrottle = Range.clip(leftThrottle,-1,1);
            rightThrottle = Range.clip(rightThrottle,-1,1);

            /* Scale the throttle values to make it easier to control the robot more precisely at slower speeds.  */
            leftThrottle = (float) OverdriveLib.scaleInput(leftThrottle);
            rightThrottle = (float) OverdriveLib.scaleInput(rightThrottle);

            getDrive().setLeftRightPowers(leftThrottle, rightThrottle);

            //D-pad controls for more precise movement
            if (Math.abs(leftThrottle) == 0 && Math.abs(rightThrottle) == 0) {
                if (gamepad1.dpad_up) {
                    getDrive().setLeftRightPowers(-0.3, -0.3);
                    sleep(250);
                }
                else if (gamepad1.dpad_down) {
                    getDrive().setLeftRightPowers(0.3, 0.3);
                    sleep(250);
                }
                else {
                    getDrive().setLeftRightPowers(0,0);
                }
            }

            //Fancy Intake FSM Controls

            /*
            boolean intakeOn = false;
            boolean intakeState = true;


            if(gamepad1.left_bumper){
                if(!intakeOn){
                    intakeOn = true;
                    intakeState = true;
                }
                if(intakeOn)
            }
            if(gamepad1.right_bumper){

            }

            if(intakeOn){
                if(intakeState)
                    getIntake().setPower(1);
                if(!intakeState)
                    getIntake().setPower(-1);
            }
            */

            //Intake Controls

            if(gamepad1.left_bumper) {
                getIntake().reverse();
            }
            else if(gamepad1.right_bumper)
                getIntake().intake();
            else
                getIntake().stop();


            /** Gamepad 2 Controls   */

            /*Lift Control  Gamepad 2 Left Joystick*/
            float liftThrottle = gamepad2.left_stick_y;
            //CLip
            liftThrottle = Range.clip(liftThrottle, -1, 1);
            //Scale
            liftThrottle = (float) OverdriveLib.scaleInput(liftThrottle);
            //Invert
            liftThrottle = liftThrottle*-1;
            //Apply power to motor
            getLift().setLiftPower(liftThrottle);

            //Slammer
            if(gamepad2.y) {
                getSlammer().setPower(0.35);
            }
            else if(gamepad2.a) {
                getSlammer().setPower(-0.2);
            }
            else
                getSlammer().setPower(0);


            //Stopper
            if(gamepad2.left_bumper)
                getSlammer().stopperUp();
            else if(gamepad2.right_bumper)
                getSlammer().stopperDown();


            //Jewel Test
            /*
            if(gamepad2.right_bumper)
                getJewel().deploy();
            else
                getJewel().retract();
             */

            //Phone servo test
            /*
            if(gamepad2.right_bumper)
                getPhoneServo.jewelPosition();
            else
                getPhoneServo.initPosition();
            */



            status.setValue("Running OpMode");
            currentOpMode.setValue("Teleop");
            phase.setValue("Driver Control");
            updateTelemetry(this);

        }

        finalAction();
    }
}
