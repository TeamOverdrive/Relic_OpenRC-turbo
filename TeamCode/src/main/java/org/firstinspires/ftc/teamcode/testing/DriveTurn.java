package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/30/2018.
 */

@Autonomous(name = "Drive Test", group = "test")
public class DriveTurn extends Team2753Linear{

    //Runs position b1 for now bc its the closest on test field

    @Override
    public void runOpMode() throws InterruptedException {

        //Set up telemetry
        telemetry.setAutoClear(false);
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        Telemetry.Item currentOpMode = telemetry.addData("Running", "UNKNOWN");
        Telemetry.Item phase = telemetry.addData("Phase", "Init Routine");
        telemetry.update();

        //Initialize Robot
        status.setValue("Initializing...");
        currentOpMode.setValue("Auto Test");
        telemetry.update();
        initializeRobot(this, AUTO);

        //Waiting for start
        status.setValue("Initialized, Waiting for Start");
        telemetry.update();
        waitForStart(this);

        status.setValue("Running OpMode");
        telemetry.update();

        int i = 0;

        while(opModeIsActive() && i == 0) {

            //the big wall of text

            //go to cryptobox
            getDrive().encoderDrive(autoSpeed, -36,-36, 5);
            getDrive().turnCW(90, autoTurnSpeed, 4);
            getDrive().encoderDrive(autoSpeed, 6, 6, 4);
            waitForTick(2500);

            //go to pile
            getDrive().encoderDrive(autoSpeed, -30, -30, 6);
            getIntake().intake();
            waitForTick(2000);
            getIntake().stop();

            //back to cryptobox
            getDrive().encoderDrive(autoSpeed, 30, 30, 6);

            //drive to an empty area
            getDrive().encoderDrive(autoSpeed, -15, -15, 4);
            waitForTick(1000);

            //turning stuff
            getDrive().turnCW(90, autoTurnSpeed, 5);
            waitForTick(1000);
            getDrive().turnCCW(90, autoTurnSpeed, 5);
            waitForTick(1000);
            getDrive().turnCW(180, autoTurnSpeed, 5);
            waitForTick(1000);
            getDrive().turnCCW(180, autoTurnSpeed, 5);
            waitForTick(1000);
            getDrive().turnCW(360, autoTurnSpeed, 5);
            waitForTick(1000);
            getDrive().turnCCW(360, autoTurnSpeed, 5);
            waitForTick(1000);

            //we should be perpendicular to the will at this point
        }

        finalAction();
    }
}
