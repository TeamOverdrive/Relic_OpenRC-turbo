package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/30/2018.
 */

@Autonomous(name = "Auto_Test", group = "test")
public class fullAutoTest extends Team2753Linear{

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
        startVuforia(FRONT);

        //Waiting for start
        status.setValue("Initialized, Waiting for Start");
        telemetry.update();
        waitForStart(this);

        status.setValue("Running OpMode");
        telemetry.update();

        int i = 0;

        while(opModeIsActive() && i == 0) {

            int Column = 0;
            switch(columnVoteLoop(this, 5)){
                case LEFT:
                    Column = 1;
                    break;
                case CENTER:
                    Column = 2;
                    break;
                case RIGHT:
                    Column = 3;
                    break;
                case UNKNOWN:
                    Column = 0;
                    break;
            }

            closeVuforia();


            //Jewel Phase
            phase.setValue("Jewel");
            telemetry.update();

            initJewelDetector();
            enableJewelDetector();
            //confirm method?
            jewelBlue();
            disableJewelDetector();



            //score cryptokey
            phase.setValue("Cryptokey");
            telemetry.update();
            glyphScoreB1();



            //grab more glyphs
            phase.setValue("Multiglyph");
            telemetry.update();
            //multiGlyphB1(13);



            //score extra glyphs



            //park
            phase.setValue("Parking");
            telemetry.update();

            //temporary code until i get glyph working
            getDrive().encoderDrive(autoSpeed, -36,-36, 5);
            getDrive().turnCW(90, autoTurnSpeed, 4);
            getDrive().encoderDrive(autoSpeed, 6, 6, 4);

            i++;
        }

        finalAction();
    }
}
