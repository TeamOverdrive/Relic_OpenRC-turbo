package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/30/2018.
 */

@Autonomous(name = "Auto Test", group = "test")
@Disabled
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
        currentOpMode.setValue("B1 CV");
        telemetry.update();
        initializeRobot(this, AUTO);
        startVuforia(BACK);

        //Waiting for start
        status.setValue("Initialized, Waiting for Start");
        telemetry.update();
        waitForStart(this);

        status.setValue("Running OpMode");
        telemetry.update();

        int i = 0;

        while(opModeIsActive() && i == 0) {

            //Vuforia
            columnVote(this);
            closeVuforia();


            //Jewel Phase
            phase.setValue("Jewel");
            telemetry.update();
            initJewelDetector();
            enableJewelDetector();
            jewelBlue(this);
            disableJewelDetector();


            //score cryptokey
            phase.setValue("Cryptokey");
            telemetry.update();
            glyphScoreB1();


            //grab more glyphs
            phase.setValue("Multiglyph");
            telemetry.update();
            //multiGlyphPos1(20);
            //disable to work on accurate turning for cryptokey

            //park
            phase.setValue("Parking");
            telemetry.update();

            i++;
        }

        finalAction();
    }
}
