package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.BLUE;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelArmDelayMS;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/10/2018.
 */
@Autonomous(name = "Blue 1 CV", group = "CV")
public class B1_CV extends Team2753Linear {

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
        currentOpMode.setValue("B1 Vuforia");
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

            vumark.closeVuforia();

            //grab cryptokey

            initialLift(BLUE);

            //lower jewel arm
            phase.setValue("Jewel");
            telemetry.update();
            getJewel().deploy();
            sleep(jewelArmDelayMS);

            //Hit off the blue jewel
            //jewelBlue();

            //raise jewel arm
            getJewel().retract();
            sleep(jewelArmDelayMS);

            //score cryptokey
            phase.setValue("Cryptokey");
            telemetry.update();
            glyphScoreB1();

            //grab more glyphs
            phase.setValue("Multiglyph");
            telemetry.update();
            multiGlyphB1(13);

            //score extra glyphs

            //park

            i++;
        }

        finalAction();
    }
}
