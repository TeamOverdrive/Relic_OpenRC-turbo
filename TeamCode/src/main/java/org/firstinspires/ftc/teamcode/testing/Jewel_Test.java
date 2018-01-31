package org.firstinspires.ftc.teamcode.testing;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelArmDelayMS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurn;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurnSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurnTimeoutS;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/27/2018.
 */

@Autonomous(group = "test")
@Disabled
public class Jewel_Test extends Team2753Linear {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        initializeRobot(this, AUTO);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initJewelDetector();

        waitForStart(this);

        int i = 0;
        while(opModeIsActive() && i == 0) {

            enableJewelDetector();

            jewelRed();

            disableJewelDetector();

            i++;

        }
    }
}
