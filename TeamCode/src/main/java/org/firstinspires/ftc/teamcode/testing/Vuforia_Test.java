package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team2753Linear;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.AUTO;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/31/2018.
 */

@Autonomous
public class Vuforia_Test extends Team2753Linear{
    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize Robot
        telemetry.addData("Status","Initializing...");
        telemetry.addData("Current Op Mode","Auto Test");
        telemetry.update();
        initializeRobot(this, AUTO);
        startVuforia(BACK);

        int Column = 0;
        switch (initColumnVoteLoop(this, 5)){
            case LEFT:
                Column = 1;
                telemetry.addData("Column", "Left");
                telemetry.update();
                break;
            case CENTER:
                Column = 2;
                telemetry.addData("Column", "Center");
                telemetry.update();
                break;
            case RIGHT:
                Column = 3;
                telemetry.addData("Column", "Right");
                telemetry.update();
                break;
            case UNKNOWN:
                Column = 0;
                telemetry.addData("Column", "Unknown");
                telemetry.update();
                break;
        }

        while(!isStarted()){}
        //does this break^?
        resetRuntime();

        int i = 0;
        while(opModeIsActive() && i == 0){

            if(Column == 0){
                switch (columnVoteLoop(this, 5)){
                    case LEFT:
                        Column = 1;
                        telemetry.addData("Column", "Left");
                        telemetry.update();
                        break;
                    case CENTER:
                        Column = 2;
                        telemetry.addData("Column", "Center");
                        telemetry.update();
                        break;
                    case RIGHT:
                        Column = 3;
                        telemetry.addData("Column", "Right");
                        telemetry.update();
                        break;
                    case UNKNOWN:
                        Column = 0;
                        telemetry.addData("Column", "Unknown");
                        telemetry.update();
                        break;
                }
            }

            waitForTick(10000); //wait ten seconds.


            i++;
        }
    }
}
