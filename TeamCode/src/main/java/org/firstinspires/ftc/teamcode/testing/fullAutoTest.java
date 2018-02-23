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
import static org.firstinspires.ftc.teamcode.subsystems.Drive.COUNTS_PER_INCH;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 1/30/2018.
 */

@Autonomous(name = "Auto Test", group = "test")
//@Disabled
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

        //Initialize
        status.setValue("Initializing...");
        currentOpMode.setValue("B1 Multiglyph");
        telemetry.update();
        initializeRobot(this, AUTO);
        startVuforia(BACK);

        //Waiting for start
        status.setValue("Initialized, Waiting for Start");
        telemetry.update();

        waitForStart(this);
        status.setValue("Running OpMode");

        int i = 0;

        while(opModeIsActive() && i == 0) {

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
            glyphScoreB1(this);

            //grab more glyphs
            phase.setValue("Multiglyph");
            telemetry.update();
            //multiGlyphPos1(15);

            /*
            getIntake().reverse();
            getSlammer().stopperDown();
            getDrive().encoderDirectDrive(autoSpeed + 0.15, -22, -22, 5);
            getIntake().intake();
            getDrive().encoderDrive(autoSpeed + 0.1, -6, -6, 2);
            //getIntake().stop();
            waitForTick(250);
            getIntake().stop();
            getDrive().encoderDrive(autoSpeed, 28, 28, 5);
            getDrive().encoderDrive(autoSpeed, 0, 3, 2);
            scoreGlyph();
            //getDrive().encoderDrive(autoSpeed, -4, -4, 3);
            */



            getIntake().reverse();
            getSlammer().stopperDown();
            getDrive().encoderDirectDrive(autoSpeed + 0.15, -20, -20, 3);
            getIntake().intake();
            getDrive().encoderDrive(0.75, -4, -4, 2);
            waitForTick(200);
            int leftPosition = getDrive().getLeftCurrentPosition();
            int rightPosition = getDrive().getRightCurrentPosition();
            getDrive().encoderDrive(0.75, -9.915, 0, 2);
            getDrive().encoderTargetDrive(autoSpeed, leftPosition, rightPosition, 2);
            //getIntake().stop();
            waitForTick(250);
            getIntake().stop();
            getDrive().encoderDrive(autoSpeed, 24, 24, 3);
            getDrive().encoderDrive(autoSpeed, 0, 3, 1.5);
            scoreGlyph();


            /*
            getIntake().reverse();
            getSlammer().stopperDown();
            getDrive().encoderDirectDrive(autoSpeed + 0.15, -22, -22, 5);
            getIntake().intake();
            getDrive().encoderDrive(autoSpeed + 0.1, -6, -6, 2);
            leftPosition = getDrive().getLeftCurrentPosition();
            rightPosition = getDrive().getRightCurrentPosition();
            getDrive().encoderDrive(autoSpeed + 0.1, 0, -6, 4);
            getDrive().encoderDrive(autoSpeed, leftPosition*COUNTS_PER_INCH, rightPosition*COUNTS_PER_INCH, 5);
            //getIntake().stop();
            waitForTick(250);
            getIntake().stop();
            getDrive().encoderDrive(autoSpeed, 30, 30, 5);
            getDrive().encoderDrive(autoSpeed, 0, 3, 2);
            scoreGlyph();
            */

            getDrive().encoderDrive(autoSpeed, -4, -4, 3);


            //score extra glyphs

            //park
            phase.setValue("Parking");
            telemetry.update();

            i++;
        }

        finalAction();
    }
}
