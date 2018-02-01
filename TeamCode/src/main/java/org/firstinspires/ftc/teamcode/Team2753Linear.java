package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static com.disnodeteam.dogecv.detectors.JewelDetector.*;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelArmDelayMS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelColorTimeoutS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurn;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurnSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurnTimeoutS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelVotes;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.vuMarkVotes;


/**
 * This class extends linearopmode and makes it
 * easier to make code for the robot and not copy
 * and pasting init code.
 *
 * See this for an example: http://bit.ly/2B8scLB
 * Created by joshua9889 on 12/10/2017.
 */

public abstract class Team2753Linear extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.subsystems.Drive Drive = new org.firstinspires.ftc.teamcode.subsystems.Drive(); // Drivetrain
    private org.firstinspires.ftc.teamcode.subsystems.Jewel Jewel = new org.firstinspires.ftc.teamcode.subsystems.Jewel(); // Jewel mech
    private org.firstinspires.ftc.teamcode.subsystems.Lift Lift = new org.firstinspires.ftc.teamcode.subsystems.Lift();
    private org.firstinspires.ftc.teamcode.subsystems.Intake Intake = new org.firstinspires.ftc.teamcode.subsystems.Intake();
    private org.firstinspires.ftc.teamcode.subsystems.Slammer Slammer = new org.firstinspires.ftc.teamcode.subsystems.Slammer();
    private org.firstinspires.ftc.teamcode.subsystems.Phone Phone = new org.firstinspires.ftc.teamcode.subsystems.Phone();

    public static VuMark vumark = new VuMark();
    public static ElapsedTime runtime = new ElapsedTime();
    private boolean isAuton = false; // Are we running auto
    private JewelDetector jewelDetector = null;

    //Init Methods

    public void waitForStart(LinearOpMode linearOpMode) {
        linearOpMode.waitForStart();
        runtime.reset();
    }

    public void initializeRobot(LinearOpMode linearOpMode, boolean auton){
        getDrive().init(linearOpMode, auton);
        getJewel().init(linearOpMode, auton);
        getLift().init(linearOpMode, auton);
        getIntake().init(linearOpMode, auton);
        getSlammer().init(linearOpMode, auton);
        phoneServo().init(linearOpMode, auton);
        if(auton){

            AutoTransitioner.transitionOnStop(linearOpMode, "Teleop"); //Auto Transitioning
            this.isAuton = auton;
        }
    }

    public void resetRuntime(){runtime.reset();}

    //Auto Methods

    //Lift

    public void liftLower(){
        getLift().setLiftPower(-0.2);
        sleep(250);
        getLift().brakeLift();
    }

    //Vuforia

    public void startVuforia(VuforiaLocalizer.CameraDirection direction){
        this.vumark.setup(direction);
    }

    public RelicRecoveryVuMark initColumnVoteLoop(LinearOpMode linearOpMode, double timeoutS){
        int leftVotes = 0;
        int centerVotes = 0;
        int rightVotes = 0;
        runtime.reset();
        while(!linearOpMode.isStarted()
                && leftVotes < vuMarkVotes  &&  centerVotes < vuMarkVotes && rightVotes < vuMarkVotes
                && runtime.seconds() < timeoutS){
            switch (vumark.targetColumn()){
                case LEFT:
                    leftVotes++;
                    break;
                case CENTER:
                    centerVotes++;
                    break;
                case RIGHT:
                    rightVotes++;
                    break;
            }
            linearOpMode.telemetry.addData("Left Votes", leftVotes);
            linearOpMode.telemetry.addData("Center Votes", centerVotes);
            linearOpMode.telemetry.addData("Right Votes", rightVotes);
            linearOpMode.telemetry.update();
        }
        if(leftVotes == vuMarkVotes)
            return RelicRecoveryVuMark.LEFT;
        else if (centerVotes == vuMarkVotes)
            return RelicRecoveryVuMark.CENTER;
        else if(rightVotes ==vuMarkVotes)
            return RelicRecoveryVuMark.RIGHT;
        else
            return RelicRecoveryVuMark.UNKNOWN;
    }

    public RelicRecoveryVuMark columnVoteLoop(LinearOpMode linearOpMode, double timeoutS){
        int leftVotes = 0;
        int centerVotes = 0;
        int rightVotes = 0;
        runtime.reset();
        while(linearOpMode.opModeIsActive()
                &&  leftVotes < vuMarkVotes  &&  centerVotes < vuMarkVotes && rightVotes < vuMarkVotes
                && runtime.seconds() < timeoutS){
            switch (vumark.targetColumn()){
                case LEFT:
                    leftVotes++;
                    break;
                case CENTER:
                    centerVotes++;
                    break;
                case RIGHT:
                    rightVotes++;
                    break;
            }
            linearOpMode.telemetry.addData("Left Votes", leftVotes);
            linearOpMode.telemetry.addData("Center Votes", centerVotes);
            linearOpMode.telemetry.addData("Right Votes", rightVotes);
            linearOpMode.telemetry.update();
        }
        if(leftVotes == vuMarkVotes)
            return RelicRecoveryVuMark.LEFT;
        else if (centerVotes == vuMarkVotes)
            return RelicRecoveryVuMark.CENTER;
        else if(rightVotes ==vuMarkVotes)
            return RelicRecoveryVuMark.RIGHT;
        else
            return RelicRecoveryVuMark.UNKNOWN;
    }

    public void closeVuforia(){vumark.closeVuforia();}


    //Jewel


    public void initJewelDetector(){
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;;
        jewelDetector.detectionMode = JewelDetectionMode.PERFECT_AREA;
        jewelDetector.perfectArea = 1600;
        jewelDetector.debugContours = false;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 800;
        //getPhone.jewelPosition();
    }

    public void enableJewelDetector(){jewelDetector.enable();}

    public JewelOrder findJewel(LinearOpMode linearOpMode, double timeoutS){

        int brVotes = 0;
        int rbVotes = 0;
        runtime.reset();

        while(opModeIsActive() &&
                brVotes < jewelVotes && rbVotes < jewelVotes &&
                runtime.seconds() < timeoutS)
        {
            switch (jewelDetector.getCurrentOrder()) {
                case BLUE_RED:
                    brVotes++;
                    break;
                case RED_BLUE:
                    rbVotes++;
                    break;
            }
            linearOpMode.telemetry.addData("Jewel Order", jewelDetector.getCurrentOrder().toString());
            linearOpMode.telemetry.addData("BLUE_RED Votes", brVotes);
            linearOpMode.telemetry.addData("RED_BLUE Votes", rbVotes);
            linearOpMode.telemetry.update();
        }
        if(brVotes + rbVotes != 0){
            if(brVotes == jewelVotes)
                return JewelOrder.BLUE_RED;
            else if(rbVotes == jewelVotes)
                return JewelOrder.RED_BLUE;
            else if (brVotes > rbVotes)
                return JewelOrder.BLUE_RED;
            else if(rbVotes > brVotes)
                return JewelOrder.RED_BLUE;
            else
                return JewelOrder.UNKNOWN;
        }
        else
            return JewelOrder.UNKNOWN;
    }

    public void jewelRed(){
        if(opModeIsActive()){
            switch(findJewel(this, 5)){
                case BLUE_RED:
                    getJewel().deploy();
                    waitForTick(jewelArmDelayMS);
                    getDrive().turnCCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(100);
                    getJewel().retract();
                    waitForTick(100);
                    getDrive().turnCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(250);
                    break;
                case RED_BLUE:
                    getJewel().deploy();
                    waitForTick(jewelArmDelayMS);
                    getDrive().turnCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(100);
                    getJewel().retract();
                    waitForTick(100);
                    getDrive().turnCCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(250);
                    break;
            }
        }
    }

    public void jewelBlue(){
        if(opModeIsActive()){
            switch(findJewel(this, 5)){
                case BLUE_RED:
                    getJewel().deploy();
                    waitForTick(jewelArmDelayMS);
                    getDrive().turnCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(100);
                    getJewel().retract();
                    waitForTick(100);
                    getDrive().turnCCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(250);
                    break;
                case RED_BLUE:
                    getJewel().deploy();
                    waitForTick(jewelArmDelayMS);
                    getDrive().turnCCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(100);
                    getJewel().retract();
                    waitForTick(100);
                    getDrive().turnCW(jewelTurn, jewelTurnSpeed, jewelTurnTimeoutS);
                    waitForTick(250);
                    break;
            }
        }
    }

    public void disableJewelDetector(){jewelDetector.disable();}

    //Glyph

    public void glyphScoreR1(){

        switch (columnVoteLoop(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 38, 38, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 32, 32, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 26, 26, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 32, 32, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreB1(){

        switch (columnVoteLoop(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -26, -26, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -32, -32, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -38, -38, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -32, -32, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreR2(){
        switch (columnVoteLoop(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 24, 24, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 18, 18, 4);
                getDrive().turnCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, -3, -3, 2);
                getDrive().turnCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, -2, -2, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 24, 24, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 12, 12, 4);
                getDrive().turnCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, -3, -3, 2);
                getDrive().turnCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, -2, -2, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 24, 24, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 6, 6, 4);
                getDrive().turnCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, -3, -3, 2);
                getDrive().turnCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, -2, -2, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 24, 24, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 12, 12, 4);
                getDrive().turnCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, -3, -3, 2);
                getDrive().turnCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, -2, -2, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreB2(){
        switch (columnVoteLoop(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -24, -24, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, 3, 3, 2);
                getDrive().turnCCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, 2, 2, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -24, -24, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -12, -12, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, 3, 3, 2);
                getDrive().turnCCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, 2, 2, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -24, -24, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -18, -18, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, 3, 3, 2);
                getDrive().turnCCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, 2, 2, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -24, -24, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -12, -12, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 3);
                getDrive().encoderDrive(0.3, 3, 3, 2);
                getDrive().turnCCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, 2, 2, 2);

                //put glyph into center column
                break;
        }
    }

    //use timeoutS to ensure we have enough time to park before the end of autonomous
    public void multiGlyphB1(double timeoutS){

        //drive to pile
        getDrive().encoderDrive(autoSpeed, 24, 24, 5);

        //double grab
        getDrive().encoderDrive(autoSpeed + 0.1, 6, 6, 3);
        //getHand().grabBackClose();
        getDrive().encoderDrive(autoSpeed, -5, -5, 4);
        getDrive().turnCW(180, autoTurnSpeed, 4);
        getDrive().encoderDrive(0.7, -6, -6, 3);
        //getHand().grabFrontClose();

        //drive back
    }

    public void multiGlyphB2(double timeoutS){

    }

    public void multiGlyphR1(double timeoutS){
        getDrive().encoderDrive(autoSpeed, -24, -24, 5);

        //double grab
        getDrive().encoderDrive(autoSpeed + 0.1, -6, -6, 3);

        //getHand().grabBackClose();
        getDrive().encoderDrive(autoSpeed, -5, -5, 4);
        getDrive().turnCW(180, autoTurnSpeed, 4);
        getDrive().encoderDrive(0.7, -6, -6, 3);
        //getHand().grabFrontClose();

    }

    public void multiGlyphR2(double timeoutS){

    }

    //Telemetry

    public void updateTelemetry(LinearOpMode linearOpMode) {

        if (!isAuton) {
            linearOpMode.telemetry.addData("Match Time", 120 - runtime.seconds());
            if(runtime.seconds() > 90){
                linearOpMode.telemetry.addData("Phase", "End game");
            }
            if(runtime.seconds() > 120){
                linearOpMode.telemetry.addData("Phase", "Overtime");
            }
        }
        getDrive().outputToTelemetry(linearOpMode.telemetry);
        getJewel().outputToTelemetry(linearOpMode.telemetry);
        getIntake().outputToTelemetry(linearOpMode.telemetry);
        getSlammer().outputToTelemetry(linearOpMode.telemetry);
        phoneServo().outputToTelemetry(linearOpMode.telemetry);
        linearOpMode.telemetry.update();
    }

    //Other

    public void finalAction(){

            getDrive().stop();
            getJewel().stop();
            getLift().stop();
            getIntake().stop();

            requestOpModeStop();

    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)runtime.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        runtime.reset();
    }



    public org.firstinspires.ftc.teamcode.subsystems.Drive getDrive() {
        return Drive;
    }

    public org.firstinspires.ftc.teamcode.subsystems.Jewel getJewel() {
        return Jewel;
    }

    public org.firstinspires.ftc.teamcode.subsystems.Lift getLift () { return Lift; }

    public org.firstinspires.ftc.teamcode.subsystems.Intake getIntake() {return Intake;}

    public org.firstinspires.ftc.teamcode.subsystems.Slammer getSlammer() {return Slammer;}

    public org.firstinspires.ftc.teamcode.subsystems.Phone phoneServo() {return Phone;}
}


