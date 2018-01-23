package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.autoTurnSpeed;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelArmDelayMS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelColorTimeoutS;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurn;
import static org.firstinspires.ftc.teamcode.auto.AutoParams.jewelTurnTimeoutS;
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
    private org.firstinspires.ftc.teamcode.subsystems.Hand Hand = new org.firstinspires.ftc.teamcode.subsystems.Hand(); // Claw for glyphs and things
    private org.firstinspires.ftc.teamcode.subsystems.Lift Lift = new org.firstinspires.ftc.teamcode.subsystems.Lift();
    public static VuMark vumark = new VuMark();
    public static ElapsedTime runtime = new ElapsedTime();
    private boolean isAuton = false; // Are we running auto

    //Init Methods

    public void waitForStart(LinearOpMode linearOpMode) {
        linearOpMode.waitForStart();
        runtime.reset();
    }

    public void initializeRobot(LinearOpMode linearOpMode, boolean auton){
        getDrive().init(linearOpMode, auton);
        getJewel().init(linearOpMode, auton);
        getHand().init(linearOpMode, auton);
        getLift().init(linearOpMode, auton);
        if(auton){
            AutoTransitioner.transitionOnStop(linearOpMode, "Teleop"); //Auto Transitioning

            this.isAuton = auton;

            //startVuforia();
            this.vumark.setup(FRONT);
        }
    }

    //Auto Methods

    //Lift

    public void initialLift(boolean color){
        if(!color) {
            //Blue
            getHand().grabFrontClose();
            getHand().grabBackOpen();
        }

        else if(color){
            //Red
            getHand().grabBackClose();
            getHand().grabFrontOpen();
        }
        sleep(300);
        getLift().setLiftPower(0.4);
        sleep(500);
        getLift().brakeLift();
    }

    public void liftLower(){
        getLift().setLiftPower(-0.2);
        sleep(250);
        getLift().brakeLift();
    }

    //Vuforia

    public RelicRecoveryVuMark columnVote(LinearOpMode linearOpMode, double timeoutS){
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

    //Jewel

    public void jewelRed(){

        switch (getJewel().vote(this, jewelColorTimeoutS)) {
            case RED:
                //getDrive().encoderDrive(0.4, -5, -5, 5);
                //rotate clockwise
                getDrive().turnCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);


                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);

                //rotate counter-clockwise
                getDrive().turnCCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);
                break;
            case BLUE:
                //getDrive().encoderDrive(0.4, 5, 5, 5);
                //rotate counter-clockwise
                getDrive().turnCCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);

                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);

                //rotate clockwise
                getDrive().turnCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);
                break;
            case UNKNOWN:
                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);
                break;
            default:
                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);
        }
    }

    public void jewelBlue(){
        switch (getJewel().vote(this, jewelColorTimeoutS)) {
            case RED:
                //getDrive().encoderDrive(0.2, -5, -5, 5);
                //rotate counter-clockwise
                getDrive().turnCCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);

                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);

                //rotate clockwise
                getDrive().turnCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);
                break;
            case BLUE:
                //getDrive().encoderDrive(0.2, 5, 5, 5);
                //rotate clockwise
                getDrive().turnCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);

                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);

                //rotate counter-clockwise
                getDrive().turnCCW(jewelTurn, autoTurnSpeed, jewelTurnTimeoutS);
                break;
            case UNKNOWN:
                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);
                break;
            default:
                getJewel().retract(); // Retract Jewel arm
                sleep(jewelArmDelayMS);
        }
    }

    //Glyph

    public void glyphScoreR1(){

        switch (columnVote(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 38, 38, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                glyphRedScore();
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 32, 32, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                glyphRedScore();
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 26, 26, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                glyphRedScore();
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 32, 32, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                glyphRedScore();
                getDrive().encoderDrive(autoSpeed, -6, -6, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreB1(){

        switch (columnVote(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -26, -26, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                glyphBlueScore();
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into left column
                break;

            case CENTER:
                telemetry.addData("Column", "Center");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -32, -32, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                glyphBlueScore();
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into center column
                break;

            case RIGHT:
                telemetry.addData("Column", "Right");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -38, -38, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                glyphBlueScore();
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into right column
                break;
            case UNKNOWN:
                telemetry.addData("Column", "Unknown");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -32, -32, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                glyphBlueScore();
                getDrive().encoderDrive(autoSpeed, 6, 6, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreR2(){
        switch (columnVote(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, 24, 24, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, 18, 18, 4);
                getDrive().turnCW(90, autoTurnSpeed, 3);
                glyphRedScore();
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
                glyphRedScore();
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
                glyphRedScore();
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
                glyphRedScore();
                getDrive().encoderDrive(0.3, -3, -3, 2);
                getDrive().turnCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, -2, -2, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphScoreB2(){
        switch (columnVote(this, 7)){

            case LEFT:
                telemetry.addData("Column", "Left");
                telemetry.update();

                getDrive().encoderDrive(autoSpeed, -24, -24, 4);
                getDrive().turnCW(90, autoTurnSpeed, 4);
                getDrive().encoderDrive(autoSpeed, -6, -6, 4);
                getDrive().turnCCW(90, autoTurnSpeed, 3);
                glyphBlueScore();
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
                glyphBlueScore();
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
                glyphBlueScore();
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
                glyphBlueScore();
                getDrive().encoderDrive(0.3, 3, 3, 2);
                getDrive().turnCCW(45, autoTurnSpeed, 3);
                getDrive().encoderDrive(autoSpeed, 2, 2, 2);

                //put glyph into center column
                break;
        }
    }

    public void glyphRedScore(){

        getDrive().encoderDrive(autoSpeed, 6, 6, 4);

        getHand().grabBackOpen();
        sleep(100);
        getDrive().encoderDrive(autoSpeed, -2,-2, 2);
        liftLower();

        getDrive().encoderDrive(autoSpeed, 6, 6, 2);
        sleep(300);
        //getDrive().encoderDrive(autoSpeed, -6, -6, 2);
    }

    public void glyphBlueScore(){

        getDrive().encoderDrive(autoSpeed, -6, -6, 4);

        getHand().grabFrontOpen();
        sleep(100);
        getDrive().encoderDrive(autoSpeed, 2, 2, 2);
        liftLower();

        getDrive().encoderDrive(0.3, -8, -8, 2);
        sleep(300);
    }

    //use timeoutS to ensure we have enough time to park before the end of autonomous
    public void multiGlyphB1(double timeoutS){

        //drive to pile
        getDrive().encoderDrive(autoSpeed, 24, 24, 5);

        //double grab
        getDrive().encoderDrive(autoSpeed + 0.1, 6, 6, 3);
        getHand().grabBackClose();
        getDrive().encoderDrive(autoSpeed, -5, -5, 4);
        getDrive().turnCW(180, autoTurnSpeed, 4);
        getDrive().encoderDrive(0.7, -6, -6, 3);
        getHand().grabFrontClose();

        //drive back
    }

    public void multiGlyphB2(double timeoutS){

    }

    public void multiGlyphR1(double timeoutS){
        getDrive().encoderDrive(autoSpeed, -24, -24, 5);

        //double grab
        getDrive().encoderDrive(autoSpeed + 0.1, -6, -6, 3);

        getHand().grabBackClose();
        getDrive().encoderDrive(autoSpeed, -5, -5, 4);
        getDrive().turnCW(180, autoTurnSpeed, 4);
        getDrive().encoderDrive(0.7, -6, -6, 3);
        getHand().grabFrontClose();

    }

    public void multiGlyphR2(double timeoutS){

    }

    //Telemetry

    public void updateTelemetry(LinearOpMode linearOpMode) {

            if (!isAuton)
                linearOpMode.telemetry.addData("Match Time", 120 - runtime.seconds());
            getDrive().outputToTelemetry(linearOpMode.telemetry);
            getJewel().outputToTelemetry(linearOpMode.telemetry);
            getHand().outputToTelemetry(linearOpMode.telemetry);
            linearOpMode.telemetry.update();

    }

    //Other

    public void finalAction(){

            getDrive().stop();
            getJewel().stop();
            getHand().stop();
            getLift().stop();

            requestOpModeStop();

    }

    public org.firstinspires.ftc.teamcode.subsystems.Drive getDrive() {
        return Drive;
    }

    public org.firstinspires.ftc.teamcode.subsystems.Jewel getJewel() {
        return Jewel;
    }

    public org.firstinspires.ftc.teamcode.subsystems.Hand getHand() {
        return Hand;
    }

    public org.firstinspires.ftc.teamcode.subsystems.Lift getLift () { return Lift; }
}


