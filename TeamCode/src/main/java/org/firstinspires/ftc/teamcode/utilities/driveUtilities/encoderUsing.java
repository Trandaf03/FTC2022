package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 * Class created to facilitate the programming of robot drive encoders
 *
 * */

public class encoderUsing {

    //The 4 robot drive motors
    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;

    //Encoder modes
    public enum ENCODER_RUNNING_MODE {
        STOP_AND_RESET, RUN_USING, RUN_WITHOUT, TO_POSITION;
    }

    public encoderUsing(){

    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear){
        a = leftFront;
        b = leftRear;
        c = rightFront;
        d = rightRear;
    }

    /**
     * Main function used for setting the encoders mode
     * */
    public void setEncoderMode(ENCODER_RUNNING_MODE encoderMode){
        switch (encoderMode){
            case STOP_AND_RESET:
                a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case RUN_USING:
                a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT:
                a.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                c.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TO_POSITION:
                a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            default:
                break;

        }
    }

    /**
     * Functions used to set the target position of the robot, used in Autonomous period
     * */
    public void setTargetPositionXmovement(int position){
        a.setTargetPosition(position);
        b.setTargetPosition(position);
        c.setTargetPosition(position);
        d.setTargetPosition(position);
    }
    public void setTargetPositionYmovement(int position){
        a.setTargetPosition(position);
        b.setTargetPosition(-position);
        c.setTargetPosition(-position);
        d.setTargetPosition(position);
    }
    public void splineSetTargetPosition1(int Position){
        c.setTargetPosition(Position);
        b.setTargetPosition(Position);
    }
    public void splineSetTargetPostion2(int Position){
        a.setTargetPosition(Position);
        d.setTargetPosition(Position);
    }


}
