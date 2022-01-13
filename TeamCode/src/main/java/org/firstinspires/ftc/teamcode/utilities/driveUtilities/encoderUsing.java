package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 *
 * Class created to facilitate the programming of robot drive encoders
 *
 * */

public class encoderUsing {

    //The 4 robot drive motors
    private DcMotorEx leftfront = null;
    private DcMotorEx leftrear = null;
    private DcMotorEx rightfront = null;
    private DcMotorEx rightrear = null;

    //Encoder modes
    public enum ENCODER_RUNNING_MODE {
        STOP_AND_RESET, RUN_USING, RUN_WITHOUT, TO_POSITION;
    }

    public encoderUsing(){

    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        leftfront = leftFront;
        leftrear = leftRear;
        rightfront = rightFront;
        rightrear = rightRear;
    }

    /**
     * Main function used for setting the encoders mode
     * */
    public void setEncoderMode(ENCODER_RUNNING_MODE encoderMode){
        switch (encoderMode){
            case STOP_AND_RESET:
                leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case RUN_USING:
                leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT:
                leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TO_POSITION:
                leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            default:
                break;

        }
    }

    /**
     * Functions used to set the target position of the robot, used in Autonomous period
     **/
    public void setTargetPositionXmovement(int position){
        leftfront.setTargetPosition(position);
        leftrear.setTargetPosition(position);
        rightfront.setTargetPosition(position);
        rightrear.setTargetPosition(position);
    }
    public void setTargetPositionYmovement(int position){
        leftfront.setTargetPosition(position);
        leftrear.setTargetPosition(-position);
        rightfront.setTargetPosition(-position);
        rightrear.setTargetPosition(position);
    }


}
