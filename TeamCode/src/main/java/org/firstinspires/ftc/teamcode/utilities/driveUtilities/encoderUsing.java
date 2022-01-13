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
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    //Encoder modes
    public enum ENCODER_RUNNING_MODE {
        STOP_AND_RESET, RUN_USING, RUN_WITHOUT, TO_POSITION;
    }

    public encoderUsing(){

    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
    }

    /**
     * Main function used for setting the encoders mode
     * */
    public void setEncoderMode(ENCODER_RUNNING_MODE encoderMode){
        switch (encoderMode){
            case STOP_AND_RESET:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case RUN_USING:
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT:
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TO_POSITION:
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            default:
                break;

        }
    }

    /**
     * Functions used to set the target position of the robot, used in Autonomous period
     **/
    public void setTargetPositionXmovement(int position){
        leftFront.setTargetPosition(position);
        leftRear.setTargetPosition(position);
        rightFront.setTargetPosition(position);
        rightRear.setTargetPosition(position);
    }
    public void setTargetPositionYmovement(int position){
        leftFront.setTargetPosition(position);
        leftRear.setTargetPosition(-position);
        rightFront.setTargetPosition(-position);
        rightRear.setTargetPosition(position);
    }
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorsEnabled(){
        leftFront.setMotorEnable();
        rightFront.setMotorEnable();
        leftRear.setMotorEnable();
        rightRear.setMotorEnable();
    }
    public void setMotorsDisabled(){
        leftFront.setMotorDisable();
        rightFront.setMotorDisable();
        leftRear.setMotorDisable();
        rightRear.setMotorDisable();
    }


}
