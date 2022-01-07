package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**
 *
 * Class created to facilitate the programming of robot power behavior of the motors
 *
 * */


public class powerBehavior {

    //The 4 robot drive motors
    private DcMotorEx leftfront = null;
    private DcMotorEx leftrear = null;
    private DcMotorEx rightfront = null;
    private DcMotorEx rightrear = null;

    //Breaking modes
    public enum ROBOT_BREAKING{
        BRAKE, FLOAT;
    }

    public powerBehavior(){


    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        leftfront = leftFront;
        leftrear = leftRear;
        rightfront = rightFront;
        rightrear = rightRear;

    }
    /**
     * Main function used for setting the power behavior
     **/
    public void setBreakingMode(ROBOT_BREAKING breakingMode){
        switch (breakingMode){
            case BRAKE:
                leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightrear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case FLOAT:
                leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightrear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default:
                break;

        }
    }



}
