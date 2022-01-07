package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;


/**
 *
 * Class created to facilitate the programming of robot power behavior of the motors
 *
 * */


public class powerBehavior {

    //The 4 robot drive motors
    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;

    //Breaking modes
    public enum ROBOT_BREAKING{
        BRAKE, FLOAT;
    }

    public powerBehavior(){


    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear){
        a = leftFront;
        b = leftRear;
        c = rightFront;
        d = rightRear;

    }
    /**
     * Main function used for setting the power behavior
     **/
    public void setBreakingMode(ROBOT_BREAKING breakingMode){
        switch (breakingMode){
            case BRAKE:
                a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case FLOAT:
                a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default:
                break;

        }
    }



}
