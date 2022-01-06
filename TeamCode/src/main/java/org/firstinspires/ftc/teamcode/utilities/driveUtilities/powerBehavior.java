package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class powerBehavior {

    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;

    public enum ROBOT_BREAKING{
        BRAKE, FLOAT;
    }

    public powerBehavior(){

    }

    public void setMotorsName(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear){
        a = leftFront;
        b = leftRear;
        c = rightFront;
        d = rightRear;

    }

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
