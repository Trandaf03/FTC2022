package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 *
 * Class created to stop the robot components
 *
 **/


public class robotStopping {
    private DcMotorEx leftfront;
    private DcMotorEx leftrear;
    private DcMotorEx rightfront;
    private DcMotorEx rightrear;

    public robotStopping(){

    }
    public void setMotorsName(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        leftfront = leftFront;
        leftrear = leftRear;
        rightfront = rightFront;
        rightrear = rightRear;

    }
    public void driveStop(){
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
    }
    public void collectorStop(DcMotor motorName){
        motorName.setPower(0);
    }
    public void duckyStop(DcMotor motorName){
        motorName.setPower(0);
    }


}
