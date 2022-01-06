package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class robotDirection {

    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;

    public enum ROBOT_DIRECTIONS{
        FORWARD, REVERSE;
    }

    public robotDirection(){

    }
    public void setMotorsName(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear){
        a = leftFront;
        b = leftRear;
        c = rightFront;
        d = rightRear;
    }
    public void setRobotDirection(ROBOT_DIRECTIONS direction){
        switch (direction){
            case FORWARD:
                a.setDirection(DcMotorSimple.Direction.FORWARD);
                b.setDirection(DcMotorSimple.Direction.REVERSE);
                c.setDirection(DcMotorSimple.Direction.REVERSE);
                d.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case REVERSE:
                a.setDirection(DcMotorSimple.Direction.REVERSE);
                b.setDirection(DcMotorSimple.Direction.FORWARD);
                c.setDirection(DcMotorSimple.Direction.FORWARD);
                d.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            default:
                break;
        }
    }




}
