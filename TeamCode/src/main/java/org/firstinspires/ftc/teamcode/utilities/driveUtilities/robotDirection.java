package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 *
 * Class created to facilitate the programming of robot direction of movement
 *
 * */

public class robotDirection {

    //The 4 robot drive motors
    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;

    //Robot movement directions
    public enum ROBOT_DIRECTIONS{
        FORWARD, REVERSE, LEFT, RIGHT;
    }

    public robotDirection(){

    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear){
        a = leftFront;
        b = leftRear;
        c = rightFront;
        d = rightRear;
    }

    /**
     * Main function used for setting the direction
     * */
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
            case LEFT:
                a.setDirection(DcMotorSimple.Direction.REVERSE);
                b.setDirection(DcMotorSimple.Direction.REVERSE);
                c.setDirection(DcMotorSimple.Direction.REVERSE);
                d.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case RIGHT:
                a.setDirection(DcMotorSimple.Direction.FORWARD);
                b.setDirection(DcMotorSimple.Direction.FORWARD);
                c.setDirection(DcMotorSimple.Direction.FORWARD);
                d.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            default:
                break;
        }
    }




}
