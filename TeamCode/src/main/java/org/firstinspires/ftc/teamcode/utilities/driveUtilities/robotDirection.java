package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 *
 * Class created to facilitate the programming of robot direction of movement
 *
 * */

public class robotDirection {

    //The 4 robot drive motors
    private DcMotorEx leftfront = null;
    private DcMotorEx leftrear = null;
    private DcMotorEx rightfront = null;
    private DcMotorEx rightrear = null;

    //Robot movement directions
    public enum ROBOT_DIRECTIONS{
        FORWARD, REVERSE, LEFT, RIGHT;
    }

    public robotDirection(){

    }
    // initialization for the motors declared here
    public void setMotorsName(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        leftfront = leftFront;
        leftrear = leftRear;
        rightfront = rightFront;
        rightrear = rightRear;
    }

    /**
     * Main function used for setting the direction
     **/
    public void setRobotDirection(ROBOT_DIRECTIONS direction){
        switch (direction){
            case FORWARD:
                leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
                leftrear.setDirection(DcMotorSimple.Direction.REVERSE);
                rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightrear.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case REVERSE:
                leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
                leftrear.setDirection(DcMotorSimple.Direction.FORWARD);
                rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
                rightrear.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case LEFT:
                leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
                leftrear.setDirection(DcMotorSimple.Direction.REVERSE);
                rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightrear.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case RIGHT:
                leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
                leftrear.setDirection(DcMotorSimple.Direction.FORWARD);
                rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
                rightrear.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            default:
                break;
        }
    }




}
