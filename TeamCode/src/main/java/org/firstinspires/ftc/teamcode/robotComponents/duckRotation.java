package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 * Class used to declare and use every component that is used in moving the ducky
 *
 * */

public class duckRotation {

    public DcMotorEx ducky = null;

    public enum DUCKY_DIRECTION{
        FORWARD, REVERSE;
    }
    public void initDuckRotation(HardwareMap map){
        ducky = map.get(DcMotorEx.class, "ducking");
    }

    public DcMotor returnDuckyMotor(){
        return ducky;
    }
    /**
     * Main function for powering the motor, where you need the power and the direction of movement
     * */
    public void startDucky(double motorPower, DUCKY_DIRECTION direction){
        if(direction == DUCKY_DIRECTION.FORWARD){
            ducky.setPower(Math.abs(motorPower));
        }
        if(direction == DUCKY_DIRECTION.REVERSE){
            ducky.setPower(-Math.abs(motorPower));
        }

    }

}
