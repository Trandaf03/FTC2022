package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public void startDucky(double motorPower, DUCKY_DIRECTION direction){
        if(direction == DUCKY_DIRECTION.FORWARD){
            ducky.setPower(abs(motorPower));
        }
        if(direction == DUCKY_DIRECTION.REVERSE){
            ducky.setPower(-abs(motorPower));
        }

    }
    public void stop(){
        ducky.setPower(0);
    }


    private double abs(double value){
        if(value < 0)
            return -value;
        else return value;
    }

}
