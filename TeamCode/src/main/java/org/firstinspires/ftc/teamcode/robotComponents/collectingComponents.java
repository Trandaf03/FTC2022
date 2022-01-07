package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 * Class used to declare and use every component that is used for collecting freight
 *
 **/


public class collectingComponents {

    private DcMotor collectorMotor = null;

    public collectingComponents(){

    }

    public enum COLLECTING_DIRECTION{
        FORWARD, REVERSE;
    }

    public void initCollecting(HardwareMap map){
        collectorMotor = map.get(DcMotor.class, "collector");

    }
    public DcMotor returnCollectingMotor(){
        return collectorMotor;
    }

    /**
     * Main function for powering the motor, where you need the power and the direction of movement
     * */
    public void setCollectingPower(double power,COLLECTING_DIRECTION direction){
        if(direction == COLLECTING_DIRECTION.FORWARD){
            collectorMotor.setPower(power);
        }
        if(direction == COLLECTING_DIRECTION.REVERSE){
            collectorMotor.setPower(-power);
        }
    }
}
