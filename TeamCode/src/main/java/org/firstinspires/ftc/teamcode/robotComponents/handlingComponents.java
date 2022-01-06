package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class handlingComponents {

    private DcMotor sliderMotor = null;
    private Servo rotationServo = null;

    //TODO Set corect data to the next 3 variables
    static final double COUNTS_PER_MOTOR_REV    = 751.8 ;
    static final double DRIVE_GEAR_REDUCTION    = 1 ;
    static final double WHEEL_DIAMETER_CM   = 4.5 ;
    static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    public enum COLLECTING_POSITIONS{
        SERVO_DOWN_POS, SERVO_UP_POS, SERVO_MID_POS;
    }
    public enum REMOVING_POSITIONS{
        LOW_POS, MID_POS, HIGH_POS, ZERO_POS
    }

    public handlingComponents(){

    }

    public Servo returnServo(){
        return rotationServo;
    }

    public void initHandlingComponents(HardwareMap map){
        sliderMotor = map.get(DcMotor.class,"slider");
        rotationServo = map.get(Servo.class,"rotationServo");

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public void sliderGoToPosition(REMOVING_POSITIONS position){
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setTargetPosition(returnPositionTicks(position));
        sliderMotor.setPower(0.25);
    }

    // TODO Set correct servo positions
    public void setServoPosition(COLLECTING_POSITIONS position){
        if(position == COLLECTING_POSITIONS.SERVO_DOWN_POS){
            rotationServo.setPosition(0.12);
        }
        if(position == COLLECTING_POSITIONS.SERVO_MID_POS){
            rotationServo.setPosition(0.20);
        }
        if(position == COLLECTING_POSITIONS.SERVO_UP_POS){
            rotationServo.setPosition(0.32);
        }
    }

    //TODO Set correct distance for the motor to run
    private int returnPositionTicks(REMOVING_POSITIONS position){
        if(position == REMOVING_POSITIONS.LOW_POS){
            return (int) (10 * COUNTS_PER_CM);
        }
        if(position == REMOVING_POSITIONS.MID_POS){
            return (int) (25 * COUNTS_PER_CM);
        }
        if(position == REMOVING_POSITIONS.HIGH_POS){
            return (int) (60 * COUNTS_PER_CM);
        }
        return  0;
    }
    public int getCurrentPosition(){
        return  sliderMotor.getCurrentPosition();
    }


}
