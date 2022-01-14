package org.firstinspires.ftc.teamcode.utilities.odometry;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;

public class Odometry {

    DcMotorEx forwardEncoder = null;
    DcMotorEx leftEncoder = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public driveComponents drive;

    private static final double robotLenght = 33.5;
    private static final double  robotWidth = 31.5;

    private static final double forwardDistanceFromCenter = -10;
    private static final double leftDistanceFromCenter = -5;

    //8949.99995556 / 6 --> valoarea perfecta 
    private static final double     COUNTS_PER_MOTOR_REV    = 8949.99995556 / 6;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    private static final double     WHEEL_DIAMETER_CM   = 4 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    public odometry(){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;
        initOdometry();
    }

    public double returnY(){
        return forwardEncoder.getCurrentPosition();
    }
    public double returnX(){
        return leftEncoder.getCurrentPosition();
    }

    private void initOdometry(){
        forwardEncoder = hardwareMap.get(DcMotorEx.class,"rotationEncoder");
        leftEncoder = hardwareMap.get(DcMotorEx.class,"ducking");

        forwardEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    


    //fata spate
    public void driveY(double distance, double power){
        
               setMotorsEnabled();
               distance = distance * COUNTS_PER_CM;
        
        
               forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

               forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
               setRobotMotorsPower(power);

               while(forwardEncoder.getCurrentPosition() < distance){
                   telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/ceva);
                   telemetry.update();
               }
        
               setRobotMotorsPower(0);

               if(forwardEncoder.getCurrentPosition() > distance){
                   driveY(-forwardEncoder.getCurrentPosition(), 1);
               }
               setMotorsDisabled();
    }
    

    //laterale
    public void driveX(double distance, double power){
        
               setMotorsEnabled();
               distance = distance * COUNTS_PER_CM;
        
        
               forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
               strafePower(power);
               while(forwardEncoder.getCurrentPosition() < distance){
                   telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/ceva);
                   telemetry.update();
               }

               if (forwardEncoder.getCurrentPosition() != 0){
                    

                }
        
               setRobotMotorsPower(0);
               setMotorsDisabled();
    }
}
