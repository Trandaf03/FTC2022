package org.firstinspires.ftc.teamcode.utilities.odometry;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;

public class odometry {

    DcMotorEx forwardEncoder = null;
    DcMotorEx leftEncoder = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private static final double robotLenght = 33.5;
    private static final double  robotWidth = 31.5;

    private static final double forwardDistanceFromCenter = -10;
    private static final double leftDistanceFromCenter = -5;

    static final double COUNTS_PER_MOTOR_REV    = 1000 ;
    static final double DRIVE_GEAR_REDUCTION    = 1 ;
    static final double WHEEL_DIAMETER_CM   = 4 ;
    static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);


    public odometry(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initOdometry();
    }
    public odometry(){

    }

    public double returnX(){
        return forwardEncoder.getCurrentPosition();
      }
    public double returnY(){
        return leftEncoder.getCurrentPosition();
    }

    public void initOdometry(){
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



}
