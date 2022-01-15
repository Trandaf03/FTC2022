package org.firstinspires.ftc.teamcode.robotComponents;


import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;

public class odometry {

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

               drive.setMotorsEnabled();
               distance = distance * COUNTS_PER_CM;


            forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

               drive.setRobotMotorsPower(power);

               while(forwardEncoder.getCurrentPosition() < distance){
                   telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/COUNTS_PER_CM);
                   telemetry.update();
               }

               drive.setRobotMotorsPower(0);

               if(forwardEncoder.getCurrentPosition() > distance){
                   driveY(-forwardEncoder.getCurrentPosition(), 1);
               }
               drive.setMotorsDisabled();
    }


    //laterale
    public void driveX(double distance, double power){

               drive.setMotorsEnabled();
               distance = distance * COUNTS_PER_CM;


               forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

               forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

               drive.strafePower(power);
               while(forwardEncoder.getCurrentPosition() < distance){
                   telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/COUNTS_PER_CM);
                   telemetry.update();
               }

               if (forwardEncoder.getCurrentPosition() != 0){


                }

               drive.setRobotMotorsPower(0);
               drive.setMotorsDisabled();
    }

    public void holonomicDrive(double xDistance, double yDistance, double speed, double t) throws InterruptedException{

        xDistance *= 1.1;
        yDistance *= 1.5;

        //distanta de mers --> ipotenuza
        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;
        double encoder_distance = Math.hypot(leftEncoder.getCurrentPosition(), forwardEncoder.getCurrentPosition());

        //double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        drive.setMotorsEnabled();

        double y = -yDistance;
        double x = xDistance;
        double rx = t;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);




        do {
            double v1 =( y + x + rx)/denominator;
            double v2 = (y - x + rx)/denominator;
            double v3 = (y - x - rx)/denominator;
            double v4 =(y + x - rx)/denominator;

            drive.leftFront.setPower(v1);
            drive.leftRear.setPower(v2);
            drive.rightFront.setPower(v3) ;
            drive.rightRear.setPower(v4);



        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && encoder_distance < distance);

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.setMotorsDisabled();

    }
}
