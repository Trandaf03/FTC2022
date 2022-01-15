package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

/** TODO Test odometry
 * Testing class.
 **/

@TeleOp(name = "testing" )

public class testingClass extends LinearOpMode {


    driveComponents drive = new driveComponents();


    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        waitForStart();
        if(opModeIsActive());

        sleep(2000);

    }


    //aicea mere unghiul dar nu si curba
    public void spline(double xDistance, double yDistance, double speed) throws InterruptedException{

        xDistance *= 1.1;
        yDistance *= 1.5;



        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        drive.setMotorsEnabled();
        Thread.sleep(100);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        drive.encoders.setTargetPositionXmovement(-(int)distance);

        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);


        double v1 = (Math.sin(angle + (Math.PI/4)) * speed )* 386.3 * 20;
        double v2 = (Math.sin(angle - (Math.PI/4)) * speed )* 386.3 * 20;
        double v3 = (Math.sin(angle + (Math.PI/4)) * speed )* 386.3 * 20;
        double v4 = (Math.sin(angle - (Math.PI/4)) * speed )* 386.3 * 20;

        double correctionAngle;
        do {


            drive.leftFront.setVelocity(v1);
            drive.rightFront.setVelocity(v2);
            drive.leftRear.setVelocity(v3);
            drive.rightRear.setVelocity(v4);


            telemetry.addData("unghi", angle);
            telemetry.addData("distance", distance / COUNTS_PER_CM);
            telemetry.addData("v4", v4);
            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);

            telemetry.update();

        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && opModeIsActive());

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.setMotorsDisabled();

    }

    //aicea mere curba
    public void spline2(double xDistance, double yDistance, double speed, double r) throws InterruptedException{

        xDistance *= 1.1;
        yDistance *= 1.5;

        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        drive.setMotorsEnabled();
        Thread.sleep(100);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        drive.encoders.setTargetPositionXmovement(-(int)distance);

        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        double power1 = drive.setMotorPower(Math.sin(angle + (Math.PI/4)) * speed);
        double power2 = drive.setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);

        final double v1 = (speed* Math.cos(angle) + r) * 386.3 * 20;
        final double v2 = (speed* Math.sin(angle) - r) * 386.3 * 20;
        final double v3 = (speed* Math.sin(angle) + r) * 386.3 * 20;
        final double v4 = (speed* Math.cos(angle) - r) * 386.3 * 20;

        double correctionAngle;
        do {
            drive.leftFront.setVelocity(v1);
            drive.rightFront.setVelocity(v2);
            drive.leftRear.setVelocity(v3);
            drive.rightRear.setVelocity(v4);


            telemetry.addData("unghi", angle);
            telemetry.addData("distance", distance / COUNTS_PER_CM);
            telemetry.addData("power1",power1);
            telemetry.addData("power2",power2);
            telemetry.update();

        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && opModeIsActive());

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.setMotorsDisabled();

    }


    //TODO test this
    public void driveWithVectors(double y, double x, double r)
    {
        double magnitude = Math.sqrt((y*y) + (x*x));
        double angle = Math.atan2(x, -y) + (Math.PI / 4);

        double FL_BR = Math.sin(angle) * magnitude;
        double FR_BL = Math.cos(angle) * magnitude;

        drive.leftFront.setPower(FL_BR + r);
        drive.rightFront.setPower(FR_BL - r);
        drive.leftRear.setPower(FR_BL + r);
        drive.rightRear.setPower(FL_BR - r);
    }


}
