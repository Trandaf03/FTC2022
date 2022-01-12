package org.firstinspires.ftc.teamcode.controlledPeriod;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.handlingComponents;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;
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

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);


        waitForStart();
        if(opModeIsActive());

        sleep(2000);

    }

    /**original teleop function

     double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
     double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
     double rightX = -gamepad1.right_stick_x;

     final double v1 = r * Math.cos(robotAngle) + rightX;
     final double v2 = r * Math.sin(robotAngle) - rightX;
     final double v3 = r * Math.sin(robotAngle) + rightX;
     final double v4 = r * Math.cos(robotAngle) - rightX;

     frontLeft.setPower(v1);
     frontRight.setPower(v2);
     backLeft.setPower(v3);
     backRight.setPower(v4);
     */

    /*//calcul care merge
    Math.sin(angle + (Math.PI/4)) * speed
     */


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
    public void spline2(double xDistance, double yDistance, double speed, double r ) throws InterruptedException{

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

    //deducere
    //TODO
    public void deducere(double xDistance, double yDistance, double speed, double turn) throws InterruptedException{

        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        drive.setMotorsEnabled();



        do {
            double r = Math.hypot(xDistance, yDistance);
            double robotAngle = Math.atan2(yDistance, -xDistance) - Math.PI / 4;
            double rightX = -turn;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;


        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && opModeIsActive());

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.setMotorsDisabled();

    }


}
