package org.firstinspires.ftc.teamcode.controlledPeriod;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.handlingComponents;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

/** TODO Test this
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

        spline(248,124,1); // PID Correction
        sleep(2000);

    }



    public void spline(double xDistance, double yDistance, double speed) throws InterruptedException{
        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.toRadians(Math.atan2(distance,yDistance));
        drive.setMotorsEnabled();
        Thread.sleep(100);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        drive.encoders.setTargetPositionXmovement(-(int)distance);

        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        double power1 = drive.setMotorPower(Math.sin(angle + (Math.PI / 4)) * speed);
        double power2 = drive.setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);

        drive.leftFront.setVelocity(power1);
        drive.rightRear.setVelocity(power1);

        drive.rightFront.setVelocity(power2);
        drive.leftRear.setVelocity(power2);

        double correctionAngle;
        do {
            correctionAngle = drive.checkDirection();
            if(correctionAngle != 0){
                drive.PIDCalculationSpline(power1,power2);
                drive.rotateRobotWithPID(drive.checkDirection());
            } else drive.PIDMovementSpline(power1,power2);

            telemetry.addData("unghi", angle);
            telemetry.addData("distance", distance);
            telemetry.addData("power1",power1);
            telemetry.addData("power2",power2);
            telemetry.update();

            power1 = drive.setMotorPower(Math.sin(angle + (Math.PI / 4)) * speed);
            power2 = drive.setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);

        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy());

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.setMotorsDisabled();

    }
}
