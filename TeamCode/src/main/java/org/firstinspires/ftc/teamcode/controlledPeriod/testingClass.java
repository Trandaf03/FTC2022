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

        spline(4*62,3*62,1, 1); // PID Correction
        //drive.xMovementWithPIDandGyroCorection(122, 1);
        sleep(2000);

    }

    /**original teleop function
     
            double y = gamepad1.left_stick_y * ;
            double x = gamepad1.left_stick_x * ;
            double rx = -gamepad1.right_stick_x * ;

            drive.stangaFata.setPower(y + x + rx);
            drive.stangaSpate.setPower(y - x + rx);
            drive.dreaptaFata.setPower(y - x - rx);
            drive.dreaptaSpate.setPower(y + x - rx);
     */



    public void spline(double xDistance, double yDistance, double speed, double invarte) throws InterruptedException{

        xDistance *= 1.1;
        yDistance *= 1.5;

        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        double turn = -Math.abs(invarte);
        drive.setMotorsEnabled();
        Thread.sleep(100);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        drive.encoders.setTargetPositionXmovement(-(int)distance);

        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        double power1 = drive.setMotorPower(Math.sin(angle + (Math.PI/4)) * speed);
        double power2 = drive.setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);



        double correctionAngle;
        do {
            drive.leftFront.setPower(Math.sin(angle + (Math.PI / 4)) * speed + turn);
            drive.rightRear.setPower(Math.sin(angle + (Math.PI / 4)) * speed + turn);

            drive.rightFront.setPower(Math.sin(angle - (Math.PI / 4)) * speed + turn );
            drive.leftRear.setPower(Math.sin(angle - (Math.PI / 4)) * speed + turn) ;


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
}
