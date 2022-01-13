package org.firstinspires.ftc.teamcode.controlledPeriod;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;


// TODO watch this when i got home, bafta danciu
@TeleOp(name = "trandaf")
public class testetrandaf extends LinearOpMode {

    driveComponents drive = new driveComponents();
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry,  robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        double xDistance = 62*4, yDistance = 62*2, speed = 1, stopHeading = 90;

        waitForStart();
        if(opModeIsActive());
        ElapsedTime time = new ElapsedTime();

        double currentHeading = drive.gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);
        xDistance *= 1.1;
        yDistance *= 1.5;

        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;
        double angle = Math.atan2(xDistance,yDistance); // corect --> unghi in radiani

        //double turn = -Math.abs(-10);
        drive.encoders.setMotorsEnabled();
        Thread.sleep(100);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        drive.encoders.setTargetPositionXmovement(-(int)distance);

        drive.encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);


        double power1,power2;
        do {
            time.reset();
            power1 = Math.sin(angle + (Math.PI / 4)) * speed;// + turn;
            power2 = Math.sin(angle - (Math.PI / 4)) * speed;// + turn;

            if(drive.gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING) < currentHeading + stopHeading){
                drive.leftFront.setVelocity(drive.setMotorPower(power1 + 1));
                drive.rightRear.setVelocity(drive.setMotorPower(power1) - 1);
                drive.rightFront.setVelocity(drive.setMotorPower(power2 + 1));
                drive.leftRear.setVelocity(drive.setMotorPower(power2) - 1) ;
            } /*else {
            drive.leftFront.setVelocity(drive.setMotorPower(power1));
            drive.rightRear.setVelocity(drive.setMotorPower(power1));
            drive.rightFront.setVelocity(drive.setMotorPower(power2));
            drive.leftRear.setVelocity(drive.setMotorPower(power2)) ;}
*/

            telemetry.addData("unghi", angle);
            telemetry.addData("distance", distance / COUNTS_PER_CM);
            telemetry.addData("power1",drive.setMotorPower(power1));
            telemetry.addData("power2",drive.setMotorPower(power2));
            telemetry.update();

        } while(drive.leftFront.isBusy() && drive.rightRear.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && opModeIsActive());

        drive.leftFront.setVelocity(0);
        drive.rightRear.setVelocity(0);
        drive.rightFront.setVelocity(0);
        drive.leftRear.setVelocity(0);

        drive.encoders.setMotorsDisabled();
    }
}
