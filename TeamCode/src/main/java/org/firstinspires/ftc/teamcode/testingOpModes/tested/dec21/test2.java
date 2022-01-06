package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec21;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;


/** TODO
*
*  test gyro functioning
*  if wrong:
*        - check return function
*        - use getangles function from example
*        - delete win32
*        - get a new life
 *
 *
 *        ROLLING US USED AS HEADING
*
* */
/*
@TeleOp(name = "GyroTesting", group = "testing")
public class test2 extends LinearOpMode {

    driveComponents drive = new driveComponents();
    Gyro gyro = new Gyro();

    private double heading;
    private double pitch;
    private double roll;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);
        gyro.initGyro(hardwareMap);

        waitForStart();


        while(opModeIsActive() && !isStopRequested()){
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            drive.setMotorPowers(r,robotAngle,rightX);

            heading = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);
            pitch = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.PITCH);
            roll = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.ROLL);

            telemetry.addData("heading", heading);
            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);
            telemetry.update();


        }
    }
}
*/