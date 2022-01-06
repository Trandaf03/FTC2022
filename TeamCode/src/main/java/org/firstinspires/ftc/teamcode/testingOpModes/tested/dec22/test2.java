package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec22;


/**
 *
 * TEST SUCCESFULY, DEPRECRATED , now using test 3 output
 *
 * */


/*
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "TeleopTesting 2", group = "testing")
public class test2 extends LinearOpMode {

    private driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.FLOAT, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            drive.setMotorPowers(r,robotAngle,rightX);

        }

    }
}
*/