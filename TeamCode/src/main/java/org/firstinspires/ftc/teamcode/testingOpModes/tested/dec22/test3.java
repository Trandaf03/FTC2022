package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec22;


/**
 *
 * TEST SUCCESFULLY, NOW USING THIS AS DRIVE METHOD
 * !DRIVE POWER MULTIPLYER IS CAPPED BY MOTOR
 *
 * */
/*
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "TeleopTesting 3  CAREFULL", group = "testing")
public class test3 extends LinearOpMode {

    private test4_driveComponents drive = new test4_driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.FLOAT, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
           drive.robotVelocityController(this.gamepad1.left_stick_x,
                   this.gamepad1.left_stick_y,
                   this.gamepad1.right_stick_x,
                   15);

        }

    }
}
*/