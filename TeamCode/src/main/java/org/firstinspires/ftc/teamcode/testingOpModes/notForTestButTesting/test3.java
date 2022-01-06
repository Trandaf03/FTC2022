package org.firstinspires.ftc.teamcode.testingOpModes.notForTestButTesting;
/*
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "TeleopTesting 1", group = "testing")
public class test3 extends LinearOpMode {

    private driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.FLOAT, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            drive.robotControl(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);
        }

    }
}
*/