package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.odometry.odometry;

@TeleOp(name = "odometry")
public class odometryTest extends LinearOpMode {

    driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        waitForStart();
        drive.encoders.resetEncoders();
        while (opModeIsActive()) {

            drive.robotTest(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x);

        }
    }
}
