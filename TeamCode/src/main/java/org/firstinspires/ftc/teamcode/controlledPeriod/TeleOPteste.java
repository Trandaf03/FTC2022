package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "teleopteste")
public class TeleOPteste extends LinearOpMode {

    driveComponents drive = new driveComponents();
    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            drive.robotVelocityController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("rx: ", gamepad1.right_stick_x);
            telemetry.addData("x: ", gamepad1.left_stick_x);
            telemetry.addData("y: ", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
