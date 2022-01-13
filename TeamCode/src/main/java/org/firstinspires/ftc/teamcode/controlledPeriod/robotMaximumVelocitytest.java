package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.opencv.core.Mat;

@TeleOp(name="maximumvelocity test", group = "testeodometrie")
public class robotMaximumVelocitytest extends LinearOpMode {

    driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

            drive.init(hardwareMap,telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

            double maximumVelocity = 0;

            waitForStart();
            while(opModeIsActive()){
                double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = -gamepad1.right_stick_x;
                final double v1 = (r * Math.cos(robotAngle) + rightX) * 386.3 * 20;
                final double v2 = (r * Math.sin(robotAngle) - rightX) * 386.3 * 20;
                final double v3 = (r * Math.sin(robotAngle) + rightX) * 386.3 * 20;
                final double v4 = (r * Math.cos(robotAngle) - rightX) * 386.3 * 20;

                drive.leftFront.setVelocity(v1);
                drive.rightFront.setVelocity(v2);
                drive.leftRear.setVelocity(v3);
                drive.rightRear.setVelocity(v4);

                maximumVelocity = Math.max(maximumVelocity,Math.max(Math.max(v1,v2),Math.max(v3,v4)));

                telemetry.addData("Maximum velocity", maximumVelocity);
                telemetry.update();
            }

        telemetry.addData("Maximum velocity", maximumVelocity);
        telemetry.update();

    }
}
