package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.odometry.odometry;

public class odometryTest extends LinearOpMode {

    driveComponents drive = new driveComponents();
    odometry odometry = new odometry(hardwareMap,telemetry,drive);
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        waitForStart();
        if(opModeIsActive()) return;

        odometry.drive.xMovement(100,10);
        telemetry.addData("xPosition",odometry.returnX());
        telemetry.update();

    }
}
