package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.odometry;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "odometryRotationTest", group = "testeodometrie")
public class odometryRotationTest extends LinearOpMode {

    driveComponents drive = new driveComponents();
    odometry odometry = new odometry();
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        double COUNTS_PER_CM_ENCODER = odometry.COUNTS_PER_CM;

        waitForStart();
        if(opModeIsActive()) return;
        sleep(2000);

        drive.rotateRobot(90,0.5);
        telemetry.addData("xTics", drive.forwardEncoder.getCurrentPosition()/COUNTS_PER_CM_ENCODER);
        telemetry.addData("yTics", drive.leftEncoder.getCurrentPosition()/COUNTS_PER_CM_ENCODER);

        // de aici valoarea o iei , o imparti la 90 si vezi cati COUNTS_PER_CM inseamna o rotire

        // deoarece gyro e kinda fked up, fa testul separat, si de mai multe ori
    }
}
