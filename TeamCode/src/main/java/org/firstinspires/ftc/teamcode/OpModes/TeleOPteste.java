package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.odometry;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "TeleopTeste")
public class TeleOPteste extends LinearOpMode {

    driveComponents drive = new driveComponents();
    odometry odometry = new odometry();


    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        //TODO : test this
        //odometry.initOdometry();

        drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            drive.robotVelocityController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            double fata = drive.forwardEncoder.getCurrentPosition();
            double dreapta = drive.leftEncoder.getCurrentPosition();

            //TODO : test X odometry wheel
            telemetry.addData("odometry fata : ", fata );
            telemetry.addData("odometry dreapta: ", dreapta );
            telemetry.update();


        }
    }
}
