package org.firstinspires.ftc.teamcode.testingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;

@TeleOp(name = "ceva")
public class Testing extends LinearOpMode {

    driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

        //drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        DcMotorEx sliderMotor = hardwareMap.get(DcMotorEx.class, "slider");


        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if(this.gamepad1.left_stick_x != 0){
                sliderMotor.setVelocity(gamepad1.left_stick_y * 386.3 * 5);
            } else {
                sliderMotor.setPower(0);
            }

        }

        //drive.moveRobot(50,0,0.1);

    }
}
