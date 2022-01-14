package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;


@TeleOp(name = "testareCorectitudineTicuri", group = "testeodometrie")
public class testareTicsEncoder extends LinearOpMode {

    //8949.99995556 / 6 --> calibrare corecta pentru Y

    private static final double     COUNTS_PER_MOTOR_REV    = 8949.99995556 / 6;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    private static final double     WHEEL_DIAMETER_CM   = 4 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {


        drive.init(hardwareMap,telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if(opModeIsActive() && !isStopRequested()) {

            double distance = 60, power = 0.25;
            drive.odometryX(distance, power, COUNTS_PER_CM);
            drive.setMotorPower(0);
        }
    }
}
