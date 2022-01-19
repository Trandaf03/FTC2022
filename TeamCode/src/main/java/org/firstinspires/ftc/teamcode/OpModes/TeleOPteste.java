package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.odometry;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "TeleopTeste")
public class TeleOPteste extends LinearOpMode {



    driveComponents drive = new driveComponents();
    odometry odometry = new odometry();

    private static final double     COUNTS_PER_MOTOR_REV    = 8949.99995556 / 6;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    private static final double     WHEEL_DIAMETER_CM   = 4 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);


        waitForStart();
        if(opModeIsActive() && !isStopRequested() ){
            //drive.robotVelocityController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            /**TODO :
            /**TODO :
             *             Test distance correction
             *             Test progressive power
             */
            drive.driveY(62, 0.5, COUNTS_PER_CM);
            this.sleep(500);
            //odometry.driveY(62, -0.5);


        }


    }

}