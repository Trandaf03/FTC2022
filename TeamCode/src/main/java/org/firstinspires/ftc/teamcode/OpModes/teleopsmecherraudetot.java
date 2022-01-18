package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.robotComponents.collectingComponents.COLLECTING_DIRECTION.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotComponents.collectingComponents;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.odometry;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@TeleOp(name = "teleopsmecherraudetot")
public class teleopsmecherraudetot extends LinearOpMode {

    driveComponents drive = new driveComponents();
    collectingComponents collectingComponents = new collectingComponents();



    @Override
    public void runOpMode() throws InterruptedException {


        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        collectingComponents.initCollecting(hardwareMap);

        boolean colecteaza = false;

        waitForStart();
        while (opModeIsActive() ){
            drive.robotVelocityController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.triangle && colecteaza == false){
                this.sleep(400);
                collectingComponents.setCollectingPower(1, FORWARD);
                colecteaza = true;
            }
            if (gamepad1.triangle && colecteaza == true){
                this.sleep(400);
                collectingComponents.setCollectingPower(0, FORWARD);
                colecteaza = false;
            }


        }


    }

}