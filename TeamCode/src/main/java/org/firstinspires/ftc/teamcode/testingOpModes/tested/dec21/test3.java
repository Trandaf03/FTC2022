package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec21;

/**TODO
*
* Test autonomous to Teleop switching
* TOTAL FAILURE
* */

/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

@Autonomous(name = "AutoTOteleopEmergency", group = "testing")
public class test3 extends LinearOpMode {

    driveComponents drive = new driveComponents();
    boolean emergency = false;


    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            drive.test3Functionformovingautonomously(150,0.5,this.gamepad1.a);
            emergency = drive.isRobotEmergency();
            if(emergency == true){
                while(opModeIsActive() && !isStopRequested()){
                double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = -gamepad1.right_stick_x;
                drive.setMotorPowers(r,robotAngle,rightX);
               }
            }
        }

    }
}
*/