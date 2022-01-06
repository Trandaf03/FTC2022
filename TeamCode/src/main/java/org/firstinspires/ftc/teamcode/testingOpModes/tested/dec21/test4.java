package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec21;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

/**TODO
 *
 * Check if the drive gear reduction is 2/1 or 1/2
 *
 * CONCLUSION : IT'S 2
 * */

/*
@Autonomous(name = "GearReduction Test", group = "testing")
public class test4 extends LinearOpMode {

    driveComponents drive = new driveComponents();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);
        waitForStart();

        if(opModeIsActive());
        //drive.test(25,1);

    }
}
*/