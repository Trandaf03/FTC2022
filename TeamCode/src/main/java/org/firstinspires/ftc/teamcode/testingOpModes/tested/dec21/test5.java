package org.firstinspires.ftc.teamcode.testingOpModes.tested.dec21;


/**
 * CONCLUSION
 * FORWARD/BACKWARD WORKS WELL , IF U PUT 25CM IT GOES 25CM
 * LEFT/RIGHT : IF U PUT 25 IT GOES 15 AND IT'S NOT GOING STRAIGHT
 * */
/*
@Disabled
@Deprecated
@TeleOp(name = "WIP with the homemade ROADRUNNER", group = "testing")
public class test5 extends LinearOpMode {

    driveComponents drive = new driveComponents();
    robotMovement movement = new robotMovement();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_WITHOUT);
        movement.setUpRobotMovement(hardwareMap, drive.getLeftFront(), drive.getLeftRear(), drive.getRightFront(), drive.getRightRear());

        waitForStart();
        if(opModeIsActive());

        movement.moveRobot(0,25,1);
    }
}
*/