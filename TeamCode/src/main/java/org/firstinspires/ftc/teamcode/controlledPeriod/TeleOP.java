package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.collectingComponents;
import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.robotComponents.duckRotation;
import org.firstinspires.ftc.teamcode.robotComponents.handlingComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotStopping;

/**
 *
 * Main TeleOp for this season, isn't yet finished.
 *
 **/

@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {

    /**
     * Calling every component on the robot
     * */
    driveComponents drive = new driveComponents();
    collectingComponents collector = new collectingComponents();
    handlingComponents handle = new handlingComponents();
    duckRotation ducky = new duckRotation();
    robotStopping stop = new robotStopping();

    /**
     * Variables used for powering the collector, ducky motor and the servo from the pulley
     * */
    boolean collectorIsPowered = false;
    boolean duckyIsPowered = false;
    int servoPosition = 1;
    handlingComponents.REMOVING_POSITIONS lastPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initializating the robot components:
         * To initialize drive class you need: the robot hardwaremap, the direction of movement, the motor braking mode and the encoder running mode
         * To initialize collector class you need: the robot hardwaremap
         * To initialize handle class you need: the robot hardwaremap
         * To initialize ducky class you need: the robot hardwaremap
         * */
        drive.init(hardwareMap, telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        handle.initHandlingComponents(hardwareMap);
        ducky.initDuckRotation(hardwareMap);

        /**
         * Waiting for the opMode to start
         * */
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            /**
             * Robot movement for every direction
             * */
            drive.robotVelocityController(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x);

            /**
             * Collector power on/off
             * */
            if(this.gamepad1.a){
                collectorIsPowered = !collectorIsPowered;
                this.sleep(200);
            }

            if(collectorIsPowered == true){
                collector.setCollectingPower(1, collectingComponents.COLLECTING_DIRECTION.FORWARD);

            } else {
                stop.collectorStop(collector.returnCollectingMotor());
            }
            /**
             * Ducky motor power on/off
             * */
            if(this.gamepad1.b){
                duckyIsPowered = !duckyIsPowered;
                this.sleep(200);
            }
            if(duckyIsPowered == true){
                ducky.startDucky(1, duckRotation.DUCKY_DIRECTION.FORWARD);
            } else {
                stop.duckyStop(ducky.returnDuckyMotor());
            }

            /**
             * Servo movement
             * */
            if(this.gamepad1.x){
                if(servoPosition == 1){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_UP_POS);
                    servoPosition = 2;
                }
                if(servoPosition == 2){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_DOWN_POS);
                    servoPosition = 0;
                }
                if(servoPosition == 0){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_MID_POS);
                }
                this.sleep(200);
            }

            /**
             * Pulley movement
             * */
            if(this.gamepad1.dpad_down){
                handle.sliderGoToPosition(handlingComponents.REMOVING_POSITIONS.LOW_POS);
                lastPosition = handlingComponents.REMOVING_POSITIONS.LOW_POS;
            }
            if(this.gamepad1.dpad_left){
                handle.sliderGoToPosition(handlingComponents.REMOVING_POSITIONS.MID_POS);
                lastPosition = handlingComponents.REMOVING_POSITIONS.MID_POS;
            }
            if(this.gamepad1.dpad_right){
                handle.sliderGoToPosition(handlingComponents.REMOVING_POSITIONS.HIGH_POS);
                lastPosition = handlingComponents.REMOVING_POSITIONS.HIGH_POS;
            }

        }
    }
}
