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


@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {

    driveComponents drive = new driveComponents();
    collectingComponents collector = new collectingComponents();
    handlingComponents handle = new handlingComponents();
    duckRotation ducky = new duckRotation();
    robotStopping stop = new robotStopping();

    boolean collectorIsPowered = false;
    boolean duckyIsPowered = false;
    int servoPosition = 1;
    handlingComponents.REMOVING_POSITIONS lastPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.driveInitialization(hardwareMap, robotDirection.ROBOT_DIRECTIONS.FORWARD,
                powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        collector.initCollecting(hardwareMap);
        handle.initHandlingComponents(hardwareMap);
        ducky.initDuckRotation(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            drive.robotVelocityController(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x);

            //COLLECTOR
            if(this.gamepad1.a){
                collectorIsPowered = !collectorIsPowered;
                this.sleep(200);
            }

            if(collectorIsPowered == true){
                collector.setCollectingPower(1, collectingComponents.COLLECTING_DIRECTION.FORWARD);

            } else {
                stop.collectorStop(collector.returnCollectingMotor());
            }
            // DUCKY
            if(this.gamepad1.b){
                duckyIsPowered = !duckyIsPowered;
                this.sleep(200);
            }
            if(duckyIsPowered == true){
                ducky.startDucky(1, duckRotation.DUCKY_DIRECTION.FORWARD);
            } else {
                stop.duckyStop(ducky.returnDuckyMotor());
            }

            if(this.gamepad1.x){
                if(servoPosition == 1){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_UP_POS);
                    servoPosition = 2;
                    this.sleep(200);
                }
                if(servoPosition == 2){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_DOWN_POS);
                    servoPosition = 0;
                    this.sleep(200);
                }
                if(servoPosition == 0){
                    handle.setServoPosition(handlingComponents.COLLECTING_POSITIONS.SERVO_MID_POS);
                }
            }

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
           /* if(this.gamepad1.dpad_up){
                handle.sliderGoToPosition(lastPosition);
            }
        */
            telemetry.addData("servopos", handle.getCurrentPosition());
            telemetry.update();
        }
    }
}
