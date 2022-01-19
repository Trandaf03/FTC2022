package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.VirtualButton;

import org.firstinspires.ftc.teamcode.robotComponents.handlingComponents;

@TeleOp(name = "culisanta")
public class clasaculisantatest extends LinearOpMode {

    handlingComponents handle = new handlingComponents();
    @Override
    public void runOpMode() throws InterruptedException {

        handle.initHandlingComponents(hardwareMap);

        handle.sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        handle.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int servoPosition = 1;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){

            if(this.gamepad1.left_stick_y != 0){
                handle.sliderMotor.setPower(this.gamepad1.left_stick_y * 0.1);
            } else {
                handle.sliderMotor.setPower(0);
            }

            //sus --> -2900
            //jos --> 1305
            //mijloc -- -2375


                if(this.gamepad1.x){
                    handle.rotationServo.setPosition(0.05);
                    this.sleep(200);
                }
                if(this.gamepad1.y){
                    handle.rotationServo.setPosition(0.15);
                    this.sleep(200);
                }
                if(this.gamepad1.b){
                    handle.rotationServo.setPosition(0.23);
                    this.sleep(200);
                }

                telemetry.addLine("am intrat aici");


            telemetry.addData("Pozitie culisanta in tics",handle.sliderMotor.getCurrentPosition());
            telemetry.addData("servo pos",handle.rotationServo.getPosition());

            telemetry.update();
        }
    }
}
