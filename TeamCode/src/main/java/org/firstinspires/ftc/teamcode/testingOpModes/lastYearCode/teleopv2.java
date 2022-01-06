package org.firstinspires.ftc.teamcode.testingOpModes.lastYearCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testingOpModes.lastYearCode.Drive;

@Disabled
@Deprecated
@TeleOp(name = "gkfkodnfjdfndjfdjfd")
public class teleopv2 extends LinearOpMode {

    Drive drive = new Drive();

    double cutieViteze = 1;
    double vitezaAruncare = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        drive.initializareDrive(hardwareMap);

        while(opModeIsActive() && !isStopRequested()){

            // drive
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            drive.setMotorPowers(r,robotAngle,rightX, cutieViteze);

            if(this.gamepad1.right_trigger >= 0.5)
                cutieViteze = 1;
            if(this.gamepad1.left_trigger >= 0.5)
                cutieViteze = 0.5;

            telemetry.addData("Viteza drive", cutieViteze);
            telemetry.addData("Viteza aruncare", vitezaAruncare);
            telemetry.update();


        }

    }
}
