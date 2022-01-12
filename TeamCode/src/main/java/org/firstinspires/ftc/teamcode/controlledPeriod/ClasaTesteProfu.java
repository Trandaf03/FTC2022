package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotComponents.testingComponents;

public class ClasaTesteProfu extends LinearOpMode {

    testingComponents testing = new testingComponents(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        testing.initializare();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(this.gamepad1.left_stick_y !=0)
                testing.setUnMotorPower(this.gamepad1.left_stick_y);
        }
    }
}
