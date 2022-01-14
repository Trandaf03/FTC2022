package org.firstinspires.ftc.teamcode.controlledPeriod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.driveComponents;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Aflaticksencoder TUNE THIS", group = "testeodometrie")
public class aflaTicsEncoder extends LinearOpMode {

    driveComponents drive = new driveComponents();

    /*private static final double     COUNTS_PER_MOTOR_REV    = 386.3 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;
    private static final double     WHEEL_DIAMETER_CM   = 10 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    counts per motor rev = counts per cm * wheel diameter * 3.1415
    counts per cm = total counts / distance
*/
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap,telemetry, robotDirection.ROBOT_DIRECTIONS.FORWARD, powerBehavior.ROBOT_BREAKING.BRAKE, encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        List<Double> valori;
        waitForStart();
        if(opModeIsActive()) return;
        double val, valTicks;

        valori = new ArrayList<>();
        double power = 0.25, distance = 50, numberOf = 10;
        for(int i = 1; i <= numberOf ; i++){
            if(i % 2 == 1){
                val = drive.getEncoderTics(distance,power);
            }
            else{
                val = drive.getEncoderTics(-distance,power);
            }

            valTicks = drive.forwardEncoder.getCurrentPosition();
            valTicks = valTicks / distance;
            valTicks = valTicks * 4 * 1415;

            valori.add(valTicks);
            sleep(1000);
        }
        double medium = 0;
        for(int i = 1 ; i<= numberOf ; i++){
            telemetry.addData("Value " + i, valori.get(i));
            medium += valori.get(i);
        }
        telemetry.addData("Final value", medium/numberOf);
        telemetry.update();

    }
}