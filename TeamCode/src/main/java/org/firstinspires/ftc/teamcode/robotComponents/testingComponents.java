package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class testingComponents {

    private DcMotorEx unMotor = null;
    private DcMotorEx altMotor = null;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public testingComponents(HardwareMap hwMap, Telemetry telemetrie){
        hardwareMap = hwMap;
        telemetry = telemetrie;
    }


    public void initializare(){

        unMotor = hardwareMap.get(DcMotorEx.class,"collector");
        altMotor = hardwareMap.get(DcMotorEx.class, "ducking");

        unMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        altMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        unMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        altMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        unMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        altMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        unMotor.setPower(0);
        altMotor.setPower(0);

        telemetry.addLine("Intializarea robotului a fost facutaa");
        telemetry.update();
    }

    public void setUnMotorPower(double power){
        unMotor.setPower(power);
    }
    public void stop(){
        unMotor.setPower(0);
        altMotor.setPower(0);
    }
}
