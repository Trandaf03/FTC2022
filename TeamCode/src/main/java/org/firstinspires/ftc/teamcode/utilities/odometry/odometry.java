package org.firstinspires.ftc.teamcode.utilities.odometry;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class odometry {

    DcMotorEx rotationEncoder = null;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public odometry(HardwareMap hardwareMap,Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        rotationEncoder = hardwareMap.get(DcMotorEx.class,"rotationEncoder");
        rotationEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotationEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
