package org.firstinspires.ftc.teamcode.testingOpModes.notForTestButTesting;
/*
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.robotStopping;

public class testDriveComponents {

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    powerBehavior motorBreaking = new powerBehavior();
    robotStopping stopping = new robotStopping();
    robotDirection robotDir = new robotDirection();
    encoderUsing encoders = new encoderUsing();


    public testDriveComponents(){

    }
    public void driveInitialization(HardwareMap map,
                                    robotDirection.ROBOT_DIRECTIONS heading,
                                    powerBehavior.ROBOT_BREAKING breakingMode,
                                    encoderUsing.ENCODER_RUNNING_MODE encoderMode){
        leftFront = map.get(DcMotorEx.class, "leftFront");
        rightFront = map.get(DcMotorEx.class, "rightFront");
        leftRear = map.get(DcMotorEx.class, "leftRear");
        rightRear = map.get(DcMotorEx.class, "rightRear");

        robotDir.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        motorBreaking.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        encoders.setMotorsName(leftFront,leftRear,rightFront,rightRear);

        robotDir.setRobotDirection(heading);
        motorBreaking.setBreakingMode(breakingMode);
        encoders.setEncoderMode(encoderMode);

        stopping.driveStop(leftFront,leftRear,rightFront,rightRear);
    }


    private static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    private PIDCoefficients v1pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v2pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v3pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v4pidGains = new PIDCoefficients(0,0,0);
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double v1IntegralPower = 0;
    private double v2IntegralPower = 0;
    private double v3IntegralPower = 0;
    private double v4IntegralPower = 0;

    private double v1LastError = 0;
    private double v2LastError = 0;
    private double v3LastError = 0;
    private double v4LastError = 0;

    public void robotControl(double left_stick_x, double left_stick_y, double right_stick_x){

        pidTimer.reset();

        double r = Math.hypot(left_stick_x, -left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;

        final double v1TargetVelocity = r * Math.cos(robotAngle) + rightX;
        final double v2TargetVelocity = r * Math.sin(robotAngle) - rightX;
        final double v3TargetVelocity = r * Math.sin(robotAngle) + rightX;
        final double v4TargetVelocity = r * Math.cos(robotAngle) - rightX;

        double v1CurrentVelocity = leftFront.getVelocity();
        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v1Error = v1TargetVelocity - v1CurrentVelocity;
        double v2Error = v1TargetVelocity - v2CurrentVelocity;
        double v3Error = v1TargetVelocity - v3CurrentVelocity;
        double v4Error = v1TargetVelocity - v4CurrentVelocity;

        v1IntegralPower += v1Error * pidTimer.time();
        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        double v1deltaError = v1Error - v1LastError;
        double v2deltaError = v1Error - v2LastError;
        double v3deltaError = v1Error - v3LastError;
        double v4deltaError = v1Error - v4LastError;

        double v1Derivative = v1deltaError / pidTimer.time();
        double v2Derivative = v1deltaError / pidTimer.time();
        double v3Derivative = v1deltaError / pidTimer.time();
        double v4Derivative = v1deltaError / pidTimer.time();

        v1pidGains.p = pidCoefficients.p * v1Error;
        v2pidGains.p = pidCoefficients.p * v2Error;
        v3pidGains.p = pidCoefficients.p * v3Error;
        v4pidGains.p = pidCoefficients.p * v4Error;

        v1pidGains.i = pidCoefficients.i * v1IntegralPower;
        v2pidGains.i = pidCoefficients.i * v2IntegralPower;
        v3pidGains.i = pidCoefficients.i * v3IntegralPower;
        v4pidGains.i = pidCoefficients.i * v4IntegralPower;

        v1pidGains.d = pidCoefficients.d * v1Derivative;
        v2pidGains.d = pidCoefficients.d * v2Derivative;
        v3pidGains.d = pidCoefficients.d * v3Derivative;
        v4pidGains.d = pidCoefficients.d * v4Derivative;

        leftFront.setVelocity(v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity);
        rightFront.setVelocity(v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity);
        leftRear.setVelocity(v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity);
        rightRear.setVelocity(v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity);

        v1LastError = v1Error;
        v2LastError = v2Error;
        v3LastError = v3Error;
        v4LastError = v4Error;



        leftFront.setVelocity(v1);
        rightFront.setVelocity(v2);
        leftRear.setVelocity(v3);
        rightRear.setVelocity(v4);
    }

}
*/