package org.firstinspires.ftc.teamcode.robotComponents;

import static org.firstinspires.ftc.teamcode.robotComponents.handlingComponents.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;

import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotStopping;

import java.util.Arrays;
import java.util.List;

public class driveComponents {


    public enum ROBOT_MOVEMENT{
        FORWARD_DIR, REVERSE_DIR, LEFT_DIR, RIGHT_DIR, SPLINE_TO_DIR, ROTATE_LEFT, ROTATE_RIGHT;
    }

    //Motor declaring
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx forwardEncoder = null;
    public DcMotorEx leftEncoder = null;

    //Gyro declaring

    public Gyro gyro = new Gyro();

    //driveUtilities declaring
    public powerBehavior motorBreaking = new powerBehavior();
    public robotStopping stopping = new robotStopping();
    public robotDirection robotDir = new robotDirection();
    public encoderUsing encoders = new encoderUsing();

    /**
     * Robot PID initialization variables
     **/
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


    private double globalAngle = 0;
    private double grade;


    private double v1Power,v2Power,v3Power,v4Power;

    public driveComponents(){ }

    /**
     * drive initialization function
     * */
    Telemetry telemetry;
    public void init(HardwareMap map, Telemetry telemetry,
                     robotDirection.ROBOT_DIRECTIONS heading,
                     powerBehavior.ROBOT_BREAKING breakingMode,
                     encoderUsing.ENCODER_RUNNING_MODE encoderMode){
        // hardware map linking
        this.telemetry = telemetry;
        leftFront = map.get(DcMotorEx.class, "leftFront");
        rightFront = map.get(DcMotorEx.class, "rightFront");
        leftRear = map.get(DcMotorEx.class, "leftRear");
        rightRear = map.get(DcMotorEx.class, "rightRear");

        // gyro init
        gyro.initGyro(map);

        //initialization for every drive utilities
        robotDir.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        motorBreaking.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        encoders.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        stopping.setMotorsName(leftFront,leftRear,rightFront,rightRear);

        //setting the heading,braking mode and the encoder mode
        robotDir.setRobotDirection(heading);
        motorBreaking.setBreakingMode(breakingMode);
        encoders.setEncoderMode(encoderMode);


        // stopping the motors to be sure that they are stopped :)
        stopping.driveStop();

        forwardEncoder = map.get(DcMotorEx.class,"rotationEncoder");
        leftEncoder = map.get(DcMotorEx.class,"ducking");

        forwardEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    /**
     * Main function used in the Controlled Period for moving the robot
     * */
    public void robotVelocityController(double left_stick_x, double left_stick_y, double right_stick_x){
        double r = Math.hypot(left_stick_x, -left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;
        final double v1 = (r * Math.cos(robotAngle) + rightX) * 386.3 * 20;
        final double v2 = (r * Math.sin(robotAngle) - rightX) * 386.3 * 20;
        final double v3 = (r * Math.sin(robotAngle) + rightX) * 386.3 * 20;
        final double v4 = (r * Math.cos(robotAngle) - rightX) * 386.3 * 20;

        leftFront.setVelocity(v1);
        rightFront.setVelocity(v2);
        leftRear.setVelocity(v3);
        rightRear.setVelocity(v4);

    }


    public double progressivePower(double targetPower){
        double pow = 0;

        while(pow != targetPower){

            pow += 0.001;
        }
        return pow;
    }



    public double getEncoderTics(double distance, double power){

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorsEnabled();
        distance*= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        encoders.setTargetPositionXmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(power);
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            telemetry.addLine("moving");
            telemetry.update();
        }
        double lfPos = leftFront.getCurrentPosition();
        double lrPos = leftRear.getCurrentPosition();
        double rfPos = rightFront.getCurrentPosition();
        double rrPos = rightRear.getCurrentPosition();
        return (lfPos + lrPos + rfPos + rrPos)/4;
    }


    //fata spate
    public void driveY(double distance, double power, double c){

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distance = distance * c;

        setRobotMotorsPower(power) ;
        //PIDmovement(power*386.3*20);

        while(Math.abs(forwardEncoder.getCurrentPosition()) < Math.abs(distance)){
            telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/c);
            telemetry.update();
        }

        if (Math.abs(forwardEncoder.getCurrentPosition()) > Math.abs(distance)){
            double diff = Math.abs(forwardEncoder.getCurrentPosition()) - Math.abs(distance);

            forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setRobotMotorsPower(0.5) ;

            while(Math.abs(forwardEncoder.getCurrentPosition()) < diff){}
            setRobotMotorsPower(0);

        }
        setRobotMotorsPower(0);
    }


    //laterale
    public void driveX(double distance, double power){

        distance = distance * COUNTS_PER_CM;


        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        strafePower(power);
        while(forwardEncoder.getCurrentPosition() < distance){
            telemetry.addData("acum sunt la cm", forwardEncoder.getCurrentPosition()/COUNTS_PER_CM);
            telemetry.update();
        }



        setRobotMotorsPower(0);
    }


    /**
     * Gyro functions
     * */

    public void resetAngle() {
        grade = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);
        globalAngle = 0;
    }
    public double getAngle() {

        double angles = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);

        double rotationAngle = angles - grade;
        if (rotationAngle < -180)
            rotationAngle += 360;
        else if (rotationAngle > 180)
            rotationAngle -= 360;
        globalAngle += rotationAngle;
        grade = angles;
        return globalAngle;
    }
    public double checkDirection() {
        double corectie, unghi, unghi_corectie = .10;

        unghi = getAngle();

        if (unghi == 0)
            corectie = 0;
        else
            corectie = -unghi;

        corectie = corectie * unghi_corectie;
        return corectie;
    }
    public void rotateRobot(double degrees, double power) {

        double  lp, rp;
        resetAngle();

        if (degrees < 0)
        {   // left rotation
            lp = -power;
            rp = power;
        }
        else if (degrees > 0)
        {   // right rotation
            lp = power;
            rp = -power;
        }
        else return;

        leftFront.setPower(lp);
        leftRear.setPower(lp);
        rightRear.setPower(rp);
        rightFront.setPower(rp);

        if (degrees < 0)
            while (getAngle() > degrees) {}
        else
            while (getAngle() < degrees) {}

        stopping.driveStop();
        resetAngle();
    }

    /**
     * PID correction for the motors
     * */
    public void PIDmovement(double speed){

        pidTimer.reset();
        final double v1TargetVelocity = setMotorPower(speed);
        final double v2TargetVelocity = setMotorPower(speed);
        final double v3TargetVelocity = setMotorPower(speed);
        final double v4TargetVelocity = setMotorPower(speed);

        double v1CurrentVelocity = leftFront.getVelocity();
        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v1Error = getError(v1TargetVelocity,v1CurrentVelocity);
        double v2Error = getError(v2TargetVelocity,v2CurrentVelocity);
        double v3Error = getError(v3TargetVelocity,v3CurrentVelocity);
        double v4Error = getError(v4TargetVelocity,v4CurrentVelocity);

        v1IntegralPower += v1Error * pidTimer.time();
        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        double v1deltaError = v1Error - v1LastError;
        double v2deltaError = v1Error - v2LastError;
        double v3deltaError = v1Error - v3LastError;
        double v4deltaError = v1Error - v4LastError;

        double v1Derivative = v1deltaError / pidTimer.time();
        double v2Derivative = v2deltaError / pidTimer.time();
        double v3Derivative = v3deltaError / pidTimer.time();
        double v4Derivative = v4deltaError / pidTimer.time();

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

        v1Power = v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity;
        v2Power = v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity;
        v3Power = v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity;
        v4Power = v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity;

        leftFront.setVelocity(v1Power);
        rightFront.setVelocity(v2Power);
        leftRear.setVelocity(v3Power);
        rightRear.setVelocity(v4Power);

        v1LastError = v1Error;
        v2LastError = v2Error;
        v3LastError = v3Error;
        v4LastError = v4Error;
    }

    public void PIDCalculation(double speed){
        pidTimer.reset();
        final double v1TargetVelocity = setMotorPower(speed);
        final double v2TargetVelocity = setMotorPower(speed);
        final double v3TargetVelocity = setMotorPower(speed);
        final double v4TargetVelocity = setMotorPower(speed);

        double v1CurrentVelocity = leftFront.getVelocity();
        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v1Error = getError(v1TargetVelocity,v1CurrentVelocity);
        double v2Error = getError(v2TargetVelocity,v2CurrentVelocity);
        double v3Error = getError(v3TargetVelocity,v3CurrentVelocity);
        double v4Error = getError(v4TargetVelocity,v4CurrentVelocity);

        v1IntegralPower += v1Error * pidTimer.time();
        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        double v1deltaError = v1Error - v1LastError;
        double v2deltaError = v1Error - v2LastError;
        double v3deltaError = v1Error - v3LastError;
        double v4deltaError = v1Error - v4LastError;

        double v1Derivative = v1deltaError / pidTimer.time();
        double v2Derivative = v2deltaError / pidTimer.time();
        double v3Derivative = v3deltaError / pidTimer.time();
        double v4Derivative = v4deltaError / pidTimer.time();

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

        v1Power = v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity;
        v2Power = v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity;
        v3Power = v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity;
        v4Power = v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity;

        v1LastError = v1Error;
        v2LastError = v2Error;
        v3LastError = v3Error;
        v4LastError = v4Error;
    }




    /**
     * Helper functions
     * */
    public double setMotorPower(double power){
        return power * COUNTS_PER_MOTOR_REV;
    }


    public void setRobotMotorsPower(double speed){
        leftFront.setVelocity(setMotorPower(speed));
        rightFront.setVelocity(setMotorPower(speed));
        leftRear.setVelocity(setMotorPower(speed));
        rightRear.setVelocity(setMotorPower(speed));
    }
    public void strafePower(double speed){
        leftFront.setVelocity(setMotorPower(-speed));
        rightFront.setVelocity(setMotorPower(speed));
        leftRear.setVelocity(setMotorPower(speed));
        rightRear.setVelocity(setMotorPower(-speed));
    }

    public void setMotorsEnabled(){
        leftFront.setMotorEnable();
        rightFront.setMotorEnable();
        leftRear.setMotorEnable();
        rightRear.setMotorEnable();
    }
    public void setMotorsDisabled(){
        leftFront.setMotorDisable();
        rightFront.setMotorDisable();
        leftRear.setMotorDisable();
        rightRear.setMotorDisable();
    }

    private double valueTo1(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private double getError(double targetVelocity, double currentVelocity){
        return  targetVelocity - currentVelocity;
    }
}