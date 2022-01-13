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
import org.firstinspires.ftc.teamcode.utilities.odometry.odometry;

import java.util.Arrays;
import java.util.List;

//192.168.43.1:8080/dash
//test
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

    public driveComponents(){

    }

    /**
     * drive initialization function
     * */
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public void init(HardwareMap map, Telemetry telemetry,
                                    robotDirection.ROBOT_DIRECTIONS heading,
                                    powerBehavior.ROBOT_BREAKING breakingMode,
                                    encoderUsing.ENCODER_RUNNING_MODE encoderMode){
        // hardware map linking
        this.hardwareMap = map;
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


        forwardEncoder = hardwareMap.get(DcMotorEx.class,"rotationEncoder");
        leftEncoder = hardwareMap.get(DcMotorEx.class,"ducking");

        forwardEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double returnX(){
        return forwardEncoder.getCurrentPosition();
    }
    public double returnY(){
        return leftEncoder.getCurrentPosition();
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
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void robotTest(double left_stick_x, double left_stick_y, double right_stick_x){
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

        telemetry.addData("xPosition", returnX()/33.5);
        telemetry.addData("yPosition", returnY()/33.5);
        telemetry.addData("leftFront position", leftFront.getCurrentPosition()/COUNTS_PER_CM);
        telemetry.addData("rightFront position", rightFront.getCurrentPosition()/COUNTS_PER_CM);
        telemetry.addData("leftRear position", leftRear.getCurrentPosition()/COUNTS_PER_CM);
        telemetry.addData("rightRear position", rightRear.getCurrentPosition()/COUNTS_PER_CM);
        telemetry.update();
    }


    /**
     * Main function for moving the robot in the Autonomous period
     * */
    public void moveRobot(double nextX, double nextY, double speed) throws InterruptedException {
        // Forward / Reverse movement
        if(nextX != 0 && nextY == 0){
            nextX = nextX * 0.87;
                xMovement(-nextX,-speed);
        } else
        // Left / Right movement
        if(nextX == 0 && nextY != 0){
            nextY = nextY * 0.95;
            yMovement(-nextY,speed);
        } else{
        // Diagonal movement{
            spline(nextX,nextY,speed);
        }
    }

    /**
     * Functions used to move the robot in the Autonomous period
     * */
    public void xMovement(double distance, double speed) throws InterruptedException {

        setMotorsEnabled();
        distance = distance * COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionXmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            //PIDmovement(setMotorPower(speed));
            telemetry.addData("leftFront position",leftFront.getCurrentPosition()/COUNTS_PER_CM);
            telemetry.addData("rightFront position",rightFront.getCurrentPosition()/COUNTS_PER_CM);
            telemetry.addData("leftRear position",leftRear.getCurrentPosition()/COUNTS_PER_CM);
            telemetry.addData("rightRear position",rightRear.getCurrentPosition()/COUNTS_PER_CM);
            telemetry.update();
        }

       setMotorsDisabled();

    }
    public void yMovement(double distance, double speed) throws InterruptedException {
        setMotorsEnabled();

        distance *= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionYmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            //PIDmovement(setMotorPower(speed));
        }

        setMotorsDisabled();

    }
    public void spline(double xDistance, double yDistance, double speed) throws InterruptedException{
        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;

        double angle = Math.toRadians(Math.atan2(xDistance,distance));
        setMotorsEnabled();
        Thread.sleep(100);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);
        Thread.sleep(100);
        encoders.setTargetPositionXmovement(-(int)distance);

        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        double power1 = setMotorPower(Math.sin(angle + (Math.PI / 4)) * speed);
        double power2 = setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);

        leftFront.setVelocity(power1);
        rightRear.setVelocity(power1);

        rightFront.setVelocity(power2);
        leftRear.setVelocity(power2);

        double correctionAngle;
        do {
           correctionAngle = checkDirection();
            if(correctionAngle != 0){
                PIDCalculationSpline(power1,power2);
                rotateRobotWithPID(checkDirection());
            } else PIDMovementSpline(power1,power2);


            power1 = setMotorPower(Math.sin(angle + (Math.PI / 4)) * speed);
            power2 = setMotorPower(Math.sin(angle - (Math.PI/4)) * speed);

        } while(leftFront.isBusy() && rightRear.isBusy() && rightFront.isBusy() && leftRear.isBusy());

       stopping.driveStop();
       setMotorsDisabled();

    }

    /**
     * Robot rotation functions
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
    public void rotateRobotWithPID(double angle){
        double  lp, rp;
        if(angle < 0){
            lp = -setMotorPower(0.1);
            rp = setMotorPower(0.1);
        } else if(angle > 0){
            lp = setMotorPower(0.1);
            rp = -setMotorPower(0.1);
        } else return;

        leftFront.setVelocity(v1Power + lp);
        rightFront.setVelocity(v2Power + rp);
        leftRear.setVelocity(v3Power + lp);
        rightRear.setVelocity(v4Power + rp);
    }


    public void xMovementWithPIDandGyroCorection(double distance, double speed){
        setMotorsEnabled();

        distance *= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionXmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        double correctionAngle;
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            PIDmovement(setMotorPower(speed));
            correctionAngle = checkDirection();
            if(correctionAngle != 0){
               PIDCalculation(setMotorPower(speed));
               rotateRobotWithPID(checkDirection());
            } else PIDmovement(setMotorPower(speed));
        }

        setMotorsDisabled();
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


    public void PIDMovementSpline(double power1,double power2){
        pidTimer.reset();
        final double v1TargetVelocity = setMotorPower(power1);
        final double v4TargetVelocity = setMotorPower(power1);

        final double v2TargetVelocity = setMotorPower(power2);
        final double v3TargetVelocity = setMotorPower(power2);


        double v1CurrentVelocity = leftFront.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();


        double v1Error = getError(v1TargetVelocity,v1CurrentVelocity);
        double v4Error = getError(v4TargetVelocity,v4CurrentVelocity);

        double v2Error = getError(v2TargetVelocity,v2CurrentVelocity);
        double v3Error = getError(v3TargetVelocity,v3CurrentVelocity);


        v1IntegralPower += v1Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();


        double v1deltaError = v1Error - v1LastError;
        double v4deltaError = v4Error - v4LastError;

        double v2deltaError = v2Error - v2LastError;
        double v3deltaError = v3Error - v3LastError;


        double v1Derivative = v1deltaError / pidTimer.time();
        double v4Derivative = v4deltaError / pidTimer.time();

        double v2Derivative = v2deltaError / pidTimer.time();
        double v3Derivative = v3deltaError / pidTimer.time();


        v1pidGains.p = pidCoefficients.p * v1Error;
        v4pidGains.p = pidCoefficients.p * v4Error;

        v2pidGains.p = pidCoefficients.p * v2Error;
        v3pidGains.p = pidCoefficients.p * v3Error;


        v1pidGains.i = pidCoefficients.i * v1IntegralPower;
        v4pidGains.i = pidCoefficients.i * v4IntegralPower;

        v2pidGains.i = pidCoefficients.i * v2IntegralPower;
        v3pidGains.i = pidCoefficients.i * v3IntegralPower;


        v1pidGains.d = pidCoefficients.d * v1Derivative;
        v4pidGains.d = pidCoefficients.d * v4Derivative;

        v2pidGains.d = pidCoefficients.d * v2Derivative;
        v3pidGains.d = pidCoefficients.d * v3Derivative;


        v1Power = v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity;
        v4Power = v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity;

        v2Power = v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity;
        v3Power = v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity;


        v1LastError = v1Error;
        v4LastError = v4Error;


        v2LastError = v2Error;
        v3LastError = v3Error;

    }
    public void PIDCalculationSpline(double power1,double power2){
        pidTimer.reset();
        final double v1TargetVelocity = setMotorPower(power1);
        final double v4TargetVelocity = setMotorPower(power1);

        final double v2TargetVelocity = setMotorPower(power2);
        final double v3TargetVelocity = setMotorPower(power2);


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
