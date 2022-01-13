package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class drivingUtil{

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    private encoderUsing encoders = new encoderUsing();

    public drivingUtil(DcMotorEx leftFront,DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear){
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
    }

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

    public void xMovement(double distance, double speed) throws InterruptedException {

        encoders.setMotorsEnabled();
        distance = distance * COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionXmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            //PIDmovement(setMotorPower(speed));
        }

        encoders. setMotorsDisabled();

    }
    public void yMovement(double distance, double speed) throws InterruptedException {
        encoders.setMotorsEnabled();

        distance *= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionYmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            //PIDmovement(setMotorPower(speed));
        }

        encoders.setMotorsDisabled();

    }
    private double setMotorPower(double power){
        return power * COUNTS_PER_MOTOR_REV;
    }

    private void setRobotMotorsPower(double speed){
        leftFront.setVelocity(setMotorPower(speed));
        rightFront.setVelocity(setMotorPower(speed));
        leftRear.setVelocity(setMotorPower(speed));
        rightRear.setVelocity(setMotorPower(speed));
    }

}
