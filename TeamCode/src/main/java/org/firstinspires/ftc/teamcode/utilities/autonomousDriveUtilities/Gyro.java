package org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.Locale;

/**
 *
 * Class used to declare and use the internal gyro from the control hub
 *
 **/


public class Gyro {

    private BNO055IMU imuSensor;
    private Orientation robotOrientation;


    private HardwareMap hardwareMap;
    private double grade;
    private double globalAngle;
    private Gyro gyro = new Gyro();
    public List<Double> motorPowers;

    public Gyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

    }
    public enum ROBOT_GYRO_DIRECTION{
        HEADING, ROLL, PITCH;
    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor = hardwareMap.get(BNO055IMU.class, "imu");
        imuSensor.initialize(parameters);
    }

    /**
     * Main gyro functions, that return the direction of HEADING,ROLL or PITCH
     * */
    public double returnAngle(ROBOT_GYRO_DIRECTION robotDirection){
        robotOrientation = imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        switch (robotDirection){
            case HEADING:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.firstAngle));
            case ROLL:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.secondAngle));
            case PITCH:
                return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.thirdAngle));
            default:
                return 0;
        }
    }


    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

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
        /*
        leftFront.setPower(lp);
        leftRear.setPower(lp);
        rightRear.setPower(rp);
        rightFront.setPower(rp);
*/
        if (degrees < 0)
            while (getAngle() > degrees) {}
        else
            while (getAngle() < degrees) {}

        resetAngle();
    }
}
