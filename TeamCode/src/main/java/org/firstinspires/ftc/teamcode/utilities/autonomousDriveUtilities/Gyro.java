package org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class Gyro {

    private BNO055IMU imuSensor;
    private Orientation robotOrientation;

    public Gyro(){

    }
    public enum ROBOT_GYRO_DIRECTION{
        HEADING, ROLL, PITCH;
    }

    public void initGyro(HardwareMap map){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor = map.get(BNO055IMU.class, "imu");
        imuSensor.initialize(parameters);
    }


    public double returnAngle(ROBOT_GYRO_DIRECTION robotDirection){
        // XYZ ROLL IS ACTUALLY THE HEADING                                                   ZXY
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

    private double returnRobotHeading(){
        return Double.parseDouble(formatAngle(robotOrientation.angleUnit, robotOrientation.firstAngle));
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
