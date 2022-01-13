package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;

import java.util.ArrayList;
import java.util.List;

public class gyroUtil {

    private HardwareMap hardwareMap;
    private double grade;
    private double globalAngle;
    private Gyro gyro = new Gyro();
    public List<Double> motorPowers;

    public gyroUtil(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        gyro.initGyro(hardwareMap);
        motorPowers = new ArrayList<>();
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
