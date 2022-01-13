package org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class robotInfo {

    /**
     *
     * Class used to store all the important robot variables
     *
     **/


    public static final double     COUNTS_PER_MOTOR_REV    = 386.3 ;
    public static final double     DRIVE_GEAR_REDUCTION    = 2 ;
    public static final double     WHEEL_DIAMETER_CM   = 10 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    public robotInfo() {
    }
}
