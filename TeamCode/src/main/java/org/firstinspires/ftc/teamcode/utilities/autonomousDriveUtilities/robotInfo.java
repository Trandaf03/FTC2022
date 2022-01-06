package org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities;

public class robotInfo {



    private static final double     COUNTS_PER_MOTOR_REV    = 386.3 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ; //TODO CHECK IF THIS IS 1/2 OR 2/1
    private static final double     WHEEL_DIAMETER_CM   = 10 ;
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    /*
    * X - Servo ports to usb ports
    * Y - Sensor ports to motor ports
    * Z - Up
    * */


    public robotInfo() {
    }
}
