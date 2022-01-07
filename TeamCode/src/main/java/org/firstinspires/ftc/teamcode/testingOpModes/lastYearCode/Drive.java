package org.firstinspires.ftc.teamcode.testingOpModes.lastYearCode;
/*
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {
    
    private DcMotor stangaFata = null;
    private DcMotor dreaptaFata = null;
    private DcMotor stangaSpate = null;
    private DcMotor dreaptaSpate = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 386.3 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2/1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM   = 10 ;     // For figuring circumference
    public static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    public Drive(){
        
    }
    
    
    public void initializareDrive(HardwareMap hwMap){

        stangaFata = hwMap.get(DcMotor.class,"leftFront");
        stangaSpate = hwMap.get(DcMotor.class,"leftRear");
        dreaptaFata = hwMap.get(DcMotor.class, "rightFront");
        dreaptaSpate = hwMap.get(DcMotor.class, "rightRear");
        
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        stangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        stangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        dreaptaFata.setPower(0);
        stangaFata.setPower(0);
        dreaptaSpate.setPower(0);
        stangaSpate.setPower(0);
    }

    public void setMotorPowers(double r, double robotAngle, double rightX, double cutie) {
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        stangaFata.setPower(v1*cutie);
        dreaptaFata.setPower(v2*cutie);
        stangaSpate.setPower(v3*cutie);
        dreaptaSpate.setPower(v4*cutie);
    }


    public void stopMotors(){
        dreaptaFata.setPower(0);
        stangaFata.setPower(0);
        dreaptaSpate.setPower(0);
        stangaSpate.setPower(0);
    }

    
}
*/