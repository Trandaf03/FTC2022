package org.firstinspires.ftc.teamcode.utilities.driveUtilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class robotStopping {

    public robotStopping(){

    }

    public void driveStop(DcMotor a, DcMotor b, DcMotor c, DcMotor d){
        a.setPower(0);
        b.setPower(0);
        c.setPower(0);
        d.setPower(0);
    }
    public void collectorStop(DcMotor motorName){
        motorName.setPower(0);
    }
    public void duckyStop(DcMotor motorName){
        motorName.setPower(0);
    }


}
