package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    // Make variables
    private DcMotor ShooterMotor;
    private CRServo ServoLeft, ServoRight;

    public void Init(HardwareMap HwMap) {
        // Set variables
        ShooterMotor = HwMap.get(DcMotor.class, "Shooter");
        ServoLeft = HwMap.get(CRServo.class, "SL");
        ServoRight = HwMap.get(CRServo.class, "SR");

        // Set it to break because when testing it wastes time by not coming to a complete stop
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetShooterPower(double Speed) {
        ShooterMotor.setPower(Speed * 8);
    }

    public void SetServoPower(double Speed) {
        ServoLeft.setPower(-Speed);
        ServoRight.setPower(Speed);
    }
}
