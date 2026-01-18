package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    // Make variables
    private DcMotor ShooterMotor;
    private CRServo ServoLeft, ServoRight;
    final private Constants Constants = new Constants();
    private double PreviousMotorRevs = 0;
    public double ShooterSpeed = 0;

    public void Init(HardwareMap HwMap) {
        // Set variables
        ShooterMotor = HwMap.get(DcMotor.class, Constants.SHOOTER_WHEEL_DEVICE_NAME);
        ServoLeft = HwMap.get(CRServo.class, Constants.SERVO_LEFT_DEVICE_NAME);
        ServoRight = HwMap.get(CRServo.class, Constants.SERVO_RIGHT_DEVICE_NAME);

        // Set it to break because when testing it wastes time by not coming to a complete stop
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetShooterPower(double Power) {
        ShooterMotor.setPower(Power * Constants.MAX_SHOOTER_POWER);
        double motorRevs = ShooterMotor.getCurrentPosition() / ShooterMotor.getMotorType().getTicksPerRev();
        ShooterSpeed = motorRevs - PreviousMotorRevs;
        PreviousMotorRevs = motorRevs;
    }

    public void SetServoPower(double Power) {
        if (ShooterSpeed > Constants.MINIMUM_SHOOTER_SPEED_FOR_SERVOS_TO_ACTIVATE) {
            ServoLeft.setPower(-Power);
            ServoRight.setPower(Power);
        }
    }
}
