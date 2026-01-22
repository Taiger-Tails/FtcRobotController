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
        ShooterMotor = HwMap.get(DcMotor.class, Constants.SHOOTER_WHEEL_NAME);
        ServoLeft = HwMap.get(CRServo.class, Constants.SERVO_LEFT_NAME);
        ServoRight = HwMap.get(CRServo.class, Constants.SERVO_RIGHT_NAME);

        // Set it to break because when testing it wastes time by not coming to a complete stop
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetShooterPower(final double Power) {
        ShooterMotor.setPower(Power * Constants.MAX_SHOOTER_POWER);
        double motorRevs = ShooterMotor.getCurrentPosition() / ShooterMotor.getMotorType().getTicksPerRev();
        ShooterSpeed = motorRevs - PreviousMotorRevs;
        PreviousMotorRevs = motorRevs;
    }

    public void SetServoPower(final double Power) {
        if (ShooterSpeed > Constants.MIN_SPEED_TO_ENABLE_SERVOS) {
            ServoLeft.setPower(-Power);
            ServoRight.setPower(Power);
        } else {
            ServoLeft.setPower(0);
            ServoRight.setPower(0);
        }
    }
}
