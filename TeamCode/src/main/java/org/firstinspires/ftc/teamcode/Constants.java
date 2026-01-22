package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

final public class Constants {
    // Shooter constants
    
    final public String SERVO_RIGHT_NAME = "SR";
    final public String SERVO_LEFT_NAME = "SL";
    final public String SHOOTER_WHEEL_NAME = "Shooter";
    final public double MAX_SHOOTER_POWER = 2;
    final public double MIN_SPEED_TO_ENABLE_SERVOS = 0.008;
    // TODO: ^ find a better name for this lol

    // Drive constants

    final public String FRONT_RIGHT_WHEEL_NAME = "FR";
    final public String FRONT_LEFT_WHEEL_NAME = "FL";
    final public String BACK_RIGHT_WHEEL_NAME = "BR";
    final public String BACK_LEFT_WHEEL_NAME = "BL";
    final public String IMU_NAME = "IMU";
    final public double MAX_DRIVE_SPEED = 1;
    final public DcMotor.RunMode WHEEL_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    final public DcMotor.ZeroPowerBehavior WHEEL_ZERO_POWER_BEHAVIOUR = DcMotor.ZeroPowerBehavior.BRAKE;
}
