import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Constants;


@TeleOp
public class FlywheelTuner extends OpMode {
    private Constants Constants = new Constants();
    public DcMotorEx FlywheelMotor;

    public double HighVelocity = 1500;
    public double lowVelocity = 900;

    public double CurTargetVelocity = HighVelocity;

    double F = 0;
    double P = 0;

    double[] StepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int StepIndex = 1;


    @Override
    public void init() {
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_WHEEL_NAME);
        FlywheelMotor.setMode(Constants.WHEEL_RUN_MODE);

        PIDFCoefficients pidfCoeffcients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelMotor.setPIDFCoefficients(Constants.WHEEL_RUN_MODE, pidfCoeffcients);


    }

    @Override
    public void loop() {
        CurTargetVelocity = gamepad1.yWasPressed() ?
                CurTargetVelocity == HighVelocity ? lowVelocity :
                CurTargetVelocity == lowVelocity ? HighVelocity :
        CurTargetVelocity : CurTargetVelocity;

        if (gamepad1.bWasPressed()) {
            
        }
    }
}
