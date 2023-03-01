package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake implements Subsystem {

    private static final PwmControl.PwmRange MAX_REV_SERVO_RANGE = new PwmControl.PwmRange(500, 2500);

    // Hardware devices
    private final DcMotorEx leftExtension;
    private final DcMotorEx rightExtension;
    private final ServoImplEx fourbarRight;
    private final ServoImplEx fourbarLeft;

    // Controllers
    private PIDFController extensionController;

    public enum ExtensionStage {
        ZERO,
        HALF,
        FULL
    }

    public enum FourbarStage {
        COLLECT,
        DEPLOY
    }


    public Intake(HardwareMap hardwareMap) {
        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");
        fourbarRight = (ServoImplEx) hardwareMap.get(Servo.class, "fourbarRight");
        fourbarLeft = (ServoImplEx) hardwareMap.get(Servo.class, "fourbarLeft");

        fourbarRight.setPwmRange(MAX_REV_SERVO_RANGE);
        fourbarLeft.setPwmRange(MAX_REV_SERVO_RANGE);
    }

    @Override
    public void periodic() {
    }
}
