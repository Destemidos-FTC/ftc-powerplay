package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Subsystem {

    private static final PwmControl.PwmRange MAX_REV_SERVO_RANGE = new PwmControl.PwmRange(500, 2500, 18000);
    // Hardware devices
    private final CRServoImplEx fourbarRight;
    private final CRServoImplEx fourbarLeft;

    private ElapsedTime fourbarTimer;
    private double maxTimeout = 0.01;

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

        fourbarRight = (CRServoImplEx) hardwareMap.get(CRServo.class, "fourbarRight");
        fourbarLeft = (CRServoImplEx) hardwareMap.get(CRServo.class, "fourbarLeft");
        fourbarLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // range dos servos
        fourbarRight.setPwmRange(MAX_REV_SERVO_RANGE);
        fourbarLeft.setPwmRange(MAX_REV_SERVO_RANGE);
    }

    public void desligaservo() {
        fourbarRight.setPower(0);
        fourbarLeft.setPower(0);
    }

    public void moveFourbar(double power) {
        fourbarLeft.setPower(power);
        fourbarRight.setPower(power);
    }

    public void setMaxTimeout(double timeout) {
        maxTimeout = timeout;
    }

    public boolean isFourbarStopped() {
        return !fourbarLeft.isPwmEnabled() && !fourbarRight.isPwmEnabled();
    }

    @Override
    public void periodic() {
        if(fourbarTimer.seconds() > maxTimeout) {
            desligaservo();
        }


    }
}
