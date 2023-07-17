package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo mecanismo de coleta (Intake),
 * inspirado no paradigma de Command-Based Programming, da FRC.
 */
public class Gripper implements Subsystem {
    public final DcMotorEx gripper;
    public final CRServoImplEx wristServoA;
    public final CRServoImplEx wristServoB;

    private final ElapsedTime wristTimer;
    private int gripperTarget = 0;

    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(DcMotorEx.class, "gripper");
        wristServoA = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_d");
        wristServoB = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_e");

        gripper.setDirection(DcMotorEx.Direction.REVERSE);
        wristServoB.setDirection(DcMotorSimple.Direction.REVERSE);

        gripper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServoA.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        wristServoB.setPwmRange(RobotConstants.MAX_SERVO_RANGE);

        wristTimer = new ElapsedTime();
        wristTimer.reset();
    }

    /**
     * Fecha a garra na posição definida
     */
    public void closeGrip() {
        gripperTarget = RobotConstants.GRIPPER_CLOSED_POSITION;
    }

    /**
     * Abre a garra
     */
    public void openGrip() {
        gripperTarget = RobotConstants.GRIPPER_OPEN_POSITION;
    }


    public void moveWrist(double power) {
        wristServoA.setPower(power);
        wristServoB.setPower(power);
        wristTimer.reset();
    }

    public void turnOffWrist() {
        wristServoA.setPwmDisable();
        wristServoB.setPwmDisable();
    }

    @Override
    public void periodic() {
        if(wristTimer.seconds() > 0.01) {
            //turnOffWrist();
        }
    }
}
