package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo mecanismo de coleta (Intake),
 * inspirado no paradigma de Command-Based Programming, da FRC.
 */
public class Servo implements Subsystem {
    //public final DcMotorEx gripper;
    public final CRServoImplEx wristServoA;
    public final CRServoImplEx wristServoB;
    public final CRServoImplEx wristServoC;
    public final CRServoImplEx wristServoD;

    private final ElapsedTime wristTimer;

    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public Servo(HardwareMap hardwareMap) {
        wristServoA = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_d");
        wristServoB = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_e");
        wristServoC = (CRServoImplEx) hardwareMap.get(CRServo.class, "servoD");
        wristServoD = (CRServoImplEx) hardwareMap.get(CRServo.class, "servoE");

        wristServoB.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServoD.setDirection(DcMotorSimple.Direction.REVERSE);


        wristServoA.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        wristServoB.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        wristServoC.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        wristServoD.setPwmRange(RobotConstants.MAX_SERVO_RANGE);

        wristTimer = new ElapsedTime();
        wristTimer.reset();
    }

    /**
     * Fecha a garra na posição definida
     */

    /**
     * Abre a garra
     */


    public void moveWrist(double power) {
        wristServoA.setPower(power);
        wristServoB.setPower(power);
        wristTimer.reset();
    }
    public void moveMonheca(double power) {
        wristServoC.setPower(power);
        wristServoD.setPower(power);
        wristTimer.reset();
    }

    public void turnOffWrist() {
        wristServoA.setPwmDisable();
        wristServoB.setPwmDisable();

    }
    public void turnOffMonheca() {
        wristServoC.setPwmDisable();
        wristServoD.setPwmDisable();

    }


    }

