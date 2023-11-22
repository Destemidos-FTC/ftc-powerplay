package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private int multiplicador = -1;

    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public Servo(HardwareMap hardwareMap) {
        wristServoA = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servo_d");
        wristServoB = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servo_e");
        wristServoC = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoD");
        wristServoD = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoE");

        wristServoB.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServoD.setDirection(DcMotorSimple.Direction.REVERSE);

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
        wristTimer.reset();

    }
    public void turnOffMonheca() {
        wristServoC.setPwmDisable();
        wristServoD.setPwmDisable();
        wristTimer.reset();

    }

    public void wristServoRotation(double rotation) {
        //reseta o temporizador
        wristTimer.reset();

        //define as variáveis que utilizaremos nesse void
        int power = -1;
        double timer = rotation / 225;

        //verifica se as rotações estão negativas.
        if(timer < 0) {
            power = 1;
            timer = timer * -1;
        }

        //loop para executar pela quantidade de tempo
        while (wristTimer.seconds() < timer) {
            wristServoA.setPower(power);
            wristServoB.setPower(power);
        }

        turnOffWrist();
        wristTimer.reset();
    }

    public void monhecaServoRotation(double rotation) {

        //reseta o temporizador
        wristTimer.reset();

        //define as variáveis que utilizaremos nesse void
        int power = -1;
        double timer = rotation / 225;

        //verifica se as rotações estão negativas.
        if(timer < 0) {
            power = 1;
            timer = timer * -1;
        }

        //loop para executar pela quantidade de tempo
        while (wristTimer.seconds() < timer) {
            wristServoC.setPower(power);
            wristServoD.setPower(power);
        }

        //desliga os servos e reinicia o temporizador
        turnOffMonheca();
        wristTimer.reset();
    }

    }

