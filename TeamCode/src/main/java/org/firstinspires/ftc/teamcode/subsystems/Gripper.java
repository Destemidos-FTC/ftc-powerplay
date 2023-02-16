package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

/**
 * Subsistema responsável pelo mecanismo de coleta (Intake),
 * inspirado no paradigma de Command-Based Programming, da FRC.
 */
public class Gripper implements Subsystem {
    public final ServoEx gripper;
    public final ServoEx rotator;

    /**
     * Construtor padrão que configura os servos do sistema
     * @param hardwareMap presente em todo OpMode
     */
    public Gripper(HardwareMap hardwareMap) {
        gripper = new SimpleServo(hardwareMap, "gripper", 0.0, 180.0);
        rotator = new SimpleServo(hardwareMap, "rotator", 0.0, 720.0);
    }

    /**
     * Fecha a garra na posição definida
     */
    public void closeGrip() {
        gripper.setPosition(0);
    }

    /**
     * Abre a garra
     */
    public void releaseGrip() {
        gripper.setPosition(1);
    }

    /**
     * Rotaciona toda a estrutura da garra em 180°,
     * para realizar a entrega do cone na base
     */
    public void rotateGripper(){
        rotator.turnToAngle(360.0);
    }

    /**
     * Retorna a posição original de coleta dos cones
     */
    public void returnToCollectPostion() {
        rotator.turnToAngle(0.0);
    }
}
