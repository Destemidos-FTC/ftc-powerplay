package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        gripper = hardwareMap.get(ServoEx.class, "gripper");
        rotator = hardwareMap.get(ServoEx.class, "rotator");
    }

    /**
     * Fecha a garra na posição definida
     */
    public void closeGrip() {
        gripper.setPosition(RobotConstants.GARRA_A_POSITION_CLOSED);
    }

    /**
     * Abre a garra
     */
    public void releaseGrip() {
        gripper.setPosition(RobotConstants.GARRA_A_POSITION_OPEN);
    }

    /**
     * Rotaciona toda a estrutura da garra em 180°,
     * para realizar a entrega do cone na base
     */
    public void rotateGripper(){
        rotator.rotateByAngle(180.0);
    }

    /**
     * Retorna a posição original de coleta dos cones
     */
    public void returnToCollectPostion() {
        rotator.rotateByAngle(0.0);
    }
}
