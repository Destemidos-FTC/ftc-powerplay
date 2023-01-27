package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

public final class GripSystem {
    public static void coletarCones(Gamepad player2, DestemidosBot robot)
    {
        // posição atual de todos os servos
        double current_garra_a_pos;
        double current_garra_b_pos;
        double current_hand_pos;

        if (player2.right_bumper) {
            current_garra_a_pos = RobotConstants.GARRA_A_POSITION_CLOSED;
            current_garra_b_pos = RobotConstants.GARRA_B_POSITION_CLOSED;

        } else {
            current_garra_a_pos = RobotConstants.GARRA_A_POSITION_OPEN;
            current_garra_b_pos = RobotConstants.GARRA_B_POSITION_OPEN;
        }
        robot.servoGarraA.setPosition(current_garra_a_pos);
        robot.servoGarraB.setPosition(current_garra_b_pos);

        // (ramalho): no caso da mão, eu prefiro usar um operador ternário msm
        current_hand_pos = (player2.left_bumper) ? RobotConstants.HAND_CLOSED_POSITION : RobotConstants.HAND_MAX_POSITION;
        robot.servoMão.setPosition(current_hand_pos);
    }
}