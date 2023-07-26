package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToHighJunction extends SequentialCommandGroup {

    public ArmToHighJunction(DestemidosBot robot) {
        super(
                //new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.HIGH)),
                //new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.HIGH))
                new InstantCommand(()->robot.simpleArm.setArmPosition(ArmSystem.ArmStage.HIGH))
        );
    }
}
