package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToMediumJunction extends SequentialCommandGroup {
    public ArmToMediumJunction(DestemidosBot robot) {
        super(
                new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.CLOSED)),
                new WaitCommand(800),
                new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.MEDIUM)),
                new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.MEDIUM))
        );
    }
}
