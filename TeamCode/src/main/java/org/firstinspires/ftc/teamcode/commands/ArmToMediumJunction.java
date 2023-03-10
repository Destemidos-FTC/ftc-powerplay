package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.ForearmSystem;

public class ArmToMediumJunction extends SequentialCommandGroup {
    public ArmToMediumJunction(DestemidosBot robot) {
        super(
                new InstantCommand(()-> robot.armSystem.setArmPosition(ArmSystem.ArmStage.MEDIUM)),
                new InstantCommand(()->robot.forearmSystem.setForearmPosition(ForearmSystem.ForearmStage.MEDIUM))
        );
    }
}
