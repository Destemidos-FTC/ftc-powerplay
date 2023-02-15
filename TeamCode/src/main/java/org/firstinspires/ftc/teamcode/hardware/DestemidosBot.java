package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSystem;

import java.util.List;

/**
 * Classe principal que representa o robô por completo, unindo
 * seus sistemas e configurações em um único objeto, que pode 
 * (e deve) ser usado em todo e qualquer tipo de OpMode que utilizamos
 */
public final class DestemidosBot {

    // lista com todos os hubs e seus IDs para fácil acesso
    private final List<LynxModule> allHubs;
    private static final int CONTROLHUB_ID = 0;
    private static final int EXPANSIONHUB_ID = 1;

    // Sistema do Drivetain
    public final Drivetrain drivetrain;

    // Sistema de localização do IMU
    public final LocalizationSystem localizationSystem;

    // Sistema de garras (estilo alavanca)
    public final ArmSystem armSystem;

    // Servos
    public final Servo servoMão;
    public final Servo servoGarraA;
    public final Servo servoGarraB;

    /**
     * Construtor padrão que recebe um {@link HardwareMap}
     * e configura todos equipamentos e seus sistemas
     * @param hardwareMap
     */
    public DestemidosBot(@NonNull HardwareMap hardwareMap){

        // listando todos os hubs conectados no robô
        allHubs = hardwareMap.getAll(LynxModule.class);

        // definindo a cor das leds do hubs
        allHubs.iterator().next().setConstant(Color.CYAN);

        // inicializando os sistemas do robô
        drivetrain = new Drivetrain(hardwareMap);
        armSystem = new ArmSystem(hardwareMap);
        localizationSystem = new LocalizationSystem(hardwareMap, "imu");

        // configurando os servos da garra
        servoMão    = hardwareMap.get(Servo.class, "mão");    // porta 1 - controlhub
        servoGarraA = hardwareMap.get(Servo.class, "garraA"); // porta 3 - controlhub
        servoGarraB = hardwareMap.get(Servo.class, "garraB"); // porta 5 - controlhub
        
        servoMão.setDirection(Servo.Direction.REVERSE);
        servoGarraA.setDirection(Servo.Direction.REVERSE);
        servoGarraB.setDirection(Servo.Direction.FORWARD);

    }

    /**
     * Ativa o modo de caching automático de ambos os Hubs
     */
    public void setBulkReadToAuto() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Ativa o modo de caching manual de ambos os Hubs
     * OBS: fica na resposabilidade do usuário, a limpeza desses dados em cache
     * se não tiver certeza de como realizar essa ação, recomendo o configurar no automático
     */
    public void setBulkCacheToManual() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Limpa o atual cache dos hubs
     */
    public void clearManualBulkCache() {
        for (LynxModule robotHub : allHubs) {
            robotHub.clearBulkCache();
        }
    }
}
