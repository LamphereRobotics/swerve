package edu.lamphere6566.lib;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class SendableCANSparkMax extends CANSparkMax implements Sendable {

    public SendableCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANSparkMax");
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("outputVoltage", () -> getAppliedOutput() * getBusVoltage(), this::setVoltage);
        builder.addDoubleProperty("outputCurrent", this::getOutputCurrent, null);
        builder.addBooleanProperty("idleModeIsBrake", () -> getIdleMode() == IdleMode.kBrake,
                (isBrake) -> setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast));
        builder.addBooleanProperty("isInverted", this::getInverted, this::setInverted);
        builder.addCloseable(builder);
        builder.addDoubleProperty("temperature", this::getMotorTemperature, null);
        builder.addDoubleProperty("closedLoopRampRate", this::getClosedLoopRampRate, this::setClosedLoopRampRate);
        builder.addDoubleProperty("openLoopRampRate", this::getOpenLoopRampRate, this::setOpenLoopRampRate);
    }
}
