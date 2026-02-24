package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsMotorConnected", isMotorConnected);
    table.put("PositionMeters", positionRotations);
    table.put("appliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    isMotorConnected = table.get("IsMotorConnected", isMotorConnected);
    positionRotations = table.get("PositionMeters", positionRotations);
    appliedVolts = table.get("appliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.isMotorConnected = this.isMotorConnected;
    copy.positionRotations = this.positionRotations;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrentAmps = this.supplyCurrentAmps;
    copy.statorCurrentAmps = this.statorCurrentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
