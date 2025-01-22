package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private TalonFX intakeRotateMotor = new TalonFX(9);
  CurrentLimitsConfigs supplyLimit = new CurrentLimitsConfigs();
  
  // Define the constant for the conversion factor
  private static final double CONVERSION_FACTOR = 13.75;
  
  // Define the maximum limit for the motor position (adjust this value based on your system)
  private static final double MAX_POSITION = 13.75;

  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  public IntakePivot() {
    supplyLimit.withStatorCurrentLimit(15);
    supplyLimit.withSupplyCurrentLimit(20);
    intakeRotateMotor.getConfigurator().apply(supplyLimit, 0.05);
  }

  public void setAngle(double angle, double armAngle) {
    // Calculate the desired position
    double desiredPosition = (angle / 360 * CONVERSION_FACTOR) + (armAngle / 360 * CONVERSION_FACTOR);

    // Ensure the position does not exceed the maximum limit
    desiredPosition = Math.min(desiredPosition, MAX_POSITION);

    // Set the motor position
    SmartDashboard.putNumber("IntakePivotPos", desiredPosition);
    intakeRotateMotor.setControl(request.withPosition(desiredPosition));
  }
}
