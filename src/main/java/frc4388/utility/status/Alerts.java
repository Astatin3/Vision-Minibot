package frc4388.utility.status;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc4388.robot.RobotContainer;

// Class to update a series of WPILIB Alerts
public class Alerts {
  public static Alert no_auto = new Alert("No auto has been selected!", AlertType.kError);

  private static HashMap<String, Alert> validations = new HashMap<>();

  public static void validate(boolean isTrue, String message, AlertType type) {
    Alert currentAlert = validations.get(message);
    
    if(currentAlert == null) {
      currentAlert = new Alert(message, type);
      validations.put(message, currentAlert);
    }

    currentAlert.set(isTrue);
  }
}
