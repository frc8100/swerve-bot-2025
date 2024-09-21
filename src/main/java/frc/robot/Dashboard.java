package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.EnumSet;
import java.util.function.Consumer;

public class Dashboard {

    private static NetworkTable sdTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    public static void watchBoolean(String keyName, boolean value, Consumer<Boolean> lambda) {
        SmartDashboard.putBoolean(keyName, value);
        sdTable.addListener(keyName, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (table, key, event) -> {
            lambda.accept(event.valueData.value.getBoolean());
        });
    }
}
