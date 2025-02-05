package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tunable extends SubsystemBase{

    public final static NetworkTableInstance INST = NetworkTableInstance.getDefault();
    public final static NetworkTable TUNABLES = INST.getTable("Tunables");

    private String key;
    private double defaultVal;
    private double previousVal;
    private DoubleTopic topic;
    private DoubleEntry entry;
    private Consumer<Double> updateCallback;
    
    public Tunable(String key, double defaultVal, Consumer<Double> updateCallback) {
        this.key = key;
        this.defaultVal = defaultVal;
        this.previousVal = defaultVal;
        this.updateCallback = updateCallback;
        publish();
    }

    public double getDefault() {
        return this.defaultVal;
    }

    public String getKey() {
        return this.key;
    }

    public double getValue() {
        return entry.getAsDouble();
    }

    public void setDefault(double defaultVal) {
        this.defaultVal = defaultVal;
        publish();
    }

    public void updateValue() {
        if(previousVal != getValue()) {
            previousVal = getValue();
            updateCallback.accept(getValue());
        }
    }

    public void publish() {
        this.topic = TUNABLES.getDoubleTopic(key);
        this.entry = topic.getEntry(defaultVal);
        topic.setRetained(true);
        entry.set(defaultVal);
    }

    @Override
    public void periodic() {
        if (DriverStation.getMatchType() == DriverStation.MatchType.None
            || DriverStation.getMatchType() == DriverStation.MatchType.Practice
            || DriverStation.isTest()
           ) {
        updateValue();
        }
    }

}
