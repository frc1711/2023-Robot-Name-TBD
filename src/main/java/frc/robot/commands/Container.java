package frc.robot.commands;

public class Container <T> {
    
    private T internalValue;
    
    public Container () {
        set(null);
    }
    
    public Container (T value) {
        set(value);
    }
    
    public T get () {
        return internalValue;
    }
    
    public void set (T value) {
        internalValue = value;
    }
    
    @Override
    public String toString () {
        return internalValue == null ? "null" : internalValue.toString();
    }
    
}
