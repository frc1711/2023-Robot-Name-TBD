package frc.robot.commands.test;

public class Container <T> {
    
    private T internalValue;
    
    public Container (T value) {
        set(value);
    }
    
    public T get () {
        return internalValue;
    }
    
    public void set (T value) {
        internalValue = value;
    }
    
}
