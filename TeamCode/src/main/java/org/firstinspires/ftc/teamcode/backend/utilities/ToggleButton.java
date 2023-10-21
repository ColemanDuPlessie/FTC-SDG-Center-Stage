package org.firstinspires.ftc.teamcode.backend.utilities;

import java.util.function.Supplier;

public class ToggleButton implements Supplier<Boolean>{

    private final Supplier<Boolean> getter;
    private boolean isAlreadyDown;
    private boolean currentState;

    public ToggleButton(Supplier<Boolean> buttonGetter) {
        getter = buttonGetter;
        isAlreadyDown = false;
        currentState = false;
    }

    public boolean update() {
        if (getter.get() != isAlreadyDown) {
            isAlreadyDown = !isAlreadyDown;
            if (isAlreadyDown) { currentState = !currentState;}
        }
        return currentState;
    }

    public void setFalse() {currentState = false;}

    public Boolean get() {return update();}

}
