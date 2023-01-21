package com.team1816.lib.auto.actions;

/**
 * The LambdaAction class is an instance of the Action interface and employs a base voidInterface that is empty. Performs nothing, old class carried over.
 */
public class LambdaAction implements AutoAction {

    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public LambdaAction(VoidInterface f) {
        this.mF = f;
    }

    @Override
    public void start() {
        mF.f();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
