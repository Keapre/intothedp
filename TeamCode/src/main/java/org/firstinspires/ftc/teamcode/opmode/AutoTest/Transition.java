//package org.firstinspires.ftc.teamcode.opmode.AutoTest;
//
//public class Transition {
//    public interface Condition { boolean isTrue(); }
//
//    private Condition condition;
//    private Runnable action;  // Optional: code to run on transition
//    private AutoState toState;
//
//    public Transition(Condition condition, Runnable action, AutoState toState) {
//        this.condition = condition;
//        this.action = action;
//        this.toState = toState;
//    }
//
//    public boolean check() {
//        return condition.isTrue();
//    }
//
//    public void runAction() {
//        if (action != null) action.run();
//    }
//
//    public AutoState getToState() {
//        return toState;
//    }
//}
