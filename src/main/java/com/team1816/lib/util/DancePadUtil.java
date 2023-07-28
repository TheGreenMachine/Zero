package com.team1816.lib.util;

import com.team1816.lib.controlboard.Controller;

public class DancePadUtil {
    private boolean upPressed;
    private boolean downPressed;
    private boolean leftPressed;
    private boolean rightPressed;
    private boolean TLPressed;
    private boolean TRPressed;
    private boolean BLPressed;
    private boolean BRPressed;

    public double straightInc;
    public double sideInc;
    public double rotateInc;

    public DancePadUtil()  {
        upPressed = false;
        downPressed = false;
        leftPressed = false;
        rightPressed = false;
        leftPressed = false;
        TLPressed = false;
        TRPressed = false;
        BLPressed = false;
        BRPressed = false;

        straightInc = 0.0;
        sideInc = 0.0;
        rotateInc = 0.0;
    }

    public boolean pressButton(Controller.Button button) {
        switch (button) {
            case UP -> {
                upPressed = !upPressed;
                return upPressed;
            }
            case DOWN -> {
                downPressed = !downPressed;
                return downPressed;
            }
            case LEFT -> {
                leftPressed = !leftPressed;
                return leftPressed;
            }
            case RIGHT -> {
                rightPressed = !rightPressed;
                return rightPressed;
            }
            case UP_LEFT -> {
                TLPressed = !TLPressed;
                return TLPressed;
            }
            case UP_RIGHT -> {
                TRPressed = !TRPressed;
                return TRPressed;
            }
            case DOWN_LEFT -> {
                BLPressed = !BLPressed;
                return BLPressed;
            }
            case DOWN_RIGHT -> {
                BRPressed = !BRPressed;
                return BRPressed;
            }
        }
        return true;
    }

    public void update() {
        updateLinear();
        updateHorizontal();
        updateRotational();
    }

    private void updateLinear() {
        if (straightInc < .6) {
            if (downPressed) {
                straightInc -= .1;
            }
        }
        if (straightInc > -.6) {
            if (upPressed) {
                straightInc += .1;
            }
        }
        if(!downPressed && !upPressed) {
            if (Math.abs(straightInc) < .2 ) {
                straightInc = 0;
            } else if( straightInc > 0) {
                straightInc -=.175;
            } else if (straightInc < 0) {
                straightInc += .175;
            }
        }
    }

    private void updateHorizontal() {
        if (sideInc < .6) {
            if (rightPressed) {
                sideInc += .1;
            }
        }
        if (sideInc > -.6) {
            if (leftPressed) {
                sideInc -= .1;
            }
        }
        if(!rightPressed && !leftPressed) {
            if (Math.abs(sideInc) < .2 ) {
                sideInc = 0;
            } else if( sideInc > 0) {
                sideInc -= .175;
            } else if (sideInc < 0) {
                sideInc += .175;
            }
        }

    }

    private void updateRotational() {
        if (rotateInc < .6) {
            if (TRPressed || BRPressed) {
                if (TRPressed) {
                    rotateInc += .1;
                } else {
                    rotateInc += .05;
                }
            }
        }
        if (rotateInc > -.6) {
            if (TLPressed || BLPressed) {
                if (TLPressed) {
                    rotateInc -= .1;
                } else {
                    rotateInc -= .05;
                }
            }
        }
        if(!TLPressed && !BRPressed && !BLPressed && !TRPressed) {
            if (Math.abs(rotateInc) < .2 ) {
                rotateInc = 0;
            } else if( rotateInc > 0) {
                rotateInc -=.175;
            } else if (rotateInc < 0) {
                rotateInc += .175;
            }
        }

    }

    public void initializeBooleans() {
        upPressed = false;
        downPressed = false;
        leftPressed = false;
        rightPressed = false;
        leftPressed = false;
        TLPressed = false;
        TRPressed = false;
        BLPressed = false;
        BRPressed = false;

    }




}
