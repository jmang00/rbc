
// Functions to calculate the left and right motor speeds based on the joystick position
// https://www.desmos.com/calculator/gsyoxissrt

int calcL(float theta) {
    if (theta < HALF_PI) { return 1; }
    else if (theta < PI) { return -cos(2*theta); }
    else if (theta < PI + HALF_PI) { return -1; }
    else { return cos(2*theta); }
}

int calcR(float theta) {
    if (theta < HALF_PI) { return -cos(2*theta); }
    else if (theta < PI) { return -1; }
    else if (theta < PI + HALF_PI) { return cos(2*theta); }
    else { return 1; }
}