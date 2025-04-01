def adjust_jostick(value: float, dead_band=0.15, smooth=False) -> float:
    joystick_value = value
    
    if abs(joystick_value) < dead_band:
        return 0
    
    if smooth:
        return (joystick_value ** 3)

    return joystick_value