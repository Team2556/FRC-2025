def smooth(joystick_value: float, exponential_for_curving: int = 3, blend: float = 0.1):
    '''
    Add curve to joystick to smooth movements.
    exponential_for_curving needs to be an odd number'''
    #make sure it is odd
    if (exponential_for_curving-1) % 2 == 0:
        exponential_for_curving = exponential_for_curving
    else:
        exponential_for_curving = int(exponential_for_curving)
    # EVEN WITH voltage deadbands, need to trim of crudy controller noise
    if joystick_value < .093 and joystick_value > -.093:
        joystick_value = 0

    blend_value = joystick_value * blend
    smoothed_blend_value = blend_value ** exponential_for_curving
    blend_mod = (1 - blend) * joystick_value
    smoothed_value = smoothed_blend_value + blend_mod
    
    # Limit smoothed vlaue to be between -1 and 1
    if smoothed_value > 1:
        smoothed_value = 1
    elif smoothed_value < -1:
        smoothed_value = -1
    
    
    # # return smoothed_value
    # smoothed_value = joystick_value ** 3
    return smoothed_value 

    
def one_side_control_only(joystick_value: float, PosOrNeg: str= 'Neg'):
    
    if type(PosOrNeg) == str:
        if 'n' in PosOrNeg.lower():
            sign_to_allow = -1
        else:
            sign_to_allow = 1
    elif (PosOrNeg) == int:
        sign_to_allow = PosOrNeg/abs(PosOrNeg)
    else:
        sign_to_allow = -1/0
    
    if sign_to_allow * joystick_value >0:
        return joystick_value
    else:
        return 0



if __name__ == '__main__':
    while True:
        test_input = input('Value to Test: ')
        print(smooth(float(test_input)))
    
    