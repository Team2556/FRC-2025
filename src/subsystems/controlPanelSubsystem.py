import keyboard
from commands2 import Command
from time import sleep

class KeyboardDetection(Command):

    # def default():
    #     ...

    def setInputFunctions(self, f1, f2, f3, f4, f5, f6, f7):
        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        self.f4 = f4
        self.f5 = f5
        self.f6 = f6
        self.f7 = f7

    def detectInputs(self):
        '''
        Each of these arguments should be a function that is called when the corresponding key is pressed. 
        For example, f1 is called when the 1 key is pressed, so you'd want to pass a function in that you 
        want to occur each time the 1 key is pressed.
        '''
        key_actions = {
            "1": (1, self.f1),
            "2": (2, self.f2),
            "3": (3, self.f3),
            "4": (4, self.f4),
            "5": (5, self.f5),
            "6": (6, self.f6),
            "7": (7, self.f7),
        }

        for key, (num, action) in key_actions.items():
            if keyboard.is_pressed(key):
                print(num)
                action()
                break

# def test():
#     print("test")

# x = KeyboardDetection()
# x.setInputFunctions(test, test, test, test, test, test, test)
# while True:
#     x.detectInputs()
#     sleep(0.1)