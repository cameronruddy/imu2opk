import color_scheme as clr

class progress_bar:

    def __init__(self, num_steps):
        self.wheel = "\\"
    def spin_da_wheel(self):
        if self.wheel == "\\":
            self.wheel = "|"
        elif self.wheel == "|":
            self.wheel = "/"
        else:
            self.wheel = "\\"
        return self.wheel