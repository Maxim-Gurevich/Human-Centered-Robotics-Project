class InterruptLogic(object):
    def __init__(self):
        self._b_interrupt_button_eight = False
        self._b_interrupt_button_five = False
        self._b_interrupt_button_four = False
        self._b_interrupt_button_two = False
        self._b_interrupt_button_six = False
        self._b_interrupt_button_seven = False
        self._b_interrupt_button_nine = False
        self._b_interrupt_button_one = False
        self._b_interrupt_button_three = False

    def process_interrupts(self):
        self._reset_flags()

    def _reset_flags(self):
        self._b_interrupt_button_eight = False
        self._b_interrupt_button_five = False
        self._b_interrupt_button_four = False
        self._b_interrupt_button_two = False
        self._b_interrupt_button_six = False
        self._b_interrupt_button_seven = False
        self._b_interrupt_button_nine = False
        self._b_interrupt_button_one = False
        self._b_interrupt_button_three = False

    @property
    def b_interrupt_button_eight(self):
        return self._b_interrupt_button_eight

    @b_interrupt_button_eight.setter
    def b_interrupt_button_eight(self, value):
        self._b_interrupt_button_eight = value

    @property
    def b_interrupt_button_five(self):
        return self._b_interrupt_button_five

    @b_interrupt_button_five.setter
    def b_interrupt_button_five(self, value):
        self._b_interrupt_button_five = value

    @property
    def b_interrupt_button_four(self):
        return self._b_interrupt_button_four

    @b_interrupt_button_four.setter
    def b_interrupt_button_four(self, value):
        self._b_interrupt_button_four = value

    @property
    def b_interrupt_button_two(self):
        return self._b_interrupt_button_two

    @b_interrupt_button_two.setter
    def b_interrupt_button_two(self, value):
        self._b_interrupt_button_two = value

    @property
    def b_interrupt_button_six(self):
        return self._b_interrupt_button_six

    @b_interrupt_button_six.setter
    def b_interrupt_button_six(self, value):
        self._b_interrupt_button_six = value

    @property
    def b_interrupt_button_seven(self):
        return self._b_interrupt_button_seven

    @b_interrupt_button_seven.setter
    def b_interrupt_button_seven(self, value):
        self._b_interrupt_button_seven = value

    @property
    def b_interrupt_button_nine(self):
        return self._b_interrupt_button_nine

    @b_interrupt_button_nine.setter
    def b_interrupt_button_nine(self, value):
        self._b_interrupt_button_nine = value
        
    @property
    def b_interrupt_button_one(self):
        return self._b_interrupt_button_one

    @b_interrupt_button_one.setter
    def b_interrupt_button_one(self, value):
        self._b_interrupt_button_one = value

    @property
    def b_interrupt_button_three(self):
        return self._b_interrupt_button_three

    @b_interrupt_button_three.setter
    def b_interrupt_button_three(self, value):
        self._b_interrupt_button_three = value
