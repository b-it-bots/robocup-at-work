from std_msgs.msg import ColorRGBA


class Color:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

    def to_msg(self):
        msg = ColorRGBA()
        msg.r = self.r / 255.0
        msg.g = self.g / 255.0
        msg.b = self.b / 255.0
        msg.a = 1.0
        return msg

colors = {'salmon':    Color(0xFA, 0x80, 0x72),
          'teal':      Color(0x00, 0x80, 0x80),
          'deep_pink': Color(0xFF, 0x14, 0x93),
          'sangria':   Color(0x92, 0x00, 0x0A),
          'sea_blue':  Color(0x00, 0x69, 0x94),
          'sea_green': Color(0x2E, 0x8B, 0x57),
          'scarlet':   Color(0xFF, 0x24, 0x00),
          'pumpkin':   Color(0xFF, 0x75, 0x18),
          'jasmine':   Color(0xF8, 0xDE, 0x7E),
          'ivory':     Color(0xFF, 0xFF, 0xF0),
          'gainsboro': Color(0xDC, 0xDC, 0xDC)}
