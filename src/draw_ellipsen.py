from math import cos, pi, sin
import subprocess

a = 4.0
b = 3.0
steps = 40
center = (5.544445, 5.544445)   # display on screen when calling /reset std_srvs/srv/Empty

# moves: list[float] = [[-a, 0]]
positions = []

single_command = '''
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute ' x: {x}
 y: {y}'
'''

command = '''#! /usr/bin/zsh
ros2 service call /reset std_srvs/srv/Empty
'''


for i in range(steps):
  x = a * cos(2 * pi * i / steps)
  y = b * sin(2 * pi * i / steps)
  positions.append((center[0] + x, center[1] + y))
  

command += ''.join([single_command.format(x=x, y=y) for x, y in positions])

command += single_command.format(x=(center[0]+a), y=center[1])  # close the loop

# print(command)

# run
subprocess.run(command, shell=True)

