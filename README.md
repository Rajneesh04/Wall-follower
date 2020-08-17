# Wall-follower
Autonomous wall following robot.

#Dependencies
  1). Arduino IDE
  
#Hardware

  1).Arduino UNO board
  2).Three ultrasonic sensors
  3).Two DC motors
  4).Caster wheel//for balancing the boot on ground
  5). A bot with 2 wheels (left and right) and a caster wheel
  
The bot uses ultrasonic sensors (one at each of its rear and front end) to get distance from the wall.

#Follows wall
A region is defined in which the bot will traverse for a fixed distance from the wall.Whenever the bot crosses that region it will get signals from the sensors
about its position and will act accordingly.

#PID for smooth movement
   To take care of the relative position of the bot with respect to the wall and to ensure that it tranverses the required path PID is implemented.It makes the
   error reduction smoother thereby ensuring the requiered path is traversed .

#Obstacle Avoidance
  For obstacle avoiding bot a sensor is put at the front end of the bot and it gives the distance of the bot from the obstacle using this distance we can turn
  the bot in such a way as to avoid the obstacle and use the obstacle as its new wall and continue to traverse in the same fashion.
