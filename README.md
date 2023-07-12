# Ball-Following TurtleBot
ROS project where a TurtleBot follows a red ball while maintaining a distance of 1 meter. I used OpenCV to detect a sphere and the most red pixel in the camera's viewport to pinpoint the ball's exact location. After finding the distance of the ball using the depth camera, linear and angular velocity commands were sent to the Turtlebot, allowing it to follow the red ball while maintaining the distance.

### Demo
[Click here](https://youtu.be/JqNIprd1HFU) to watch the full demo.
<br><br>
![Demo_small](https://github.com/zainasir/BallFollower/assets/49543216/fa6e6405-4272-48c6-b419-658db0727950)
