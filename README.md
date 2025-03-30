# eyantra
GUIDE THROUGH TEAM #1136 LOGISTIC COBOT CODE-BASE

This is how the operation is carried out :
- Arm moves to the payload to pick it up, while the ebot navigates to arm position.
- The ebot docks to collect the payload.
- The ebot requests the arm to place the payload on it.
- Then ebot undocks to a certain distance.
- Then the ebot navigates to the conveyor according to th box name.
- It docks to the conveyor station and drops the box.
- Then it undocks to a certain distance from the station
- It then navigates to the drop position once again and the amr moves to pick up the next payload.

There are four scrips made by our team controlling the operation :

1) ebot_nav.py : Handles the whole operation - navigates the ebot, and transports the payload from the drop station to the conveyor station. It requests the arm to pick the payload and be ready with the payload to be dropped, while the ebot navigates to the drop pose.

2) ebot_dock.py : Handles the docking operation - linear and angular docking is achieved

3) passing.py : This server handles the passing operation - requests the pick and drop position, manipulates the arm to perform the pick and drop operation.

4) detec_aruco.py : Custom server created by our team which processes the camera data gets the position of aruco markers. It returns response as pick, drop position and payload ids. This submission also includes the srv file created ArucoSW to enable this service
final results are as in the videos 


[![Watch the video](https://img.youtube.com/vi/GV8gkHFqm1A/0.jpg)](https://youtu.be/GV8gkHFqm1A?si=HaXrYE9bc-79YkQm)
