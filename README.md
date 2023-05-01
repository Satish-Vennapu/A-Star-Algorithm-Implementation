# Path Planning for a Point Robot with obstacle avoidance using A-star Search Algorithm
Project - 03 (Phase 01) for the course ENPM661 - Planning for Autonomous Robots

## Team Members
- Mothish Raj Venkatesan Kumararaj (mr2997@umd.edu)   Directory ID: mr2997    UID: 117553727 


- Satish Vennapu (satish@umd.edu)    Directory ID : satish     UID : 118306759

## Project Description
Implement A* Algorithm to find a path between the start and end point on a given map for a mobile
robot (radius = 5mm; clearance = 5 mm).

* **A-star** : A* Search algorithm is one of the best and popular technique used in path-finding and graph traversals.
* **Action Set** : The Action Set used by the algorithm contains 5 actions each having a Euclidean distance threshold of 0.5 unit and a theta. Using these actions, the algorithm generates new states at every state and checks if each new state is the required `goal state`.
* **Distance Metric** : We are using the Euclidean Distance as the Metric. It's being used with a threshold of 0.5 unit (We have scaled the obstacle coordinates to accomodate the 0.5 clearance).



### Dependencies

* NumPy
* cv2
* Matplotlib
* math
* heapq
* pygame
* time
* itertools
* threading
* sys




### Steps to run the implementation
```
git clone:  https://github.com/Mothish97/A_star_algorithm_planning.git
```
1. Clone the repository
2. Install latest version of Python and the required libraries mentinoned above prior to running the code
3. Open the project file in any IDE. (eg: VSC)
4. Run the program
5. The program prompts for user inputs and takes the user inputs from the console
6. Exploration of nodes starts and the optimal path is displayed

## Results



https://user-images.githubusercontent.com/86384730/226221759-cd75954e-70fb-4db0-a62e-f4dcb8d25510.mp4


#### Understanding the Output 

- Obstacle space is represented in purple
- Explored path is represented in yellow
- Optimal path is represented in red/black

### Test Case 1: 
  [co-ordinates w.r.t the bottom left corner origin of the window] 
	<Format: (x-coord, y-coord, theta)> 
	
	Start-Node: (11, 11, 0)

	Goal-Node:  (400, 100, 60)

### Constraints in the program
1. The sum of the clearance and the robot always equals to 10 because the problem statement was to keep a 5+5 clearance. 
2. The goal thresold is mentioned to be 1.5 in the instructions but in our case we have altered according to the step size to reach the closest point of goal quickly. 
3. If you wan the goal thresold to be 1.5 them do not increase the step size more than 3.
4. Since the clearance is 5+5, a path cannot be generated if x>460 since the triangle is covering the path to reach further.



### [Implementation Video Link](https://drive.google.com/file/d/1n5SmnlW9Tq08KdRU7UKI8Rp8X4Ycr2td/view?usp=share_link)
### Video URL
https://drive.google.com/file/d/1n5SmnlW9Tq08KdRU7UKI8Rp8X4Ycr2td/view?usp=share_link
