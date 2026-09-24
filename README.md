# Lab 6: Motion Planning

## I. Learning Goals

- Motion Planning basic concepts
  * Configuration space vs. Workspace: you should understand the difference between configuration space and workspace, and the advantages and disadvantages of planning in each of them.
  * Free space vs. Obstacle space: you should understand the difference between free space and obstacle space.
  * Occupancy grids and Costmaps: you should understand what occupancy grids and costmaps are, how to use them, and how to create them.
- Motion Planning algorithms
  * Sampling based algorithms: RRT and its variants.

## II. Overview

The goal of this lab is to provide you with tools that will help you in a head-to-head race on a race track. After finishing this lab, your car should be able to do something like [this](https://www.youtube.com/watch?v=llHCRqwIllM).

Before you start this lab, you should read the [paper](https://arxiv.org/pdf/1105.1186.pdf). Pay close attention to sections 3.1, 3.2, Algorithm 3, 3.3, Algorithm 6.

### RRT Pseudocode

![rrt_algo](imgs/rrt_algo.png)

The pseudocode of the basic version of RRT is listed as above. You can find all the details of the functions used by RRT in the paper. If you're implementing RRT*, or another version of RRT, read the RRT* section of the provided paper, and do some research to figure out how to modify the basic version of RRT.

### F1TENTH RRT vs. Generic RRT

In general, RRT is often used as a global planner where the tree is kept throughout the time steps. Whenever there is a new obstacle, and the occupancy grid changes, the tree will change accordingly. In our case, RRT is used as a local planner for obstacle avoidance. This is due to the fact that we don't have a well-defined starting point and goal point when we're racing on a track and we want to run continuous laps. In our implementation, we are only keeping a tree for the current time step in an area around the car. You could try to keep one tree that populates the map throughout the time steps, but speed is going to be an issue if you don't optimize how you're finding nodes, and traversing the tree.

For more details, you can check the slides from class for more details on the local planning scheme.

## III. Coding assignment

You can choose to implement RRT in either the workspace or configuration space. Since we're working with a car-like robot, the workspace will be the car's position in the world, and the configuration space will be whatever you decided to add on top of that (heading angle, velocity, etc.).

### Implementing an Occupancy Grid

You'll need to implement an occupancy grid for collision checking. Think about what is available to you (the map, the Laserscan messages, etc.), and construct a occupancy grid using those information. You can choose to either implement a binary occupancy grid (a grid cell is either 0 for unoccupied, or 1 for occupied), or use a probabilistic occupancy grid (a grid cell has values between 0 and 1 for probability that it is occupied). You could choose to implement either an occupancy grid in the car's local frame (our recommendation), or in the map's global frame. Depending on the size of the map that you use, think about how to compute updates to the occupancy grid and storing/using the occupancy grid efficiently. Since we're using RRT as a local planner, as in it comes up with a new path at every time step, you need to run everything relatively fast.  You'll also want to visualize the occupancy grid to ensure its correctness. You don't have to implement a multi-layer one like the one shown in the Figure.

![grid](imgs/grid.png)


Figure 2

### Working in the simulator and on the car

By this point, you should be pretty comfortable using the simulator to test your code. The ground truth pose of the car is available in the simulator, this would be useful for testing your algorithm in the simulator. When you test on the car, make sure you start slow, and increase your speed gradually. You'll see the effectiveness of your algorithm at a higher speed.

### Trajectory Execution

After you've found a path to your goal with RRT, there are different algorithms that you could use to follow that trajectory. The most obvious solution is Pure Pursuit that you have done from the last lab. Picking the waypoint out of the path for pure pursuit will be the most important part if you decided to go down this route. You want a balance between having the car steering smoothly, and at the same time, reactive enough to avoid obstacles. Also, up-sampling the path for pure pursuit to pick out a waypoint is a good way to go.

### Hints

Think about how you could change the way that you're sampling the free space to speed up the process of finding a path to the goal. Also think about how the restrict the area in which you're sampling to make sure you don't have too big of a tree. Besides RRT*, there are other versions of RRT that takes into consideration of other variables (the dynamics of the car for example). After you're done with the basic version of RRT, you should do some research and implement a better version of RRT.

Make sure you visualize the tree you've expanded, and the path you've chosen as the trajectory.


## IV. Part C: RRT* (Extra credit)

![rrt](imgs/rrt.png)

Figure 3

You'll be rewarded extra credit (10%) for implementing RRT*, or another modified version of RRT (if you do, make a good argument on why it deserves extra credit). On top of the basic version of RRT, RRT* uses a cost function, and rewiring the tree, to find a better path to the goal. When the tree has expanded infinite number of nodes, RRT*'s solution is close to optimal. Figure 3 shows the difference in the tree expanded and path found between RRT and RRT*. The skeleton code provided has sections for functions in RRT* as well.

## V: What the autograder runs

The autograder grades your planner on **`levine_obs`**, the simulator's Levine map with obstacles (the one from lab 4), counter-clockwise like every Levine run. Pick it on your laptop with `map_path: 'maps/levine_obs'` in `config/sim.yaml`, and set the start pose of the run you want to try:

| Graded run | `sx`, `sy`, `stheta` | What ends it |
| --- | --- | --- |
| Straight path | `8.08`, `8.67`, `3.1416` (the north hallway just out of the north-east corner, heading west) | 16 m along the hallway, under both triangles, 1.4 m before the L-shaped wall |
| Turn | `9.96`, `2.8`, `1.5708` (the east hallway, heading north: lab 4's start) | 3 m after the north-east corner, before the first triangle |

A run also ends at the first touch of a wall or an obstacle, or after 12 s without progress; a run that ends early earns partial credit for the fraction of its stretch covered.

Before the simulator the autograder runs a quick bench check, worth no points: it holds the car still in the middle of the south hallway, publishes the pose and a laser scan with a box in it 1.3 m ahead, and reports whether your planner steers around it (further right with the box on the left than with its mirror on the right, and to one side or the other with a 0.4 m box straight ahead). It tells you whether your planner reacts to what it sees before you watch it drive.

**One launch file.** The autograder starts your code with `lab7_pkg/launch/levine_launch.py`, and nothing else, for the bench check and both runs:

```bash
ros2 launch lab7_pkg levine_launch.py
```

It is yours to edit: set `EXECUTABLE` to the node you wrote (`rrt_node.py` for Python, `rrt_node` for C++), put your tuned values in `PARAMETERS` (or give it a `.yaml` file), and start as many nodes as you like, in either language. Start your own nodes only: the autograder runs the simulator. Without the launch file the autograder falls back to `ros2 run lab7_pkg <executable>` with no parameter file, so tuned values must then be your node's defaults. Line F of your result tells you which way each run was started.

**Ship your waypoints with your package.** RRT plans locally towards a goal taken from a global path: put your CSV files in `lab7_pkg/waypoints/`; the skeleton's `CMakeLists.txt` installs that folder, and your node finds it with `get_package_share_directory('lab7_pkg')` (Python, `ament_index_python.packages`) or `ament_index_cpp::get_package_share_directory("lab7_pkg")` (C++). A path like `/home/you/sim_ws/...` only exists on your laptop: on the autograder your node would die at start-up.


## VI: Deliverables and Submission

**This lab is done in teams**, the same teams as lab 5. Your team is already formed — you do not create one or invite anyone. **Every member of the team runs the same command**:

```bash
gh student accept RoboRacer-Class ese-6150 lab-6-motion-planning
```

Whoever runs it first creates the team's shared repository, `ese-6150-lab-6-motion-planning-group-<n>`; everyone else gets `Repository already exists` and the same URL. All of you push to that one repository, so **pull before you push**. One submission is the whole team's submission, and every member gets the same grade.

- **Deliverable 1**: Commit your `lab7_pkg` package to your team's repository, waypoints included. Your committed code should run smoothly in simulation on `levine_obs`: down the north hallway under both triangles and round the north-east corner, without touching a wall or an obstacle.
- **Deliverable 2**: Submit links to two videos in **`SUBMISSION.md`** (YouTube unlisted, or Google Drive shared as **"Anyone with the link can view"**): your RRT running in the simulator with the tree and the chosen path visualized in RViz, and the real car running RRT in Levine hallway.

### Submitting

You can commit and push your work as often as you need, but a plain push does **not** count as a submission. When your team is ready to submit, any one of you pushes a tag named `submission` — it counts for the whole team, so agree on the commit first:

```bash
# Make sure you've pulled before or switch branches
git push                            # your commits
git tag submission
git push origin submission          # this triggers the autograder
```

The autograder builds your package, runs the bench check and drives your planner through the two runs in the simulator, then posts your score as a **Release** on your repo (check the Releases page or the commit's status check a few minutes after you tag). To resubmit, move the tag to a new commit:

```bash
git tag -f submission
git push --force origin submission
```

The best scored `submission` push is counted as your team's final submission, and its grade is every member's grade for the lab.

**The autograder finds your work by name.** Package `lab7_pkg`, launch file `levine_launch.py` (or, without it, an executable it can start with `ros2 run lab7_pkg <executable>`, the skeleton's `rrt_node`), taking its pose from `/ego_racecar/odom`, reading `/scan` and publishing `AckermannDriveStamped` on `/drive`. Otherwise, the autograder will not be able to grade your work and your submission may get the wrong grade.

## VII: Grading Rubric
- Compilation: **30** Points (autograded)
- Performance on straight path: **25** Points (autograded in simulation: the north hallway of `levine_obs` under both triangles, without touching anything; a run that ends early earns partial credit for the fraction covered)
- Performance on turn: **25** Points (autograded in simulation, the same way: the north-east corner of `levine_obs`)
- Videos (TA-graded from the links in `SUBMISSION.md`):
  - **10** Points working in Simulation
  - **10** Points working on Car
- RRT* or another improved RRT: extra credit **10** Points (TA-graded from the code and the video; the autograder does not tell RRT and RRT* apart)
