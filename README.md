# basic_mission_pixhawk_gazebo

In order to execute the mission, perform the following steps:

- Execute the script that compiles px4_autopilot and launches Gazebo for this project:

        $ ./launcher_gazebo.sh

- Wait until the following window is presented:

<img src="https://github.com/aerostack/basic_mission_pixhawk_gazebo/blob/v5-libeccio/docs/gazebobasicmission.png" width=600>

- Execute the script that launches the Aerostack components for this project using df motion control:

        $ ./df_main_launcher.sh

	or using pid motion control instead:

	    $ ./pid_main_launcher.sh

  As a result of this command, a set of windows are presented to monitor the execution of the mission. These windows include:
  - Alphanumeric behavior viewer
  - Keyboard Teleoperation

  You can navigate the windows using tmux commands:
  - Press Ctrl-b and release, the use the special keys, such as:
	- "n" : go to the next window.
	- "Up"/"Down" : go to the upper/lower window if divided.
	- *any number such as "1" : go to the desired terminal number.

  In order to stop executing all tmux windows, press Ctrl-c and execute

	    $ ./stop.sh

- Execute the following command to run the mission:

        $ rosservice call /drone1/python_based_mission_interpreter_process/start

The following video illustrates how to launch and execute the project:

[ ![Launch](https://img.youtube.com/vi/K_o4Vcv4A9I/0.jpg)](https://www.youtube.com/watch?v=K_o4Vcv4A9I)

