# CH 2: ROSKY2 - Keyboard teleop

In this note, user will use keyboard teleop for ROSKY2.

## Hardware user have to prepare before doing...

1. [ROSKY2](https://www.icshop.com.tw/product-page.php?28598)
2. A host machine with the operating system is Window 10(above) or Ubuntu.

---

## Some things user should know before operating...

Please finished the notes below: 
- [CH 1. ROSKY2 - remotely log in](1_remotely_log_in.md)

Now let’s go ahead to use keyboard teleop for ROSKY2.

---

## Step 1. Open Terminator in ROSKY2 through remote desktop

### Step 1-1. Open terminal on the remote desktop 

Because of the keyboard shortcut keys in NoMachine, user can not use the keyboard keys 【ctrl】+【alt】+【T】to open the terminal quickly. 

Please right-click the mouse on the remote desktop to open the menu item and then choose【Open Terminal】to open terminal. 

Once user see the roe_menu is shown on the terminal, please choose【2】to start the operating environment for ROSKY2.

![Open terminal on the remote desktop](https://i.imgur.com/zsH0Zuq.png)

### Step 1-2. Open terminator on the remote desktop

Terminator is a nice terminal emulator, please check the [Terminator’s documentation](https://terminator-gtk3.readthedocs.io/en/latest/) to know more detail.

User can type the command command to start terminator:
```bash=
$ terminator
```
Once start the terminator successfully, user can see ros_menu is shown on the terminator. Like the operation in [step 1-1](#Step-1-1-Open-terminal-on-the-remote-desktop), user have to choose [2] to start the environment for ROSKY2.

![Open terminator on the remote desktop](https://i.imgur.com/dgprdav.png)

More keyboard shortcut in the terminator user can find on the [Terminator’s documentation - Using the keyboard](https://terminator-gtk3.readthedocs.io/en/latest/gettingstarted.html#using-the-keyboard).

## Step 2. Start the communication with all sensors, motor control board and more on ROSKY2

Now user just running the system on the SBC and it can not send/receive data from sensor and more. Hence, user have to start the communication with these to send/receive data. 

> **Note**
> Seneors on ROSKY2 will start running after typing the command below. Please make sure **no more things** result in stopping these sensor work, such as **something is on the LiDAR**, **transfer cable connect between host machine and ROSKY2** and more.

Please type the command below in the terminator to start the communication:
```bash=
ros2 launch rosky2_bringup bringup.launch.py
```
![launch bringup.launch.py](https://i.imgur.com/eCJKtdZ.png)


## Step 3. Use node "teleop_twist_keyboard" for ROSKY2

Awesome! User start the communication between ROSKY2 and sensors. Before doing anything, user have to make sure:

> **Note**
> **PUT ROSKY2 ON THE GROUND.**

ROSKY2 may fall from high place if user do not put ROSKY2 on the ground. The most important thing than everyone: make sure that ROSKY2 is safe.

From [Step 1-2 in this note](#Step-1-2-Open-terminator-on-the-remote-desktop), user can know that use the keyboard shortcut 【ctrl】+【shift】+【E】 in terminator to split terminal vertically. The same thing is choosing 【2】 to start the environment for ROSKY2 after user opening ros_menu successfully.

![split terminal vertically](https://i.imgur.com/hYP1edP.png)

Now is ready to use keyboard teleop for ROSKY2. Please type the command below to run the node "teleop_twist_keyboard":

```bash=
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

![Run the node teleop_twist_keyboard](https://i.imgur.com/c4WH8Te.png)

Base on the information shown on the terminal, user can press the keyboard key to control ROSKY2. Please feel free to adjust the velocity and check what happen to ROSKY2.

## Step 4. Shutdown all running node.

If user finished the keyboard teleop for ROSKY2, please remember to shutdown all running node through the shortcut key 【ctrl】+【C】 in terminal. 


## Next

User can continue the note -> [CH3: wall_follower](3_wall_follower.md)

Please have fun to use ROSKY2 :)