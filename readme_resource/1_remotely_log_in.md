
# CH 1: ROSKY2 - remotely log in

In this example, user will use NoMachine to remotely log in ROSKY2.

## Hardware user have to prepare before doing...

1. [ROSKY2](https://www.icshop.com.tw/product-page.php?28598)
2. Transfer cable: diferent SBC need different type.
    | Single Board Computer(SBC)   | Transfer Cable Type |
    | ---------------------------- | ------------------- |
    | [Jetson Nano Developer Kit](https://www.icshop.com.tw/product-page.php?27812)| [MicroUSB](https://www.icshop.com.tw/product-page.php?15784) |
    | [reComputer-J1010](https://www.icshop.com.tw/product-page.php?28703)| USB-A to Type-c |
    
3. A host machine with the operating system is Window 10(above) or Ubuntu.

---

## Some things user should know before operating...

Different SBC had to finish different notes:

| Single Board Computer(SBC)   | Note had to read first |
| ---------------------------- | ------------------- |
| [Jetson Nano Developer Kit](https://www.icshop.com.tw/product-page.php?27812)| [CH 0.0: ROSKY2 - setup environment with Jetson Nano 4G Developer Kit](0_0_setup_environment_with_jetson_nano.md) |
| [reComputer-J1010](https://www.icshop.com.tw/product-page.php?28703)| [CH 0.1: ROSKY2 - setup environment with reComputer](0_1_setup_environment_with_recomputer.md) |


This note will use these acronym below to stand for:

1. [SBC: Single Board Computer](https://en.wikipedia.org/wiki/Single-board_computer)
2. [LAN: Local Area Network](https://en.wikipedia.org/wiki/Local_area_network)
3. [IP address: Internet Protocal address](https://en.wikipedia.org/wiki/IP_address)
4. [SSID: Service Set Identifier](https://en.wikipedia.org/wiki/Service_set_(802.11_network)#SSID)


Now let's go ahead to setup the environment for operating ROSKY2 with remote desktop.

---

## Step 1. Use transfer cable to connect the host machine and ROSKY2

First, user must make sure that the host machine and ROSKY2 are in the LAN, Please follow the steps to setup the LAN between the host machine and ROSKY2:

No matter what the SBC user use, always putting USB Type-A of the transfer cable into user's host machine, and put another side into the SBC user use.

Assume everything is ready in this step, user can find there is a new drive call "L4T-README" in user's host machine. The similar image with Windows 10 system is shown below:

![L4T-README](https://i.imgur.com/Vu28imo.png)

If there is not the similar image as shown above in the host machine, please check the transfer cable is a "transfer cable". Some cables are just the "Charging Cable", that mean the cable can not transfer datas.

## Step 2. Remotely log in the ROSKY2 with the transfer cable

All right, now the host machine and ROSKY2 can communicate with each other through the transfer cable. Let's see how to remotely log in with NoMachine.

### Step 2-1. Check NoMachine in the host machine
Please check NoMachine is in the host machine. You can check [NoMachine offical site](https://www.nomachine.com/) to see how to install in the host machine. 

### Step 2-2. Launch NoMachine in the host machine

Now user can launch NoMachine. If launch successfully then user can see the similar image as shown below:

![NoMachine](https://i.imgur.com/8Ddk1yy.png)

### Step 2-3. Add remote machine address in NoMachine

Next is adding remote machine to remotely control. Please use the mouse to click the button "Add" on the left top corner in the NoMachine main window, and then user can see the window with page "Machine address".

No matter what the SBC user use, the static IP through the transfer cable setup is always 192.168.55.1. Please fill the filed "Host" with IP address "192.168.55.1". The operating example is shown below:

![fill the field HOST](https://i.imgur.com/o4uSR5o.png)

The filed "Name" user can fill with anything but this note will kepp it empty.

After checking the IP address is correct, please use mouse to click the botton "Connect" on the right top corner in the page "Machine address", and then NoMachine will open a new "connected window" for setting login information. Please fill the filed "Username" and "Password" with those you setup in the SBC. If user got ROSKY2 from iCShop then those will be:
- Usename: user
- Password: 075564686

After filling the login information, user can click the selection "Save this password in the connection file" to log in automaticlly next time. The operating example is shown below:

![filling the login information](https://i.imgur.com/Thunp4Y.png)

Awesome! Now user can click the button "Login" on the right botton corner in the NoMachine connected window. If everything is ok, now user can remotely log in the ROSKY2 through NoMachine. 

### Step 2-4. Remotely Operateing ROSKY2 through NoMachine

Every time after remotely logging in ROSKY2 through NoMachine, the connected window will show dialog. User can select "Don't show this dialog anymore" and next time the dialog will not happen. 

![dialog on the connected window](https://i.imgur.com/8YFBI1P.png)

Click the button "OK" and then user can see the remote desktop in ROSKY2 on connected window. The image shown below is remote desltop from reComputer-J1010.

![remote desktop in ROSKY2 on connected window](https://i.imgur.com/iR28EDm.png)

> **Note**
> Remote desktop will be different in different SBC user use.

## Step 3. Using remote desktop through wireless network

Now user can use remote dekstop to control ROSKY2 through the transfer cable, but this is not convenient while ROSKY2 moving. Another way using remote desktop is wireless network. 
Please followe the steps to setup environment for using remote desktop through wireless network

### Step 3-1. Setup Wi-Fi Network in ROSKY2

Now is normally set up multiple computers for LAN through Wi-Fi router. Thanks to the remote desktop through transfer cable, user can easily setup Wi-Fi Network in ROSKY2. Please click the tool "Network" in the menu which on right top corner(red square in the image shown below), and then choose the SSID of Wi-Fi Network user want to connect. In this step, it will be "iCShop_Test_5G".

![Click tool "Network"](https://i.imgur.com/Kd8ZsJP.png)

After typing the correct password for SSID, user can find the icon of the tool "Network" change from LAN to Wi-Fi(red square in the image shown below). That means user setup the Wi-Fi network successfully in ROSKY2.

![setup the wireless network successfully](https://i.imgur.com/hwbY9xI.png)

### Step 3-2. Check IP address of Wi-Fi Network in ROSKY2.

Such as the step 2 in this note, user have to know what the IP address of the Wi-Fi Network is in ROSKY2 or can not use NoMachine to control ROSKY2 remotely.

Please click the tool "Network" on right top corner again, and this time choose "Connection Information". After that, you can see the new window "Connection Information"on the remote desktop. Please check "IP Address" under label "SSID user connect to". In this example, it is "192.168.4.98".

![IP Address of SSID](https://i.imgur.com/rU0jy2q.png)

### Step 3-3. Add remote machine address in NoMachine

All right, now repeat the actions in [step 2-3 this note](#step-2-3-add-remote-machine-in-nomachine) to add new machine address. And after repeating the actionss in [step 2-4 this note](#Step-2-4-Remotely-Operateing-ROSKY2-through-NoMachine), user can use remote desktop in NoMachine through Wi-Fi Network. 

## Next

User can continue the note -> [CH 2: ROSKY2 - Keyboard teleop](2_keyboard_teleop.md)

Please have fun to use ROSKY2 :)