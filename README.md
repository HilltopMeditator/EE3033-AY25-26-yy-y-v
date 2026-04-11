# EE3033-AY25-26-yy-y-v

Repository to host (blood, sweat and tears) our collective efforts towards making this clanker move.

PS: ples no hack, student grades matter

## Dependancies

Assumed system runs on ROS-noetic:

```bash
sudo apt-get update

# 1. State Machine dependencies
sudo apt-get install ros-noetic-smach ros-noetic-smach-ros

# 2. Navigation Stack (Includes move_base, nav_msgs, costmap_2d, etc.)
sudo apt-get install ros-noetic-navigation

# 3. Actionlib and TF (Usually pre-installed with desktop-full, but good to ensure)
sudo apt-get install ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-tf

# 4. Frontier exploration package: Explore-lite 
sudo apt-get install ros-noetic-m-explore

# 5. Core Message Packages (If not already installed)
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-visualization-msgs ros-noetic-rosgraph-msgs
```

## Usage

### Setup

```bash
scp "./ee3033 rviz setup.rviz" ~

## change robot directory perms
ssh wheeltec@192.168.0.100 ## then type password
chmod 755 ~  
exit

scp -r Robot/scripts wheeltec@192.168.0.100:~/ros1_shared_dir
scp -r Robot/turn_on_wheeltec_robot wheeltec@192.168.0.100:~/wheeltec_robot/src/
```

#### Key-based auth

<details>
  <summary><i>Pssst...</i></summary>

If you'd like, follow these extra steps:

```bash
## in the robot machine ...
ssh wheeltec@192.168.0.100 ## then type password

## change robot directory perms
chmod 755 ~  ## The home directory must not be writable by group/others
chmod 700 ~/.ssh  ## The .ssh directory must only be accessible by the user
chmod 600 ~/.ssh/authorized_keys ## The authorized_keys file must only be readable/writable by the user
chown -R wheeltec:wheeltec ~/.ssh ## Ensure the user actually owns the files (if you created them via root/sudo)

sudo nano /etc/ssh/sshd_config ## include the lines below
```

> ```scary config file
> PubkeyAuthentication yes
> AuthorizedKeysFile .ssh/authorized_keys .ssh/authorized_keys2
> ```

```bash
## ...exit robot machine
sudo systemctl restart sshd  # or 'systemctl restart ssh' on Debian/Ubuntu
## ...exited? no? type "exit" and enter

ssh-keygen -t ed25519 -C "your_email@example.com"
ssh-copy-id wheeltec@192.168.0.100 ## then type password
```

During the ssh-keygen process, it will ask where to save it (default ~/.ssh/id_ed25519). If you already have keys you use for GitHub or other servers, do not overwrite them. Give this new key a custom name like ~/.ssh/my_server_key

You should be done at this stage, but why not go the extra mile to setup a ssh profile?

```bash
nano ~/.ssh/config
```

> ```another scary config file
> Host bot
>     HostName 192.168.0.100
>     User wheeltec
>     IdentityFile ~/.ssh/id_rsa # or the custom identity file
>     IdentitiesOnly yes
>     PreferredAuthentications publickey,password
> ```

</details>

### Running it

On robot:
```bash
## Timesync for amcl: needs key-based auth
./timesync_script.sh ## then type password REALLY FAST: dongguan

## Logging in 
ssh wheeltec@192.168.0.100 ## then type password
ros1_start
ros1

## Run the script
/home/wheeltec/ros1_shared_dir/scripts/robot_worker.py
```

On laptop:
```bash
## Run the script
./Laptop/laptop_commander.py
```

## Credits + Acknowledgements

May the contributors of these packages have their beers forever chilly and pillows forever comfy:

1. [m-explore](https://github.com/hrnr/m-explore)
2. [SMACH](https://github.com/ros/executive_smach)