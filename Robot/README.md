# About this folder
ENV files and launch configurations for the robot

## turn_on_wheeltec_robot:
Mostly a read only reference.

This has to be uploaded to `<bot>:~/wheeltec/wheeltec_robot`

NOTE ANY CHANGES TO LAUNCH CONFIGS HERE:

> (it's empty here for now)
>
> (perhaps you should try looking in another castle?)

## scripts

Scripts planned to be run on the robot

This has to be uploaded to `<bot>:~/ros1_shared_dir`

## Commands

Replace the /path/to/ with your own path \
(use `pwd` while in the folder to get the path to your current folder!)

```bash
scp -r </path/to/>scripts wheeltec@192.168.0.100:~/ros1_shared_dir
scp -r </path/to/>turn_on_wheeltec_robot wheeltec@192.168.0.100:~/wheeltec/wheeltec_robot/src/
```