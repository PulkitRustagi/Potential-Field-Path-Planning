# Potential-Field-Path-Planning
Python code for Simulating a Potential Field based obstable avoidance and path planning

### To setup:
You can use a code editor of your choice, but I personally find **PyCharm** very user friendly and easy to use.

### Steps to install **PyCharm** (Ubuntu):
1. Download the tarball .tar.gz from [Here](https://www.jetbrains.com/pycharm/download/#section=linux)
2. Extract the tarball to a directory that supports file execution.For example, if the downloaded version is 1.17.7391, you can extract it to the recommended /opt directory using the following command:
    - `sudo tar -xzf jetbrains-toolbox-1.17.7391.tar.gz -C /`
3. Switch to the bin subdirectory:
    - `cd /opt/pycharm-*/bin`
4. Run pycharm.sh from the bin subdirectory:
    - `sh pycharm.sh`

### Clone the Script file:
1. Go to your workspace (any folder where you which to maintain this code)
2. Open terminal at that location (Right-Click -> Open in Terminal)
3. Clone the repostory by pasting the following into the opened terminal:
    - `git clone "https://github.com/PulkitRustagi/Potential-Field-Path-Planning.git"`

## What output should look like:
The obstacle locations have been preset in the `main()` function which you can scroll to the bottomof the script and specify **(x,y)** for obstables:
![](https://github.com/PulkitRustagi/Potential-Field-Path-Planning/blob/main/obstacle_code_snapshot.png)
Upon execution you should get an animation such as shown below *(this is a different obstacle setting than shown above)*:

<p align="center">
  <img width="450" height="450" src="https://github.com/PulkitRustagi/Potential-Field-Path-Planning/blob/main/potential-PathPlanning-2.gif">
</p>


