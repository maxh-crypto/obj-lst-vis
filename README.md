# obj-lst-vis
Developement of an autonomous driving environment model visualization based on object list level

This repository can be your catkin workspace for ROS

# Installation in Ubuntu
1. Install git:
```bash
sudo apt-get install git
```
2. Fork repository for your TP (one fork for each TP)
Fork button on GitHub

3. Clone forked repository:
  In your home folder:
``` bash
git clone https://github.com/<fork-owner>/obj-lst-vis.git
```
4. Branch repository:
``` bash
cd obj-lst-vis
git checkout -b origin <your-branch-name>
```
5. Make catkin workspace:
``` bash
catkin_make
```
Now you can work on your own branch of the project. 

# Typical Workflow
1. Pull master (in your local git repository)
``` bash
git pull 
```
2. Work on your branch
3. Add new created files
``` bash
git add <new files>
```
4. Commit and comment your changes
``` bash
git commit -m "comment"
```
5. Push changes to online remote repository (fork of your TP)
``` bash
git push 
```
*Important: only add files of workspace directory 'src' to your git repo!*
  
# Merging into your fork master
If you have a working state of your package you can merge it into the master branch
1. Push your branch (git push)
2. Go to GitHub 
3. Make pull request of your branch

# Merging of fork to origin repository
If your whole TP has an working state of your repository you can make a pull request to merge it into the orgin repository (maxh-crypto/obj-lst-vis)

*A visual respresentation of the workflow can be found under orga/workflow*

