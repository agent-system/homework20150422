# homework 20150624

1. Update homework directory and move to 20150624

  ```
cd /home/<your account>/agentsystem
git pull origin master
cd 20150624
```
2. Take the screenshots of rviz and gazebo
3. Checkout a branch to push

  ```
git checkout -b 20150624-homework
```
4. Copy the screenshots to this directory and rename them like this

  ```
rviz-<your student id>.png gazebo-<your student id>.png
```
5. Commit your change

  ```
git add rviz-<your student id>.png gazebo-<your student id>.png
git commit -m "add my homework"
```
6. Push your change to github

  ```
git push origin 20150624-homework
```
7. Create Pull-request to `agent-system/homework/20150624