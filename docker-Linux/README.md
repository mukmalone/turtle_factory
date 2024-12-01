### Install Docker on the EC2 Instance ###

sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl start docker
sudo systemctl enable docker

### Build dockerimage ###
`docker build -t turtle_factory:001 .`

### Run docker container ###
For VNC:
`sudo docker run -d --name turtle_facotry -p 5901:5901 turtle_factory:001`

### Connecting to VNC: ###
`vnc://ec2-ip-address:5901`

### in the bash ###
`cd /catkin_ws/src/turtle_factory/launch`
`source /catkin_ws/devel/setup.bash`
`roslaunch turtle_factory.launch`