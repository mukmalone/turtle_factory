### Install Docker on the EC2 Instance ###
sudo apt-get update
sudo apt-get install git
git clone https://github.com/mukmalone/turtle_factory.git

cd ~/turtle_factory/docker-Linux/

sudo apt-get install -y docker.io

### Build dockerimage ###
`sudo docker build -t turtle_factory:001 .`

### Run docker container ###
For VNC:
`sudo docker run -d --name turtle_facotry -p 5901:5901 turtle_factory:001`
sudo docker exec -it turtle_factory

### Connecting through SSH: ###
`ssh -i 11-30.pem -X ec2-user@54.162.186.228`

### in the bash ###
`cd /catkin_ws/src/turtle_factory/launch`
`source /catkin_ws/devel/setup.bash`
`roslaunch turtle_factory.launch`


Tutorial on GUI: https://www.youtube.com/watch?v=foCG_mH8bxk&t=24s
