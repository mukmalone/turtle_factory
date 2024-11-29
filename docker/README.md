### Build dockerimage ###
`docker build -t turtle_factory:001 .`

### Run docker container ###
Windows:
`docker run --name turtle_facotry -e DISPLAY=host.docker.internal:0.0 -it turtle_factory:001`

### in the bash ###
`docker exec -it 541dafb788f0 bash`
`cd /catkin_ws/src/turtle_factory/launch`
`source /catkin_ws/devel/setup.bash`
`roslaunch turtle_factory.launch`