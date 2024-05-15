# Bout to hack NASA

This git contain the code, ROS work space and testing enviroment

## Requirement

* docker desktop
* vs code
* Knowledge


## Enviroment setup
After install the docker open your comand-line and enter to env file to pull create container

```bash
docker image build -t env .
```

To run the container use this command and pull the godang_ws in to sources. (for the first time)

```bash
docker run -it -v $PWD/source:/ros2_file env
```

To run the container
```bash
docker run -it env
```

## Question

Feel free to ask Jiratanun@kmutt.ac.th about all of this. I know it's kinda hard but trust me I'm engineer.



## License

Made on earth. Design by human.
