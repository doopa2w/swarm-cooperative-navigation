

# swarm-cooperative-navigation

Cooperative Navigation Algorithm for Swarm Robot in Indoor Environment. The navigation of mobile robots are aided by random exploration, communication to exchange navigation information, collision avoidance and goal reaching behaviour. The algorithm uses the foot-bot robot model which is found within the ARGoS's library.

## Installation 

Refer to the Dependecy section to install the necessary libraries. Advised to install ARGoS3 firstmost since it required most of the listed dependencies. The installation guide for ARGoS3 can be found on the README on https://github.com/ilpincy/argos3.

### Dependency

- C++
- cmake >= 3.5.1
- ARGos3 beta58 (latest version)
- g++ >= 5.4 (on Linux) or clang >= 3.1 (on MacOSX)
- A UNIX system (Linux or MacOSX; Microsoft Windows is not supported)
- lua == 5.3
- QT >= 5.5
- *freeglut* >= 2.6.0
- *libxi-dev* (on Ubuntu and other Debian-based systems)
- *libxmu-dev* (on Ubuntu and other Debian-based systems)

### Compilation

The compilation of the scripts is configured through CMAKE. However, the project's scripts have been compiled. To compile the project, make sure to run the following commands in the root folder (swarm-cooperative-navigation/):

```
$ cd swarm-cooperative-navigation
$ bash build.sh
```

The compilation mode is BUILD_TYPE=Debug by default.

### Usage

The experiments can be executed by the following commands:

```
$ argos3 -c experiments/env1.argos
```

The command above is used to run the first layout of the uncluttered environment. To run other setups, just change to the relevant argos file. The files for the experiment can be located in the next section.


## Important Directories

The directories that are relevant to the project would be relevant while those unrelevant directories are ignored and not listed in the tree below.

```
swarm-cooperative-navigation/
├── build .............................................. Contains the compiled libraries so	there is no need to recompile.
├── build.sh ........................................... A simple bash script to compile the project
├── controllers 
│   ├── CMakeLists.txt
│   ├── footbot_swarmnav ................................ The cooperative navigation algorithm is located here
│   │   ├── CMakeLists.txt
│   │   ├── mobile ....................................... This contains the algorithm for mobile robots + target robots
│   │   │   ├── CMakeLists.txt
│   │   │   ├── footbot_mobile.cpp ..................................... The cooperative navigation algorithm for both mobile and target robots
│   │   │   ├── footbot_mobile.h
│   │   │   ├── state.h ................................... enums of the robot's states
│   │   │   ├── trace_message.cpp ......................... For debug purpose only
│   │   │   └── trace_message.h
│   │   └── static ........................................ This contains the algorithm for target robots
│   │       ├── CMakeLists.txt
│   │       ├── footbot_static.cpp ......................... The cooperative navigation algorithm for target robots
│   │       └── footbot_static.h
├── experiments
│   ├── custom_distributions.argos
│   ├── env1.argos .........................................  The first layout of the uncluttered environment in XML
│   ├── env2.argos .......................................... The second maze layout of the cluttered environment in XML
│   ├── env3.argos ..........................................  The third layout of the cluttered environment in XML
│   ├── env4.argos ........................................... The second layout of the uncluttered encironment in XML
│   └── env5.argos ........................................... The first maze layout of the cluttered environment in XML
├── README.md
└── results .................................................. Contains the spreadsheet of raw data

```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## Authors and acknowledgment

- Darryl Tan Zhe Liang - The author of this algorithm
- Dr. Richard Wong Teck Ken - The supervisor for this project.
