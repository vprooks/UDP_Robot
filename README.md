# UDP Robot

This code is an example of UDP communication program for robotic applications (and not only). 

For robotic applications requiring high frequency communication between computers, like visual servoing or bilateral teleoperation, the best solution is to exchange messages using UDP protocol. Of course, it is preferable to use an existing framework like ROS rather than having custom implementation of the communication logic. However, when one robot runs on Ubuntu, and another - on Windows, we cannot use ROS.


There are two classes in the code: UDP_server and Robot.

UDP_server class that can run as either a client or a server depending on the provided command line robotics.

Robot class is a mock-up of a robotic system that interacts with an environment and performs some tasks and exchanges its state with another robot over a network. 

Both UDP_server and Robot run their logic in separate threads. Also, the implementation ensures that the communication between the nodes is not affected by either classes' logic: the UDP server sends and receives messages at the same instant as they arrive.

This example is based on [Boost Asio tutorials](
 https://www.boost.org/doc/libs/1_58_0/doc/html/boost_asio/tutorial.html). The code runs with Boost 1.58 which is default for Ubuntu 16.04. 

Additionally, the code uses [boost command line argument parser](https://theboostcpplibraries.com/boost.program_options).

To run the example, execute the program in two different terminals: one for a server side, another for a client side:

```bash 
./udp_robot -p 55555 -n ALPHA --period 100  # server
./udp_robot --host 127.0.0.1 -p 55555 -n BETA --period 100  # client
```

The code is tested in Ubuntu 16.04, but it should work on Windows 10 as well.
