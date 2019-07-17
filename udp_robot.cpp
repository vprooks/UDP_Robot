#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <functional>
#include <iostream>
#include <thread>
#include <utility>

using namespace boost::program_options;
using boost::asio::ip::udp;
using namespace std;

/**
 * A helper function that returns a current epoch time with nanosecond
 * resolution.
 * @return time in seconds
 */
double create_timestamp() {
  long timestamp = chrono::duration_cast<chrono::nanoseconds>(
                       chrono::system_clock::now().time_since_epoch())
                       .count();
  return timestamp * 1e-9;
}

/**
 * A mock-up class of a state message that robots exchange with each other.
 */
class RobotState {

public:
  RobotState(unsigned long value, double timestamp) {
    this->value = value;
    this->timestamp = timestamp;
  }
  unsigned long value = 0;
  double timestamp = 0.0;
};

/**
 * A function handle type that the communication and robot code use to exchange
 * data between each other.
 */
typedef function<void(const RobotState &)> robot_state_function_type;

/**
 * The communication logic
 */
class UDP_server {
public:
  UDP_server(const string &host, unsigned short port) {

    io_service = make_shared<boost::asio::io_service>();

    if (host.empty()) {
      // Start UDP server
      socket =
          make_shared<udp::socket>(*io_service, udp::endpoint(udp::v4(), port));
      udp::endpoint endpoint;
      remote_endpoint_ = make_shared<udp::endpoint>(endpoint);
    } else {
      // Start UDP client
      socket =
          make_shared<udp::socket>(*io_service, udp::endpoint(udp::v4(), 0));
      udp::resolver resolver(*io_service);
      remote_endpoint_ = make_shared<udp::endpoint>(
          *resolver.resolve({udp::v4(), host, to_string(port)}));
    }
  }

  /**
   * Set the function that the UDP_server should use to send messages from a
   * remote endpoint to the robot.
   * @param function_handle
   */
  void set_receive_function(const robot_state_function_type &function_handle) {
    receive_function_handle = function_handle;
  }

  /**
   * The destructor closes the socket connection.
   */
  ~UDP_server() {
    if (is_running) {
      stop();
    } else {
      socket->close();
    }
  }

  /**
   * Run the server in a thread
   */
  void start() {
    // Start the thread
    udp_thread = make_shared<thread>(&UDP_server::execute, this);
  }

  /**
   * Graceful shutdown logic
   */
  void stop() {
    cout << "[UDP server]: start stopping" << endl;
    socket->close();
    if (udp_thread->joinable()) {
      udp_thread->join();
    }
    cout << "[UDP server]: stopped" << endl;
  }

  /**
   * Send the robot state to the remote endpoint
   * @param state - a state of the robot to be sent over the network
   */
  void send(const RobotState &state) {
    // Serialize the message and put it into the buffer
    unsigned long buffer[2];
    buffer[0] = state.value;
    memcpy(&buffer[1], &state.timestamp, sizeof(double));
    // Send the message
    socket->async_send_to(
        boost::asio::buffer(buffer), *remote_endpoint_,
        boost::bind(&UDP_server::handle_send, this, buffer,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

private:
  /**
   * The execution loop that is run in a thread created in "start" function.
   */
  void execute() {
    is_running = true;
    start_receive();
    io_service->run();
  }

  /**
   * Initiate receiving a message asynchronously.
   */
  void start_receive() {
    socket->async_receive_from(
        boost::asio::buffer(recv_buffer_), *remote_endpoint_,
        boost::bind(&UDP_server::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

  /**
   * The callback that is called once a message is received.
   * @param error
   * @param bytes_transferred
   */
  void handle_receive(const boost::system::error_code &error,
                      std::size_t bytes_transferred) {
    if (!error || error == boost::asio::error::message_size) {
      if (bytes_transferred == 8 + 8) {
        // Deserialize the input data
        unsigned long value;
        double timestamp;
        memcpy(&value, &recv_buffer_[0], sizeof(unsigned long));
        memcpy(&timestamp, &recv_buffer_[1], sizeof(double));

        // Create a robot state value
        RobotState state(value, timestamp);
        // Send the state to the robot
        receive_function_handle(state);

      } else {
        cerr << "Received a corrupted message!" << endl;
      }
      // Start waiting for another message.
      start_receive();
    }
  }

  /**
   * A sending method function. We're not going to use it, but we need it
   * for sending the message over the network.
   * @param message
   */
  void handle_send(unsigned long *message,
                   const boost::system::error_code & /*error*/,
                   std::size_t /*bytes_transferred*/) {}

  shared_ptr<udp::endpoint> remote_endpoint_;
  boost::array<unsigned long, 2> recv_buffer_;
  shared_ptr<thread> udp_thread = nullptr;
  shared_ptr<udp::socket> socket = nullptr;
  shared_ptr<boost::asio::io_service> io_service;
  robot_state_function_type receive_function_handle;
  bool is_running = false;
};

/**
 * A robot class. It does some work and communicates with another remote robot.
 */
class Robot {
public:
  explicit Robot(string name, unsigned int period = 100)
      : robot_name(std::move(name)) {}

  ~Robot() {
    cout << "[" << robot_name << "]: start destroying" << endl;
    stop();
    cout << "[" << robot_name << "]: destroyed" << endl;
  }

  /**
   * Set the function that the robot should use to send messages via the UDP
   * server to the remote endpoint.
   * @param function_handle
   */
  void set_send_function(const robot_state_function_type &function_handle) {
    send_function_handle = function_handle;
  }

  /**
   * Create a thread where the robot executes its logic.
   */
  void start() { robot_thread = make_shared<thread>(&Robot::execute, this); }

  /**
   * Graceful shutdown of the robot.
   */
  void stop() {
    cout << "[" << robot_name << "]: stopping ..." << endl;
    break_flag = true;
    if (robot_thread->joinable()) {
      cout << "[" << robot_name << "]: the thread is running - stopping"
           << endl;
      robot_thread->join();
    }
    cout << "[" << robot_name << "]: ... stopped" << endl;
  }

  /**
   * Update the robot state using the state of the remote robot. Also
   * @param state - a state of the remote robot.
   */
  void receive(const RobotState &state) {
    // Update the robot state
    counter_in = state.value;
    // Measure the delay. Its value will be printed out in "update_robot_state"
    const double delay_update = create_timestamp() - state.timestamp;
    if (delay > 0) {
      delay = 0.9 * delay + 0.1 * delay_update;
    } else {
      if (delay_update > 0) {
        delay = delay_update;
      }
    }
  }

private:
  /**
   * This is the main loop of the robot. Here the robot updates its state,
   * prints it out and sends the state to the remote robot.
   */
  void execute() {
    while (!break_flag) {
      // Print out the robot state and the measured time delay
      stringstream state_msg;
      state_msg << "[" << robot_name << "] State in = " << counter_in
                << ", out = " << counter_out;
      if (delay > 0) {
        state_msg << "; delay = " << delay * 1e3 << " ms";
      }
      cout << state_msg.str() << endl;

      // Update the robot state and send via UDP
      RobotState state(counter_out++, create_timestamp());
      send_function_handle(state);

      // We use sleep here to imitate the computational process taking a
      // significant time. In a real application there is usually no need to
      // make the thread sleep.
      this_thread::sleep_for(std::chrono::milliseconds(period));
    }
  }

  /**
   * A value representing state of the remote robot
   */
  unsigned long counter_in = 0;
  /**
   * A value representing state of the local robot
   */
  unsigned long counter_out = 0;
  shared_ptr<thread> robot_thread = nullptr;
  bool break_flag = false;
  const string robot_name;
  robot_state_function_type send_function_handle;
  double delay = -1.0;
  unsigned int period = 10;
};

// Create shared pointers to a robot and a udp_server. We need the pointers to
// implement the graceful shutdown logic (function int_handler).
shared_ptr<Robot> robot = nullptr;
shared_ptr<UDP_server> udp_server = nullptr;

/**
 * The code to shutdown the system gracefully. It stops the threads of both the
 * robot and udp_server.
 * @param int_signal
 */
void int_handler(int int_signal) {
  // shutdown gracefully here
  cout << "SIGINT received; shutting down" << endl;
  if (robot != nullptr) {
    robot->stop();
  }
  if (udp_server != nullptr) {
    udp_server->stop();
  }
  exit(int_signal);
}

int main(int argc, const char *argv[]) {
  // Specify an int handler to make the program shut down gracefully when the
  // program is stopped in the console.
  signal(SIGINT, int_handler);

  try {
    // Parse command line options.
    options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")(
        "host", value<string>()->default_value(""),
        "Host")("port,p", value<unsigned short>()->default_value(0), "Port")(
        "robot_name,n", value<string>()->default_value("SERVERBOT"),
        "RobotName")("period,P", value<unsigned int>()->default_value(100),
                     "time delay between robot updates in milliseconds");
    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);
    // Printout the help message if no arguments provided
    if (argc == 1 || vm.count("help")) {
      cout << desc << endl;
      cout << "To run a server application:" << endl;
      cout << "\t./udp_robot -p 55555 -n ALPHA --period 100" << endl;
      cout << "To run a client application example:" << endl;
      cout << "\t./udp_robot --host 127.0.0.1 -p 55555 -n BETA --period 100"
           << endl;
      return 0;
    }

    // Put the command line arguments into special variables
    string host = vm["host"].as<string>();
    unsigned short port = vm["port"].as<unsigned short>();
    string robot_name = vm["robot_name"].as<string>();
    unsigned int period = vm["period"].as<unsigned int>();

    // Create udp_server object
    udp_server = make_shared<UDP_server>(host, port);
    // Create robot object
    robot = make_shared<Robot>(robot_name, period);
    // Create a function handle that will be used by the robot to send a message
    // using the udp_server
    auto send_function_handle = [=](const RobotState &value) {
      udp_server->send(value);
    };
    // Tell the robot how to send messages over network
    robot->set_send_function(send_function_handle);

    // Create a function handle that the udp_server will use to update robot
    // state
    auto receive_function_handle = [=](const RobotState &value) {
      robot->receive(value);
    };
    // Tell the udp_server how to send messages received from a remote host to
    // the robot
    udp_server->set_receive_function(receive_function_handle);

    // Start the server and the robot threads
    udp_server->start();
    robot->start();
  } catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << "\n";
    exit(1);
  }

  // Start the infinite loop so the program does not exit on its own. If we
  // don't do this, the program will exit immediately. An alternative to the
  // loop below is running ros::spin() or a loop with ros::spinOnce().
  cout << "Start infinite loop" << endl;
  while (true) {
    // Prevent the loop becoming a busy loop
    this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

