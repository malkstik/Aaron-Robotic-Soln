#include "rclcpp/rclcpp.hpp"

#include "sensor_interfaces/msg/sensor_data_array.hpp"
#include "sensor_interfaces/msg/sensor_data.hpp"
#include "sensor_interfaces/srv/get_load.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <memory>

#include <boost/asio.hpp>   
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <queue>


using namespace std::chrono_literals;

class MinimalService : public rclcpp::Node
{
public:
    MinimalService(boost::asio::ip::tcp::socket && socket) : Node("minimal_service"),
                        _socket(std::move(socket))
                         // Constructor for the node
    {
        _timer = this->create_wall_timer(
            50ms, std::bind(&MinimalService::timer_callback, this));
        
        this->declare_parameter("address", "127.0.0.3");
        this->declare_parameter("port", 10000);
        this->declare_parameter("num_samples", 10);
        this->declare_parameter("polling_rate", 2000);
        this->declare_parameter("dof", 3);
        this->declare_parameter("sensor_id", "sensor1");



        // Get parameters
        this -> get_parameter("address", _address);
        this -> get_parameter("port", _port);
        this -> get_parameter("num_samples", _num_samples);
        this -> get_parameter("polling_rate", _polling_rate);
        this -> get_parameter("dof", _dof);
        this -> get_parameter("sensor_id", _sensor_id);
        _num_samples_str = std::to_string(_num_samples);

  
        // Create a service with the same type as our service definition.
        // Bind the incoming request to the handle_service callback method.
        _service = this->create_service<sensor_interfaces::srv::GetLoad>("get_" + _sensor_id + "_data",
            std::bind(&MinimalService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    
    
        for (int i = 0; i <_num_samples; i++){
            float dt = ((float)i )/_polling_rate * 1e9;
            _dt_array.push_back(rclcpp::Duration::from_nanoseconds((int64_t)dt));
        }
    }

    std::string _address;
    int _port;
private:


    int _num_samples;
    int _polling_rate;
    int _dof;
    std::string _sensor_id;
    std::string _num_samples_str;
    std::queue<double> _sensor_queue;
    std::queue<rclcpp::Time> _timestamp_queue;
    rclcpp::TimerBase::SharedPtr _timer;
    std::vector<rclcpp::Duration> _dt_array;


    boost::asio::io_context _io_context;
    boost::asio::ip::tcp::socket _socket;

    void timer_callback()
    {

        std::size_t expected_num_doubles = _num_samples * _dof;
        std::size_t bytes_to_read = expected_num_doubles * sizeof(double);

        // Get timestamps
        rclcpp::Time time_now = this->now();
        for (auto dt: _dt_array){
            _timestamp_queue.push(time_now + dt);
        }
        RCLCPP_INFO(this->get_logger(), "Requesting data"); 
        // Request  data
        boost::asio::write(_socket, boost::asio::buffer(_num_samples_str));
        // Read the response into a dynamic buffer.
        boost::asio::streambuf response;

        boost::asio::read(_socket, response.prepare(bytes_to_read));
        response.commit(bytes_to_read); // Commit the results.

        // Convert data from the buffer to a vector for processing.
        std::istream stream(&response);
        while (stream.good()) {
            RCLCPP_INFO(this->get_logger(), "Yummy data"); 
            double value;
            stream >> value;
            _sensor_queue.push(value);
        }
    }


    builtin_interfaces::msg::Time toTimeMsg(rclcpp::Time time){
        builtin_interfaces::msg::Time time_msg;

        time_msg.sec = time.seconds(); 
        time_msg.nanosec = time.nanoseconds() - time.seconds() * 1e9; 
        return time_msg;
    }


    // Callback to handle incoming service requests
    void handle_service(const std::shared_ptr<sensor_interfaces::srv::GetLoad::Request> request,
                        std::shared_ptr<sensor_interfaces::srv::GetLoad::Response> response)
    {
        // request->req_time;
        RCLCPP_INFO(this->get_logger(), "Received request");       

        sensor_interfaces::msg::SensorData sensor_data_msg;
        sensor_data_msg.sensor_id.data = _sensor_id;

        sensor_interfaces::msg::SensorDataArray sensor_data_array_msg;
        std::vector<sensor_interfaces::msg::SensorData> data_array;

        RCLCPP_INFO(this->get_logger(), "Timestamp queue size: %d", _timestamp_queue.size());       
        RCLCPP_INFO(this->get_logger(), "Sensor queue size: %d", _sensor_queue.size());     


        if (!_timestamp_queue.empty()){
            int num_pop = _timestamp_queue.size();

            for (int i =0; i < num_pop; i++){
    

                std::vector<double> data;
                for(int j=0; j< _dof; j++){
                    data.push_back(_sensor_queue.front());
                    _sensor_queue.pop();
                }
                sensor_data_msg.data = data;

                rclcpp::Time ts = _timestamp_queue.front();
                sensor_data_msg.timestamp = toTimeMsg(ts);
                if (i ==0){
                    sensor_data_array_msg.oldest_timestamp = toTimeMsg(ts);
                    }

                _timestamp_queue.pop();

                data_array.push_back(sensor_data_msg);
            }

            sensor_data_array_msg.data = data_array;
            sensor_data_array_msg.sensor_id.data = _sensor_id;

        }
        RCLCPP_INFO(this->get_logger(), "Sending request");        
        response->data = sensor_data_array_msg;
    }


    rclcpp::Service<sensor_interfaces::srv::GetLoad>::SharedPtr _service; // Define the service member variable
};


// Function to configure and connect the socket using parameters from the ROS node.
boost::asio::ip::tcp::socket configure_and_connect_socket(
    const rclcpp::Node::SharedPtr& node,
    boost::asio::io_context& io_context) {

    // Retrieve address and port parameters
    std::string address = node->get_parameter("address").as_string();

    int p = node->get_parameter("port").as_int();
    std::string port = std::to_string(p);

    // Resolve the endpoint and connect the socket
    boost::asio::ip::tcp::resolver resolver(io_context);
    boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(address, port);
    boost::asio::ip::tcp::socket socket(io_context);
    
    // Perform the connection, while handling potential exceptions
    try {
        boost::asio::connect(socket, endpoints);
    } catch(const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect: %s", e.what());
        throw e; // Rethrow the exception so that main can handle the failure.
    }
    return socket;
}


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create and configure a node with parameters for address and port
    auto node = std::make_shared<rclcpp::Node>("minimal_service");
    node->declare_parameter("address", "127.0.0.3");
    node->declare_parameter("port", 10000);

    // Configure the socket using the parameters and connect it
    boost::asio::io_context io_context;
    auto socket = configure_and_connect_socket(node, io_context);

    // Pass the connected socket to your node service
    auto minimal_service = std::make_shared<MinimalService>(std::move(socket));
    
    rclcpp::spin(minimal_service);
    rclcpp::shutdown();

    return 0; 
}