#include "ros/ros.h"

#include "mqtt/async_client.h"
#include "ros_msg_parser/ros_parser.hpp"
#include "rosmsg_cpp/rosmsg_cpp.h"
#include "yaml-cpp/yaml.h"

struct MQTT_to_ROS_Publisher {
    std::string topic_name;
    std::string topic_type;

    MQTT_to_ROS_Publisher(std::string name, std::string type) {
        topic_name = name;
        topic_type = type;
    }
};

class MQTT_ROS {
public:
    MQTT_ROS() {}

protected:
    //Reference variable to the Node Handler.
    ros::NodeHandle nh;

    //Reference to the MQTT Client.
    mqtt::async_client* mqtt_client;

    //Stores the MQTT Broker Address.
    std::string mqtt_broker_address;

    //Stores the name of the robot.
    std::string robot_name;

    //Stores all ROS publisher reference data.
    std::vector<MQTT_to_ROS_Publisher> stored_publishers_info;

    //Stores all ROS publishers.
    std::vector<ros::Publisher> stored_publishers;

    //Stores all ROS subscribers.
    std::vector<ros::Subscriber> stored_subscribers;

    //Stores all MQTT topics.
    std::vector<mqtt::topic_ptr> stored_topics; 

    //The MQTT message pointer.
    mqtt::message_ptr pubmsg;

    //The ROS message parsers pointer.
    RosMsgParser::ParsersCollection* parsers;

public:
    void ROS_to_MQTT(const ros::MessageEvent<RosMsgParser::ShapeShifter const>& ros_event) {
        //Get the topic name.
        const ros::M_string& header = ros_event.getConnectionHeader();
        std::string topic_name = header.at("topic");
        RosMsgParser::ShapeShifter::ConstPtr ros_msg = ros_event.getMessage();

        uint32_t serialization_size = ros_msg->size();
        boost::shared_array<uint8_t> buffer(new uint8_t[serialization_size]);
        ros::serialization::OStream stream(buffer.get(), serialization_size);
        ros::serialization::serialize(stream, *ros_msg);

        //Publish the MQTT message.
        pubmsg = mqtt::make_message(robot_name + topic_name, ros_msg->raw_data(), ros_msg->size());
        pubmsg->set_qos(0);
        mqtt_client->publish(pubmsg);
    }

    void MQTT_to_ROS(mqtt::const_message_ptr mqtt_msg) {
        std::string mqtt_topic = mqtt_msg->get_topic();
        std::string topic_name = mqtt_topic.erase(0, robot_name.length());
        ros::Publisher curr_publisher;
        for (int i = 0; i < stored_publishers_info.size(); i++) {
            if (stored_publishers_info[i].topic_name == topic_name) {
                curr_publisher = stored_publishers[i];
                break;
            }
        }
        if (!curr_publisher) {
            return;
        }

        RosMsgParser::ShapeShifter ros_msg;
        uint32_t msg_size = mqtt_msg->get_payload().size();
        boost::shared_array<uint8_t> buffer(new uint8_t[msg_size]);
        for (int i = 0; i < msg_size; i++) {
            buffer[i] = mqtt_msg->get_payload_ref()[i];
        }
        ros::serialization::IStream stream(buffer.get(), msg_size);
        ros::serialization::deserialize(stream, ros_msg);
        curr_publisher.publish(ros_msg);
    }

    void run() {
        //Get the MQTT Broker Address using ROS Parameters.
        nh.getParam("/mqtt_ros/mqtt_broker_address", mqtt_broker_address);

        //Get the robot name.
        nh.getParam("/mqtt_ros/robot_name", robot_name);

        //Get the config file name.
        std::string config_file;
        nh.getParam("/mqtt_ros/config_file", config_file);

        //Debug message for starting the connection.
        ROS_INFO("Starting MQTT Client with Broker at %s and robot %s.", mqtt_broker_address.c_str(), robot_name.c_str());

        //Initialize MQTT Client.
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        mqtt::async_client client(mqtt_broker_address, "mqtt_ros_client_" + robot_name + "_" + std::to_string(ms.count()), 100000);
        mqtt::connect_options connection_options;
        connection_options.set_clean_session(false);
        connection_options.set_automatic_reconnect(true);
        connection_options.set_max_inflight(65000);
        client.connect(connection_options)->wait();
        mqtt_client = &client;

        //Load the config.yaml file
        std::string yaml_path = ros::package::getPath("mqtt_ros");
        YAML::Node config = YAML::LoadFile(yaml_path + "/config/" + config_file);

        //Set up a ROS subscriber that will publish a message to MQTT.
        //TODO: Topic names should be declared via a config file.
        std::vector<std::string> ros_to_mqtt_publishers = config["output_topics"].as<std::vector<std::string>>();
        for (std::string topic : ros_to_mqtt_publishers) {
            stored_subscribers.push_back(nh.subscribe(topic, 1000, &MQTT_ROS::ROS_to_MQTT, this));
        }

        //Set up a ROS publisher that will receive messages from MQTT.
        client.set_message_callback([this](mqtt::const_message_ptr mqtt_msg) {
            MQTT_to_ROS(mqtt_msg);
        });
        auto mqtt_sub_options = mqtt::subscribe_options(true);
        const YAML::Node& mqtt_to_ros_publisher = config["input_topics"];
        for (auto value : mqtt_to_ros_publisher) {
            MQTT_to_ROS_Publisher pub(value["name"].as<std::string>(), value["type"].as<std::string>());
            stored_publishers_info.push_back(pub);
        }
        for (auto pub : stored_publishers_info) {
            std::string topic_md5sum = ros::message::getMD5Sum(pub.topic_type);
            std::string topic_definition = ros::message::getFullDef(pub.topic_type);
            ros::AdvertiseOptions adv_options(pub.topic_name, 1000, topic_md5sum, pub.topic_type, topic_definition);
            stored_publishers.push_back(nh.advertise(adv_options));
            mqtt::topic mqttTopic(client, robot_name + pub.topic_name, 0);
            mqttTopic.subscribe(mqtt_sub_options)->wait();
        }

        //Everything successful.
        ROS_INFO("MQTT Client started and connected to Broker.");

        //Main Loop
        while (ros::ok()) {
            ros::spinOnce();
        }

        // Disconnect MQTT Client after ending the Node.
        mqtt_client->disconnect()->wait();
        for (auto publisher : stored_publishers) {
            publisher.shutdown();
        }
        for (auto subscriber : stored_subscribers) {
            subscriber.shutdown();
        }
        ROS_INFO("Disconnecting MQTT Client and ending MQTT_ROS Node.");
    }
};

int main(int argc, char **argv) {
    //Initialize ROS Node.
    ros::init(argc, argv, "mqtt_ros_node");

    MQTT_ROS mqtt_ros;
    mqtt_ros.run();

    return 0;
}
