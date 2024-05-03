#include<imu_serial_node.hpp>

ImuSerialNode::ImuSerialNode():Node("imu_serial")
{
    initserial();
    this->__imu_pub__ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",1000);
    this->__timer__ = this->create_wall_timer(0.005s,std::bind(&ImuSerialNode::timer_callback,this));
}

void ImuSerialNode::initserial()
{
    this->declare_parameter("serial_port","/dev/tty_IMU");

    this->serial_port = this->get_parameter("serial_port").as_string();

    this->serial = std::make_shared<LibSerial::SerialPort>(
        this->serial_port,
        LibSerial::BaudRate::BAUD_115200,
        LibSerial::CharacterSize::CHAR_SIZE_8,
        LibSerial::FlowControl::FLOW_CONTROL_NONE,
        LibSerial::Parity::PARITY_NONE,
        LibSerial::StopBits::STOP_BITS_1
    );

    this->stage = STAGE_LOOKING_SOF;
    this->remaining_byte = 1;

    RCLCPP_DEBUG(this->get_logger(),"Init IMU serial done");
}

bool ImuSerialNode::checksum(const uint8_t *data,size_t length)
{
    uint32_t sum = 0;
    for(int i=0;i<length;i++)
    {
        sum+=data[i];
    }
    return (sum & 0xFF) == data[length];
}

short ImuSerialNode::hex_to_short(uint8_t high_byte,uint8_t low_byte)
{
    return (short)((short)high_byte << 8 | low_byte);
}

void ImuSerialNode::publish_imu(const sensor_msgs::msg::Imu& msg)
{
    this->__imu_pub__->publish(msg);
}

void ImuSerialNode::timer_callback()
{
    try
    {
        if(!this->serial->IsOpen())
        {
            this->serial->Open(this->serial_port);
            return;
        }
        RCLCPP_DEBUG(this->get_logger(),"now available %d bytes",this->serial->GetNumberOfBytesAvailable());
        while(this->serial->GetNumberOfBytesAvailable() >= this->remaining_byte)
        {
            switch(this->stage)
            {
                case STAGE_LOOKING_SOF:
                {
                    char ch;
                    this->serial->ReadByte(ch);
                    if((uint8_t)ch == 0x55)
                    {
                        this->stage = STAGE_READING_PACKET;
                        this->remaining_byte = 4;
                        this->rx_buffer[0] = (uint8_t)ch;
                        RCLCPP_DEBUG(this->get_logger(),"find SOF");
                        break;
                    }
                    else
                    {
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        RCLCPP_WARN(this->get_logger(), "SOF mismatch, found 0x%x", ch);
                    }
                    break;
                }
                case STAGE_READING_PACKET:
                {
                    int data_length = 11;
                    for(int i = 1;i < data_length;++i)
                    {
                        char ch;
                        this->serial->ReadByte(ch);
                        this->rx_buffer[i] = (uint8_t)ch;
                    }
                    if(checksum(this->rx_buffer,10))
                    {
                        this->cmd_id = this->rx_buffer[1];
                        if(this->cmd_id == 0x51)
                        {
                            acc_x = hex_to_short(this->rx_buffer[3],this->rx_buffer[2]) / 32768.0 * 16 * 9.8;
                            acc_y = hex_to_short(this->rx_buffer[5],this->rx_buffer[4]) / 32768.0 * 16 * 9.8;
                            acc_z = hex_to_short(this->rx_buffer[7],this->rx_buffer[6]) / 32768.0 * 16 * 9.8;
                        }
                        else if(this->cmd_id == 0x52)
                        {
                            gym_x = hex_to_short(this->rx_buffer[3],this->rx_buffer[2]) / 32768.0 * 2000 * M_PI / 180;
                            gym_y = hex_to_short(this->rx_buffer[5],this->rx_buffer[4]) / 32768.0 * 2000 * M_PI / 180;
                            gym_z = hex_to_short(this->rx_buffer[7],this->rx_buffer[6]) / 32768.0 * 2000 * M_PI / 180;
                        }
                        else if(this->cmd_id == 0x53)
                        {
                            angle_row = hex_to_short(this->rx_buffer[3],this->rx_buffer[2]) / 32768.0 * 180;
                            angle_pitch = hex_to_short(this->rx_buffer[5],this->rx_buffer[4]) / 32768.0 * 180;
                            angle_yaw = hex_to_short(this->rx_buffer[7],this->rx_buffer[6]) / 32768.0 * 180;
                        }
                        else if(this->cmd_id == 0x54)
                        {

                        }
                        sensor_msgs::msg::Imu imu_msg;
                        imu_msg.header.stamp = this->get_clock()->now();
                        imu_msg.linear_acceleration.x = acc_x;
                        imu_msg.linear_acceleration.y = acc_y;
                        imu_msg.linear_acceleration.z = acc_z;
                        imu_msg.angular_velocity.x = gym_x;
                        imu_msg.angular_velocity.y = gym_y;
                        imu_msg.angular_velocity.z = gym_z;
                        Eigen::Quaterniond q = Eigen::AngleAxisd(angle_yaw,Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd(angle_pitch,Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(angle_row,Eigen::Vector3d::UnitX());
                        imu_msg.orientation.w = q.w();
                        imu_msg.orientation.x = q.x();
                        imu_msg.orientation.y = q.y();
                        imu_msg.orientation.z = q.z();
                        publish_imu(imu_msg);
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        break;
                    }
                    else
                    {
                        this->stage = STAGE_LOOKING_SOF;
                        this->remaining_byte = 1;
                        RCLCPP_WARN(this->get_logger(), "found packet 0x%x, checksum mismatch",
                                    this->rx_buffer[6] << 8 | this->rx_buffer[5]);
                    }
                    break;
                }
            }

        }
    }
    catch(std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),"Caught Error: %s",e.what());
    }

}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImuSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}