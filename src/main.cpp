/* CMAVNode
 * Monash UAS
 */

#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <stdio.h>
#include <signal.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include "../include/mavlink/ardupilotmega/mavlink.h"
#include "../include/logging/src/easylogging++.h"

#include "mlink.h"
#include "ConfigFile.h"
#include "asyncsocket.h"
#include "serial.h"
#include "exception.h"
#include "shell.h"

std::string filename;

bool exitMainLoop = false;
bool shellen = true;

//timing stuff, not happy with this implementation
long now_ms = 0;
long last_update_sysid_ms = 0;
long lastPrintLinkStatsMs = 0;
long myclock();

bool verbose = false;

//Global storage of pointers to links
std::vector<std::shared_ptr<mlink>> links;
ConfigFile _configFile;

// -------- FUNCTION DECLARATIONS IN THIS FILE ---------------
void runMainLoop(std::vector<std::shared_ptr<mlink>> *links);
void printLinkStats(std::vector<std::shared_ptr<mlink>> *links);
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);
void parseConfigFile(std::string filepath);
void exitGracefully(int a);

//Periodic function timings
#define UPDATE_SYSID_INTERVAL_MS 10
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10
#define PRINT_LINK_STATS_MS n10000

namespace
{
//RETURN CODES
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv)
{
    START_EASYLOGGINGPP(argc, argv);
    el::Loggers::configureFromGlobal("log.conf");
    signal(SIGINT, exitGracefully);
    try
    {
        /** Define and parse the program options
        */
        boost::program_options::options_description desc("Options");
        desc.add_options()
        ("help", "Print help messages")
        ("file,f", boost::program_options::value<std::string>(&filename),
         "configuration file, usage: --file=path/to/file.conf")
        ("interface,i", boost::program_options::bool_switch(&shellen),
         "start in interactive mode with cmav shell")
        ("verbose,v", boost::program_options::bool_switch(&verbose),
         "verbose output including dropped packets");

        boost::program_options::variables_map vm;
        try
        {
            boost::program_options::store(
                boost::program_options::parse_command_line(argc, argv, desc), vm);
            if ( vm.count("help")  )
            {
                std::cout << "CMAVNode - Flexible MAVLink Router" << std::endl
                          << desc << std::endl;
                return SUCCESS;
            }

            boost::program_options::notify(vm); // throws on error, so do after help in case
            // there are any problems

            if( !vm.count("file") )
            {
                LOG(ERROR) << "CMAVNode requires a config file";
                std::cerr << desc << std::endl;
                return ERROR_IN_COMMAND_LINE;
            }

            //parseConfigFile and allocate links
            parseConfigFile(filename);

        }
        catch(boost::program_options::error& e)
        {
            LOG(ERROR) << "ERROR: " << e.what();
            std::cerr << desc << std::endl;
            return ERROR_IN_COMMAND_LINE;
        }
        /*--------------END COMMAND LINE PARSING------------------*/

        if(links.size() == 0)
        {
            LOG(ERROR) << "No links specified, exiting";
            return ERROR_IN_COMMAND_LINE;
        }
        LOG(INFO) << "Command line arguments parsed succesfully";
        LOG(INFO) << "Links Initialized, routing loop starting";

        //Assign link id... should do this at allocation but it breaks something
        int counter = 0;
        for(unsigned int i = 0; i < links.size(); i++)
        {
            links.at(i)->link_id = counter++;
        }

        boost::thread shell;
        if(shellen)
        {
            shell = boost::thread(runShell);
        }

        while(!exitMainLoop)
        {
            runMainLoop(&links);
        }


        if(shellen)
            shell.join();

        /*----------------END MAIN CODE------------------*/
    }
    catch(std::exception& e)
    {
        LOG(FATAL) << "Unhandled Exception reached the top of main: "
                   << e.what() << ", application will now exit";
        return ERROR_UNHANDLED_EXCEPTION;

    }
    LOG(INFO) << "Links deallocated, stack unwound, exiting";
    return SUCCESS;
} //main

void parseConfigFile(std::string filepath)
{
    _configFile = ConfigFile(filepath);

    std::vector<std::string> sections = _configFile.GetSections();

    for (std::vector<std::string>::iterator i = sections.begin(); i < sections.end(); i++)
    {
        std::string type;
        bool isSerial = false;
        bool isUDP = false;
        std::cout<<"Link: " << *i << std::endl;
        try
        {
            type = _configFile.Value(*i,"type");
            std::cout<<"Type: " << type << std::endl;
        }
        catch(int e)
        {
            LOG(ERROR) << "Link has no type";
            continue;
        }
        std::string serialport;
        int baud;
        std::string targetip;
        int targetport, localport;

        //hack for now to hardcode the link_info
        std::vector<int> output_only_from;
        output_only_from.push_back(0);
        link_info infoloc;
        infoloc.link_name = *i;
        infoloc.output_only_from = output_only_from;
        infoloc.output_only_heartbeat_from = 0;

        if( type.compare("serial") == 0)
        {
            try
            {
                serialport = _configFile.Value(*i, "port");
                baud = _configFile.iValue(*i, "baud");
            }
            catch(int e)
            {
                LOG(ERROR) << "Invalid serial link";
                continue;
            }
            isSerial = true;
            LOG(INFO) << "Valid Serial Link Found " << serialport << " " << baud;
        }
        else if(type.compare("udp") == 0)
        {
            try
            {
                targetip = _configFile.Value(*i, "targetip");
                targetport = _configFile.iValue(*i, "targetport");
                localport = _configFile.iValue(*i, "localport");
            }
            catch(int e)
            {
                LOG(ERROR) << "Invalid UDP link";
                continue;
            }
            isUDP = true;
            LOG(INFO) << "Valid UDP Link Found " << targetip << ":" << targetport << " -> " << localport;
        }
        else
        {
            LOG(ERROR) << "Invalid link type: " << type;
            continue;
        }

        //if we made it this far without break we have a valid link of some sort
        if(isSerial)
        {
            links.push_back(std::shared_ptr<mlink>(new serial(serialport
                                                   ,std::to_string(baud)
                                                   ,infoloc)));
        }
        else if (isUDP)
        {
            links.push_back(
                std::shared_ptr<mlink>(new asyncsocket(targetip,
                                       std::to_string(targetport)
                                       ,std::to_string(localport)
                                       ,infoloc)));
        }
    }
}

void runMainLoop(std::vector<std::shared_ptr<mlink>> *links)
{
//Gets run in a while loop once links are setup

    for(unsigned int i = 0; i < links->size(); i++)
    {
        links->at(i)->getSysID_thisLink();
    }

    for(unsigned int i = 0; i < links->size(); i++)
    {
        mavlink_message_t msg;
        //while reading off buffer for link i
        while(links->at(i)->qReadIncoming(&msg))
        {
            int16_t sysIDmsg = 0;
            int16_t compIDmsg = 0;
            getTargets(&msg, sysIDmsg, compIDmsg);

            //we have got a message, work out where to send it
            LOG(DEBUG) << "Message received from sysID: " << (int)msg.sysid << " msgID: " << (int)msg.msgid << " target system: " << (int)sysIDmsg;

            bool wasForwarded = false;
            for(unsigned int n = 0; n < links->size(); n++)
            {

                bool dontSendOnThisLink = true;
                bool sysOnThisLink = false;
                //if the packet came from this link, dont bother
                if(n == i) sysOnThisLink = true;
                if(links->at(n)->info.output_only_from.at(0) != 0)
                {
                    for(unsigned int z = 0; z < links->at(n)->info.output_only_from.size(); z++)
                    {
                        if(msg.sysid == links->at(n)->info.output_only_from.at(z))
                            dontSendOnThisLink = false;
                    }
                }
                else dontSendOnThisLink = false;
                if(links->at(n)->info.output_only_heartbeat_from != 0)
                {
                    if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != links->at(n)->info.output_only_heartbeat_from)
                    {
                        dontSendOnThisLink = true;
                    }
                }

                //If this link doesn't point to the system that sent the message, send here
                if(!sysOnThisLink && !dontSendOnThisLink && links->at(n)->up)
                {
                    links->at(n)->qAddOutgoing(msg);
                    wasForwarded = true;
                }
            }

            if(!wasForwarded && verbose)
            {
                LOG(ERROR) << "Packet dropped from sysID: " << (int)msg.sysid << " msgID: " << (int)msg.msgid << " target system: " << (int)sysIDmsg << " link name: " << links->at(i)->info.link_name;
            }
        }
    }


    boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS));
}

void printLinkStats(std::vector<std::shared_ptr<mlink>> *links)
{
    LOG(INFO) << "=====================================";
    for(unsigned int i = 0; i < links->size(); i++)
    {
        std::ostringstream buffer;

        buffer << "Link: " << links->at(i)->link_id << " " << links->at(i)->info.link_name;
        if(links->at(i)->is_kill) buffer << " DEAD ";
        else if(links->at(i)->up) buffer << " UP ";
        else buffer << " DOWN ";

        buffer << " Received: " << links->at(i)->recentPacketCount << " Sent: " <<
               links->at(i)->recentPacketSent << " Systems on link: ";

        if(links->at(i)->sysIDpub.size() != 0)
        {
            for(unsigned int k = 0; k < links->at(i)->sysIDpub.size(); k++)
            {
                buffer << (int)links->at(i)->sysIDpub.at(k) << " ";
            }
        }
        else buffer << "none";

        links->at(i)->recentPacketCount = 0;
        links->at(i)->recentPacketSent = 0;
        LOG(INFO) << buffer.str();
    }
    LOG(INFO) << "+++++++++++++++++++++++++++++++++++++";
}

void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
{
    /* --------METHOD TAKEN FROM ARDUPILOT ROUTING LOGIC CODE ------------*/
    // unfortunately the targets are not in a consistent position in
    // the packets, so we need a switch. Using the single element
    // extraction functions (which are inline) makes this a bit faster
    // than it would otherwise be.
    // This list of messages below was extracted using:
    //
    // cat ardupilotmega/*h common/*h|egrep
    // 'get_target_system|get_target_component' |grep inline | cut
    // -d'(' -f1 | cut -d' ' -f4 > /tmp/targets.txt
    //
    // TODO: we should write a python script to extract this list
    // properly

    switch (msg->msgid)
    {
        // these messages only have a target system
    case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
        sysid = mavlink_msg_camera_feedback_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CAMERA_STATUS:
        sysid = mavlink_msg_camera_status_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        sysid = mavlink_msg_change_operator_control_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        sysid = mavlink_msg_set_mode_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        sysid = mavlink_msg_set_gps_global_origin_get_target_system(msg);
        break;

        // these support both target system and target component
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
        sysid  = mavlink_msg_digicam_configure_get_target_system(msg);
        compid = mavlink_msg_digicam_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        sysid  = mavlink_msg_digicam_control_get_target_system(msg);
        compid = mavlink_msg_digicam_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        sysid  = mavlink_msg_fence_fetch_point_get_target_system(msg);
        compid = mavlink_msg_fence_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_POINT:
        sysid  = mavlink_msg_fence_point_get_target_system(msg);
        compid = mavlink_msg_fence_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
        sysid  = mavlink_msg_mount_configure_get_target_system(msg);
        compid = mavlink_msg_mount_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        sysid  = mavlink_msg_mount_control_get_target_system(msg);
        compid = mavlink_msg_mount_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_STATUS:
        sysid  = mavlink_msg_mount_status_get_target_system(msg);
        compid = mavlink_msg_mount_status_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        sysid  = mavlink_msg_rally_fetch_point_get_target_system(msg);
        compid = mavlink_msg_rally_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_POINT:
        sysid  = mavlink_msg_rally_point_get_target_system(msg);
        compid = mavlink_msg_rally_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
        sysid  = mavlink_msg_set_mag_offsets_get_target_system(msg);
        compid = mavlink_msg_set_mag_offsets_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_INT:
        sysid  = mavlink_msg_command_int_get_target_system(msg);
        compid = mavlink_msg_command_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        sysid  = mavlink_msg_command_long_get_target_system(msg);
        compid = mavlink_msg_command_long_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        sysid  = mavlink_msg_file_transfer_protocol_get_target_system(msg);
        compid = mavlink_msg_file_transfer_protocol_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        sysid  = mavlink_msg_gps_inject_data_get_target_system(msg);
        compid = mavlink_msg_gps_inject_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        sysid  = mavlink_msg_log_erase_get_target_system(msg);
        compid = mavlink_msg_log_erase_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        sysid  = mavlink_msg_log_request_data_get_target_system(msg);
        compid = mavlink_msg_log_request_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        sysid  = mavlink_msg_log_request_end_get_target_system(msg);
        compid = mavlink_msg_log_request_end_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        sysid  = mavlink_msg_log_request_list_get_target_system(msg);
        compid = mavlink_msg_log_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        sysid  = mavlink_msg_mission_ack_get_target_system(msg);
        compid = mavlink_msg_mission_ack_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        sysid  = mavlink_msg_mission_clear_all_get_target_system(msg);
        compid = mavlink_msg_mission_clear_all_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        sysid  = mavlink_msg_mission_count_get_target_system(msg);
        compid = mavlink_msg_mission_count_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM:
        sysid  = mavlink_msg_mission_item_get_target_system(msg);
        compid = mavlink_msg_mission_item_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        sysid  = mavlink_msg_mission_item_int_get_target_system(msg);
        compid = mavlink_msg_mission_item_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        sysid  = mavlink_msg_mission_request_get_target_system(msg);
        compid = mavlink_msg_mission_request_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        sysid  = mavlink_msg_mission_request_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_request_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        sysid  = mavlink_msg_mission_set_current_get_target_system(msg);
        compid = mavlink_msg_mission_set_current_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_write_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_write_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        sysid  = mavlink_msg_param_request_list_get_target_system(msg);
        compid = mavlink_msg_param_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        sysid  = mavlink_msg_param_request_read_get_target_system(msg);
        compid = mavlink_msg_param_request_read_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        sysid  = mavlink_msg_param_set_get_target_system(msg);
        compid = mavlink_msg_param_set_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PING:
        sysid  = mavlink_msg_ping_get_target_system(msg);
        compid = mavlink_msg_ping_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        sysid  = mavlink_msg_rc_channels_override_get_target_system(msg);
        compid = mavlink_msg_rc_channels_override_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        sysid  = mavlink_msg_request_data_stream_get_target_system(msg);
        compid = mavlink_msg_request_data_stream_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
        sysid  = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
        compid = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        sysid  = mavlink_msg_set_attitude_target_get_target_system(msg);
        compid = mavlink_msg_set_attitude_target_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        sysid  = mavlink_msg_set_position_target_global_int_get_target_system(msg);
        compid = mavlink_msg_set_position_target_global_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        sysid  = mavlink_msg_set_position_target_local_ned_get_target_system(msg);
        compid = mavlink_msg_set_position_target_local_ned_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_V2_EXTENSION:
        sysid  = mavlink_msg_v2_extension_get_target_system(msg);
        compid = mavlink_msg_v2_extension_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        sysid  = mavlink_msg_gimbal_report_get_target_system(msg);
        compid = mavlink_msg_gimbal_report_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_CONTROL:
        sysid  = mavlink_msg_gimbal_control_get_target_system(msg);
        compid = mavlink_msg_gimbal_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT:
        sysid  = mavlink_msg_gimbal_torque_cmd_report_get_target_system(msg);
        compid = mavlink_msg_gimbal_torque_cmd_report_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK:
        sysid  = mavlink_msg_remote_log_data_block_get_target_system(msg);
        compid = mavlink_msg_remote_log_data_block_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        sysid  = mavlink_msg_remote_log_block_status_get_target_system(msg);
        compid = mavlink_msg_remote_log_block_status_get_target_component(msg);
        break;
    }
}

long myclock()
{
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::duration<float, std::milli> duration;

    static clock::time_point start = clock::now();
    duration elapsed = clock::now() - start;
    return elapsed.count();
}

void exitGracefully(int a)
{
    std::cout << "Exit code " << a << std::endl;
    if(shellen)
        std::cout << "SIGINT blocked. to exit type 'quit'" << std::endl;
    else
    {
        LOG(INFO) << "SIGINT caught, deconstructing links and exiting";
        exitMainLoop = true;
    }
}
