/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Mission::var_info[] = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, AP_Mission, _cmd_total, 0),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    // @User: Advanced
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Mission options bitmask
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0:Clear Mission on reboot
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  2, AP_Mission, _options, AP_MISSION_OPTIONS_DEFAULT),

	AP_GROUPINFO("BREAK_INDEX", 3, AP_Mission, _break_index, 0),  //断点的航点号
	AP_GROUPINFO("BREAK_ALT", 4, AP_Mission, _break_alt, 0),
	AP_GROUPINFO("BREAK_LAT", 5, AP_Mission, _break_lat, 0),
	AP_GROUPINFO("BREAK_LNG", 6, AP_Mission, _break_lng, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // changes in Content size break the storage
    static_assert(sizeof(union Content) == 12, "AP_Mission: Content must be 12 bytes");

    // If Mission Clear bit is set then it should clear the mission, otherwise retain the mission.
    if (AP_MISSION_MASK_MISSION_CLEAR & _options) {
    	gcs().send_text(MAV_SEVERITY_INFO, "Clearing Mission");
    	clear();	
    }

    _last_change_time_ms = AP_HAL::millis();
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;

    reset(); // reset mission to the first command, resets jump tracking
    
    // advance to the first command
    if (!advance_current_nav_cmd()) {
        // on failure set mission complete
        complete();
    }
}

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
void AP_Mission::stop()
{
    _flags.state = MISSION_STOPPED;
    breakPosition(GET_BREAK_POSITION);
    autoClimb(RESET_HOLD_INDEX);
}

/// resume - continues the mission execution from where we last left off
///     previous running commands will be re-initialized
void AP_Mission::resume()
{
    // if mission had completed then start it from the first command
    if (_flags.state == MISSION_COMPLETE) {
        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid nav command index restart from beginning
        if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }
    }

    // ensure cache coherence
    if (!read_cmd_from_storage(_nav_cmd.index, _nav_cmd)) {
        // if we failed to read the command from storage, then the command may have
        // been from a previously loaded mission it is illogical to ever resume
        // flying to a command that has been excluded from the current mission
        start();
        return;
    }

    // restart active navigation command. We run these on resume()
    // regardless of whether the mission was stopped, as we may be
    // re-entering AUTO mode and the nav_cmd callback needs to be run
    // to setup the current target waypoint

    // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
    //      update will take care of finding and starting the nav command
    if (_flags.nav_cmd_loaded) {
    	breakPosition(CHECK_GO_BREAK_RESUME);
    	autoClimb(GET_CLIMB_POSITION);
		_cmd_start_fn(_nav_cmd);
    }

    // restart active do command
    if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
        _cmd_start_fn(_do_cmd);
    }
}

/// check mission starts with a takeoff command
bool AP_Mission::starts_with_takeoff_cmd()
{
    Mission_Command cmd = {};
    uint16_t cmd_index = _restart ? AP_MISSION_CMD_INDEX_NONE : _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }

    // check a maximum of 16 items, remembering that missions can have
    // loops in them
    for (uint8_t i=0; i<16; i++, cmd_index++) {
        if (!get_next_nav_cmd(cmd_index, cmd)) {
            return false;
        }
        switch (cmd.id) {
        // any of these are considered a takeoff command:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_TAKEOFF_LOCAL:
            return true;
        // any of these are considered "skippable" when determining if
        // we "start with a takeoff command"
        case MAV_CMD_NAV_DELAY:
            continue;
        default:
            return false;
        }
    }
    return false;
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{
    if (_restart) {
        start();
    } else {
        resume();
    }
}

/// reset - reset mission to the first command
void AP_Mission::reset()
{
    _flags.nav_cmd_loaded  = false;
    _flags.do_cmd_loaded   = false;
    _flags.do_cmd_all_done = false;
    _nav_cmd.index         = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index          = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_index    = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_id       = AP_MISSION_CMD_ID_NONE;
    init_jump_tracking();
    autoClimb(RESET_VERTICAL_CLIMB);//每次重新开始执行航线都将isVerticalClimb复位
}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Mission::clear()
{
    // do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // remove all commands
    _cmd_total.set_and_save(0);

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // return success
    return true;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{
    if ((unsigned)_cmd_total > index) {        
        _cmd_total.set_and_save(index);
    }
    breakPosition(DELETE_BREAK_POSITION);
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    // check if we have an active nav command检查我们是否有一个主动的导航命令
    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        // advance in mission if no active nav command
        if (!advance_current_nav_cmd()) {
            // failure to advance nav command means mission has completed 没有前进导航指令意味着任务已经完成
            complete(); //结束任务
            return;
        }
    }else{
        // run the active nav command 检查nav命令是否执行完毕
        if (_cmd_verify_fn(_nav_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.nav_cmd_loaded = false;
            // immediately advance to the next mission command
            if (!advance_current_nav_cmd()) {
                // failure to advance nav command means mission has completed
                complete();
                return;
            }
        }
    }

    // check if we have an active do command
    if (!_flags.do_cmd_loaded) {
        advance_current_do_cmd();
    }else{
        // run the active do command 检查do命令是否执行完毕
        if (_cmd_verify_fn(_do_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.do_cmd_loaded = false; //标记命令执行完毕
        }
    }
}

///
/// public command methods
///

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Mission::add_cmd(Mission_Command& cmd) //将航点写入存储器
{
    // attempt to write the command to storage
    bool ret = write_cmd_to_storage(_cmd_total, cmd);

    if (ret) {
        // update command's index
        cmd.index = _cmd_total;
        // increment total number of commands
        _cmd_total.set_and_save(_cmd_total + 1); //航点总数加一
    }

    return ret;
}

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Mission::replace_cmd(uint16_t index, Mission_Command& cmd)
{
    // sanity check index
    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // attempt to write the command to storage
    return write_cmd_to_storage(index, cmd);
}

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    // NAV commands all have ids below MAV_CMD_NAV_LAST except NAV_SET_YAW_SPEED
    return (cmd.id <= MAV_CMD_NAV_LAST || cmd.id == MAV_CMD_NAV_SET_YAW_SPEED);
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    uint16_t cmd_index = start_index;

    // search until the end of the mission command list
    while(cmd_index < (unsigned)_cmd_total) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, false)) {
            // no more commands so return failure
            return false;
        }
        // if found a "navigation" command then return it
        if (is_nav_cmd(cmd)) {
            return true;
        }
        // move on in list
        cmd_index++;
    }

    // if we got this far we did not find a navigation command
    return false;
}

/// get the ground course of the next navigation leg in centidegrees
/// from 0 36000. Return default_angle if next navigation
/// leg cannot be determined
int32_t AP_Mission::get_next_ground_course_cd(int32_t default_angle)
{
    Mission_Command cmd;
    if (!get_next_nav_cmd(_nav_cmd.index+1, cmd)) {
        return default_angle;
    }
    // special handling for nav commands with no target location
    if (cmd.id == MAV_CMD_NAV_GUIDED_ENABLE ||
        cmd.id == MAV_CMD_NAV_DELAY) {
        return default_angle;
    }
    if (cmd.id == MAV_CMD_NAV_SET_YAW_SPEED) {
        return (_nav_cmd.content.set_yaw_speed.angle_deg * 100);
    }
    return get_bearing_cd(_nav_cmd.content.location, cmd.content.location);
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index)
{
    Mission_Command cmd;

    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // stop current nav cmd
    _flags.nav_cmd_loaded = false;

    // if index is zero then the user wants to completely restart the mission
    if (index == 0 || _flags.state == MISSION_COMPLETE) {
        _prev_nav_cmd_id    = AP_MISSION_CMD_ID_NONE;
        _prev_nav_cmd_index = AP_MISSION_CMD_INDEX_NONE;
        _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
        // reset the jump tracking to zero
        init_jump_tracking();
        if (index == 0) {
            index = 1;
        }
    }

    // if the mission is stopped or completed move the nav_cmd index to the specified point and set the state to stopped
    // so that if the user resumes the mission it will begin at the specified index
    if (_flags.state != MISSION_RUNNING) {
        // search until we find next nav command or reach end of command list
        while (!_flags.nav_cmd_loaded) {
            // get next command
            if (!get_next_cmd(index, cmd, true)) {
                _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
                return false;
            }

            // check if navigation or "do" command
            if (is_nav_cmd(cmd)) {
                // set current navigation command
                _nav_cmd = cmd;
                _flags.nav_cmd_loaded = true;
            }else{
                // set current do command
                if (!_flags.do_cmd_loaded) {
                    _do_cmd = cmd;
                    _flags.do_cmd_loaded = true;
                }
            }
            // move onto next command
            index = cmd.index+1;
        }

        // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
        if (!_flags.do_cmd_loaded) {
            _flags.do_cmd_all_done = true;
        }

        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        return true;
    }

    // the state must be MISSION_RUNNING
    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded) {
        // get next command
        if (!get_next_cmd(index, cmd, true)) {
            // if we run out of nav commands mark mission as complete
            complete();
            // return true because we did what was requested
            // which was apparently to jump to a command at the end of the mission
            return true;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_id = _nav_cmd.id;
            _prev_nav_cmd_index = _nav_cmd.index;
            // save separate previous nav command index if it contains lat,long,alt
            if (!(cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
                _prev_nav_cmd_wp_index = _nav_cmd.index;
            }
            // set current navigation command and start it
            _nav_cmd = cmd;
            _flags.nav_cmd_loaded = true;
            _cmd_start_fn(_nav_cmd);
        }else{
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                _cmd_start_fn(_do_cmd);
            }
        }
        // move onto next command
        index = cmd.index+1;
    }

    // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
    if (!_flags.do_cmd_loaded) {
        _flags.do_cmd_all_done = true;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
    // exit immediately if index is beyond last command but we always let cmd #0 (i.e. home) be read
    if (index >= (unsigned)_cmd_total && index != 0) {
        return false;
    }

    // special handling for command #0 which is home
    if (index == 0) {
        cmd.index = 0;
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = _ahrs.get_home();
    }else{
        // Find out proper location in memory by using the start_byte position + the index
        // we can load a command, we don't process it yet
        // read WP position
        uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

        uint8_t b1 = _storage.read_byte(pos_in_storage);
        if (b1 == 0) {
            cmd.id = _storage.read_uint16(pos_in_storage+1);
            cmd.p1 = _storage.read_uint16(pos_in_storage+3);
            _storage.read_block(cmd.content.bytes, pos_in_storage+5, 10);
        } else {
            cmd.id = b1;
            cmd.p1 = _storage.read_uint16(pos_in_storage+1);
            _storage.read_block(cmd.content.bytes, pos_in_storage+3, 12);
        }

        // set command's index to it's position in eeprom
        cmd.index = index;
    }

    // return success
    return true;
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint16_t index, Mission_Command& cmd)
{
    // range check cmd's index
    if (index >= num_commands_max()) { //检查航点号是否超过存储器可接收的最大值
        return false;
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE); //航点在存储器中的位置

    if (cmd.id < 256) {
        _storage.write_byte(pos_in_storage, cmd.id);
        _storage.write_uint16(pos_in_storage+1, cmd.p1);
        _storage.write_block(pos_in_storage+3, cmd.content.bytes, 12);
    } else {
        // if the command ID is above 256 we store a 0 followed by the 16 bit command ID
        _storage.write_byte(pos_in_storage, 0);
        _storage.write_uint16(pos_in_storage+1, cmd.id);
        _storage.write_uint16(pos_in_storage+3, cmd.p1);
        _storage.write_block(pos_in_storage+5, cmd.content.bytes, 10);
    }

    // remember when the mission last changed
    _last_change_time_ms = AP_HAL::millis();

    // return success
    return true;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{
    Mission_Command home_cmd = {};
    home_cmd.id = MAV_CMD_NAV_WAYPOINT;
    home_cmd.content.location = _ahrs.get_home();
    write_cmd_to_storage(0,home_cmd);
}

MAV_MISSION_RESULT AP_Mission::sanity_check_params(const mavlink_mission_item_int_t& packet) {
    uint8_t nan_mask;
    switch (packet.command) {
        case MAV_CMD_NAV_WAYPOINT:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_LAND:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_LAND:
            nan_mask = ~((1 << 2) | (1 << 3)); // param 3 and 4 can be nan
            break;
        default:
            nan_mask = 0xff;
            break;
    }

    if (((nan_mask & (1 << 0)) && isnan(packet.param1)) ||
        isinf(packet.param1)) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (((nan_mask & (1 << 1)) && isnan(packet.param2)) ||
        isinf(packet.param2)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (((nan_mask & (1 << 2)) && isnan(packet.param3)) ||
        isinf(packet.param3)) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    if (((nan_mask & (1 << 3)) && isnan(packet.param4)) ||
        isinf(packet.param4)) {
        return MAV_MISSION_INVALID_PARAM4;
    }
    return MAV_MISSION_ACCEPTED;
}

// mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd)
{
    bool copy_location = false;
    bool copy_alt = false;

    // command's position in mission list and mavlink id
    cmd.index = packet.seq;
    cmd.id = packet.command;
    cmd.content.location.options = 0;

    MAV_MISSION_RESULT param_check = sanity_check_params(packet);
    if (param_check != MAV_MISSION_ACCEPTED) {
        return param_check;
    }

    // command specific conversions from mavlink packet to mission command
    switch (cmd.id) {

    case 0:
        // this is reserved for storing 16 bit command IDs
        return MAV_MISSION_INVALID;
        
    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
    {
        copy_location = true;
        /*
          the 15 byte limit means we can't fit both delay and radius
          in the cmd structure. When we expand the mission structure
          we can do this properly
         */
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters and pass by distance in meters
        uint16_t acp = packet.param2;           // param 2 is acceptance radius in meters is held in low p1
        uint16_t passby = packet.param3;        // param 3 is pass by distance in meters is held in high p1

        // limit to 255 so it does not wrap during the shift or mask operation
        passby = MIN(0xFF,passby);
        acp = MIN(0xFF,acp);

        cmd.p1 = (passby << 8) | (acp & 0x00FF);
#else
        // delay at waypoint in seconds (this is for copters???)
        cmd.p1 = packet.param1;
#endif
    }
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        cmd.p1 = fabsf(packet.param3);                  // store radius as 16bit since no other params are competing for space
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
    {
        copy_location = true;
        uint16_t num_turns = packet.param1;              // param 1 is number of times to circle is held in low p1
        uint16_t radius_m = fabsf(packet.param3);        // param 3 is radius in meters is held in high p1
        cmd.p1 = (radius_m<<8) | (num_turns & 0x00FF);   // store radius in high byte of p1, num turns in low byte of p1
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.flags.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    }
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        cmd.p1 = packet.param1;                         // loiter time in seconds uses all 16 bits, 8bit seconds is too small. No room for radius.
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.flags.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        cmd.p1 = packet.param1;                         // abort target altitude(m)  (plane only)
        cmd.content.location.flags.loiter_ccw = is_negative(packet.param4); // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        cmd.p1 = packet.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        copy_location = true;                           // lat/lng used for heading lock
        cmd.p1 = packet.param1;                         // Climb/Descend
                        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
                        // 1 = Climb, cmd complete at or above indicated alt.
                        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        copy_location = true;
        cmd.p1 = fabsf(packet.param2);                  // param2 is radius in meters
        cmd.content.location.flags.loiter_ccw = (packet.param2 < 0);
        cmd.content.location.flags.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        copy_location = true;
        cmd.p1 = packet.param1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        cmd.p1 = packet.param1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 94
        cmd.content.nav_delay.seconds = packet.param1; // delay in seconds
        cmd.content.nav_delay.hour_utc = packet.param2;// absolute time's hour (utc)
        cmd.content.nav_delay.min_utc = packet.param3;// absolute time's min (utc)
        cmd.content.nav_delay.sec_utc = packet.param4; // absolute time's second (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        cmd.content.delay.seconds = packet.param1;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        cmd.content.distance.meters = packet.param1;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        cmd.content.yaw.angle_deg = packet.param1;      // target angle in degrees
        cmd.content.yaw.turn_rate_dps = packet.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        cmd.content.yaw.direction = packet.param3;      // -1 = ccw, +1 = cw
        cmd.content.yaw.relative_angle = packet.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        cmd.p1 = packet.param1;                         // flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = packet.param1;        // jump-to command number
        cmd.content.jump.num_times = packet.param2;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        cmd.content.speed.speed_type = packet.param1;   // 0 = airspeed, 1 = ground speed
        cmd.content.speed.target_ms = packet.param2;    // target speed in m/s
        cmd.content.speed.throttle_pct = packet.param3; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:
        copy_location = true;
        cmd.p1 = packet.param1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        cmd.content.relay.num = packet.param1;          // relay number
        cmd.content.relay.state = packet.param2;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        cmd.content.repeat_relay.num = packet.param1;           // relay number
        cmd.content.repeat_relay.repeat_count = packet.param2;  // count
        cmd.content.repeat_relay.cycle_time = packet.param3;    // time converted from seconds to milliseconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = packet.param1;      // channel
        cmd.content.servo.pwm = packet.param2;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        cmd.content.repeat_servo.channel = packet.param1;      // channel
        cmd.content.repeat_servo.pwm = packet.param2;          // PWM
        cmd.content.repeat_servo.repeat_count = packet.param3; // count
        cmd.content.repeat_servo.cycle_time = packet.param4;   // time in seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        copy_location = true;
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        copy_location = true;
        cmd.p1 = packet.param1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        cmd.content.digicam_configure.shooting_mode = packet.param1;
        cmd.content.digicam_configure.shutter_speed = packet.param2;
        cmd.content.digicam_configure.aperture = packet.param3;
        cmd.content.digicam_configure.ISO = packet.param4;
        cmd.content.digicam_configure.exposure_type = packet.x;
        cmd.content.digicam_configure.cmd_id = packet.y;
        cmd.content.digicam_configure.engine_cutoff_time = packet.z;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        cmd.content.digicam_control.session = packet.param1;
        cmd.content.digicam_control.zoom_pos = packet.param2;
        cmd.content.digicam_control.zoom_step = packet.param3;
        cmd.content.digicam_control.focus_lock = packet.param4;
        cmd.content.digicam_control.shooting_cmd = packet.x;
        cmd.content.digicam_control.cmd_id = packet.y;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        cmd.content.mount_control.pitch = packet.param1;
        cmd.content.mount_control.roll = packet.param2;
        cmd.content.mount_control.yaw = packet.param3;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = packet.param1;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        cmd.p1 = packet.param1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                         // MAV ID: 208
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        cmd.p1 = packet.param1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        cmd.content.gripper.num = packet.param1;        // gripper number
        cmd.content.gripper.action = packet.param2;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        cmd.p1 = packet.param1;                         // max time in seconds the external controller will be allowed to control the vehicle
        cmd.content.guided_limits.alt_min = packet.param2;  // min alt below which the command will be aborted.  0 for no lower alt limit
        cmd.content.guided_limits.alt_max = packet.param3;  // max alt above which the command will be aborted.  0 for no upper alt limit
        cmd.content.guided_limits.horiz_max = packet.param4;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:                    // MAV ID: 211
        cmd.p1 = packet.param1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        cmd.content.altitude_wait.altitude = packet.param1;
        cmd.content.altitude_wait.descent_rate = packet.param2;
        cmd.content.altitude_wait.wiggle_time = packet.param3;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        copy_location = true;
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        copy_location = true;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        cmd.content.do_vtol_transition.target_state = packet.param1;
        break;

    case MAV_CMD_DO_SET_REVERSE:
        cmd.p1 = packet.param1; // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        cmd.content.do_engine_control.start_control = (packet.param1>0);
        cmd.content.do_engine_control.cold_start = (packet.param2>0);
        cmd.content.do_engine_control.height_delay_cm = packet.param3*100;
        break;        

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd.p1 = packet.param1*100; // copy max-descend parameter (m->cm)
        copy_location = true;
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        cmd.content.set_yaw_speed.angle_deg = packet.param1;        // target angle in degrees
        cmd.content.set_yaw_speed.speed = packet.param2;            // speed in meters/second
        cmd.content.set_yaw_speed.relative_angle = packet.param3;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:                              // MAV ID: 42600
        cmd.content.winch.num = packet.param1;          // winch number
        cmd.content.winch.action = packet.param2;       // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        cmd.content.winch.release_length = packet.param3;   // cable distance to unwind in meters, negative numbers to wind in cable
        cmd.content.winch.release_rate = packet.param4; // release rate in meters/second
        break;

    default:
        // unrecognised command
        return MAV_MISSION_UNSUPPORTED;
    }

    // copy location from mavlink to command
    if (copy_location || copy_alt) {

        // sanity check location
        if (copy_location) {
            if (!check_lat(packet.x)) {
                return MAV_MISSION_INVALID_PARAM5_X;
            }
            if (!check_lng(packet.y)) {
                return MAV_MISSION_INVALID_PARAM6_Y;
            }
        }
        if (isnan(packet.z) || fabsf(packet.z) >= LOCATION_ALT_MAX_M) {
            return MAV_MISSION_INVALID_PARAM7;
        }

        if (copy_location) {
            cmd.content.location.lat = packet.x;
            cmd.content.location.lng = packet.y;
        }

        cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)

        switch (packet.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
            cmd.content.location.flags.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            cmd.content.location.flags.relative_alt = 1;
            break;

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            cmd.content.location.flags.relative_alt = 1;
            // mark altitude as above terrain, not above home
            cmd.content.location.flags.terrain_alt = 1;
            break;
#endif

        default:
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }
    }

    // if we got this far then it must have been successful
    return MAV_MISSION_ACCEPTED;
}

// converts a mission_item to mission_item_int and returns a mission_command
MAV_MISSION_RESULT AP_Mission::mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd)
{
    mavlink_mission_item_int_t mav_cmd = {};

    mav_cmd.param1 = packet.param1;
    mav_cmd.param2 = packet.param2;
    mav_cmd.param3 = packet.param3;
    mav_cmd.param4 = packet.param4;
    mav_cmd.z = packet.z;
    mav_cmd.seq = packet.seq;
    mav_cmd.command = packet.command;
    mav_cmd.target_system = packet.target_system;
    mav_cmd.target_component = packet.target_component;
    mav_cmd.frame = packet.frame;
    mav_cmd.current = packet.current;
    mav_cmd.autocontinue = packet.autocontinue;
    
    /*
      the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
      is to pass the lat/lng in MISSION_ITEM_INT straight through, and
      for MISSION_ITEM multiply by 1e7 here. We need an exception for
      any commands which use the x and y fields not as
      latitude/longitude.
     */
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        mav_cmd.x = packet.x;
        mav_cmd.y = packet.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e7 to convert to int32_t
        if (!check_lat(packet.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(packet.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        mav_cmd.x = packet.x * 1.0e7f;
        mav_cmd.y = packet.y * 1.0e7f;
        break;
    }
    
    MAV_MISSION_RESULT ans = mavlink_int_to_mission_cmd(mav_cmd, cmd);
    
    return ans;
}

// converts a Mission_Command to mission_item_int and returns a mission_item
bool AP_Mission::mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet)
{
    mavlink_mission_item_int_t mav_cmd = {};
    
    bool ans = mission_cmd_to_mavlink_int(cmd, (mavlink_mission_item_int_t&)mav_cmd);
    
    packet.param1 = mav_cmd.param1;
    packet.param2 = mav_cmd.param2;
    packet.param3 = mav_cmd.param3;
    packet.param4 = mav_cmd.param4;
    packet.z = mav_cmd.z;
    packet.seq = mav_cmd.seq;
    packet.command = mav_cmd.command;
    packet.target_system = mav_cmd.target_system;
    packet.target_component = mav_cmd.target_component;
    packet.frame = mav_cmd.frame;
    packet.current = mav_cmd.current;
    packet.autocontinue = mav_cmd.autocontinue;

    /*
      the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
      is to pass the lat/lng in MISSION_ITEM_INT straight through, and
      for MISSION_ITEM multiply by 1e-7 here. We need an exception for
      any commands which use the x and y fields not as
      latitude/longitude.
     */
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        packet.x = mav_cmd.x;
        packet.y = mav_cmd.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e-7 to convert to int32_t
        packet.x = mav_cmd.x * 1.0e-7f;
        packet.y = mav_cmd.y * 1.0e-7f;
        break;
    }
    
    return ans;
}

// mavlink_cmd_long_to_mission_cmd - converts a mavlink cmd long to an AP_Mission::Mission_Command object which can be stored to eeprom
// return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_cmd_long_to_mission_cmd(const mavlink_command_long_t& packet, AP_Mission::Mission_Command& cmd) 
{
    mavlink_mission_item_t miss_item = {0};
 
    miss_item.param1 = packet.param1;
    miss_item.param2 = packet.param2;
    miss_item.param3 = packet.param3;
    miss_item.param4 = packet.param4;

    miss_item.command = packet.command;
    miss_item.target_system = packet.target_system;
    miss_item.target_component = packet.target_component;

    return mavlink_to_mission_cmd(miss_item, cmd);
}

// mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
bool AP_Mission::mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet)
{
    bool copy_location = false;
    bool copy_alt = false;

    // command's position in mission list and mavlink id
    packet.seq = cmd.index;
    packet.command = cmd.id;

    // set defaults
    packet.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    packet.param1 = 0;
    packet.param2 = 0;
    packet.param3 = 0;
    packet.param4 = 0;
    packet.autocontinue = 1;

    // command specific conversions from mission command to mavlink packet
    switch (cmd.id) {
    case 0:
        // this is reserved for 16 bit command IDs
        return false;
        
    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
        copy_location = true;
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters

        packet.param2 = LOWBYTE(cmd.p1);        // param 2 is acceptance radius in meters is held in low p1
        packet.param3 = HIGHBYTE(cmd.p1);       // param 3 is pass by distance in meters is held in high p1
#else
        // delay at waypoint in seconds
        packet.param1 = cmd.p1;
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        packet.param3 = (float)cmd.p1;
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 *= -1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        copy_location = true;
        packet.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        packet.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -packet.param3;
        }
        packet.param4 = cmd.content.location.flags.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        packet.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -1;
        } else {
            packet.param3 = 1;
        }
        packet.param4 = cmd.content.location.flags.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        packet.param1 = cmd.p1;                        // abort target altitude(m)  (plane only)
        packet.param4 = cmd.content.location.flags.loiter_ccw ? -1 : 1; // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        packet.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        copy_location = true;                           // lat/lng used for heading lock
        packet.param1 = cmd.p1;                         // Climb/Descend
                        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
                        // 1 = Climb, cmd complete at or above indicated alt.
                        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        copy_location = true;
        packet.param2 = cmd.p1;                        // loiter radius(m)
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param2 = -packet.param2;
        }
        packet.param4 = cmd.content.location.flags.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        copy_location = true;
        packet.param1 = cmd.p1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        packet.param1 = cmd.p1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 94
        packet.param1 = cmd.content.nav_delay.seconds; // delay in seconds
        packet.param2 = cmd.content.nav_delay.hour_utc; // absolute time's day of week (utc)
        packet.param3 = cmd.content.nav_delay.min_utc; // absolute time's hour (utc)
        packet.param4 = cmd.content.nav_delay.sec_utc; // absolute time's min (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        packet.param1 = cmd.content.delay.seconds;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        packet.param1 = cmd.content.distance.meters;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        packet.param1 = cmd.content.yaw.angle_deg;      // target angle in degrees
        packet.param2 = cmd.content.yaw.turn_rate_dps;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        packet.param3 = cmd.content.yaw.direction;      // -1 = ccw, +1 = cw
        packet.param4 = cmd.content.yaw.relative_angle; // 0 = absolute angle provided, 1 = relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        packet.param1 = cmd.p1;                         // set flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        packet.param1 = cmd.content.jump.target;        // jump-to command number
        packet.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        packet.param1 = cmd.content.speed.speed_type;   // 0 = airspeed, 1 = ground speed
        packet.param2 = cmd.content.speed.target_ms;    // speed in m/s
        packet.param3 = cmd.content.speed.throttle_pct; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:                           // MAV ID: 179
        copy_location = true;
        packet.param1 = cmd.p1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        packet.param1 = cmd.content.relay.num;          // relay number
        packet.param2 = cmd.content.relay.state;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        packet.param1 = cmd.content.repeat_relay.num;           // relay number
        packet.param2 = cmd.content.repeat_relay.repeat_count;  // count
        packet.param3 = cmd.content.repeat_relay.cycle_time;    // time in seconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        packet.param1 = cmd.content.servo.channel;      // channel
        packet.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        packet.param1 = cmd.content.repeat_servo.channel;       // channel
        packet.param2 = cmd.content.repeat_servo.pwm;           // PWM
        packet.param3 = cmd.content.repeat_servo.repeat_count;  // count
        packet.param4 = cmd.content.repeat_servo.cycle_time;    // time in milliseconds converted to seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        copy_location = true;
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        copy_location = true;
        packet.param1 = cmd.p1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        packet.param1 = cmd.content.digicam_configure.shooting_mode;
        packet.param2 = cmd.content.digicam_configure.shutter_speed;
        packet.param3 = cmd.content.digicam_configure.aperture;
        packet.param4 = cmd.content.digicam_configure.ISO;
        packet.x = cmd.content.digicam_configure.exposure_type;
        packet.y = cmd.content.digicam_configure.cmd_id;
        packet.z = cmd.content.digicam_configure.engine_cutoff_time;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        packet.param1 = cmd.content.digicam_control.session;
        packet.param2 = cmd.content.digicam_control.zoom_pos;
        packet.param3 = cmd.content.digicam_control.zoom_step;
        packet.param4 = cmd.content.digicam_control.focus_lock;
        packet.x = cmd.content.digicam_control.shooting_cmd;
        packet.y = cmd.content.digicam_control.cmd_id;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        packet.param1 = cmd.content.mount_control.pitch;
        packet.param2 = cmd.content.mount_control.roll;
        packet.param3 = cmd.content.mount_control.yaw;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        packet.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                          // MAV ID: 208
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        packet.param1 = cmd.p1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        packet.param1 = cmd.content.gripper.num;        // gripper number
        packet.param2 = cmd.content.gripper.action;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        packet.param1 = cmd.p1;                         // max time in seconds the external controller will be allowed to control the vehicle
        packet.param2 = cmd.content.guided_limits.alt_min;  // min alt below which the command will be aborted.  0 for no lower alt limit
        packet.param3 = cmd.content.guided_limits.alt_max;  // max alt above which the command will be aborted.  0 for no upper alt limit
        packet.param4 = cmd.content.guided_limits.horiz_max;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        packet.param1 = cmd.p1;                         // disable=0 enable=1
        break;

    case MAV_CMD_DO_SET_REVERSE:
        packet.param1 = cmd.p1;   // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        packet.param1 = cmd.content.altitude_wait.altitude;
        packet.param2 = cmd.content.altitude_wait.descent_rate;
        packet.param3 = cmd.content.altitude_wait.wiggle_time;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        copy_location = true;
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        copy_location = true;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        packet.param1 = cmd.content.do_vtol_transition.target_state;
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        packet.param1 = cmd.content.do_engine_control.start_control?1:0;
        packet.param2 = cmd.content.do_engine_control.cold_start?1:0;
        packet.param3 = cmd.content.do_engine_control.height_delay_cm*0.01f;
        break;        

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        copy_location = true;
        packet.param1 = cmd.p1/100.0f; // copy max-descend parameter (m->cm)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        packet.param1 = cmd.content.set_yaw_speed.angle_deg;        // target angle in degrees
        packet.param2 = cmd.content.set_yaw_speed.speed;            // speed in meters/second
        packet.param3 = cmd.content.set_yaw_speed.relative_angle;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:
        packet.param1 = cmd.content.winch.num;              // winch number
        packet.param2 = cmd.content.winch.action;           // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        packet.param3 = cmd.content.winch.release_length;   // cable distance to unwind in meters, negative numbers to wind in cable
        packet.param4 = cmd.content.winch.release_rate;     // release rate in meters/second
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (copy_location) {
        packet.x = cmd.content.location.lat;
        packet.y = cmd.content.location.lng;
    }
    if (copy_location || copy_alt) {
        packet.z = cmd.content.location.alt / 100.0f;   // cmd alt in cm to m
        if (cmd.content.location.flags.relative_alt) {
            packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }else{
            packet.frame = MAV_FRAME_GLOBAL;
        }
#if AP_TERRAIN_AVAILABLE
        if (cmd.content.location.flags.terrain_alt) {
            // this is a above-terrain altitude
            if (!cmd.content.location.flags.relative_alt) {
                // refuse to return non-relative terrain mission
                // items. Internally we do have these, and they
                // have home.alt added, but we should never be
                // returning them to the GCS, as the GCS doesn't know
                // our home.alt, so it would have no way to properly
                // interpret it
                return false;
            }
            packet.z = cmd.content.location.alt * 0.01f;
            packet.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        }
#else
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.flags.terrain_alt) {
            return false;
        }
#endif
    }

    // if we got this far then it must have been successful
    return true;
}

///
/// private methods
///

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AP_Mission::complete()
{
    // flag mission as complete
    _flags.state = MISSION_COMPLETE;

    //gcs().send_text(MAV_SEVERITY_ALERT, "MISSION_COMPLETE");
    // callback to main program's mission complete function
    _mission_complete_fn();
}

//函数中包含了垂直爬升功能的所有程序
void AP_Mission::autoClimb(autoClimbEvent event,uint16_t *index)
{
	struct Location loc;
	int32_t relativeAlt;
	switch (event) {
	case CHECK_NEED_CLIMB:
		 if(isVerticalClimb){ //检测是否进行垂直爬升，如果需要则进行爬升
			 isVerticalClimb = 0;
		 }else{
			 return;
		 }
	case GET_CLIMB_POSITION:
		isHoldIndex = 1;
		_ahrs.get_position(loc); //获取当前位置
		relativeAlt = loc.alt - _ahrs.get_home().alt; //获取相对高度
		if(_nav_cmd.content.location.flags.relative_alt && (!_nav_cmd.content.location.flags.terrain_alt)){ //如果航线是相对高度
			if(relativeAlt <= _nav_cmd.content.location.alt){ //如果当前高度小于目标航点高度则垂直爬升至目标高度，否则保持目前高度飞行至目标航点
				_nav_cmd.content.location.lat = loc.lat;
				_nav_cmd.content.location.lng = loc.lng;
			}else{
				_nav_cmd.content.location.alt = relativeAlt;
			}
		}else if(_nav_cmd.content.location.flags.relative_alt && _nav_cmd.content.location.flags.terrain_alt){
			/*if(relativeAlt<=_nav_cmd.content.location.alt){ //如果当前高度小于目标航点高度则垂直爬升至目标高度，否则保持目前高度飞行至目标航点
				_nav_cmd.content.location.lat = loc.lat;
				_nav_cmd.content.location.lng = loc.lng;
			}else{
				_nav_cmd.content.location.alt = relativeAlt;
			}*/
		}else{
			if(loc.alt<=_nav_cmd.content.location.alt){ //如果当前高度小于目标航点高度则垂直爬升至目标高度，否则保持目前高度飞行至目标航点
				_nav_cmd.content.location.lat = loc.lat;
				_nav_cmd.content.location.lng = loc.lng;
			}else{
				_nav_cmd.content.location.alt = loc.alt;
			}
		}
		return;
	case GET_WAYPOINT_INDEX:
		if(isHoldIndex){ //将航点号减一，重复执行上次的航点，因为上次执行的是原地爬升航点或者是中断点
			isHoldIndex = 0;
			*index = (*index) - 1;
		}
		return;
	case RESET_VERTICAL_CLIMB:
		isVerticalClimb = 1; //每次重新开始执行航线都将isVerticalClimb复位
		return;
	case RESET_HOLD_INDEX:
		isHoldIndex = 0; //复位保持航点位
		return;
	default:
		return;
	}
}
//函数中包含了记录中断点功能的所有程序
void AP_Mission::breakPosition(breakPositionEvent event)
{
	struct Location loc;
	int32_t relativeAlt;
	switch (event) {
	case GET_BREAK_POSITION:
	    //如果当前航点大于1（排除起飞过程中）且不是返航航点（排除在返航过程中）且上个中断点已完成，则进行记录中断点
		if(_nav_cmd.index > 1 && _nav_cmd.id != MAV_CMD_NAV_RETURN_TO_LAUNCH && (breakState == 3 || breakState == 0)){
			_ahrs.get_position(loc); //获取当前位置
			relativeAlt = loc.alt - _ahrs.get_home().alt; //获取相对高度
			if(_nav_cmd.content.location.flags.relative_alt && (!_nav_cmd.content.location.flags.terrain_alt)){ //如果航线是相对高度
				_break_index.set_and_save(_nav_cmd.index);
				_break_lat.set_and_save(loc.lat);
				_break_lng.set_and_save(loc.lng);
				_break_alt.set_and_save(relativeAlt);
			}else if(_nav_cmd.content.location.flags.relative_alt &&_nav_cmd.content.location.flags.terrain_alt){
				_break_index.set_and_save(_nav_cmd.index);
				_break_lat.set_and_save(loc.lat);
				_break_lng.set_and_save(loc.lng);
				_break_alt.set_and_save(_nav_cmd.content.location.alt);
			}else{
				_break_index.set_and_save(_nav_cmd.index);
				_break_lat.set_and_save(loc.lat);
				_break_lng.set_and_save(loc.lng);
				_break_alt.set_and_save(loc.alt);
			}
		}
		breakState = 0; //复位中断点状态
		return;
	case CHECK_GO_BREAK:
		if((_break_index != 0) && (breakState == 0)){ //如果有中断点则第一次执行中断点

		}else if((_break_index != 0) && (_break_index == _nav_cmd.index) && (breakState == 1)){ //自动爬升高度后第二次执行中断点
			breakState = 2;
			isHoldIndex = 1; //到达中断点后执行后面的航点
			_nav_cmd.id = MAV_CMD_NAV_WAYPOINT;
			_nav_cmd.content.location.lat = _break_lat;
			_nav_cmd.content.location.lng = _break_lng;
			_nav_cmd.content.location.alt = _break_alt;
			return;
		}else{
			return;
		}
	case CHECK_GO_BREAK_RESUME:
		if((_break_index != 0) && (breakState !=3)){ //如果有中断点则第一次执行中断点
			breakState = 1;
			isHoldIndex = 1;
			_nav_cmd.index = _break_index;
			_nav_cmd.id = MAV_CMD_NAV_WAYPOINT;
			_nav_cmd.content.location.lat = _break_lat;
			_nav_cmd.content.location.lng = _break_lng;
			_nav_cmd.content.location.alt = _break_alt;
		}
		return;
	case SET_BREAK_STATE:
		if(_break_index == _nav_cmd.index && breakState == 2){ //如果当前航点号等于中断点航点号且中断状态为飞向中断点，则标记状态为已到达中断点
			breakState = 3;
		}
		return;
	case CHECK_DELETE_BREAK:
		if(_nav_cmd.index == (unsigned)(_cmd_total-1)){ //如果执行到了最后一个航点（返航点）则取消中断点

		}else{
			return;
		}
	case DELETE_BREAK_POSITION:
	    _break_index.set_and_save(0); //新航线上传则取消中断点
		return;
	default:
		return;
	}
}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current nav command has not completed
    if (_flags.nav_cmd_loaded) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // get starting point for search
    cmd_index = _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {  //航点索引超限
        // start from beginning of the mission command list
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current nav command
        cmd_index++;
    }

    autoClimb(GET_WAYPOINT_INDEX,&cmd_index);
    breakPosition(SET_BREAK_STATE);
    // avoid endless loops
    uint8_t max_loops = 255;

    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded) { //搜索直到我们找到下一个导航命令或到达命令列表的结束
        // get next command
        if (!get_next_cmd(cmd_index, cmd, true)) {//获取下一个航点命令
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {  //如果是导航命令
            // save previous nav command index 保存以前的导航命令索引
            _prev_nav_cmd_id = _nav_cmd.id;
            _prev_nav_cmd_index = _nav_cmd.index;
            // save separate previous nav command index if it contains lat,long,alt
            if (!(cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
                _prev_nav_cmd_wp_index = _nav_cmd.index;  //保存上一个航点索引
            }
            // set current navigation command and start it
            _nav_cmd = cmd;
            _flags.nav_cmd_loaded = true;
            breakPosition(CHECK_DELETE_BREAK);
			breakPosition(CHECK_GO_BREAK);
			autoClimb(CHECK_NEED_CLIMB);
            _cmd_start_fn(_nav_cmd);  //启动这个导航命令，直接通过指针调用mode_auto.cpp的函数
        }else{  //do命令
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) { //如果没有加载do命令
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                _cmd_start_fn(_do_cmd); //启动do命令
            } else {
                // protect against endless loops of do-commands
                if (max_loops-- == 0) { //避免死循环
                    return false;
                }
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
    if (!_flags.do_cmd_loaded) {
        _flags.do_cmd_all_done = true;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running or we've completed all possible "do" commands
    if (_flags.state != MISSION_RUNNING || _flags.do_cmd_all_done) {
        return;
    }

    // get starting point for search
    cmd_index = _do_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current do command
        cmd_index++;
    }

    // check if we've reached end of mission
    if (cmd_index >= (unsigned)_cmd_total) {
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return;
    }

    // find next do command
    if (get_next_do_cmd(cmd_index, cmd)) {
        // set current do command and start it
        _do_cmd = cmd;
        _flags.do_cmd_loaded = true;
        _cmd_start_fn(_do_cmd);
    }else{
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
    }
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found)
{
    uint16_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint16_t jump_index = AP_MISSION_CMD_INDEX_NONE;

    // search until the end of the mission command list
    uint8_t max_loops = 64;
    while(cmd_index < (unsigned)_cmd_total) {
        // load the next command
        if (!read_cmd_from_storage(cmd_index, temp_cmd)) {
            // this should never happen because of check above but just in case
            return false;
        }

        // check for do-jump command
        if (temp_cmd.id == MAV_CMD_DO_JUMP) {

            if (max_loops-- == 0) {
                return false;
            }

            // check for invalid target
            if ((temp_cmd.content.jump.target >= (unsigned)_cmd_total) || (temp_cmd.content.jump.target == 0)) {
                // To-Do: log an error?
                return false;
            }

            // check for endless loops
            if (!increment_jump_num_times_if_found && jump_index == cmd_index) {
                // we have somehow reached this jump command twice and there is no chance it will complete
                // To-Do: log an error?
                return false;
            }

            // record this command so we can check for endless loops
            if (jump_index == AP_MISSION_CMD_INDEX_NONE) {
                jump_index = cmd_index;
            }

            // check if jump command is 'repeat forever'
            if (temp_cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER) {
                // continue searching from jump target
                cmd_index = temp_cmd.content.jump.target;
            }else{
                // get number of times jump command has already been run
                int16_t jump_times_run = get_jump_times_run(temp_cmd);
                if (jump_times_run < temp_cmd.content.jump.num_times) {
                    // update the record of the number of times run
                    if (increment_jump_num_times_if_found) {
                        increment_jump_times_run(temp_cmd);
                    }
                    // continue searching from jump target
                    cmd_index = temp_cmd.content.jump.target;
                }else{
                    // jump has been run specified number of times so move search to next command in mission
                    cmd_index++;
                }
            }
        }else{
            // this is a non-jump command so return it
            cmd = temp_cmd;
            return true;
        }
    }

    // if we got this far we did not find a navigation command
    return false;
}

/// get_next_do_cmd - gets next "do" or "conditional" command after start_index
///     returns true if found, false if not found
///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_do_cmd(uint16_t start_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;

    // check we have not passed the end of the mission list
    if (start_index >= (unsigned)_cmd_total) {
        return false;
    }

    // get next command
    if (!get_next_cmd(start_index, temp_cmd, false)) {
        // no more commands so return failure
        return false;
    }else if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
        return false;
    }else{
        // this must be a "do" or "conditional" and is not a do-jump command so return it
        cmd = temp_cmd;
        return true;
    }
}

///
/// jump handling methods
///

// init_jump_tracking - initialise jump_tracking variables
void AP_Mission::init_jump_tracking()
{
    for(uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i].index = AP_MISSION_CMD_INDEX_NONE;
        _jump_tracking[i].num_times_run = 0;
    }
}

/// get_jump_times_run - returns number of times the jump command has been run
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command or target is invalid
    if ((cmd.id != MAV_CMD_DO_JUMP) || (cmd.content.jump.target >= (unsigned)_cmd_total) || (cmd.content.jump.target == 0)) {
        // To-Do: log an error?
        return AP_MISSION_JUMP_TIMES_MAX;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            return _jump_tracking[i].num_times_run;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 0;
            return 0;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error?
    return AP_MISSION_JUMP_TIMES_MAX;
}

/// increment_jump_times_run - increments the recorded number of times the jump command has been run
void AP_Mission::increment_jump_times_run(Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command
    if (cmd.id != MAV_CMD_DO_JUMP) {
        // To-Do: log an error?
        return;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            _jump_tracking[i].num_times_run++;
            return;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 1;
            return;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error
    return;
}

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Mission::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_MISSION_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_MISSION_EEPROM_VERSION);
        }
    }
}

/*
  return total number of commands that can fit in storage space
 */
uint16_t AP_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}

// find the nearest landing sequence starting point (DO_LAND_START) and
// return its index.  Returns 0 if no appropriate DO_LAND_START point can
// be found.
uint16_t AP_Mission::get_landing_sequence_start() 
{
    struct Location current_loc;

    if (!_ahrs.get_position(current_loc)) {
        return 0;
    }

    uint16_t landing_start_index = 0;
    float min_distance = -1;

    // Go through mission looking for nearest landing start command
    for (uint16_t i = 0; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == MAV_CMD_DO_LAND_START) {
            float tmp_distance = get_distance(tmp.content.location, current_loc);
            if (min_distance < 0 || tmp_distance < min_distance) {
                min_distance = tmp_distance;
                landing_start_index = i;
            }           
        }
    }

    return landing_start_index;
}

/*
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
bool AP_Mission::jump_to_landing_sequence(void)
{
    uint16_t land_idx = get_landing_sequence_start();
    if (land_idx != 0 && set_current_cmd(land_idx)) {

        //if the mission has ended it has to be restarted
        if (state() == AP_Mission::MISSION_STOPPED) {
            resume();
        }

        gcs().send_text(MAV_SEVERITY_INFO, "Landing sequence start");
        return true;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Unable to start landing sequence");
    return false;
}

const char *AP_Mission::Mission_Command::type() const {
    switch(id) {
    case MAV_CMD_NAV_WAYPOINT:
        return "WP";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "RTL";
    case MAV_CMD_NAV_LOITER_UNLIM:
        return "LoitUnlim";
    case MAV_CMD_NAV_LOITER_TIME:
        return "LoitTime";
    case MAV_CMD_NAV_SET_YAW_SPEED:
        return "SetYawSpd";
    case MAV_CMD_CONDITION_DELAY:
        return "CondDelay";
    case MAV_CMD_CONDITION_DISTANCE:
        return "CondDist";
    case MAV_CMD_DO_CHANGE_SPEED:
        return "ChangeSpeed";
    case MAV_CMD_DO_SET_HOME:
        return "SetHome";
    case MAV_CMD_DO_SET_SERVO:
        return "SetServo";
    case MAV_CMD_DO_SET_RELAY:
        return "SetRelay";
    case MAV_CMD_DO_REPEAT_SERVO:
        return "RepeatServo";
    case MAV_CMD_DO_REPEAT_RELAY:
        return "RepeatRelay";
    case MAV_CMD_DO_CONTROL_VIDEO:
        return "CtrlVideo";
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        return "DigiCamCfg";
    case MAV_CMD_DO_DIGICAM_CONTROL:
        return "DigiCamCtrl";
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return "SetCamTrigDst";
    case MAV_CMD_DO_SET_ROI:
        return "SetROI";
    case MAV_CMD_DO_SET_REVERSE:
        return "SetReverse";
    default:
        return "?";
    }
}
