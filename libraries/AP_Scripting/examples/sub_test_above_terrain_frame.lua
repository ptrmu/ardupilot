

local UPDATE_PERIOD_MS = 50

-- These strings must match the strings used by the test driver for interpreting the output from this test.
local TEST_ID_STR = "STTF"
local COMPLETE_STR = "#complete#"
local SUCCESS_STR = "!!success!!"
local FAILURE_STR = "!!failure!!"

-- Copied from libraries/AP_Math/rotation.h enum Rotation {}.
local RNGFND_ORIENTATION_DOWN = 25
local RNGFND_ORIENTATION_FORWARD = 0
-- Copied from libraries/AP_RangeFinder/AP_RanggeFinder.h enum RangeFinder::Type {}.
local RNGFND_TYPE_LUA = 36.0
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Status {}.
local RNGFND_STATUS_NOT_CONNECTED = 0
local RNGFND_STATUS_NO_DATA = 1
local RNGFND_STATUS_OUT_OF_RANGE_LOW = 2
local RNGFND_STATUS_OUT_OF_RANGE_HIGH = 3
local RNGFND_STATUS_GOOD = 4
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h
local SIGNAL_QUALITY_MIN = 0
local SIGNAL_QUALITY_MAX = 100
local SIGNAL_QUALITY_UNKNOWN = -1



-------------------------------------------------------------------------------

-- gcs messaging function. Pass two strings and the function will drop frequent messages

local gcs_send_funcfactory = function(name, eat_messages_period_s, msg_severity)
    local gcs_send_times = {}
    local gcs_eaten_count = {}

    ---@param str1 string
    ---@param str2 string|nil
    return function(str1, str2)
        if not str1 or #str1 == 0 then return end
        if not msg_severity then msg_severity = 6 end
        if not eat_messages_period_s then eat_messages_period_s = 1.0 end
        
        local send_str = nil

        if not str2 or #str2 == 0 then
            send_str = string.format("%s: %s", name, str1)
        
        else
            local time_curr_s = millis():tofloat() / 1000.0
            local time_first_s = gcs_send_times[str1]
            if (time_first_s) then
                local dur_since_first = time_curr_s - time_first_s
                if dur_since_first < eat_messages_period_s then
                    if not gcs_eaten_count[str1] then
                        gcs_eaten_count[str1] = 0
                    end
                    gcs_eaten_count[str1] = gcs_eaten_count[str1] + 1
                    return
                end
            end

            local eaten_count = gcs_eaten_count[str1]
            if eaten_count then
                gcs_eaten_count[str1] = nil
                if str2 then
                    send_str = string.format("%s: %s %s (+%i)", name, str1, str2, eaten_count)
                else
                    send_str = string.format("%s: %s (+%i)", name, str1, eaten_count)
                end
            else
                if str2 then
                    send_str = string.format("%s: %s %s", name, str1, str2)
                else
                    send_str = string.format("%s: %s", name, str1)
                end
            end

            gcs_send_times[str1] = time_curr_s
        end

        gcs:send_text(msg_severity, send_str)
    end
end

local send = gcs_send_funcfactory(TEST_ID_STR, 1.0, 3)

local function failed(error)
    send(string.format("%s %s %s", COMPLETE_STR, FAILURE_STR, error))
end

-------------------------------------------------------------------------------

-- Profile definition and intersection used in Sea Floor Model

local function section_factory(x0, z0, x1, z1)

    local function intersect(self, seg)
        local den = - self.vx*seg.vz + self.vz*seg.vx
        if math.abs(den) < 1.0e-6 then
            return nil
        end
        local x1m0 = seg.x0 - self.x0
        local z1m0 = seg.z0 - self.z0
        local t0 = - x1m0*seg.vz + z1m0*seg.vx
        local t1 = self.vx*z1m0 - self.vz*x1m0
        return t0 / den, t1 / den
    end

    return {
        x0 = x0,
        z0 = z0,
        vx = x1 - x0,
        vz = z1 - z0,
        intersect = intersect,
    }
end

-- psi is an angle in radians rotating around the y axis with zero at the z axis
local function ray_factory(x, z, psi)
    return section_factory(x, z, x + math.sin(psi), z + math.cos(psi))
end

local function profile_factory(vertices)
    local sections = {}

    -- The first section covers to -infinity
    if #vertices > 0 then
        local vertex1 = vertices[1]
        table.insert(sections, section_factory(vertex1[1], vertex1[2], vertex1[1] - 1, vertex1[2]))
    end

    local last_vertex = nil
    for _, vertex in pairs(vertices) do
        if not last_vertex then
            last_vertex = vertex
        else
            local section = section_factory(last_vertex[1], last_vertex[2], vertex[1], vertex[2])
            if math.abs(section.vx) > 1.e-6 or math.abs(section.vy) > 1.e-6 then
                -- Only add section if it has non-zero length, otherwise ignore this vertex.
                table.insert(sections, section)
                last_vertex = vertex
            end
        end
    end

    -- The last section covers to +infinity
    if last_vertex then
        table.insert(sections, section_factory(last_vertex[1], last_vertex[2], last_vertex[1] + 1, last_vertex[2]))
    end


    -- Returns the distance to the closest section.
    sections.intersect = function(self, ray)
        local d = nil

        for i, segment in ipairs(self) do

            local function find_valid_intersection()
                local s, r = segment:intersect(ray)
                if not s then
                    return nil
                end
                if s < 0 or r < 0 then
                    return nil
                end
                if i ~= 1 and i ~= #self then
                    if s > 1 then
                        return nil
                    end
                end
                return r
            end

            local d1 = find_valid_intersection()
            if d1 then
                -- Found a valid intersection, look for the shortest distance
                if not d or d > d1 then
                    d = d1
                end
            end
        end

        return d
    end

    return sections
end

do
    local section = section_factory(0, 1, 1, 1)

    local function test(x, z, psi, s_expected, r_expected)
        local ray = ray_factory(x, z, psi)

        -- Test the section:intersect method.
        local s_actual, r_actual = section:intersect(ray)

        if not s_actual or not s_expected then
            if s_actual or s_expected then
                local actual_str = not s_actual and "nil" or string.format("%.2f", s_actual)
                local expected_str = not s_expected and "nil" or string.format("%.2f", s_expected)
                send(string.format("intersect not nil  : x %.2f, z %.2f, psi %.2f", x, z, psi) ..
                    string.format(" : s_actual %s, s_expected %s", actual_str, expected_str))
            end
        else
            if math.abs(s_actual - s_expected) > 1.e-6 or math.abs(r_actual - r_expected) > 1.e-6 then
                send(string.format("intersect failed  : x %.2f, z %.2f, psi %.2f", x, z, psi) ..
                    string.format(" : s_actual %.2f, s_expected %.2f, r_actual %.2f, r_expected %.2f", 
                        s_actual, s_expected, r_actual, r_expected))
            end
        end
    end

    test(-1, 0, 0,  -1, 1)
    test(0, 0, 0,  0, 1)
    test(1, 0, 0,  1, 1)
    test(2, 0, 0,  2, 1)

    test(-1, 0.5, 0,  -1, .5)
    test(0, 0.5, 0,  0, .5)
    test(1, 0.5, 0,  1, .5)
    test(2, 0.5, 0,  2, .5)

    test(-1, 1, 0,  -1, 0)
    test(0, 1, 0,  0, 0)
    test(1, 1, 0,  1, 0)
    test(2, 1, 0,  2, 0)

    test(0, 0, math.pi/2,  nil, 0)
    test(0, 1, math.pi/2,  nil, 0)
    test(1, 1, math.pi/2,  nil, 0)
    test(1, 2, math.pi/2,  nil, 0)

    test(0, 0, math.pi/4,  1, math.sqrt(2))
    test(1, 0, math.pi/4,  2, math.sqrt(2))
    test(0, 0, -math.pi/4,  -1, math.sqrt(2))
    test(1, 0, -math.pi/4,  0, math.sqrt(2))
end

do
    local profile = profile_factory({{0, 1}, {1, 2}})

    local function test(x, z, psi, d_expected)
        local ray = ray_factory(x, z, psi)

        -- Test the profile:intersect method.
        local d_actual = profile:intersect(ray)

        if math.abs(d_actual - d_expected) > 1.e-6 then
            send(string.format("intersect failed x %.2f, z %.2f, psi %.2f, d_actual %.2f, d_expected %.2f",
                x, z, psi, d_actual, d_expected))
        end
    end

    test(-20, 0, 0, 1)
    test(-1, 0, 0, 1)
    test(0, 0, 0, 1)
    test(0.5, 0, 0, 1.5)
    test(1, 0, 0, 2)
    test(1.5, 0, 0, 2)
    test(20, 0, 0, 2)

    test(-20, 0, math.pi/4, math.sqrt(2))
    test(-1, 0, math.pi/4, math.sqrt(2))
    test(-0.99, 0, math.pi/4, 2*math.sqrt(2))
    test(0, 0, math.pi/4, 2*math.sqrt(2))
    test(0.5, 0, math.pi/4, 2*math.sqrt(2))
    test(1, 0, math.pi/4, 2*math.sqrt(2))
    test(1.5, 0, math.pi/4, 2*math.sqrt(2))
    test(20, 0, math.pi/4, 2*math.sqrt(2))

    test(-20, 0, -math.pi/4, math.sqrt(2))
    test(-1, 0, -math.pi/4, math.sqrt(2))
    test(0, 0, -math.pi/4, math.sqrt(2))
    test(0.5, 0, -math.pi/4, math.sqrt(2))
    test(1, 0, -math.pi/4, math.sqrt(2))
    test(1.5, 0, -math.pi/4, 1.25*math.sqrt(2))
    test(2, 0, -math.pi/4, 1.5*math.sqrt(2))
    test(3, 0, -math.pi/4, 2*math.sqrt(2))
    test(20, 0, -math.pi/4, 2*math.sqrt(2))
end

-------------------------------------------------------------------------------

-- Sea Floor Model

local function sea_floor_model_factory(model_bearing_N_rad, model_depth_m, vertices)

    ---@class Location_ud
    local origin_loc

    local profile = profile_factory(vertices)

    ---@param sub_loc Location_ud
    local function get_range(self, sub_loc)

        -- Figure out the depth of the sub in absolute frame
        sub_loc:change_alt_frame(0)
        self.sub_z_m = -sub_loc:alt()/100

        -- If the origin has not been set then do not use the profile
        if not origin_loc then
            self.bottom_z_m = model_depth_m
            self.range_m = model_depth_m - self.sub_z_m
            return self.range_m
        end

        -- The model origin has been set so use the profile.
        -- N in name means relative to Earth frame, M in name means relative to Sea floor model frame.
        local sub_bearing_N_rad = origin_loc:get_bearing(sub_loc)
        local sub_distance_M_m = origin_loc:get_distance(sub_loc)

        local sub_bearing_M_rad = sub_bearing_N_rad - model_bearing_N_rad
        local sub_northly_M_m = math.cos(sub_bearing_M_rad) * sub_distance_M_m

        send("SFM", string.format("sub_z %.2f, bearing %.2f, range %.2f", self.sub_z_m, sub_bearing_N_rad, sub_distance_M_m))

        -- profile:intersect has an origin at zero so we must calculate how far 
        -- the sub is above that origin. And also calculate the depth of the
        -- sea floor below the sub's location.
        -- For the sea floor depth:
        -- bottom_z = model_depth_m + profile(0)
        local bottom_ray = ray_factory(sub_northly_M_m, 0, 0)
        self.bottom_z_m = model_depth_m + profile:intersect(bottom_ray)

        -- For the sub range measurement:
        -- model_range = model_depth_m - sub_z
        -- range = model_depth_m + profile(0) - sub_z;
        -- range = profile(-model_range);
        local sub_above_floor_m = model_depth_m - self.sub_z_m
        local sub_ray = ray_factory(sub_northly_M_m, -sub_above_floor_m, 0)
        self.range_m = profile:intersect(sub_ray)
        return self.range_m
    end

    return {
        sub_z_m = 0,
        bottom_z_m = model_depth_m,
        range_m = model_depth_m,
        set_origin = function(self, origin) origin_loc = origin end,
        valid_origin = function(self) return origin_loc ~= nil end,
        get_range = get_range,
    }
end

local function get_sea_floor_model()
    return sea_floor_model_factory(math.pi, 45, {{5, 0}, {25, 10}, {50, 0}})
end

-------------------------------------------------------------------------------

-- Range Finder Driver

-- The range finder backend is initialized in the update_init function.
---@type AP_RangeFinder_Backend_ud
local rngfnd_backend
local sea_floor_model

local function range_finder_driver(sub_loc)

    local rf_state = RangeFinder_State()
    -- The full state udata must be initialized.
    rf_state:last_reading(millis():toint())
    rf_state:voltage(0)

    local range_m = nil
    if sub_loc then
        range_m = sea_floor_model:get_range(sub_loc)
    end

    if not range_m then
        rf_state:status(RNGFND_STATUS_NO_DATA)
        rf_state:range_valid_count(0)
        rf_state:distance(0)
        rf_state:signal_quality(SIGNAL_QUALITY_MIN)
    else
        rf_state:status(RNGFND_STATUS_GOOD)
        rf_state:range_valid_count(10)
        rf_state:distance(range_m)
        rf_state:signal_quality(SIGNAL_QUALITY_MAX)
    end

    rngfnd_backend:handle_script_msg(rf_state) -- state as arg
end

-------------------------------------------------------------------------------

-- update functions

local function update_run()

    local loc_c = ahrs:get_location()

    -- Check if we have to set or clear the origin
    if arming:is_armed() ~= sea_floor_model:valid_origin() then
        if arming:is_armed() then
            send("Starting to send sea floor range data")
            sea_floor_model:set_origin(loc_c)
        else
            send("Ending range data")
            sea_floor_model:set_origin(nil)
        end
    end

    -- Update with range finder driver
    local error_str = range_finder_driver(loc_c)
    if error_str then
        return failed(error_str)
    end

    -- Log some data
    if sea_floor_model.range_m then
        send("RNGFND", string.format("range %.2f, sub_z %.2f, bottom_z %.2f",
            sea_floor_model.range_m, sea_floor_model.sub_z_m, sea_floor_model.bottom_z_m))
        logger:write('RNFN', 'sub_z,bottom_z,true_rngfnd,rngfnd', 'ffff', 'mmmm', '----',
            sea_floor_model.sub_z_m, sea_floor_model.bottom_z_m, sea_floor_model.range_m, sea_floor_model.range_m)
    else
        send("RNGFND", "failed to find a range. Maybe sub below bottom.")
    end

    return update_run, UPDATE_PERIOD_MS
end

local function update_init()
    if Parameter('RNGFND1_TYPE'):get() ~= RNGFND_TYPE_LUA then
        return failed("LUA range finder driver not enabled")
    end
    if rangefinder:num_sensors() < 1 then
        return failed("LUA range finder driver not connected")
    end
    rngfnd_backend = rangefinder:get_backend(0)
    if not rngfnd_backend then
        return failed("Range Finder 1 does not exist")
    end
    if (rngfnd_backend:type() ~= RNGFND_TYPE_LUA) then
        return failed("Range Finder 1 is not a LUA driver")
    end

    sea_floor_model = get_sea_floor_model()

    return update_run, 0
end

send("Loaded sub_test_above_terrain_frame.lua")

return update_init, 0
