require('config_utils')

is_real_bot = false

num_push_up = 3
push_up_time_scale = 0.8

local graph = require('common_graph_define')
graph.init(is_real_bot)

local options = require('common_module_options_final_no_record')
options.init(is_real_bot)
options.resetPushUp(num_push_up, push_up_time_scale)
